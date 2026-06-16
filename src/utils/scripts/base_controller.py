# =============================================================================
# HARDWARE INTERFACE NODE (BASE CONTROLLER)
# This script bridges the high-level ROS 2 ecosystem with the low-level physical 
# hardware. It manages the serial communication with the Dynamixel XH430-W350-R 
# servomotors, converts /cmd_vel twist messages into raw wheel velocities, and 
# calculates the real-time odometry using the differential drive kinematic model 
# to be published to the FastSLAM node.
# [See Section 4.2.4: Hardware Interface ROS2 Nodes]
# [See Section 2.1.1: Differential Drive Kinematics]
# =============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from dynamixel_sdk import *
import math

# =============================================================================
# HARDWARE CONFIGURATIONS & KINEMATIC PARAMETERS
# Specifies the serial communication parameters for the RS-485 bus and the 
# physical dimensions of the DIY TurtleBot3 chassis. The wheel radius (R) is 
# adjusted to account for the larger LEGO 44771 tires.
# [See Section 4.1.1: Component List and Electronic Architecture]
# [See Section 4.2.2: Microcontroller Firmware]
# =============================================================================
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

DXL_ID_L = 4
DXL_ID_R = 3

# Real measurements of your robot (adjusted for LEGO tires)
R = 0.0688 / 2   # Wheel radius (m)
L = 0.176        # Distance between wheels (m)
TICKS_PER_REV = 4096 # XH430 Resolution

# Dynamixel Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132 # Where we read the encoders

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        # Publishers and Broadcasters
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Odometry State Variables
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_l_ticks = None
        self.last_r_ticks = None
        self.last_time = self.get_clock().now()

        # Handlers base
        self.ph = PacketHandler(PROTOCOL_VERSION)
        
        # --- DIRECT MOTOR CONNECTION ---
        self.port = self.find_dynamixel_port()

        if self.port is None:
            self.get_logger().error('❌ ERRO FATAL: Falha ao conectar na porta /dev/motores!')
            return

        self.setup_motors()

        # Timer to calculate Odometry (20Hz - ideal for SLAM)
        self.timer = self.create_timer(0.2, self.update_odometry)

    def find_dynamixel_port(self):
        """Connects directly to the fixed port defined by the UDEV rule"""
        port_name = '/dev/motores'
        self.get_logger().info(f"🔍 Conectando aos motores na porta fixa: {port_name}")

        test_port = PortHandler(port_name)
        
        if test_port.openPort() and test_port.setBaudRate(BAUDRATE):
            # Attempts to "Ping" the Left motor (ID 4) to confirm communication
            _, comm_result, _ = self.ph.ping(test_port, DXL_ID_L)
            
            if comm_result == COMM_SUCCESS:
                self.get_logger().info(f"✅ Dynamixel conectado com sucesso em {port_name}!")
                return test_port 
            else:
                test_port.closePort()
                self.get_logger().error(f"❌ Porta aberta, mas falha de comunicação (Ping ID {DXL_ID_L}).")
        else:
            self.get_logger().warn(f"⚠️ Não foi possível abrir a porta {port_name}.")
            
        return None

    def setup_motors(self):
        # Using the discovered port to initialize both motor IDs
        for dxl_id in [DXL_ID_L, DXL_ID_R]:
            self.ph.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            self.ph.write1ByteTxRx(self.port, dxl_id, ADDR_OPERATING_MODE, 1)
            self.ph.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)

    # =============================================================================
    # ODOMETRY CALCULATION LOOP (20Hz)
    # Reads the magnetic encoders from both wheels, calculates the physical 
    # displacement using the differential drive kinematic model, and broadcasts 
    # the relative motion (control data u_t) via the /odom topic and TF2 tree.
    # [See Section 2.1.1: Differential Drive Kinematics, Eq. 4 and 5]
    # =============================================================================
    def update_odometry(self):

        self.port.clearPort()
        
        # 1. Read encoders: Both read from the discovered port
        p_l, comm_l, _ = self.ph.read4ByteTxRx(self.port, DXL_ID_L, ADDR_PRESENT_POSITION)
        p_r, comm_r, _ = self.ph.read4ByteTxRx(self.port, DXL_ID_R, ADDR_PRESENT_POSITION)

        if comm_l != COMM_SUCCESS:
            self.get_logger().error(f"❌ Left Motor Failure (ID {DXL_ID_L}): {self.ph.getTxRxResult(comm_l)}")
            return
            
        if comm_r != COMM_SUCCESS:
            self.get_logger().error(f"❌ Right Motor Failure (ID {DXL_ID_R}): {self.ph.getTxRxResult(comm_r)}")
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if self.last_l_ticks is not None:
            # 2. Calculate displacement (Right inverted on chassis)
            d_l = (p_l - self.last_l_ticks) / TICKS_PER_REV * (2 * math.pi * R)
            d_r = -(p_r - self.last_r_ticks) / TICKS_PER_REV * (2 * math.pi * R)

            # 3. Differential Kinematics
            # [See Section 2.1.1: Eqs. 4 and 5]
            dS = (d_r + d_l) / 2.0
            dTh = (d_r - d_l) / L

            self.x += dS * math.cos(self.th + dTh/2.0)
            self.y += dS * math.sin(self.th + dTh/2.0)
            self.th += dTh

            # 4. Publish TF (For SLAM Toolbox/Rviz)
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = math.sin(self.th / 2.0)
            t.transform.rotation.w = math.cos(self.th / 2.0)
            self.tf_broadcaster.sendTransform(t)

            # 5. Publish Odometry Message
            odom = Odometry()
            odom.header = t.header
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = t.transform.rotation
            self.odom_pub.publish(odom)

        self.last_l_ticks = p_l
        self.last_r_ticks = p_r
        self.last_time = now

    # =============================================================================
    # VELOCITY COMMAND CALLBACK
    # Subscribes to the /cmd_vel topic, translates the linear (v) and angular (w) 
    # velocities into specific motor rotations (RPM), and writes them to the hardware.
    # [See Section 4.2.4: Hardware Interface ROS2 Nodes]
    # =============================================================================
    def cmd_vel_callback(self, msg):
        v, w = msg.linear.x, msg.angular.z
        v_l = (v - w * (L / 2.0)) / R
        v_r = (v + w * (L / 2.0)) / R
        u_l = int((v_l * 60 / (2 * math.pi)) / 0.229)
        u_r = -int((v_r * 60 / (2 * math.pi)) / 0.229)
        
        # Sending velocity commands to the hardware
        self.ph.write4ByteTxRx(self.port, DXL_ID_L, ADDR_GOAL_VELOCITY, u_l)
        self.ph.write4ByteTxRx(self.port, DXL_ID_R, ADDR_GOAL_VELOCITY, u_r)

def main():
    rclpy.init()
    node = BaseController()
    try:
        if node.port is not None:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.port is not None:
            node.port.closePort() # Safely closes the communication port
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()