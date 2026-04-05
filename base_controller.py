import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from dynamixel_sdk import *
import math

# --- CONFIGURAÇÕES DO HARDWARE ---
BAUDRATE = 57600
DEVICE_L = '/dev/ttyUSB1' # ID 4
DEVICE_R = '/dev/ttyUSB0' # ID 3
PROTOCOL_VERSION = 2.0

DXL_ID_L = 4
DXL_ID_R = 3

# Medidas reais do seu robô (ajuste se necessário)
R = 0.0688  # Raio da roda (m)
L = 0.176   # Distância entre rodas (m)
TICKS_PER_REV = 4096 # Resolução do XH430

# Endereços Dynamixel
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132 # Onde lemos os encoders

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        # Publishers e Broadcasters
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Variáveis de Estado da Odometria
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_l_ticks = None
        self.last_r_ticks = None
        self.last_time = self.get_clock().now()

        # Handlers
        self.portL = PortHandler(DEVICE_L)
        self.portR = PortHandler(DEVICE_R)
        self.ph = PacketHandler(PROTOCOL_VERSION)

        if self.portL.openPort() and self.portL.setBaudRate(BAUDRATE) and \
           self.portR.openPort() and self.portR.setBaudRate(BAUDRATE):
            self.get_logger().info('✅ Conectado aos Motores e pronto para Odometria!')
        else:
            self.get_logger().error('❌ Erro nas portas USB!')
            return

        self.setup_motors()

        # Timer para calcular Odometria (20Hz - ideal para SLAM)
        self.timer = self.create_timer(0.05, self.update_odometry)

    def setup_motors(self):
        for port, dxl_id in [(self.portL, DXL_ID_L), (self.portR, DXL_ID_R)]:
            self.ph.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            self.ph.write1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE, 1)
            self.ph.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)

    def update_odometry(self):
        # 1. Ler encoders
        p_l, comm_l, _ = self.ph.read4ByteTxRx(self.portL, DXL_ID_L, ADDR_PRESENT_POSITION)
        p_r, comm_r, _ = self.ph.read4ByteTxRx(self.portR, DXL_ID_R, ADDR_PRESENT_POSITION)

        if comm_l != COMM_SUCCESS:
            self.get_logger().error(f"❌ Falha no Motor Esq (ID {DXL_ID_L}): {self.ph.getTxRxResult(comm_l)}")
            return
            
        if comm_r != COMM_SUCCESS:
            self.get_logger().error(f"❌ Falha no Motor Dir (ID {DXL_ID_R}): {self.ph.getTxRxResult(comm_r)}")
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if self.last_l_ticks is not None:
            # 2. Calcular deslocamento (Direita invertida no chassi)
            d_l = (p_l - self.last_l_ticks) / TICKS_PER_REV * (2 * math.pi * R)
            d_r = -(p_r - self.last_r_ticks) / TICKS_PER_REV * (2 * math.pi * R)

            # 3. Cinemática Diferencial
            dS = (d_r + d_l) / 2.0
            dTh = (d_r - d_l) / L

            self.x += dS * math.cos(self.th + dTh/2.0)
            self.y += dS * math.sin(self.th + dTh/2.0)
            self.th += dTh

            # 4. Publicar TF (Para o SLAM Toolbox/Rviz)
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = math.sin(self.th / 2.0)
            t.transform.rotation.w = math.cos(self.th / 2.0)
            self.tf_broadcaster.sendTransform(t)

            # 5. Publicar Odometry Message
            odom = Odometry()
            odom.header = t.header
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = t.transform.rotation
            self.odom_pub.publish(odom)

        self.last_l_ticks = p_l
        self.last_r_ticks = p_r
        self.last_time = now

    def cmd_vel_callback(self, msg):
        v, w = msg.linear.x, msg.angular.z
        v_l = (v - w * (L / 2.0)) / R
        v_r = (v + w * (L / 2.0)) / R
        u_l = int((v_l * 60 / (2 * math.pi)) / 0.229)
        u_r = -int((v_r * 60 / (2 * math.pi)) / 0.229)
        self.ph.write4ByteTxRx(self.portL, DXL_ID_L, ADDR_GOAL_VELOCITY, u_l)
        self.ph.write4ByteTxRx(self.portR, DXL_ID_R, ADDR_GOAL_VELOCITY, u_r)

def main():
    rclpy.init()
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portL.closePort()
        node.portR.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()