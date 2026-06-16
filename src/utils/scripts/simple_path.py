# =============================================================================
# KINEMATIC PATH PLANNING NODE
# This script implements the autonomous navigation state-machine. It executes 
# a predefined sequence of geometric primitives (straight lines and arcs) to 
# ensure comprehensive LiDAR coverage of the experimental arena without relying 
# on harsh in-place rotations.
# [See Section 4.3.2: Complementary Nodes]
# [See Section 4.4.2: Kinematic Path Planning and Trajectory Execution]
# =============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SequenceNavNode(Node):
    def __init__(self):
        super().__init__('sequence_nav_node')
        
        # =============================================================================
        # GENERAL SETTINGS & ROS 2 SETUP
        # Configures the base linear speed and turn tolerances, alongside the 
        # publishers (/cmd_vel) and subscribers (/odom) necessary for closed-loop 
        # relative navigation.
        # =============================================================================
        self.linear_speed = 0.05  # Base linear speed in m/s
        self.turn_tolerance = 0.04 # Tolerance for 90-degree turn completion
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # State Variables
        self.state = 'FETCH_CMD' 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.received_odom = False
        
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        
        # Sequence Execution Variables
        self.command_sequence = []
        self.current_cmd_idx = 0
        self.current_target_val = 0.0  # Stores either distance (m) or radius (m)

        # Debug Variables
        self.print_counter = 0 # Used to prevent terminal flooding
        self.get_logger().info("🚀 Nó de Navegação Iniciado! Aguardando o sensor de odometria...")

        # =============================================================================
        # PREDEFINED NAVIGATION TRAJECTORY
        # Defines the specific path to navigate the experimental arena.
        # [See Section 4.4.2: Figure 16 - Predefined autonomous navigation trajectory]
        # =============================================================================
        self.add_straight(200)       
        self.add_right(0.25)          
        self.add_right(0.25) 
        self.add_straight(20)      
        self.add_left(0.25)  
        self.add_straight(500) 
        self.add_left(0.25) 

    # =============================================================================
    # COMMAND BUILDER FUNCTIONS
    # Populates the trajectory queue.
    # =============================================================================
    def add_straight(self, distance_mm):
        # Converts mm to meters internally
        distance_m = distance_mm / 1000.0
        self.command_sequence.append({'type': 'STRAIGHT', 'value': distance_m})

    def add_left(self, radius_m):
        self.command_sequence.append({'type': 'TURN_LEFT', 'value': radius_m})

    def add_right(self, radius_m):
        self.command_sequence.append({'type': 'TURN_RIGHT', 'value': radius_m})

    # =============================================================================
    # ODOMETRY SENSOR CALLBACK
    # Updates the internal spatial state of the robot required to evaluate 
    # the completion of the current geometric maneuver.
    # =============================================================================
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_x = pos.x
        self.current_y = pos.y
        
        # Convert quaternion to Euler angle (yaw/theta)
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        # DEBUG: Notify only the first time data is received
        if not self.received_odom:
            self.get_logger().info("✅ ODOMETRIA RECEBIDA! O robô agora sabe onde está. Iniciando a sequência.")
            
        self.received_odom = True

    def normalize_angle(self, angle):
        """ Keeps the angle strictly between -PI and PI """
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    # =============================================================================
    # MAIN CONTROL LOOP (STATE MACHINE)
    # Iterates at 20Hz, evaluating the current odometry against the active 
    # command target. Calculates the necessary linear and angular velocities 
    # to fulfill the maneuver.
    # [See Section 4.3.2: Complementary Nodes]
    # =============================================================================
    def control_loop(self):
        # DEBUG: Check if odometry is arriving
        if not self.received_odom:
            if not hasattr(self, 'odom_wait_printed'):
                self.get_logger().warning("⚠️ TRAVADO: Aguardando dados em '/odom'. Verifique se o base_controller.py está rodando!")
                self.odom_wait_printed = True
            return

        msg = Twist()
        
        # Print filter (prints every 10 cycles = 0.5 seconds)
        self.print_counter += 1
        should_print = (self.print_counter % 10 == 0)

        # 1. FETCH NEXT COMMAND
        if self.state == 'FETCH_CMD':
            if self.current_cmd_idx >= len(self.command_sequence):
                self.get_logger().info("🏁 Sequência finalizada! Parando o robô.")
                self.state = 'DONE'
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                return

            # Get current command dictionary
            cmd = self.command_sequence[self.current_cmd_idx]
            self.state = cmd['type']
            self.current_target_val = cmd['value']
            
            # Save starting position/orientation for relative calculation
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta
            
            self.get_logger().info(f"▶️ Iniciando passo {self.current_cmd_idx + 1}/{len(self.command_sequence)}: {self.state} com valor {self.current_target_val}")

        # 2. EXECUTE STRAIGHT COMMAND
        elif self.state == 'STRAIGHT':
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance_driven = math.sqrt(dx*dx + dy*dy)

            if should_print:
                self.get_logger().info(f"   Andando Reto: {distance_driven:.3f}m / {self.current_target_val:.3f}m")

            if distance_driven < self.current_target_val:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            else:
                self.get_logger().info("✅ Movimento Reto concluído.")
                self.current_cmd_idx += 1
                self.state = 'FETCH_CMD'

        # 3. EXECUTE LEFT OR RIGHT TURN (90 DEGREES)
        elif self.state in ['TURN_LEFT', 'TURN_RIGHT']:
            # Calculate how much we have turned since the start of the command
            delta_theta = self.normalize_angle(self.current_theta - self.start_theta)
            
            # 90 degrees is pi/2 radians
            target_angle = (math.pi / 2.0)
            
            if should_print:
                self.get_logger().info(f"   Girando: {math.degrees(abs(delta_theta)):.1f}° / {math.degrees(target_angle):.1f}°")
            
            if abs(delta_theta) < (target_angle - self.turn_tolerance):
                msg.linear.x = self.linear_speed
                
                # Calculate angular speed (w = v / r)
                turn_speed = self.linear_speed / self.current_target_val
                
                # Apply direction (Positive for Left, Negative for Right)
                if self.state == 'TURN_LEFT':
                    msg.angular.z = turn_speed
                else:
                    msg.angular.z = -turn_speed
            else:
                self.get_logger().info(f"✅ Curva {self.state} concluída.")
                self.current_cmd_idx += 1
                self.state = 'FETCH_CMD'
                
        # 4. DONE STATE
        elif self.state == 'DONE':
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        if should_print and self.state not in ['FETCH_CMD', 'DONE']:
             self.get_logger().info(f"   📡 Enviando para as rodas -> Velocidade: {msg.linear.x:.2f}, Giro: {msg.angular.z:.2f}")

        # Send speed to robot
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SequenceNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
    finally:
        # Send zero velocity before shutting down
        zero_msg = Twist()
        node.publisher_.publish(zero_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()