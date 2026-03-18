import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SequenceNavNode(Node):
    def __init__(self):
        super().__init__('sequence_nav_node')
        
        # --- GENERAL SETTINGS ---
        self.linear_speed = 0.05  # Base linear speed in m/s
        self.turn_tolerance = 0.04 # Tolerance for 90-degree turn completion
        
        # --- ROS SETUP ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # --- STATE VARIABLES ---
        self.state = 'FETCH_CMD' 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.received_odom = False
        
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        
        # --- SEQUENCE EXECUTION VARIABLES ---
        self.command_sequence = []
        self.current_cmd_idx = 0
        self.current_target_val = 0.0  # Stores either distance (m) or radius (m)

        # =====================================================================
        # DEFINE YOUR SEQUENCE HERE
        # Use the methods:
        #   self.add_straight(distance_in_mm)
        #   self.add_left(radius_in_meters)
        #   self.add_right(radius_in_meters)
        # =====================================================================
        
        self.add_straight(200)       
        self.add_right(0.25)          
        self.add_right(0.25) 
        self.add_straight(20)      
        self.add_left(0.25)  
        self.add_straight(500) 
        self.add_left(0.25) 
        self.add_left(0.25) 
        self.add_left(0.25) 
         
        
        # =====================================================================

    # --- COMMAND BUILDER FUNCTIONS ---
    def add_straight(self, distance_mm):
        # Converts mm to meters internally
        distance_m = distance_mm / 1000.0
        self.command_sequence.append({'type': 'STRAIGHT', 'value': distance_m})

    def add_left(self, radius_m):
        self.command_sequence.append({'type': 'TURN_LEFT', 'value': radius_m})

    def add_right(self, radius_m):
        self.command_sequence.append({'type': 'TURN_RIGHT', 'value': radius_m})

    # --- SENSOR CALLBACK ---
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_x = pos.x
        self.current_y = pos.y
        
        # Convert quaternion to Euler angle (yaw/theta)
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.received_odom = True

    def normalize_angle(self, angle):
        """ Keeps the angle between -pi and pi """
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    # --- MAIN CONTROL LOOP ---
    def control_loop(self):
        if not self.received_odom:
            return

        msg = Twist()

        # 1. FETCH NEXT COMMAND
        if self.state == 'FETCH_CMD':
            if self.current_cmd_idx >= len(self.command_sequence):
                self.get_logger().info("Sequence finished! Stopping robot.")
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
            
            self.get_logger().info(f"Executing step {self.current_cmd_idx + 1}/{len(self.command_sequence)}: {self.state} with value {self.current_target_val}")

        # 2. EXECUTE STRAIGHT COMMAND
        elif self.state == 'STRAIGHT':
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance_driven = math.sqrt(dx*dx + dy*dy)

            if distance_driven < self.current_target_val:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            else:
                self.get_logger().info("Straight movement completed.")
                self.current_cmd_idx += 1
                self.state = 'FETCH_CMD'

        # 3. EXECUTE LEFT OR RIGHT TURN (90 DEGREES)
        elif self.state in ['TURN_LEFT', 'TURN_RIGHT']:
            # Calculate how much we have turned since the start of the command
            delta_theta = self.normalize_angle(self.current_theta - self.start_theta)
            
            # 90 degrees is pi/2 radians
            target_angle = (math.pi / 2.0)
            
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
                self.get_logger().info(f"Turn {self.state} completed.")
                self.current_cmd_idx += 1
                self.state = 'FETCH_CMD'
                
        # 4. DONE STATE
        elif self.state == 'DONE':
            msg.linear.x = 0.0
            msg.angular.z = 0.0

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