import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RoundedRectangleNode(Node):
    def __init__(self):
        super().__init__('rounded_rectangle_node')
        
        # --- CONFIGURAÇÕES DE DIMENSÕES (Ajuste aqui) ---
        self.len_long = 1.35           # Tamanho total da aresta MAIOR
        self.len_short = 0          # Tamanho total da aresta MENOR
        self.corner_radius = 0.35      # Raio da curva
        self.linear_speed = 0.05      
        
        # Cálculos das distâncias retas padrão (Descontando os 2 raios de curva)
        self.dist_straight_long = self.len_long - (2 * self.corner_radius)
        self.dist_straight_short = self.len_short - (2 * self.corner_radius)
        
        # Velocidade angular (w = v/r)
        self.turn_speed = self.linear_speed / self.corner_radius
        
        # --- ROS SETUP ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Variáveis de Estado
        self.state = 'INIT' 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.received_odom = False
        
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        
        self.side_count = 0  # 0 e 2: Longo | 1 e 3: Curto
        self.current_target_dist = 0.0

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_x = pos.x
        self.current_y = pos.y
        
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        self.received_odom = True

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.received_odom: return

        msg = Twist()

        if self.state == 'INIT':
            # 1. Determina se é aresta maior ou menor
            if (self.side_count % 2) == 0:
                self.current_target_dist = self.dist_straight_long
                tipo = "MAIOR"
            else:
                self.current_target_dist = self.dist_straight_short
                tipo = "MENOR"

            # 2. COMPENSAÇÃO DA PRIMEIRA RETA (Se for o primeiro movimento da vida)
            if self.side_count == 0:
                self.current_target_dist += self.corner_radius
                self.get_logger().info(f"--- LARGADA: Aresta {tipo} com compensação de raio ---")
            
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'STRAIGHT'
            self.get_logger().info(f"Lado {self.side_count}: {tipo}. Alvo: {self.current_target_dist:.2f}m")

        elif self.state == 'STRAIGHT':
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < self.current_target_dist:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            else:
                self.start_theta = self.current_theta
                self.state = 'CURVE'
                self.get_logger().info("Curva...")

        elif self.state == 'CURVE':
            delta_theta = self.normalize_angle(self.current_theta - self.start_theta)
            
            # Curva de 90 graus
            if abs(delta_theta) < (math.pi / 2.0) - 0.04:
                msg.linear.x = self.linear_speed
                msg.angular.z = self.turn_speed
            else:
                # Incrementa o contador de lados (0 -> 1 -> 2 -> 3 -> 0...)
                self.side_count = (self.side_count + 1) % 4
                self.state = 'INIT'

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoundedRectangleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()