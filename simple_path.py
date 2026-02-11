import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RoundedRectangle4x2(Node):
    def __init__(self):
        super().__init__('rounded_rectangle_4x2')
        
        # --- CONFIGURAÇÕES DAS DIMENSÕES ---
        self.len_long = 4.0   # Lado Longo (Frente/Trás)
        self.len_short = 2.0  # Lado Curto (Esquerda/Direita)
        self.corner_radius = 0.5
        self.linear_speed = 0.15
        
        # Cálculos Geométricos para cada tipo de lado
        # Descontamos 2x o raio (entrada e saída da curva)
        self.straight_dist_long = self.len_long - (2 * self.corner_radius)
        self.straight_dist_short = self.len_short - (2 * self.corner_radius)
        
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
        
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        
        self.received_odom = False
        self.side_count = 0 # 0=Longo, 1=Curto, 2=Longo, 3=Curto...
        self.current_target_dist = 0.0 # Vai mudar dinamicamente

        self.get_logger().info("--- INICIANDO RETÂNGULO 4x2 ---")
        self.get_logger().info(f"Longo: {self.len_long}m | Curto: {self.len_short}m | Raio: {self.corner_radius}m")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_x = pos.x
        self.current_y = pos.y
        
        # Quaternion para Euler
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
            # === LÓGICA DO RETÂNGULO ===
            # Se o contador for Par (0, 2, 4...), é o lado LONGO (4m)
            # Se o contador for Ímpar (1, 3, 5...), é o lado CURTO (2m)
            if (self.side_count % 2) == 0:
                self.current_target_dist = self.straight_dist_long
                tipo = "LONGO (4m)"
            else:
                self.current_target_dist = self.straight_dist_short
                tipo = "CURTO (2m)"

            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'STRAIGHT'
            
            self.get_logger().info(f"--> Lado {self.side_count}: {tipo}. Andando {self.current_target_dist:.2f}m reto.")

        elif self.state == 'STRAIGHT':
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx*dx + dy*dy)

            # Usa a distancia alvo dinâmica (4m ou 2m menos as curvas)
            if distance < self.current_target_dist:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            else:
                self.start_theta = self.current_theta
                self.state = 'CURVE'
                self.get_logger().info("--> Curva...")

        elif self.state == 'CURVE':
            delta_theta = self.normalize_angle(self.current_theta - self.start_theta)
            
            # Curva de 90 graus padrão
            if abs(delta_theta) < (math.pi / 2.0) - 0.02:
                msg.linear.x = self.linear_speed
                msg.angular.z = self.turn_speed
            else:
                self.side_count += 1
                self.state = 'INIT' # Reinicia para o próximo lado

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoundedRectangle4x2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except: pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()