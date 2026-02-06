import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math
import time

class PrecisionMover(Node):
    def __init__(self):
        super().__init__('precision_mover')
        
        # Publica velocidade e Ouve Odometria
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )
        
        # Variáveis de Estado
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        
        self.get_logger().info('Sistema de ULTRA Precisão (Lento) Iniciado...')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True

    def stop(self):
        self.publisher_.publish(Twist())
        time.sleep(1.0) 

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def girar(self, graus_objetivo):
        """ Gira quase parando (0.02 rad/s) """
        if not self.odom_received: return
        
        if graus_objetivo == 0:
            self.get_logger().info("Pausa técnica...")
            time.sleep(2.0)
            return

        target_rad = math.radians(graus_objetivo)
        target_angle = self.normalize_angle(self.yaw + target_rad)
        
        self.get_logger().info(f"Girando {graus_objetivo} graus em CÂMERA LENTA...")
        
        cmd = Twist()
        
        while rclpy.ok():
            rclpy.spin_once(self)
            
            error = self.normalize_angle(target_angle - self.yaw)
            
            # Tolerância
            if abs(error) < 0.01:
                break
            
            # Controle Proporcional
            speed = error * 0.2
            
            # --- NOVOS LIMITES (5x mais lento que antes) ---
            # Máximo: 0.02 rad/s (Super lento)
            # Mínimo: 0.005 rad/s (Mínimo absoluto para vencer inércia)
            
            if speed > 0: 
                speed = max(0.005, min(0.02, speed))
            else:         
                speed = min(-0.005, max(-0.02, speed))
            
            cmd.angular.z = float(speed)
            self.publisher_.publish(cmd)
            # Sleep pequeno para não sobrecarregar CPU
            time.sleep(0.02)
            
        self.stop()

    def frente(self, dist_metros):
        if not self.odom_received: return
        
        start_x = self.x
        start_y = self.y
        
        self.get_logger().info(f"Andando {dist_metros} metros...")
        
        cmd = Twist()
        
        while rclpy.ok():
            rclpy.spin_once(self)
            
            dx = self.x - start_x
            dy = self.y - start_y
            distance_moved = math.sqrt(dx*dx + dy*dy)
            
            error = abs(dist_metros) - distance_moved

            if error <= 0.01:
                break
            
            speed = error * 0.5
            
            # Velocidade linear mantida em 0.1 (lento, mas não parado)
            speed = max(0.02, min(0.1, speed))
            
            if dist_metros < 0:
                speed = -speed

            cmd.linear.x = float(speed)
            self.publisher_.publish(cmd)
            time.sleep(0.02)
            
        self.stop()

def main(args=None):
    rclpy.init(args=args)
    robot = PrecisionMover()
    
    while not robot.odom_received:
        rclpy.spin_once(robot)
        time.sleep(0.1)

    try:
        robot.get_logger().info(">>> LOOP LENTO INICIADO <<<")

        while rclpy.ok():
            # 1. Anda 4 metros
            robot.frente(4.0)
            
            # 2. Pausa
            robot.girar(0) 
            
            # 3. Gira 90 graus (Vai demorar ~80 segundos)
            robot.girar(90)
            
            time.sleep(1.0)

            # 4. Anda 1 metro
            robot.frente(1.0)
            
            # 5. Pausa
            robot.girar(0) 
            
            # 6. Gira 90 graus
            robot.girar(90)
            
            time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()