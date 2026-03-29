import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *
import math
import time

# --- CONFIGURAÇÕES EXATAS DO SEU PRINT DO WIZARD ---
BAUDRATE = 57600        # Conforme seu print
DXL_ID_LEFT = 4         # Conforme seu print
DXL_ID_RIGHT = 2        # Assumindo que seu outro motor é o ID 2
DEVICENAME = '/dev/ttyUSB0'
PROTOCOL_VERSION = 2.0

# Medidas do robô (metros)
R = 0.0688
L = 0.176

# Endereços XH430
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # ABRIR PORTA
        if self.portHandler.openPort() and self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info(f'✅ Conectado em {BAUDRATE} bps!')
        else:
            self.get_logger().error('❌ Falha na porta USB. O Wizard está fechado?')
            return

        # AJUSTE DE PACIÊNCIA (O "pulo do gato" para o seu Return Delay de 250)
        # Aumentamos o timeout para o Python esperar a resposta lenta do motor
        self.portHandler.setPacketTimeout(2.0) 

        self.setup_motors()

    def setup_motors(self):
        # Lista de IDs para tentar configurar
        ids = [DXL_ID_LEFT, DXL_ID_RIGHT]
        
        for dxl_id in ids:
            # Testa se o motor responde
            model, comm, err = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, 0)
            if comm == COMM_SUCCESS:
                self.get_logger().info(f'--- Motor ID:{dxl_id} ONLINE')
                # Configura modo velocidade sem alterar permanentemente a EEPROM
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 1)
                self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
            else:
                self.get_logger().warn(f'⚠️ Motor ID:{dxl_id} não respondeu. Verifique fiação/ID.')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        
        # Log de recepção
        self.get_logger().info(f'Movendo: v={v:.2f} w={w:.2f}')

        # Cálculo de Velocidade (Cinemática)
        v_l = (v - w * (L / 2.0)) / R
        v_r = (v + w * (L / 2.0)) / R

        # Converter rad/s para unidades Dynamixel (0.229 rpm/unit)
        unit_l = int((v_l * 60 / (2 * math.pi)) / 0.229)
        unit_r = int((v_r * 60 / (2 * math.pi)) / 0.229)

        # Inversão para o motor da direita
        unit_r = -unit_r

        # Enviar comandos
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_LEFT, ADDR_GOAL_VELOCITY, unit_l)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_RIGHT, ADDR_GOAL_VELOCITY, unit_r)

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()