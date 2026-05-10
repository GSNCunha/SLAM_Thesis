import sys
import subprocess
import signal
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QPushButton, QFrame)
from PyQt5.QtGui import QFont, QCursor
from PyQt5.QtCore import Qt

# ==========================================
# ⚙️ CONFIGURAÇÕES DA IHM
# ==========================================
PI_USER = "burguer"
CONTAINER = "slam_container_thesis"
WORKSPACE = "~/SLAM_Thesis/Desktop/SLAM_Thesis"

# Comando base atualizado com CycloneDDS
DOCKER_BASE = f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && cd {WORKSPACE} && source install/setup.bash && export ROS_DOMAIN_ID=30 && "

class IHMRobot(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # Configurações da Janela
        self.setWindowTitle("IHM de Operação - Burguer Bot")
        self.setFixedSize(400, 700) # Altura ajustada para caber o novo botão
        self.setStyleSheet("background-color: #1e272e; color: white;")

        # Layout Principal (Vertical)
        layout = QVBoxLayout()
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)

        # Cabeçalho
        lbl_title = QLabel("🤖 IHM de Operação")
        lbl_title.setFont(QFont("Arial", 18, QFont.Bold))
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setStyleSheet("color: #d2dae2;")
        layout.addWidget(lbl_title)

        # Campo de IP (Layout Horizontal)
        ip_layout = QHBoxLayout()
        lbl_ip = QLabel("Endereço IP do Robô:")
        lbl_ip.setFont(QFont("Arial", 10))
        lbl_ip.setStyleSheet("color: #bdc3c7;")
        
        self.entry_ip = QLineEdit("192.168.1.203")
        self.entry_ip.setFont(QFont("Arial", 12))
        self.entry_ip.setStyleSheet("""
            QLineEdit {
                background-color: #2f3640; 
                border: 1px solid #485460; 
                padding: 5px; 
                border-radius: 4px;
                color: white;
            }
        """)
        
        ip_layout.addWidget(lbl_ip)
        ip_layout.addWidget(self.entry_ip)
        layout.addLayout(ip_layout)

        # Função auxiliar para criar botões padronizados
        def create_btn(text, color, callback):
            btn = QPushButton(text)
            btn.setFont(QFont("Arial", 11, QFont.Bold))
            btn.setCursor(QCursor(Qt.PointingHandCursor))
            btn.setFixedHeight(50)
            # Aplicando CSS para um design moderno
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {color};
                    color: white;
                    border: none;
                    border-radius: 6px;
                }}
                QPushButton:hover {{
                    background-color: {color};
                    border: 2px solid white; /* Efeito de hover */
                }}
                QPushButton:pressed {{
                    background-color: #2c3e50; /* Cor ao clicar */
                }}
            """)
            btn.clicked.connect(callback)
            return btn

        # Adicionando os Botões
        layout.addWidget(create_btn("0. Inicializar Sistema (Docker)", "#8e44ad", self.btn_start_docker))
        
        # NOVO BOTÃO: REINICIAR DOCKER
        layout.addWidget(create_btn("🔄 Reiniciar Docker (Reset)", "#34495e", self.btn_restart_docker))

        layout.addWidget(create_btn("⚙️ Recompilar Código (Build)", "#7f8c8d", self.btn_build))
        layout.addWidget(create_btn("1. Iniciar Sensor (Lidar)", "#c0392b", self.btn_lidar))
        layout.addWidget(create_btn("2. Acoplar Tração (Motores)", "#d35400", self.btn_motores))
        layout.addWidget(create_btn("🛑 PARAR MOTORES", "#e74c3c", self.btn_parar_motores))
        layout.addWidget(create_btn("3. Processamento SLAM", "#f39c12", self.btn_slam))
        layout.addWidget(create_btn("4. Iniciar Rota Autônoma", "#27ae60", self.btn_nav))
        
        # Linha Divisória
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #485460;")
        layout.addWidget(line)

        layout.addWidget(create_btn("💻 Abrir Visualizador 3D", "#2980b9", self.btn_rviz))

        self.setLayout(layout)

    # ==========================================
    # 🚀 PROTOCOLOS DE COMUNICAÇÃO
    # ==========================================
    def run_remote(self, command_suffix, title="Terminal"):
        current_ip = self.entry_ip.text().strip()
        if not current_ip:
            print("[Erro] O campo de IP está vazio!")
            return

        # 1. Comando base que vai rodar DENTRO do docker
        linux_cmd = f"{DOCKER_BASE} {command_suffix}"
        
        # 2. Comando SSH: Usamos aspas duplas escapadas (\") para o bash interno do docker
        ssh_cmd = f"ssh -t {PI_USER}@{current_ip} \"docker exec -it {CONTAINER} bash -c \\\"{linux_cmd}\\\"\""
        
        # 3. Comando Terminal: Envolvemos o bash -c com aspas SIMPLES ('...') 
        # Isso evita que as aspas duplas do comando SSH quebrem a string principal
        terminal_cmd = f"gnome-terminal --title=\"{title}\" -- bash -c '{ssh_cmd}; exec bash'"
        
        subprocess.Popen(terminal_cmd, shell=True)
        print(f"[IHM] Conectando subsistema: {title} no IP {current_ip}")

    def run_remote_silent(self, command_suffix):
        """ Executa um comando SSH no docker sem abrir uma nova janela do terminal """
        current_ip = self.entry_ip.text().strip()
        if not current_ip:
            print("[Erro] O campo de IP está vazio!")
            return

        linux_cmd = f"{DOCKER_BASE} {command_suffix}"
        ssh_cmd = f"ssh {PI_USER}@{current_ip} \"docker exec {CONTAINER} bash -c \\\"{linux_cmd}\\\"\""
        
        subprocess.Popen(ssh_cmd, shell=True)
        print(f"[IHM] 🛑 Comando de parada enviado para o IP {current_ip}")

    # ==========================================
    # 🎛️ ROTINAS DE OPERAÇÃO
    # ==========================================
    def btn_start_docker(self):
        current_ip = self.entry_ip.text().strip()
        if not current_ip:
            print("[Erro] O campo de IP está vazio!")
            return
        ssh_cmd = f"ssh -t {PI_USER}@{current_ip} 'docker start {CONTAINER}'"
        terminal_cmd = f"gnome-terminal --title=\"Inicializando Sistema\" -- bash -c \"{ssh_cmd}; exec bash\""
        subprocess.Popen(terminal_cmd, shell=True)
        print(f"[IHM] Acordando o robô no IP {current_ip}")

    # NOVA FUNÇÃO: REINICIAR DOCKER
    def btn_restart_docker(self):
        current_ip = self.entry_ip.text().strip()
        if not current_ip:
            print("[Erro] O campo de IP está vazio!")
            return
        ssh_cmd = f"ssh -t {PI_USER}@{current_ip} 'docker restart {CONTAINER}'"
        terminal_cmd = f"gnome-terminal --title=\"Reiniciando Docker\" -- bash -c \"{ssh_cmd}; exec bash\""
        subprocess.Popen(terminal_cmd, shell=True)
        print(f"[IHM] Reiniciando o contêiner no IP {current_ip} (Limpando processos zumbis)")

    def btn_build(self):
        # Encadeia os builds: Primeiro compila o Lidar, depois o FastSLAM (raiz) e finaliza com source
        build_cmd = "colcon build --base-paths src/ldlidar_stl_ros2 --symlink-install && colcon build --symlink-install && source install/setup.bash"
        self.run_remote(build_cmd, "Compilador (Colcon Build)")

    def run_local(self, command, title="Terminal Local"):
        """Abre um gnome-terminal local no seu PC Ubuntu"""
        terminal_cmd = f"gnome-terminal --title=\"{title}\" -- bash -c \"{command}; exec bash\""
        subprocess.Popen(terminal_cmd, shell=True)
        print(f"[IHM] Iniciando rotina local: {title}")

    def btn_lidar(self):
        self.run_remote("ros2 launch ldlidar_stl_ros2 ld19.launch.py", "Sensor Lidar")

    def btn_motores(self):
        self.run_remote("python3 base_controller.py", "Tração e Odometria")

    def btn_parar_motores(self):
        stop_cmd = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
        self.run_remote_silent(stop_cmd)

    def btn_slam(self):
        self.run_remote("ros2 launch fastslam_thesis fastslam.launch.py", "Mapeamento SLAM")

    def btn_nav(self):
        self.run_remote("python3 simple_path.py", "Navegação Autônoma")

    def btn_rviz(self):
        self.run_local("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && rviz2", "Visualizador 3D")

if __name__ == '__main__':
    # Permitir fechamento via Ctrl+C no terminal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    app = QApplication(sys.argv)
    ihm = IHMRobot()
    ihm.show()
    sys.exit(app.exec_())