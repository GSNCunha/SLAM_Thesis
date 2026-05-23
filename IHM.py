import sys
import os
import subprocess
import signal
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QPushButton, QFrame, QScrollArea, QComboBox)
from PyQt5.QtGui import QFont, QCursor
from PyQt5.QtCore import Qt

# ==========================================
# ⚙️ CONFIGURAÇÕES DA IHM E AMBIENTE ROS 2
# ==========================================
PI_USER = "burguer"
CONTAINER = "slam_container_thesis"

# SEPARAÇÃO DE CAMINHOS (LOCAL VS RASPBERRY PI)
PI_WORKSPACE = "~/SLAM_Thesis"                 # Caminho dentro do Docker na Raspberry Pi
LOCAL_WORKSPACE = "~/Desktop/SLAM_Thesis"      # Caminho no seu PC (Ubuntu Local)

# Comando base que roda remotamente na Raspberry Pi
DOCKER_BASE = f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && cd {PI_WORKSPACE} && source install/setup.bash && export ROS_DOMAIN_ID=30 && "

class IHMRobot(QWidget):
    def __init__(self):
        super().__init__()
        
        # Dicionário interno para armazenar o estado de todos os parâmetros
        self.params = {
            # Parâmetros Estáticos (Inicialização)
            "particle_count": 300,
            "map_resolution": 0.05,
            "linear_update": 0.05,
            "angular_update": 0.10,
            "meas_z_hit": 0.95,
            "meas_z_rand": 0.05,
            "meas_sigma": 0.50,
            "laser_max_range": 3.50,
            
            # Parâmetros Dinâmicos (Tempo Real)
            "beam_skip": 5,
            "kernel_size": 1,
            "alpha1": 0.05,
            "alpha2": 0.005,
            "alpha3": 0.05,
            "alpha4": 0.005
        }
        
        self.initUI()

    def initUI(self):
        # Configurações da Janela Principal
        self.setWindowTitle("Estação de Controlo - FastSLAM 1.0")
        self.setFixedSize(450, 880)
        self.setStyleSheet("background-color: #1e272e; color: white;")

        # --- CONTAINER PRINCIPAL PARA A BARRA DE ROLAGEM ---
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setStyleSheet("QScrollArea { border: none; } QScrollBar { background: #2f3640; }")

        # --- CONTEÚDO DENTRO DO SCROLL ---
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        # Cabeçalho
        lbl_title = QLabel("🤖 IHM de Operação")
        lbl_title.setFont(QFont("Arial", 18, QFont.Bold))
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setStyleSheet("color: #d2dae2;")
        layout.addWidget(lbl_title)

        # Campo de IP do Robô
        ip_layout = QHBoxLayout()
        lbl_ip = QLabel("Endereço IP:")
        lbl_ip.setFont(QFont("Arial", 10))
        self.entry_ip = QLineEdit("192.168.1.203")
        self.entry_ip.setStyleSheet("background-color: #2f3640; border: 1px solid #485460; padding: 5px; border-radius: 4px; color: white;")
        ip_layout.addWidget(lbl_ip)
        ip_layout.addWidget(self.entry_ip)
        layout.addLayout(ip_layout)

        # ==========================================
        # 🌟 ADIÇÃO: SELETOR DE MODO DE OPERAÇÃO
        # ==========================================
        mode_layout = QHBoxLayout()
        lbl_mode = QLabel("🖥️ Ambiente:")
        lbl_mode.setFont(QFont("Arial", 10, QFont.Bold))
        lbl_mode.setStyleSheet("color: #f39c12;")
        
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["Físico (Robô Real)", "Simulação (Gazebo)", "Gravação (Rosbag)"])
        self.combo_mode.setStyleSheet("background-color: #2f3640; border: 1px solid #f39c12; padding: 5px; border-radius: 4px; color: white; font-weight: bold;")
        self.combo_mode.setFixedHeight(35)
        
        mode_layout.addWidget(lbl_mode)
        mode_layout.addWidget(self.combo_mode, stretch=1)
        layout.addLayout(mode_layout)
        # ==========================================

        # Função universal para criar botões com design consistente
        def create_btn(text, color, callback):
            btn = QPushButton(text)
            btn.setFont(QFont("Arial", 10, QFont.Bold))
            btn.setCursor(QCursor(Qt.PointingHandCursor))
            btn.setFixedHeight(45)
            btn.setStyleSheet(f"QPushButton {{ background-color: {color}; color: white; border: none; border-radius: 6px; }} QPushButton:hover {{ border: 2px solid white; }}")
            btn.clicked.connect(callback)
            return btn

        # ==========================================
        # 🔧 OPERAÇÕES DO SISTEMA (DOCKER E SENSORES)
        # ==========================================
        layout.addWidget(create_btn("0. Inicializar Sistema (Docker)", "#8e44ad", self.btn_start_docker))
        layout.addWidget(create_btn("🔄 Reiniciar Docker (Reset)", "#34495e", self.btn_restart_docker))
        
        # --- DIVISÃO DO BUILD (COMPILAÇÃO) ---
        build_layout = QHBoxLayout()
        build_layout.setSpacing(10)
        build_layout.addWidget(create_btn("⚙️ Build (Pi)", "#7f8c8d", self.btn_build_remote))
        build_layout.addWidget(create_btn("⚙️ Build (Local)", "#95a5a6", self.btn_build_local))
        layout.addLayout(build_layout)
        
        # --- DIVISÃO DE LIMPEZA DO BUILD (CLEAN) ---
        clean_layout = QHBoxLayout()
        clean_layout.setSpacing(10)
        clean_layout.addWidget(create_btn("🧹 Limpar (Pi)", "#e67e22", self.btn_clean_remote))
        clean_layout.addWidget(create_btn("🧹 Limpar (Local)", "#d35400", self.btn_clean_local))
        layout.addLayout(clean_layout)
        
        layout.addWidget(create_btn("1. Iniciar Sensor (Lidar)", "#c0392b", self.btn_lidar))
        layout.addWidget(create_btn("2. Acoplar Tração (Motores)", "#d35400", self.btn_motores))
        layout.addWidget(create_btn("🛑 PARAR MOTORES", "#e74c3c", self.btn_parar_motores))

        # ==========================================
        # 🟢 PARÂMETROS ESTÁTICOS (PRÉ-BOOT)
        # ==========================================
        line1 = QFrame(); line1.setFrameShape(QFrame.HLine); line1.setStyleSheet("background-color: #485460; margin-top: 10px;")
        layout.addWidget(line1)

        lbl_estaticos = QLabel("🟢 Configuração de Inicialização (Pré-Boot)")
        lbl_estaticos.setFont(QFont("Arial", 11, QFont.Bold))
        lbl_estaticos.setStyleSheet("color: #2ecc71;")
        layout.addWidget(lbl_estaticos)

        # Função universal para criar Steppers (+ / - e Campo de Texto Editável)
        def create_stepper(title, param_name, min_val, max_val, step, is_float, is_dynamic):
            frame = QFrame()
            frame.setStyleSheet("background-color: #2f3640; border-radius: 6px;")
            h_layout = QHBoxLayout(frame)
            h_layout.setContentsMargins(10, 5, 10, 5)
            
            lbl_title = QLabel(title)
            lbl_title.setStyleSheet("color: white;")
            
            btn_minus = QPushButton("-")
            btn_minus.setFixedSize(30, 30)
            btn_minus.setStyleSheet("background-color: #e74c3c; border-radius: 4px; font-weight: bold;")
            
            init_val = self.params[param_name]
            val_str = f"{init_val:.3f}" if is_float else str(init_val)
            
            # Mudança: Usando QLineEdit em vez de QLabel para permitir digitação
            entry_val = QLineEdit(val_str)
            entry_val.setFont(QFont("Courier", 11, QFont.Bold))
            entry_val.setAlignment(Qt.AlignCenter)
            entry_val.setFixedSize(65, 30)
            entry_val.setStyleSheet("background-color: white; color: black; border-radius: 4px; border: none;")
            
            btn_plus = QPushButton("+")
            btn_plus.setFixedSize(30, 30)
            btn_plus.setStyleSheet("background-color: #2ecc71; border-radius: 4px; font-weight: bold;")
            
            # Função para validar e aplicar o novo valor
            def update_value(new_val):
                # Restringe entre o mínimo e o máximo
                new_val = max(min_val, min(new_val, max_val))
                
                if is_float:
                    new_val = round(new_val, 4)
                    entry_val.setText(f"{new_val:.3f}")
                else:
                    new_val = int(new_val)
                    entry_val.setText(str(new_val))
                    
                self.params[param_name] = new_val
                
                # Se for dinâmico, envia o comando ao ROS 2
                if is_dynamic:
                    cmd = f"ros2 param set /fastslam_node {param_name} {new_val}"
                    self.run_remote_silent(cmd)

            # Callback para os botões +/-
            def on_click(delta):
                current = self.params[param_name]
                update_value(current + delta)

            # Callback para quando o usuário digita no campo e aperta Enter (ou clica fora)
            def on_text_edit():
                try:
                    # Substitui vírgula por ponto para evitar erros em valores decimais
                    text_val = entry_val.text().replace(',', '.')
                    new_val = float(text_val) if is_float else int(float(text_val))
                    update_value(new_val)
                except ValueError:
                    # Se digitar texto inválido (ex: letras), reverte para o valor atual
                    current = self.params[param_name]
                    if is_float:
                        entry_val.setText(f"{current:.3f}")
                    else:
                        entry_val.setText(str(current))
                        
            btn_minus.clicked.connect(lambda: on_click(-step))
            btn_plus.clicked.connect(lambda: on_click(step))
            entry_val.editingFinished.connect(on_text_edit)
            
            h_layout.addWidget(lbl_title)
            h_layout.addStretch()
            h_layout.addWidget(btn_minus)
            h_layout.addWidget(entry_val) # Agora adiciona o campo de texto
            h_layout.addWidget(btn_plus)
            layout.addWidget(frame)

        # 1. Estrutura e Gatilhos
        create_stepper("Número de Partículas", "particle_count", 1, 1000, 50, is_float=False, is_dynamic=False)
        create_stepper("Resolução do Mapa (m/px)", "map_resolution", 0.01, 0.20, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Gatilho Linear (m)", "linear_update", 0.01, 0.50, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Gatilho Angular (rad)", "angular_update", 0.01, 0.50, 0.05, is_float=True, is_dynamic=False)
        
        # 2. Measurement Model
        create_stepper("Z_hit (Prob. Acerto)", "meas_z_hit", 0.01, 1.0, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Z_rand (Ruído Uniforme)", "meas_z_rand", 0.01, 1.0, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Sigma Hit (Variância)", "meas_sigma", 0.05, 2.0, 0.05, is_float=True, is_dynamic=False)
        create_stepper("Alcance Máx. Lidar (m)", "laser_max_range", 1.0, 12.0, 0.5, is_float=True, is_dynamic=False)

        # --- DIVISÃO DO SLAM (LAYOUT HORIZONTAL) ---
        slam_layout = QHBoxLayout()
        slam_layout.setSpacing(10)
        slam_layout.addWidget(create_btn("3a. SLAM (Pi)", "#f39c12", self.btn_slam_remote))
        slam_layout.addWidget(create_btn("3b. SLAM (PC Local)", "#f1c40f", self.btn_slam_local))
        layout.addLayout(slam_layout)

        # ==========================================
        # 🎛️ PARÂMETROS DINÂMICOS (LIVE TUNING)
        # ==========================================
        line2 = QFrame(); line2.setFrameShape(QFrame.HLine); line2.setStyleSheet("background-color: #485460; margin-top: 10px;")
        layout.addWidget(line2)

        lbl_dinamicos = QLabel("🎛️ Sintonia Fina (Ajuste em Tempo Real)")
        lbl_dinamicos.setFont(QFont("Arial", 11, QFont.Bold))
        lbl_dinamicos.setStyleSheet("color: #f1c40f;")
        layout.addWidget(lbl_dinamicos)

        # 1. Custo Computacional do Lidar
        create_stepper("Beam Skip (Pulo de Feixes)", "beam_skip", 1, 15, 1, is_float=False, is_dynamic=True)
        create_stepper("Kernel Size (Busca Likelihood)", "kernel_size", 0, 3, 1, is_float=False, is_dynamic=True)
        
        # 2. Odometria Alphas
        create_stepper("Alpha 1 (Giro-Giro)", "alpha1", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        create_stepper("Alpha 2 (Giro-Reto)", "alpha2", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        create_stepper("Alpha 3 (Reto-Reto)", "alpha3", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        create_stepper("Alpha 4 (Reto-Giro)", "alpha4", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)

        # ==========================================
        # 🚀 OPERAÇÕES FINAIS E VISUALIZAÇÃO
        # ==========================================
        line3 = QFrame(); line3.setFrameShape(QFrame.HLine); line3.setStyleSheet("background-color: #485460; margin-top: 10px;")
        layout.addWidget(line3)

        # --- DIVISÃO DO ROSBAG (LAYOUT HORIZONTAL) ---
        rosbag_layout = QHBoxLayout()
        rosbag_layout.setSpacing(10)
        rosbag_layout.addWidget(create_btn("⏺️ Gravar Dados (Pi)", "#16a085", self.btn_rosbag_remote))
        rosbag_layout.addWidget(create_btn("⏺️ Gravar Dados (PC)", "#1abc9c", self.btn_rosbag_local))
        layout.addLayout(rosbag_layout)
        
        # --- LEITOR DE ROSBAG (PLAY) ---
        bag_play_layout = QHBoxLayout()
        bag_play_layout.setSpacing(10)
        
        self.combo_bags = QComboBox()
        self.combo_bags.setStyleSheet("background-color: #2f3640; border: 1px solid #485460; padding: 5px; border-radius: 4px; color: white; font-weight: bold;")
        self.combo_bags.setFixedHeight(45)
        self.update_bag_list()
        
        bag_play_layout.addWidget(self.combo_bags, stretch=3)
        bag_play_layout.addWidget(create_btn("🔄", "#7f8c8d", self.update_bag_list), stretch=1)
        bag_play_layout.addWidget(create_btn("▶️ Tocar Bag", "#2980b9", self.btn_play_bag_local), stretch=2)
        layout.addLayout(bag_play_layout)

        # --- DIVISÃO DA ROTA AUTÔNOMA (LAYOUT HORIZONTAL) ---
        nav_layout = QHBoxLayout()
        nav_layout.setSpacing(10)
        nav_layout.addWidget(create_btn("4a. Rota (Pi)", "#27ae60", self.btn_nav_remote))
        nav_layout.addWidget(create_btn("4b. Rota (PC Local)", "#2ecc71", self.btn_nav_local))
        layout.addLayout(nav_layout)
        
        # Botões de Interface Gráfica Local (Gazebo e RViz)
        gui_layout = QHBoxLayout()
        gui_layout.setSpacing(10)
        gui_layout.addWidget(create_btn("🌍 Simulação (Gazebo)", "#9b59b6", self.btn_gazebo))
        gui_layout.addWidget(create_btn("💻 Visão 3D (RViz)", "#2980b9", self.btn_rviz))
        layout.addLayout(gui_layout)

        # Adiciona todo o conteúdo construído à Scroll Area
        scroll_area.setWidget(content_widget)
        main_layout.addWidget(scroll_area)


    # ==========================================
    # 📡 PROTOCOLOS DE COMUNICAÇÃO (SSH / DOCKER / LOCAL)
    # ==========================================
    def run_remote(self, command_suffix, title="Terminal Remoto"):
        current_ip = self.entry_ip.text().strip()
        if not current_ip: return
        linux_cmd = f"{DOCKER_BASE} {command_suffix}"
        ssh_cmd = f"ssh -t {PI_USER}@{current_ip} \"docker exec -it {CONTAINER} bash -c \\\"{linux_cmd}\\\"\""
        terminal_cmd = f"gnome-terminal --title=\"{title}\" -- bash -c '{ssh_cmd}; exec bash'"
        subprocess.Popen(terminal_cmd, shell=True)

    def run_remote_silent(self, command_suffix):
        current_ip = self.entry_ip.text().strip()
        if not current_ip: return
        linux_cmd = f"{DOCKER_BASE} {command_suffix}"
        ssh_cmd = f"ssh {PI_USER}@{current_ip} \"docker exec {CONTAINER} bash -c \\\"{linux_cmd}\\\"\""
        subprocess.Popen(ssh_cmd, shell=True)

    def run_local(self, command, title="Terminal Local"):
        terminal_cmd = f"gnome-terminal --title=\"{title}\" -- bash -c \"{command}; exec bash\""
        subprocess.Popen(terminal_cmd, shell=True)

    # ==========================================
    # 🎛️ ROTINAS DE OPERAÇÃO (CALLBACKS)
    # ==========================================
    def btn_start_docker(self):
        self.run_local(f"ssh -t {PI_USER}@{self.entry_ip.text()} 'docker start {CONTAINER}'", "Inicializando Docker")

    def btn_restart_docker(self):
        self.run_local(f"ssh -t {PI_USER}@{self.entry_ip.text()} 'docker restart {CONTAINER}'", "Reiniciando Docker")

    # --- CALLBACKS DE LIMPEZA E COMPILAÇÃO (CLEAN & BUILD) ---
    def btn_clean_remote(self):
        # Remove as pastas de compilação dentro do Docker na Pi
        cmd = "rm -rf build/ install/ log/ && echo 'Build Remoto Limpo!'"
        self.run_remote(cmd, "Limpando Build (Pi)")

    def btn_clean_local(self):
        # Utiliza sudo para garantir a remoção local de arquivos do Docker. Vai pedir senha no terminal Ubuntu.
        cmd = f"cd {LOCAL_WORKSPACE} && sudo rm -rf build/ install/ log/ && echo 'Build Local Limpo!'"
        self.run_local(cmd, "Limpando Build (PC Local)")

    def btn_build_remote(self):
        build_cmd = "colcon build --base-paths src/ldlidar_stl_ros2 --symlink-install && colcon build --symlink-install && source install/setup.bash"
        self.run_remote(build_cmd, "Compilador Remoto (Pi)")

    def btn_build_local(self):
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"colcon build --base-paths src/ldlidar_stl_ros2 --symlink-install && "
            f"colcon build --symlink-install && "
            f"source install/setup.bash"
        )
        self.run_local(cmd, "Compilador Local (PC Ubuntu)")

    def btn_lidar(self):
        self.run_remote("ros2 launch ldlidar_stl_ros2 ld19.launch.py", "Sensor Lidar")

    def btn_motores(self):
        self.run_remote("python3 base_controller.py", "Tração e Odometria")

    def btn_parar_motores(self):
        self.run_remote_silent("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")

    # --- FUNÇÃO AUXILIAR PARA PARÂMETROS DO SLAM ---
    # ==========================================
    # 🌟 ADIÇÃO: LÓGICA DE CAPTURA DO MODO
    # ==========================================
    def _get_slam_args(self):
        # Lê a caixa de seleção e define a string que o ROS 2 entende
        mode_text = self.combo_mode.currentText()
        if "Gazebo" in mode_text:
            r_mode = "gazebo"
        elif "Rosbag" in mode_text:
            r_mode = "rosbag"
        else:
            r_mode = "real"

        return (
            f"robot_mode:={r_mode} "  # <-- Enviando a escolha para o Cérebro do Launch
            f"particle_count:={int(self.params['particle_count'])} "
            f"map_resolution:={self.params['map_resolution']} "
            f"linear_update:={self.params['linear_update']} "
            f"angular_update:={self.params['angular_update']} "
            f"meas_z_hit:={self.params['meas_z_hit']} "
            f"meas_z_rand:={self.params['meas_z_rand']} "
            f"meas_sigma:={self.params['meas_sigma']} "
            f"laser_max_range:={self.params['laser_max_range']}"
        )

    # --- CALLBACKS DO SLAM ---
    def btn_slam_remote(self):
        cmd = f"ros2 launch fastslam_thesis fastslam.launch.py {self._get_slam_args()}"
        self.run_remote(cmd, "Mapeamento SLAM (Pi)")

    def btn_slam_local(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 launch fastslam_thesis fastslam.launch.py {self._get_slam_args()}"
        )
        self.run_local(cmd, "Mapeamento SLAM (Local PC)")

    # --- CALLBACKS DO ROSBAG ---
    def btn_rosbag_remote(self):
        self.run_remote("ros2 bag record /scan /tf /tf_static /odom", "Gravador Rosbag (Pi)")

    def btn_rosbag_local(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 bag record /scan /tf /tf_static /odom"
        )
        self.run_local(cmd, "Gravador Rosbag (Local PC)")

    # --- CALLBACKS DO LEITOR DE ROSBAG ---
    def update_bag_list(self):
        self.combo_bags.clear()
        # O caminho usa o LOCAL_WORKSPACE definido no início do arquivo
        bag_dir = os.path.expanduser(f"{LOCAL_WORKSPACE.replace('~', '~')}/ROSBAG")
        
        if os.path.exists(bag_dir):
            # No ROS 2, as bags são pastas que contêm os arquivos .db3
            bags = [f for f in os.listdir(bag_dir) if os.path.isdir(os.path.join(bag_dir, f))]
            bags.sort(reverse=True) # Exibe as gravações mais recentes primeiro
            if bags:
                self.combo_bags.addItems(bags)
            else:
                self.combo_bags.addItem("Pasta ROSBAG vazia")
        else:
            self.combo_bags.addItem("Pasta ROSBAG não encontrada")

    def btn_play_bag_local(self):
        selected_bag = self.combo_bags.currentText()
        if not selected_bag or selected_bag in ["Pasta ROSBAG vazia", "Pasta ROSBAG não encontrada"]:
            return
            
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 bag play ROSBAG/{selected_bag} --clock" # <-- ALTERADO PARA INCLUIR --clock
        )
        self.run_local(cmd, f"Leitor Rosbag: {selected_bag}")

    # --- CALLBACKS DA ROTA AUTÔNOMA ---
    def btn_nav_remote(self):
        self.run_remote("python3 simple_path.py", "Navegação Autônoma (Pi)")

    def btn_nav_local(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"python3 simple_path.py"
        )
        self.run_local(cmd, "Navegação Autônoma (Local PC)")

    # --- CALLBACKS DE VISUALIZAÇÃO GRÁFICA ---
    def btn_gazebo(self):
        cmd = (
            f"export TURTLEBOT3_MODEL=burger && "
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"source {LOCAL_WORKSPACE}/install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 launch custom_map.launch.py"
        )
        self.run_local(cmd, "Simulador Gazebo")

    def btn_rviz(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"source {LOCAL_WORKSPACE}/install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=True" # <-- ALTERADO PARA INCLUIR use_sim_time
        )
        self.run_local(cmd, "Visualizador 3D")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    ihm = IHMRobot()
    ihm.show()
    sys.exit(app.exec_())