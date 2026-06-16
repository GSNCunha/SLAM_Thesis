import sys
import os
import subprocess
import signal
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QPushButton, QFrame, QScrollArea, 
                             QComboBox, QStackedWidget, QDialog, QSizePolicy)
from PyQt5.QtGui import QFont, QCursor
from PyQt5.QtCore import Qt

# ==========================================
# HMI AND ROS 2 ENVIRONMENT SETTINGS
# ==========================================
PI_USER = "burguer"
CONTAINER = "slam_container_thesis"

# PATH SEPARATION
DOCKER_WORKSPACE = "/root/SLAM_Thesis"         
PI_HOST_WORKSPACE = "~/Desktop/SLAM_Thesis"    
LOCAL_WORKSPACE = "~/Desktop/SLAM_Thesis"      

# Base command running remotely on the Raspberry Pi
DOCKER_BASE = f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && cd {DOCKER_WORKSPACE} && if [ -f install/setup.bash ]; then source install/setup.bash; fi && export ROS_DOMAIN_ID=30 && "


class SlamParamsDialog(QDialog):
    """ Independent window for configuring all SLAM parameters """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_hmi = parent
        self.setWindowTitle("SLAM Parameters (Setup & Fine Tuning)")
        
        self.setFixedWidth(450)
        self.setStyleSheet("background-color: #1e272e; color: white;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setStyleSheet("QScrollArea { border: none; } QScrollBar { background: #2f3640; }")

        content_widget = QWidget()
        self.vbox = QVBoxLayout(content_widget)
        self.vbox.setSpacing(10)
        self.vbox.setContentsMargins(20, 20, 20, 20)

        # Title
        lbl_title = QLabel("SLAM Parameters")
        lbl_title.setFont(QFont("Arial", 14, QFont.Bold))
        lbl_title.setAlignment(Qt.AlignCenter)
        self.vbox.addWidget(lbl_title)

        # Adding parameters
        self.create_stepper("Particle Count", "particle_count", 1, 1000, 50, is_float=False, is_dynamic=False)
        self.create_stepper("Map Resolution (m/px)", "map_resolution", 0.01, 0.20, 0.01, is_float=True, is_dynamic=False)
        self.create_stepper("Linear Trigger (m)", "linear_update", 0.01, 0.50, 0.01, is_float=True, is_dynamic=False)
        self.create_stepper("Angular Trigger (rad)", "angular_update", 0.01, 0.50, 0.05, is_float=True, is_dynamic=False)
        
        self.create_stepper("Z_hit (Hit Prob.)", "meas_z_hit", 0.01, 1.0, 0.01, is_float=True, is_dynamic=False)
        self.create_stepper("Z_rand (Uniform Noise)", "meas_z_rand", 0.01, 1.0, 0.01, is_float=True, is_dynamic=False)
        self.create_stepper("Sigma Hit (Variance)", "meas_sigma", 0.05, 2.0, 0.05, is_float=True, is_dynamic=False)
        self.create_stepper("Max Lidar Range (m)", "laser_max_range", 1.0, 12.0, 0.5, is_float=True, is_dynamic=False)

        self.create_stepper("Beam Skip", "beam_skip", 1, 15, 1, is_float=False, is_dynamic=True)
        self.create_stepper("Kernel Size (Likelihood Search)", "kernel_size", 0, 3, 1, is_float=False, is_dynamic=True)
        self.create_stepper("Alpha 1 (Turn-Turn)", "alpha1", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        self.create_stepper("Alpha 2 (Turn-Straight)", "alpha2", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        self.create_stepper("Alpha 3 (Straight-Straight)", "alpha3", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        self.create_stepper("Alpha 4 (Straight-Turn)", "alpha4", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)

        scroll_area.setWidget(content_widget)
        layout.addWidget(scroll_area)

        # Close Button
        btn_close = QPushButton("Apply and Close")
        btn_close.setFont(QFont("Arial", 10, QFont.Bold))
        btn_close.setFixedHeight(45)
        btn_close.setStyleSheet("background-color: #2ecc71; color: white; border-radius: 6px; margin: 10px;")
        btn_close.clicked.connect(self.accept)
        layout.addWidget(btn_close)
        
        self.setMinimumHeight(600)
        self.adjustSize()

    def create_stepper(self, title, param_name, min_val, max_val, step, is_float, is_dynamic):
        frame = QFrame()
        frame.setStyleSheet("background-color: #2f3640; border-radius: 6px;")
        h_layout = QHBoxLayout(frame)
        h_layout.setContentsMargins(10, 5, 10, 5)
        
        lbl_title = QLabel(title)
        lbl_title.setStyleSheet("color: white;")
        
        btn_minus = QPushButton("-")
        btn_minus.setFixedSize(30, 30)
        btn_minus.setStyleSheet("background-color: #e74c3c; border-radius: 4px; font-weight: bold;")
        
        init_val = self.parent_hmi.params[param_name]
        val_str = f"{init_val:.3f}" if is_float else str(init_val)
        
        entry_val = QLineEdit(val_str)
        entry_val.setFont(QFont("Courier", 11, QFont.Bold))
        entry_val.setAlignment(Qt.AlignCenter)
        entry_val.setFixedSize(65, 30)
        entry_val.setStyleSheet("background-color: white; color: black; border-radius: 4px; border: none;")
        
        btn_plus = QPushButton("+")
        btn_plus.setFixedSize(30, 30)
        btn_plus.setStyleSheet("background-color: #2ecc71; border-radius: 4px; font-weight: bold;")
        
        def update_value(new_val):
            new_val = max(min_val, min(new_val, max_val))
            if is_float:
                new_val = round(new_val, 4)
                entry_val.setText(f"{new_val:.3f}")
            else:
                new_val = int(new_val)
                entry_val.setText(str(new_val))
                
            self.parent_hmi.params[param_name] = new_val
            
            if is_dynamic:
                cmd = f"ros2 param set /fastslam_node {param_name} {new_val}"
                self.parent_hmi.run_remote_silent(cmd)

        def on_click(delta):
            current = self.parent_hmi.params[param_name]
            update_value(current + delta)

        def on_text_edit():
            try:
                text_val = entry_val.text().replace(',', '.')
                new_val = float(text_val) if is_float else int(float(text_val))
                update_value(new_val)
            except ValueError:
                current = self.parent_hmi.params[param_name]
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
        h_layout.addWidget(entry_val)
        h_layout.addWidget(btn_plus)
        self.vbox.addWidget(frame)


class IHMRobot(QWidget):
    def __init__(self):
        super().__init__()
        
        self.params = {
            "particle_count": 300, "map_resolution": 0.05, "linear_update": 0.05,
            "angular_update": 0.10, "meas_z_hit": 0.95, "meas_z_rand": 0.05,
            "meas_sigma": 0.50, "laser_max_range": 3.50, "beam_skip": 5,
            "kernel_size": 1, "alpha1": 0.05, "alpha2": 0.005,
            "alpha3": 0.05, "alpha4": 0.005
        }
        
        self.current_robot_mode = "real" # real, gazebo, rosbag
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Control Station - FastSLAM 1.0")
        
        self.setFixedWidth(450)
        self.setStyleSheet("background-color: #1e272e; color: white;")

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        self.stacked_widget = QStackedWidget()
        main_layout.addWidget(self.stacked_widget)

        self.page_menu = self.create_menu_page()
        self.page_physical = self.create_physical_page()
        self.page_sim = self.create_sim_page()
        self.page_record = self.create_record_page()

        self.stacked_widget.addWidget(self.page_menu)
        self.stacked_widget.addWidget(self.page_physical)
        self.stacked_widget.addWidget(self.page_sim)
        self.stacked_widget.addWidget(self.page_record)

        self.change_page(self.page_menu)

    # --- NAVIGATION AND UI FUNCTIONS ---
    def create_btn(self, text, color, callback):
        btn = QPushButton(text)
        btn.setFont(QFont("Arial", 10, QFont.Bold))
        btn.setCursor(QCursor(Qt.PointingHandCursor))
        btn.setFixedHeight(45)
        btn.setStyleSheet(f"QPushButton {{ background-color: {color}; color: white; border: none; border-radius: 6px; }} QPushButton:hover {{ border: 2px solid white; }}")
        btn.clicked.connect(callback)
        return btn

    def change_page(self, page):
        for i in range(self.stacked_widget.count()):
            widget = self.stacked_widget.widget(i)
            widget.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            
        page.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        self.stacked_widget.setCurrentWidget(page)
        
        self.adjustSize()

    def set_mode(self, mode, page):
        self.current_robot_mode = mode
        self.change_page(page)

    def back_to_menu(self):
        self.change_page(self.page_menu)

    def open_slam_params(self):
        dialog = SlamParamsDialog(self)
        dialog.exec_()

    # ==========================================
    # PAGE 1: MAIN MENU
    # ==========================================
    def create_menu_page(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(20)

        lbl = QLabel("Control Station - Select Mode")
        lbl.setFont(QFont("Arial", 16, QFont.Bold))
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        layout.addWidget(self.create_btn("Physical Robot", "#c0392b", lambda: self.set_mode("real", self.page_physical)))
        layout.addWidget(self.create_btn("Simulation (Gazebo)", "#9b59b6", lambda: self.set_mode("gazebo", self.page_sim)))
        layout.addWidget(self.create_btn("Recorded Data (physical robot)", "#2980b9", lambda: self.set_mode("rosbag", self.page_record)))

        return widget

    # ==========================================
    # PAGE 2: PHYSICAL ROBOT
    # ==========================================
    def create_physical_page(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(20, 20, 20, 20)

        lbl = QLabel("Mode: Physical Robot")
        lbl.setFont(QFont("Arial", 14, QFont.Bold)); lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        # IP
        ip_layout = QHBoxLayout()
        ip_layout.addWidget(QLabel("IP Address:"))
        self.entry_ip = QLineEdit("192.168.1.203")
        self.entry_ip.setStyleSheet("background-color: #2f3640; padding: 5px; border-radius: 4px; color: white;")
        ip_layout.addWidget(self.entry_ip)
        layout.addLayout(ip_layout)

        # System & Docker
        layout.addWidget(self.create_btn("Initialize System (Docker)", "#8e44ad", self.btn_start_docker))
        layout.addWidget(self.create_btn("Restart Docker", "#34495e", self.btn_restart_docker))
        
        build_layout = QHBoxLayout()
        build_layout.addWidget(self.create_btn("Build (Pi)", "#7f8c8d", self.btn_build_remote))
        build_layout.addWidget(self.create_btn("Clean (Pi)", "#e67e22", self.btn_clean_remote))
        layout.addLayout(build_layout)

        # Sensors & Movement
        layout.addWidget(self.create_btn("Start Sensor (Lidar)", "#c0392b", self.btn_lidar))
        layout.addWidget(self.create_btn("Engage Traction (Motors)", "#d35400", self.btn_motores))
        
        # SLAM
        layout.addWidget(self.create_btn("Set SLAM Parameters", "#f39c12", self.open_slam_params))
        layout.addWidget(self.create_btn("Start SLAM (Pi)", "#2ecc71", self.btn_slam_remote))

        # Rosbag
        bag_layout = QHBoxLayout()
        bag_layout.addWidget(QLabel("Rosbag Name:"))
        self.entry_bag_name_pi = QLineEdit("my_recording")
        self.entry_bag_name_pi.setStyleSheet("background-color: #2f3640; padding: 5px; border-radius: 4px; color: white;")
        bag_layout.addWidget(self.entry_bag_name_pi)
        layout.addLayout(bag_layout)

        layout.addWidget(self.create_btn("Record Data (Pi)", "#16a085", self.btn_rosbag_remote))
        layout.addWidget(self.create_btn("Sync Rosbags (Pi -> PC)", "#2980b9", self.btn_sync_bags))
        
        # Route & View
        nav_layout = QHBoxLayout()
        nav_layout.addWidget(self.create_btn("Route (Pi)", "#27ae60", self.btn_nav_remote))
        nav_layout.addWidget(self.create_btn("3D Vision (RViz)", "#2980b9", self.btn_rviz))
        layout.addLayout(nav_layout)

        layout.addStretch()
        layout.addWidget(self.create_btn("Back to Menu", "#7f8c8d", self.back_to_menu))
        return widget

    # ==========================================
    # PAGE 3: SIMULATION (GAZEBO)
    # ==========================================
    def create_sim_page(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(20, 20, 20, 20)

        lbl = QLabel("Mode: Simulation")
        lbl.setFont(QFont("Arial", 14, QFont.Bold)); lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        build_layout = QHBoxLayout()
        build_layout.addWidget(self.create_btn("Build (Local)", "#95a5a6", self.btn_build_local))
        build_layout.addWidget(self.create_btn("Clean (Local)", "#d35400", self.btn_clean_local))
        layout.addLayout(build_layout)

        # SLAM
        layout.addWidget(self.create_btn("Set SLAM Parameters", "#f39c12", self.open_slam_params))
        layout.addWidget(self.create_btn("Start SLAM (Local PC)", "#f1c40f", self.btn_slam_local))

        # Gazebo, Route, RViz
        layout.addWidget(self.create_btn("Route (Local PC)", "#2ecc71", self.btn_nav_local))
        layout.addWidget(self.create_btn("Simulation (Gazebo)", "#9b59b6", self.btn_gazebo))
        layout.addWidget(self.create_btn("3D Vision (RViz)", "#2980b9", self.btn_rviz))

        layout.addStretch()
        layout.addWidget(self.create_btn("Back to Menu", "#7f8c8d", self.back_to_menu))
        return widget

    # ==========================================
    # PAGE 4: RECORDED DATA (ROSBAG)
    # ==========================================
    def create_record_page(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(20, 20, 20, 20)

        lbl = QLabel("Mode: Recorded Data")
        lbl.setFont(QFont("Arial", 14, QFont.Bold)); lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl)

        layout.addWidget(self.create_btn("Set SLAM Parameters", "#f39c12", self.open_slam_params))
        layout.addWidget(self.create_btn("Start SLAM (Local PC)", "#f1c40f", self.btn_slam_local))

        # Rosbag Player
        self.combo_bags = QComboBox()
        self.combo_bags.setStyleSheet("background-color: #2f3640; padding: 5px; border-radius: 4px; color: white;")
        self.combo_bags.setFixedHeight(40)
        self.update_bag_list()

        play_layout = QHBoxLayout()
        play_layout.addWidget(self.combo_bags, stretch=3)
        play_layout.addWidget(self.create_btn("Refresh", "#7f8c8d", self.update_bag_list), stretch=1)
        layout.addLayout(play_layout)

        layout.addWidget(self.create_btn("Play Selected Rosbag", "#2980b9", self.btn_play_bag_local))
        layout.addWidget(self.create_btn("3D Vision (RViz)", "#2980b9", self.btn_rviz))

        layout.addStretch()
        layout.addWidget(self.create_btn("Back to Menu", "#7f8c8d", self.back_to_menu))
        return widget

    # ==========================================
    # COMMUNICATION PROTOCOLS (SSH / DOCKER / LOCAL)
    # ==========================================
    def run_remote(self, command_suffix, title="Remote Terminal"):
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

    def run_local(self, command, title="Local Terminal"):
        terminal_cmd = f"gnome-terminal --title=\"{title}\" -- bash -c \"{command}; exec bash\""
        subprocess.Popen(terminal_cmd, shell=True)

    # ==========================================
    # OPERATION ROUTINES (CALLBACKS)
    # ==========================================
    def btn_start_docker(self):
        self.run_local(f"ssh -t {PI_USER}@{self.entry_ip.text()} 'docker start {CONTAINER}'", "Initializing Docker")

    def btn_restart_docker(self):
        self.run_local(f"ssh -t {PI_USER}@{self.entry_ip.text()} 'docker restart {CONTAINER}'", "Restarting Docker")

    def btn_clean_remote(self):
        cmd = "rm -rf build/ install/ log/ && echo Remote Build Cleaned Successfully"
        self.run_remote(cmd, "Cleaning Build (Pi)")
        
    def btn_clean_local(self):
        cmd = f"cd {LOCAL_WORKSPACE} && sudo rm -rf build/ install/ log/ && echo 'Local Build Cleaned!'"
        self.run_local(cmd, "Cleaning Build (Local PC)")

    def btn_build_remote(self):
        build_cmd = "colcon build --symlink-install --executor sequential && source install/setup.bash"
        self.run_remote(build_cmd, "Remote Compiler (Pi)")

    def btn_build_local(self):
        cmd = f"source /opt/ros/humble/setup.bash && cd {LOCAL_WORKSPACE} && colcon build --symlink-install && source install/setup.bash"
        self.run_local(cmd, "Local Compiler (Ubuntu PC)")

    def btn_lidar(self):
        self.run_remote("ros2 launch ldlidar_stl_ros2 ld19.launch.py", "Lidar Sensor")

    def btn_motores(self):
        self.run_remote("python3 src/utils/scripts/base_controller.py", "Traction and Odometry")

    def _get_slam_args(self):
        r_mode = self.current_robot_mode
        return (
            f"robot_mode:={r_mode} "
            f"particle_count:={int(self.params['particle_count'])} "
            f"map_resolution:={self.params['map_resolution']} "
            f"linear_update:={self.params['linear_update']} "
            f"angular_update:={self.params['angular_update']} "
            f"meas_z_hit:={self.params['meas_z_hit']} "
            f"meas_z_rand:={self.params['meas_z_rand']} "
            f"meas_sigma:={self.params['meas_sigma']} "
            f"laser_max_range:={self.params['laser_max_range']}"
        )

    def btn_slam_remote(self):
        cmd = f"ros2 launch fastslam_thesis fastslam.launch.py {self._get_slam_args()}"
        self.run_remote(cmd, "SLAM Mapping (Pi)")

    def btn_slam_local(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && export ROS_DOMAIN_ID=30 && "
            f"ros2 launch fastslam_thesis fastslam.launch.py {self._get_slam_args()}"
        )
        self.run_local(cmd, "SLAM Mapping (Local PC)")

    def btn_rosbag_remote(self):
        name = self.entry_bag_name_pi.text().strip() or "rosbag_pi"
        cmd = f"mkdir -p ROSBAG && ros2 bag record -o ROSBAG/{name} /scan /tf /tf_static /odom"
        self.run_remote(cmd, f"Rosbag Recorder (Pi): {name}")

    def btn_sync_bags(self):
        current_ip = self.entry_ip.text().strip()
        if not current_ip: return
        cmd = f"mkdir -p {LOCAL_WORKSPACE}/ROSBAG && rsync -avz --progress {PI_USER}@{current_ip}:{PI_HOST_WORKSPACE}/ROSBAG/ {LOCAL_WORKSPACE}/ROSBAG/"
        self.run_local(cmd, "Syncing Rosbags Database (Pi -> Local PC)")

    def update_bag_list(self):
        self.combo_bags.clear()
        bag_dir = os.path.expanduser(f"{LOCAL_WORKSPACE.replace('~', '~')}/ROSBAG")
        if os.path.exists(bag_dir):
            bags = [f for f in os.listdir(bag_dir) if os.path.isdir(os.path.join(bag_dir, f))]
            bags.sort(reverse=True)
            if bags:
                self.combo_bags.addItems(bags)
            else:
                self.combo_bags.addItem("Empty ROSBAG folder")
        else:
            self.combo_bags.addItem("ROSBAG folder not found")

    def btn_play_bag_local(self):
        selected_bag = self.combo_bags.currentText()
        if not selected_bag or selected_bag in ["Empty ROSBAG folder", "ROSBAG folder not found"]:
            return
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && source install/setup.bash && export ROS_DOMAIN_ID=30 && "
            f"ros2 bag play ROSBAG/{selected_bag} --clock"
        )
        self.run_local(cmd, f"Rosbag Player: {selected_bag}")

    def btn_nav_remote(self):
        self.run_remote("python3 src/utils/scripts/simple_path.py", "Autonomous Navigation (Pi)")

    def btn_nav_local(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && source install/setup.bash && export ROS_DOMAIN_ID=30 && "
            f"python3 src/utils/scripts/simple_path.py"
        )
        self.run_local(cmd, "Autonomous Navigation (Local PC)")

    def btn_gazebo(self):
        cmd = (
            f"export TURTLEBOT3_MODEL=burger && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && export ROS_DOMAIN_ID=30 && "
            f"ros2 launch ./src/utils/launch/custom_map.launch.py"
        )
        self.run_local(cmd, "Gazebo Simulator")

    def btn_rviz(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && "
            f"source {LOCAL_WORKSPACE}/install/setup.bash && export ROS_DOMAIN_ID=30 && "
            f"ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=True"
        )
        self.run_local(cmd, "3D Viewer")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    ihm = IHMRobot()
    ihm.show()
    sys.exit(app.exec_())