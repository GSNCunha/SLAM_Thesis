# =============================================================================
# HUMAN-MACHINE INTERFACE (HMI) CONTROL STATION
# This script acts as the centralized control dashboard for the robotic system. 
# It manages the execution of ROS 2 nodes across three operational modes: 
# Physical Robot (via SSH/Docker), Gazebo Simulation (Local), and Recorded Data 
# Playback (Rosbags). It also allows real-time tuning of the FastSLAM parameters.
# [See Section 4.3.4: Human-Machine Interface (HMI)]
# [See Section 4.3.3: Node Structure and Communication Flow]
# =============================================================================

import sys
import os
import subprocess
import signal
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QPushButton, QFrame, QScrollArea, QComboBox)
from PyQt5.QtGui import QFont, QCursor
from PyQt5.QtCore import Qt

# =============================================================================
# HMI AND ROS 2 ENVIRONMENT SETTINGS
# =============================================================================
PI_USER = "burguer"
CONTAINER = "slam_container_thesis"

# =============================================================================
# PATH CONFIGURATIONS
# =============================================================================
DOCKER_WORKSPACE = "/root/SLAM_Thesis"         # Absolute path INSIDE Docker
PI_HOST_WORKSPACE = "~/Desktop/SLAM_Thesis"    # Real path on the Raspberry Pi disk
LOCAL_WORKSPACE = "~/Desktop/SLAM_Thesis"      # Path on your PC (Local Ubuntu)

# Base command running remotely on the Raspberry Pi (without subshell execution)
DOCKER_BASE = f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && cd {DOCKER_WORKSPACE} && if [ -f install/setup.bash ]; then source install/setup.bash; fi && export ROS_DOMAIN_ID=30 && "

class IHMRobot(QWidget):
    def __init__(self):
        super().__init__()
        
        # =============================================================================
        # FASTSLAM ALGORITHM PARAMETERS
        # Internal dictionary to store the state of all configurable parameters 
        # exposed to the user.
        # [See Section 4.3.4: Figure 22 - SLAM Params window]
        # =============================================================================
        self.params = {
            # Static Parameters (Initialization)
            "particle_count": 1000,
            "map_resolution": 0.02,
            "linear_update": 0.05,
            "angular_update": 0.260,
            "meas_z_hit": 0.950,
            "meas_z_rand": 0.05,
            "meas_sigma": 0.05,
            "laser_max_range": 3.50,
            
            # Dynamic Parameters (Real-Time)
            "beam_skip": 5,
            "kernel_size": 1,
            "alpha1": 0.5,
            "alpha2": 0.5,
            "alpha3": 0.5,
            "alpha4": 0.5
        }
        
        self.initUI()

    def initUI(self):
        # Main Window Settings
        self.setWindowTitle("Control Station - FastSLAM 1.0")
        self.setFixedSize(450, 880)
        self.setStyleSheet("background-color: #1e272e; color: white;")

        # --- MAIN CONTAINER FOR SCROLL BAR ---
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setStyleSheet("QScrollArea { border: none; } QScrollBar { background: #2f3640; }")

        # --- CONTENT INSIDE THE SCROLL ---
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        # Header
        lbl_title = QLabel("🤖 Operation HMI")
        lbl_title.setFont(QFont("Arial", 18, QFont.Bold))
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setStyleSheet("color: #d2dae2;")
        layout.addWidget(lbl_title)

        # Robot IP Field (SSH Target)
        ip_layout = QHBoxLayout()
        lbl_ip = QLabel("IP Address:")
        lbl_ip.setFont(QFont("Arial", 10))
        self.entry_ip = QLineEdit("192.168.1.203")
        self.entry_ip.setStyleSheet("background-color: #2f3640; border: 1px solid #485460; padding: 5px; border-radius: 4px; color: white;")
        ip_layout.addWidget(lbl_ip)
        ip_layout.addWidget(self.entry_ip)
        layout.addLayout(ip_layout)

        # =============================================================================
        # OPERATION MODE SELECTOR
        # Switches between the physical robot, the Gazebo simulation, and the 
        # offline Rosbag player data sources.
        # [See Section 4.3.3: Node Structure and Communication Flow]
        # =============================================================================
        mode_layout = QHBoxLayout()
        lbl_mode = QLabel("🖥️ Environment:")
        lbl_mode.setFont(QFont("Arial", 10, QFont.Bold))
        lbl_mode.setStyleSheet("color: #f39c12;")
        
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["Physical (Real Robot)", "Simulation (Gazebo)", "Recording (Rosbag)"])
        self.combo_mode.setStyleSheet("background-color: #2f3640; border: 1px solid #f39c12; padding: 5px; border-radius: 4px; color: white; font-weight: bold;")
        self.combo_mode.setFixedHeight(35)
        
        mode_layout.addWidget(lbl_mode)
        mode_layout.addWidget(self.combo_mode, stretch=1)
        layout.addLayout(mode_layout)

        # Universal function to create buttons with consistent design
        def create_btn(text, color, callback):
            btn = QPushButton(text)
            btn.setFont(QFont("Arial", 10, QFont.Bold))
            btn.setCursor(QCursor(Qt.PointingHandCursor))
            btn.setFixedHeight(45)
            btn.setStyleSheet(f"QPushButton {{ background-color: {color}; color: white; border: none; border-radius: 6px; }} QPushButton:hover {{ border: 2px solid white; }}")
            btn.clicked.connect(callback)
            return btn

        # =============================================================================
        # SYSTEM OPERATIONS (DOCKER AND SENSORS)
        # =============================================================================
        layout.addWidget(create_btn("0. Initialize System (Docker)", "#8e44ad", self.btn_start_docker))
        layout.addWidget(create_btn("🔄 Restart Docker (Reset)", "#34495e", self.btn_restart_docker))
        
        # --- BUILD DIVISION (COMPILATION) ---
        build_layout = QHBoxLayout()
        build_layout.setSpacing(10)
        build_layout.addWidget(create_btn("⚙️ Build (Pi)", "#7f8c8d", self.btn_build_remote))
        build_layout.addWidget(create_btn("⚙️ Build (Local)", "#95a5a6", self.btn_build_local))
        layout.addLayout(build_layout)
        
        # --- CLEAN BUILD DIVISION (CLEAN) ---
        clean_layout = QHBoxLayout()
        clean_layout.setSpacing(10)
        clean_layout.addWidget(create_btn("🧹 Clean (Pi)", "#e67e22", self.btn_clean_remote))
        clean_layout.addWidget(create_btn("🧹 Clean (Local)", "#d35400", self.btn_clean_local))
        layout.addLayout(clean_layout)
        
        layout.addWidget(create_btn("1. Start Sensor (Lidar)", "#c0392b", self.btn_lidar))
        layout.addWidget(create_btn("2. Engage Traction (Motors)", "#d35400", self.btn_motores))

        # =============================================================================
        # STATIC PARAMETERS (PRE-BOOT)
        # =============================================================================
        line1 = QFrame(); line1.setFrameShape(QFrame.HLine); line1.setStyleSheet("background-color: #485460; margin-top: 10px;")
        layout.addWidget(line1)

        lbl_estaticos = QLabel("🟢 Initialization Setup (Pre-Boot)")
        lbl_estaticos.setFont(QFont("Arial", 11, QFont.Bold))
        lbl_estaticos.setStyleSheet("color: #2ecc71;")
        layout.addWidget(lbl_estaticos)

        # Universal function to create Steppers (+ / - and Editable Text Field)
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
            
            # Using QLineEdit instead of QLabel to allow direct manual typing
            entry_val = QLineEdit(val_str)
            entry_val.setFont(QFont("Courier", 11, QFont.Bold))
            entry_val.setAlignment(Qt.AlignCenter)
            entry_val.setFixedSize(65, 30)
            entry_val.setStyleSheet("background-color: white; color: black; border-radius: 4px; border: none;")
            
            btn_plus = QPushButton("+")
            btn_plus.setFixedSize(30, 30)
            btn_plus.setStyleSheet("background-color: #2ecc71; border-radius: 4px; font-weight: bold;")
            
            # Function to validate and apply the new value
            def update_value(new_val):
                # Restricts input strictly between defined minimum and maximum limits
                new_val = max(min_val, min(new_val, max_val))
                
                if is_float:
                    new_val = round(new_val, 4)
                    entry_val.setText(f"{new_val:.3f}")
                else:
                    new_val = int(new_val)
                    entry_val.setText(str(new_val))
                    
                self.params[param_name] = new_val
                
                # If designated as dynamic, transmits the update command directly to ROS 2
                if is_dynamic:
                    cmd = f"ros2 param set /fastslam_node {param_name} {new_val}"
                    self.run_remote_silent(cmd)

            # Callback for +/- step buttons
            def on_click(delta):
                current = self.params[param_name]
                update_value(current + delta)

            # Callback for manual text entry completion (Enter key or focus loss)
            def on_text_edit():
                try:
                    # Replaces comma with period to avoid parsing errors in decimal values
                    text_val = entry_val.text().replace(',', '.')
                    new_val = float(text_val) if is_float else int(float(text_val))
                    update_value(new_val)
                except ValueError:
                    # If invalid text is typed (e.g., characters), reverts to the previous valid state
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
            h_layout.addWidget(entry_val) # Adds the interactive text field
            h_layout.addWidget(btn_plus)
            layout.addWidget(frame)

        # 1. Structure and Triggers
        create_stepper("Particle Count", "particle_count", 1, 1000, 50, is_float=False, is_dynamic=False)
        create_stepper("Map Resolution (m/px)", "map_resolution", 0.01, 0.20, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Linear Trigger (m)", "linear_update", 0.01, 0.50, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Angular Trigger (rad)", "angular_update", 0.01, 0.50, 0.05, is_float=True, is_dynamic=False)
        
        # 2. Measurement Model
        create_stepper("Z_hit (Hit Prob.)", "meas_z_hit", 0.01, 1.0, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Z_rand (Uniform Noise)", "meas_z_rand", 0.01, 1.0, 0.01, is_float=True, is_dynamic=False)
        create_stepper("Sigma Hit (Variance)", "meas_sigma", 0.05, 2.0, 0.05, is_float=True, is_dynamic=False)
        create_stepper("Max Lidar Range (m)", "laser_max_range", 1.0, 12.0, 0.5, is_float=True, is_dynamic=False)

        # --- SLAM DIVISION (HORIZONTAL LAYOUT) ---
        slam_layout = QHBoxLayout()
        slam_layout.setSpacing(10)
        slam_layout.addWidget(create_btn("3a. SLAM (Pi)", "#f39c12", self.btn_slam_remote))
        slam_layout.addWidget(create_btn("3b. SLAM (Local PC)", "#f1c40f", self.btn_slam_local))
        layout.addLayout(slam_layout)

        # =============================================================================
        # DYNAMIC PARAMETERS (LIVE TUNING)
        # =============================================================================
        line2 = QFrame(); line2.setFrameShape(QFrame.HLine); line2.setStyleSheet("background-color: #485460; margin-top: 10px;")
        layout.addWidget(line2)

        lbl_dinamicos = QLabel("🎛️ Fine Tuning (Real-Time Adjustment)")
        lbl_dinamicos.setFont(QFont("Arial", 11, QFont.Bold))
        lbl_dinamicos.setStyleSheet("color: #f1c40f;")
        layout.addWidget(lbl_dinamicos)

        # 1. Lidar Computational Cost
        create_stepper("Beam Skip", "beam_skip", 1, 15, 1, is_float=False, is_dynamic=True)
        create_stepper("Kernel Size (Likelihood Search)", "kernel_size", 0, 3, 1, is_float=False, is_dynamic=True)
        
        # 2. Odometry Alphas
        create_stepper("Alpha 1 (Turn-Turn)", "alpha1", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        create_stepper("Alpha 2 (Turn-Straight)", "alpha2", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        create_stepper("Alpha 3 (Straight-Straight)", "alpha3", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)
        create_stepper("Alpha 4 (Straight-Turn)", "alpha4", 0.0, 0.5, 0.005, is_float=True, is_dynamic=True)

        # =============================================================================
        # FINAL OPERATIONS AND VISUALIZATION
        # =============================================================================
        line3 = QFrame(); line3.setFrameShape(QFrame.HLine); line3.setStyleSheet("background-color: #485460; margin-top: 10px;")
        layout.addWidget(line3)

        # --- TEXT BOX FOR ROSBAG NAME ---
        bag_name_layout = QHBoxLayout()
        lbl_bag_name = QLabel("Rosbag Name:")
        lbl_bag_name.setFont(QFont("Arial", 10, QFont.Bold))
        lbl_bag_name.setStyleSheet("color: #1abc9c;")
        self.entry_bag_name = QLineEdit("my_recording")
        self.entry_bag_name.setStyleSheet("background-color: #2f3640; border: 1px solid #1abc9c; padding: 5px; border-radius: 4px; color: white;")
        bag_name_layout.addWidget(lbl_bag_name)
        bag_name_layout.addWidget(self.entry_bag_name)
        layout.addLayout(bag_name_layout)

        # --- ROSBAG RECORDING DIVISION (HORIZONTAL LAYOUT) ---
        rosbag_layout = QHBoxLayout()
        rosbag_layout.setSpacing(10)
        rosbag_layout.addWidget(create_btn("⏺️ Record Data (Pi)", "#16a085", self.btn_rosbag_remote))
        rosbag_layout.addWidget(create_btn("⏺️ Record Data (PC)", "#1abc9c", self.btn_rosbag_local))
        layout.addLayout(rosbag_layout)
        
        # --- BUTTON TO SYNC ROSBAGS (REMOTE -> LOCAL) ---
        layout.addWidget(create_btn("🔄 Sync Rosbags (Pi ➔ PC)", "#2980b9", self.btn_sync_bags))
        
        # --- ROSBAG PLAYER (PLAY) ---
        bag_play_layout = QHBoxLayout()
        bag_play_layout.setSpacing(10)
        
        self.combo_bags = QComboBox()
        self.combo_bags.setStyleSheet("background-color: #2f3640; border: 1px solid #485460; padding: 5px; border-radius: 4px; color: white; font-weight: bold;")
        self.combo_bags.setFixedHeight(45)
        self.update_bag_list()
        
        bag_play_layout.addWidget(self.combo_bags, stretch=3)
        bag_play_layout.addWidget(create_btn("🔄", "#7f8c8d", self.update_bag_list), stretch=1)
        bag_play_layout.addWidget(create_btn("▶️ Play Bag", "#2980b9", self.btn_play_bag_local), stretch=2)
        layout.addLayout(bag_play_layout)

        # --- AUTONOMOUS ROUTE DIVISION (HORIZONTAL LAYOUT) ---
        nav_layout = QHBoxLayout()
        nav_layout.setSpacing(10)
        nav_layout.addWidget(create_btn("4a. Route (Pi)", "#27ae60", self.btn_nav_remote))
        nav_layout.addWidget(create_btn("4b. Route (Local PC)", "#2ecc71", self.btn_nav_local))
        layout.addLayout(nav_layout)
        
        # Local GUI Buttons (Gazebo and RViz)
        gui_layout = QHBoxLayout()
        gui_layout.setSpacing(10)
        gui_layout.addWidget(create_btn("🌍 Simulation (Gazebo)", "#9b59b6", self.btn_gazebo))
        gui_layout.addWidget(create_btn("💻 3D Vision (RViz)", "#2980b9", self.btn_rviz))
        layout.addLayout(gui_layout)

        # Adds all built content to the Scroll Area
        scroll_area.setWidget(content_widget)
        main_layout.addWidget(scroll_area)


    # =============================================================================
    # COMMUNICATION PROTOCOLS (SSH / DOCKER / LOCAL)
    # Functions responsible for securely tunneling execution commands to the 
    # remote hardware or spawning local processes.
    # =============================================================================
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

    # =============================================================================
    # OPERATION ROUTINES (CALLBACKS)
    # Implementation of individual button behaviors triggering the ROS 2 ecosystem.
    # =============================================================================
    def btn_start_docker(self):
        self.run_local(f"ssh -t {PI_USER}@{self.entry_ip.text()} 'docker start {CONTAINER}'", "Initializing Docker")

    def btn_restart_docker(self):
        self.run_local(f"ssh -t {PI_USER}@{self.entry_ip.text()} 'docker restart {CONTAINER}'", "Restarting Docker")

    # --- CLEAN AND COMPILATION CALLBACKS (CLEAN & BUILD) ---
    def btn_clean_remote(self):
        # Removes the single quotes from the echo to prevent gnome-terminal breakage
        cmd = "rm -rf build/ install/ log/ && echo Remote Build Cleaned Successfully"
        self.run_remote(cmd, "Cleaning Build (Pi)")
        
    def btn_clean_local(self):
        # Employs sudo privileges to ensure complete local removal of Docker-generated files.
        cmd = f"cd {LOCAL_WORKSPACE} && sudo rm -rf build/ install/ log/ && echo 'Local Build Cleaned!'"
        self.run_local(cmd, "Cleaning Build (Local PC)")

    def btn_build_remote(self):
        # Sequentially compiles packages within the src directory to prevent RAM saturation on the Pi
        build_cmd = (
            "colcon build --symlink-install --executor sequential && "
            "source install/setup.bash"
        )
        self.run_remote(build_cmd, "Remote Compiler (Pi)")

    def btn_build_local(self):
        # Standard unconstrained parallel compilation for the local workstation
        cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"colcon build --symlink-install && "
            f"source install/setup.bash"
        )
        self.run_local(cmd, "Local Compiler (Ubuntu PC)")

    def btn_lidar(self):
        # Port assignment and hardware permissions are autonomously resolved by UDEV rules
        self.run_remote("ros2 launch ldlidar_stl_ros2 ld19.launch.py", "Lidar Sensor")

    def btn_motores(self):
        # The base controller script interfaces directly with /dev/motores symlink
        self.run_remote("python3 src/utils/scripts/base_controller.py", "Traction and Odometry")

    # =============================================================================
    # MODE CAPTURE LOGIC (SLAM PARAMETERS COMPILATION)
    # Extracts the GUI parameter values and dynamically formats them as launch 
    # arguments for the primary FastSLAM node.
    # =============================================================================
    def _get_slam_args(self):
        # Parses the environment combo box to define the operational mode
        mode_text = self.combo_mode.currentText()
        if "Gazebo" in mode_text:
            r_mode = "gazebo"
        elif "Rosbag" in mode_text:
            r_mode = "rosbag"
        else:
            r_mode = "real"

        return (
            f"robot_mode:={r_mode} "  # <-- Forwards the selected execution mode
            f"particle_count:={int(self.params['particle_count'])} "
            f"map_resolution:={self.params['map_resolution']} "
            f"linear_update:={self.params['linear_update']} "
            f"angular_update:={self.params['angular_update']} "
            f"meas_z_hit:={self.params['meas_z_hit']} "
            f"meas_z_rand:={self.params['meas_z_rand']} "
            f"meas_sigma:={self.params['meas_sigma']} "
            f"laser_max_range:={self.params['laser_max_range']}"
        )

    # --- SLAM EXECUTION CALLBACKS ---
    def btn_slam_remote(self):
        cmd = f"ros2 launch fastslam_thesis fastslam.launch.py {self._get_slam_args()}"
        self.run_remote(cmd, "SLAM Mapping (Pi)")

    def btn_slam_local(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 launch fastslam_thesis fastslam.launch.py {self._get_slam_args()}"
        )
        self.run_local(cmd, "SLAM Mapping (Local PC)")

    # --- ROSBAG RECORDING CALLBACKS (DATA ACQUISITION) ---
    def btn_rosbag_remote(self):
        name = self.entry_bag_name.text().strip()
        if not name: name = "rosbag_pi"
        # Verifies directory existence and records targeted topics into the specified folder
        cmd = f"mkdir -p ROSBAG && ros2 bag record -o ROSBAG/{name} /scan /tf /tf_static /odom"
        self.run_remote(cmd, f"Rosbag Recorder (Pi): {name}")

    def btn_rosbag_local(self):
        name = self.entry_bag_name.text().strip()
        if not name: name = "rosbag_local"
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"mkdir -p ROSBAG && "
            f"ros2 bag record -o ROSBAG/{name} /scan /tf /tf_static /odom"
        )
        self.run_local(cmd, f"Rosbag Recorder (Local PC): {name}")

    # --- SYNC VIA RSYNC (SKIPS DUPLICATES OPTIMIZATION) ---
    def btn_sync_bags(self):
        current_ip = self.entry_ip.text().strip()
        if not current_ip: return
        # Ensures local directory availability and pulls strictly missing/new data via rsync
        cmd = (
            f"mkdir -p {LOCAL_WORKSPACE}/ROSBAG && "
            f"rsync -avz --progress {PI_USER}@{current_ip}:{PI_HOST_WORKSPACE}/ROSBAG/ {LOCAL_WORKSPACE}/ROSBAG/"
        )
        self.run_local(cmd, "Syncing Rosbags Database (Pi ➔ Local PC)")

    # --- ROSBAG PLAYER CALLBACKS (DATA REPLAY) ---
    def update_bag_list(self):
        self.combo_bags.clear()
        # Evaluates the localized workspace path established globally
        bag_dir = os.path.expanduser(f"{LOCAL_WORKSPACE.replace('~', '~')}/ROSBAG")
        
        if os.path.exists(bag_dir):
            # In ROS 2 architecture, bags are container folders housing the underlying .db3 databases
            bags = [f for f in os.listdir(bag_dir) if os.path.isdir(os.path.join(bag_dir, f))]
            bags.sort(reverse=True) # Displays chronological acquisitions descending
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
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 bag play ROSBAG/{selected_bag} --clock" # Ensures simulated clock broadcast
        )
        self.run_local(cmd, f"Rosbag Player: {selected_bag}")

    # --- AUTONOMOUS ROUTE EXECUTION CALLBACKS ---
    def btn_nav_remote(self):
        self.run_remote("python3 src/utils/scripts/simple_path.py", "Autonomous Navigation (Pi)")

    def btn_nav_local(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"python3 src/utils/scripts/simple_path.py"
        )
        self.run_local(cmd, "Autonomous Navigation (Local PC)")

    # --- GRAPHICAL VISUALIZATION CALLBACKS ---
    def btn_gazebo(self):
        # Bypass package recognition by directly executing the explicit file path
        cmd = (
            f"export TURTLEBOT3_MODEL=burger && "
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"cd {LOCAL_WORKSPACE} && "
            f"source install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 launch ./src/utils/launch/custom_map.launch.py"
        )
        self.run_local(cmd, "Gazebo Simulator")

    def btn_rviz(self):
        cmd = (
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"source /opt/ros/humble/setup.bash && "
            f"source {LOCAL_WORKSPACE}/install/setup.bash && "
            f"export ROS_DOMAIN_ID=30 && "
            f"ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=True" # Forces synchronization with /clock
        )
        self.run_local(cmd, "3D Viewer")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    ihm = IHMRobot()
    ihm.show()
    sys.exit(app.exec_())