import tkinter as tk
import subprocess

# ==========================================
# ⚙️ CONFIGURAÇÕES DA IHM
# ==========================================
PI_USER = "burguer"
CONTAINER = "slam_container_thesis"
WORKSPACE = "~/SLAM_Thesis/Desktop/SLAM_Thesis"

# Comando base: Primeiro acessa a pasta, DEPOIS roda o source do install!
DOCKER_BASE = f"source /opt/ros/humble/setup.bash && cd {WORKSPACE} && source install/setup.bash && export ROS_DOMAIN_ID=30 && "

# ==========================================
# 🚀 PROTOCOLOS DE COMUNICAÇÃO
# ==========================================

def run_remote(command_suffix, title="Terminal"):
    """Abre o CMD do Windows, faz SSH, entra no Docker e roda o ROS"""
    current_ip = entry_ip.get().strip()
    if not current_ip:
        print("[Erro] O campo de IP está vazio!")
        return

    # O comando completo que vai rodar lá dentro
    linux_cmd = f"{DOCKER_BASE} {command_suffix}"
    
    # O truque final: aspas duplas por fora para o SSH enviar, e aspas simples por dentro para o bash ler
    ssh_cmd = f'ssh -t {PI_USER}@{current_ip} "docker exec -it {CONTAINER} bash -c \'{linux_cmd}\'"'
    
    # Removemos as aspas em volta de {ssh_cmd} para o Windows não se confundir
    terminal_cmd = f'start "{title}" cmd /k {ssh_cmd}'
    
    subprocess.Popen(terminal_cmd, shell=True)
    print(f"[IHM] Conectando subsistema: {title} no IP {current_ip}")

def run_local(command, title="Terminal Local"):
    """Abre um CMD local no seu PC Windows"""
    terminal_cmd = f'start "{title}" cmd /k "{command}"'
    subprocess.Popen(terminal_cmd, shell=True)
    print(f"[IHM] Iniciando rotina local: {title}")

# ==========================================
# 🎛️ ROTINAS DE OPERAÇÃO
# ==========================================

def btn_start_docker():
    """Acorda o contêiner do Docker no Raspberry Pi"""
    current_ip = entry_ip.get().strip()
    if not current_ip:
        print("[Erro] O campo de IP está vazio!")
        return
        
    ssh_cmd = f'ssh -t {PI_USER}@{current_ip} "docker start {CONTAINER}"'
    terminal_cmd = f'start "Inicializando Sistema" cmd /k "{ssh_cmd}"'
    subprocess.Popen(terminal_cmd, shell=True)
    print(f"[IHM] Acordando o robô no IP {current_ip}")

def btn_lidar():
    run_remote("ros2 launch ldlidar_stl_ros2 ld19.launch.py", "Sensor Lidar")

def btn_motores():
    run_remote("python3 base_controller.py", "Tração e Odometria")

def btn_slam():
    run_remote("ros2 launch fastslam_thesis fastslam.launch.py", "Mapeamento SLAM")

def btn_nav():
    run_remote("python3 simple_path.py", "Navegação Autônoma")

def btn_rviz():
    run_local("set ROS_DOMAIN_ID=30 && rviz2", "Visualizador 3D")

# ==========================================
# 🎨 INTERFACE HOMEM-MÁQUINA (GUI)
# ==========================================

# Inicialização da tela
root = tk.Tk()
root.title("IHM de Operação - Burguer Bot")
root.geometry("380x560") 
root.configure(bg="#1e272e")

# Cabeçalho da IHM
lbl_title = tk.Label(root, text="🤖 IHM de Operação", font=("Arial", 18, "bold"), bg="#1e272e", fg="#d2dae2")
lbl_title.pack(pady=(20, 5))

# Campo de IP
frame_ip = tk.Frame(root, bg="#1e272e")
frame_ip.pack(pady=10)

lbl_ip = tk.Label(frame_ip, text="Endereço IP do Robô:", font=("Arial", 10), bg="#1e272e", fg="#bdc3c7")
lbl_ip.pack(side=tk.LEFT, padx=5)

entry_ip = tk.Entry(frame_ip, font=("Arial", 12), width=15, bg="#2f3640", fg="white", insertbackground="white")
entry_ip.insert(0, "192.168.1.203") 
entry_ip.pack(side=tk.LEFT)

# Estilo dos botões
btn_style = {"font": ("Arial", 11, "bold"), "width": 26, "height": 2, "cursor": "hand2", "borderwidth": 0}

# Botão NOVO de Inicialização
tk.Button(root, text="0. Inicializar Sistema (Docker)", bg="#8e44ad", fg="white", command=btn_start_docker, **btn_style).pack(pady=(5, 15))

# Botões de Ação do Robô
tk.Button(root, text="1. Iniciar Sensor (Lidar)", bg="#c0392b", fg="white", command=btn_lidar, **btn_style).pack(pady=5)
tk.Button(root, text="2. Acoplar Tração (Motores)", bg="#d35400", fg="white", command=btn_motores, **btn_style).pack(pady=5)
tk.Button(root, text="3. Processamento SLAM", bg="#f39c12", fg="white", command=btn_slam, **btn_style).pack(pady=5)
tk.Button(root, text="4. Iniciar Rota Autônoma", bg="#27ae60", fg="white", command=btn_nav, **btn_style).pack(pady=5)

tk.Frame(root, height=2, bg="#485460", width=330).pack(pady=15)

tk.Button(root, text="💻 Abrir Visualizador 3D", bg="#2980b9", fg="white", command=btn_rviz, **btn_style).pack(pady=5)

# Permitir que Ctrl+C no terminal feche o programa graciosamente
import signal
try:
    signal.signal(signal.SIGINT, signal.SIG_DFL)
except AttributeError:
    pass

root.mainloop()