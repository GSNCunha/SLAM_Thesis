# =============================================================================
# DYNAMIXEL PORT SCANNER UTILITY
# This script is a diagnostic tool used to identify the connected Dynamixel 
# XH430-W350-R actuators across the available serial USB ports. It sweeps 
# through the hardware IDs, pinging the RS-485 communication link to verify 
# the motor addresses before launching the main ROS 2 base controller.
# [See Section 4.1.1: Component List and Electronic Architecture]
# [See Section 4.2.2: Microcontroller Firmware]
# =============================================================================

from dynamixel_sdk import *

# =============================================================================
# SCAN PORT FUNCTION
# Initializes the serial port handler and packet handler for Protocol 2.0, 
# iterating through IDs 1 to 10 to find active motors on the given device.
# =============================================================================
def scan_port(device, baudrate):
    portHandler = PortHandler(device)
    packetHandler = PacketHandler(2.0)
    
    if not portHandler.openPort() or not portHandler.setBaudRate(baudrate):
        print(f"❌ Não foi possível abrir a porta {device}")
        return

    print(f"🔍 Escaneando {device} a {baudrate} bps...")
    for dxl_id in range(1, 11):
        model, comm, err = packetHandler.read2ByteTxRx(portHandler, dxl_id, 0)
        if comm == COMM_SUCCESS:
            print(f"✅ MOTOR ENCONTRADO! ID: {dxl_id} na porta {device}")
    portHandler.closePort()

# =============================================================================
# EXECUTION
# Tests the two specific ports discovered by the system 'ls' command 
# at the configured Dynamixel baudrate (57600 bps).
# =============================================================================
scan_port('/dev/ttyUSB0', 57600)
scan_port('/dev/ttyUSB1', 57600)