from dynamixel_sdk import *

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

# Testa nas duas portas que o seu ls mostrou
scan_port('/dev/ttyUSB0', 57600)
scan_port('/dev/ttyUSB1', 57600)