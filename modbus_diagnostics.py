import serial.tools.list_ports
import serial
import time
import struct

# --- CONFIGURATION ---
PORT = 'COM10'  # <--- CHANGE THIS to your COM port (e.g., 'COM5')
BAUD = 230400
PARITY = serial.PARITY_EVEN
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS
SLAVE_ID = 21

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder='little')

def run_diagnostics():
    print(f"--- STM32 Modbus Diagnostic Tool (Debug Mode) ---")

    print("\n[Step 0] Available Ports:")
    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(f"  - {p.device}: {p.description}")

    print(f"\nConnecting to {PORT} at {BAUD} baud (8E1)...")
    
    try:
        ser = serial.Serial(
            port=PORT, 
            baudrate=BAUD, 
            parity=PARITY, 
            stopbits=STOPBITS, 
            bytesize=BYTESIZE, 
            timeout=1
        )
    except Exception as e:
        print(f"ERROR: Could not open port {PORT}. Is it in use by another program?")
        print(f"Details: {e}")
        return

    # STEP 1: SNIFFING (Is the robot sending anything?)
    print("\n[Step 1] Sniffing for 3 seconds (Is the robot sending anything?)...")
    start_time = time.time()
    raw_data = b''
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            raw_data += ser.read(ser.in_waiting)
    
    if raw_data:
        print(f"  SUCCESS: Received {len(raw_data)} raw bytes from Robot.")
        print(f"  Raw Hex: {raw_data.hex(' ')}")
        if b'YA' in raw_data or b'Y' in raw_data:
            print("  DETECTED: Heartbeat string 'YA' found in stream!")
    else:
        print("  FAILED: No data received from Robot. Check if switch is in BASE mode.")

    # STEP 2: MODBUS READ (Can we talk to it?)
    print("\n[Step 2] Attempting Modbus Read (Slave 21, Reg 0)...")
    # Function 03: Read Holding Registers (Addr 0, Count 1)
    # [SlaveID] [FC] [AddrHi] [AddrLo] [CountHi] [CountLo] [CRCLo] [CRCHi]
    request = bytearray([SLAVE_ID, 0x03, 0x00, 0x00, 0x00, 0x01])
    request += calculate_crc(request)
    
    print(f"  Sending: {request.hex(' ')}")
    ser.write(request)
    
    response = ser.read(7) # Expected: [21][3][2][ByteHi][ByteLo][CRC][CRC]
    if response:
        print(f"  Response: {response.hex(' ')}")
        if len(response) >= 5 and response[0] == SLAVE_ID and response[1] == 0x03:
            val = struct.unpack('>H', response[3:5])[0]
            print(f"  SUCCESS! Heartbeat Register Value: {val}")
            if val == 22881:
                print("  ROBOT SAYS: 'YA' (Heartbeat is correct)")
        else:
            print("  ERROR: Received invalid Modbus response.")
    else:
        print("  FAILED: No Modbus response from Robot.")

    ser.close()
    print("\nDiagnostics Complete.")

if __name__ == "__main__":
    # Note: You need to 'pip install pyserial' if you don't have it.
    run_diagnostics()
