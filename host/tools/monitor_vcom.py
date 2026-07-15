import serial
from serial.tools import list_ports
import sys
import time
import subprocess
import threading

def reset_board():
    time.sleep(1.0)
    print("[Host] Resetting board...")
    subprocess.run([r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe", "-c", "port=SWD", "-rst"], capture_output=True)
    print("[Host] Reset complete")

def main():
    stlink_port = None
    for p in list_ports.comports():
        if p.vid == 0x0483: # ST-Link VCOM
            stlink_port = p.device
            break
            
    if not stlink_port:
        print("ST-Link VCOM not found!")
        sys.exit(1)
        
    print(f"Monitoring ST-Link VCOM on {stlink_port} at 115200 baud for 10 seconds...")
    with serial.Serial(stlink_port, 115200, timeout=1) as ser:
        threading.Thread(target=reset_board, daemon=True).start()
        
        t0 = time.time()
        while time.time() - t0 < 10.0:
            line = ser.readline()
            if line:
                decoded = line.decode('ascii', 'replace').strip()
                if decoded:
                    print(decoded)
                    sys.stdout.flush()

if __name__ == "__main__":
    main()
