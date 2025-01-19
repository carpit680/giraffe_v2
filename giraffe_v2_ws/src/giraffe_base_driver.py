import serial
import time

# Open serial port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def send_cmd_vel(left_vel, right_vel):
    cmd = f"{left_vel} {right_vel}\n"
    ser.write(cmd.encode())

def read_odometry():
    while True:
        data = ser.readline().decode().strip()
        if data:
            print(f"{data}")

if __name__ == "__main__":
    try:
        while True:
            send_cmd_vel(0.5, 0.5)  
            read_odometry()
            time.sleep(1)
    except KeyboardInterrupt:
        ser.close()
