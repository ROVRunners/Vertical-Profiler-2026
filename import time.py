import time
import serial

PORT = "COM7"      # change this
BAUD = 115200

with serial.Serial(PORT, BAUD, timeout=2) as ser:
    time.sleep(2)  # give board time to reset
    utc_epoch = str(int(time.time())) + "\n"
    ser.write(utc_epoch.encode("utf-8"))
    print("Sent UTC epoch:", utc_epoch.strip())

    time.sleep(1)
    while ser.in_waiting:
        print(ser.readline().decode(errors="ignore").strip())