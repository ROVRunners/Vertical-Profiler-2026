import serial
import time

PORT = "COM7"      # change this
BAUD = 115200

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)  # allow ESP32 to reset

        print("Connected.")
        print("Type a PWM value from 500 to 2500.")
        print("Type q to quit.\n")

        while True:
            value = input("PWM us> ").strip()

            if value.lower() == "q":
                break

            try:
                pwm = int(value)
            except ValueError:
                print("Enter a number.")
                continue

            if pwm < 500 or pwm > 2500:
                print("Out of range. Use 500 to 2500.")
                continue

            ser.write(f"{pwm}\n".encode("utf-8"))
            time.sleep(0.1)

            while ser.in_waiting:
                print(ser.readline().decode(errors="ignore").strip())

if __name__ == "__main__":
    main()