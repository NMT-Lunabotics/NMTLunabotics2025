import serial
import time

# Configure the serial connection
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # wait for the connection to initialize

# Define the string to send
for _ in range(5):
    message = "Hello Arduino\n"
    ser.write(message.encode('utf-8'))
    print("Sent:", message.strip())
    time.sleep(1)

ser.close()