import serial
import time

# Set up serial communication (adjust port and baud rate if needed)
ser = serial.Serial('/dev/ttyACM0', 9600)  # Replace with your Arduino port


def send_motor_data(motor1, motor2):
    # Ensure the values are within the expected range
    motor1 = max(-255, min(255, motor1))
    motor2 = max(-255, min(255, motor2))

    # Create a message string with start '<' and end '>' characters
    message = f"<{motor1},{motor2}>"

    # Send the message to the Arduino
    ser.write(message.encode())

    # Optionally, wait for a response from the Arduino
    time.sleep(0.1)  # Wait a moment for Arduino to process
    if ser.in_waiting > 0:
        # Print the response from Arduino
        print(ser.read(ser.in_waiting).decode('utf-8'))


try:
    while True:
        motor1 = -128
        motor2 = -128
        send_motor_data(motor1, motor2)

except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
