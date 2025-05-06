import serial

# Configure the serial connection
serial_device = '/dev/ttyACM0'  # Replace with your serial port
baud_rate = 115200

# Construct the command
start_byte = b'\x02'
end_byte = b'\x03'
payload = struct.pack('>BBBBB', ord('L'), 0, 0, 0, 1)  # Command 'L', blue LED on
length_byte = struct.pack('>B', len(payload))
command = start_byte + length_byte + payload + end_byte

# Send the command
with serial.Serial(serial_device, baud_rate, timeout=1) as ser:
    ser.write(command)
    print("Command sent:", command)