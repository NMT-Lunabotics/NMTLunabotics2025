import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from moon_messages.msg import Bytes
import serial
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Declare parameters
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_device', '/dev/ttyUSB0')
        self.declare_parameter('reconnect_timeout', 1.0)  # Seconds to wait before reconnection attempt
        
        # Get parameters
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        self.reconnect_timeout = self.get_parameter('reconnect_timeout').get_parameter_value().double_value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Bytes,
            '/serial_write',
            self.serial_write_callback,
            10)
        self.publisher = self.create_publisher(String, '/serial_read', 10)
        
        # Timer to periodically read from serial
        self.timer = self.create_timer(0.1, self.serial_read_callback)
        
        # Timer to attempt reconnection if needed
        self.reconnect_timer = self.create_timer(self.reconnect_timeout, self.check_connection)
    
    def connect_serial(self):
        """Establish a serial connection with proper error handling."""
        try:
            # Close existing connection if any
            if self.serial_conn is not None:
                self.get_logger().info('Closing existing serial connection...')
                self.serial_conn.close()
                time.sleep(1)  # Give it time to properly close
            
            self.get_logger().info(f'Opening serial connection to {self.serial_device} at {self.baud_rate} baud...')
            self.serial_conn = serial.Serial(
                self.serial_device, 
                self.baud_rate, 
                timeout=0.1, 
                dsrdtr=True, 
                rtscts=True
            )
            
            # Reset the Arduino by toggling DTR
            self.serial_conn.dtr = False
            time.sleep(0.1)
            self.serial_conn.dtr = True
            
            time.sleep(2)  # Wait for Arduino to initialize after reset
            self.get_logger().info('Serial connection established')
            
            # Flush any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial connection: {e}')
            return False
    
    def check_connection(self):
        """Check if the serial connection is still valid and reconnect if needed."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection lost, attempting to reconnect...')
            if self.connect_serial():
                self.get_logger().info('Successfully reconnected')
            else:
                self.get_logger().error('Reconnection failed')
    
    def serial_write_callback(self, msg):
        """Write data to the Arduino."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn('Cannot write to serial: connection not available')
            return
        
        try:
            # # Define start and end bytes
            # START_BYTE = b'\x02'
            # END_BYTE = b'\x03'

            # # Calculate message length and checksum
            # message_length = len(msg.data).to_bytes(2, byteorder='big')
            # checksum = (sum(msg.data) + sum(message_length)) & 0xFF

            # # Construct the full message
            # full_message = START_BYTE + message_length + msg.data + checksum.to_bytes(1, byteorder='big') + END_BYTE

            # # Write the full message to the serial connection
            # self.serial_conn.write(full_message)
            self.serial_conn.write(msg.data)
            self.serial_conn.flush()  # Ensure data is written
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.check_connection()  # Try to reconnect
    
    def serial_read_callback(self):
        """Read data from the Arduino."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        
        try:
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.readline()
                try:
                    decoded_data = data.decode('utf-8', errors='ignore').strip()
                    if decoded_data:  # Only publish if there's actual data
                        self.publisher.publish(String(data=decoded_data))
                except Exception as e:
                    self.get_logger().warn(f'Failed to decode serial data: {e}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}')
            self.check_connection()  # Try to reconnect
    
    def destroy_node(self):
        """Clean up resources when the node is shutting down."""
        self.get_logger().info('Shutting down serial_bridge_node...')
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.get_logger().info('Closing serial connection...')
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()