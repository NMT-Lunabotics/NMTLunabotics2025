import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Declare parameters
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('serial_device', '/dev/ttyUSB0')
        
        # Get parameters
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        
        # Initialize serial connection
        self.serial_conn = serial.Serial(serial_device, baud_rate, timeout=1)
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            String,
            '/serial_write',
            self.serial_write_callback,
            10)
        self.publisher = self.create_publisher(String, '/serial_read', 10)
        
        # Timer to periodically read from serial
        self.timer = self.create_timer(0.1, self.serial_read_callback)
        
    def serial_write_callback(self, msg):
        self.serial_conn.write(msg.data.encode())
        
    def serial_read_callback(self):
        if self.serial_conn.in_waiting > 0:
            data = self.serial_conn.readline().decode().strip()
            self.publisher.publish(String(data=data))

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()