import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from moon_messages.msg import Actuators, Motors,  Leds, Bytes
import struct

class SerialConvertNode(Node):
    def __init__(self):
        super().__init__('serial_convert_node')
        
        self.serial_write_publisher = self.create_publisher(Bytes, '/serial_write', 10)
        
        self.create_subscription(Actuators, '/actuator_control', self.actuator_control_subscriber, 10)
        self.create_subscription(Motors, '/motor_control', self.motor_control_subscriber, 10)
        self.create_subscription(Bool, '/heartbeat_control', self.servo_control_subscriber, 10)
        self.create_subscription(Leds, '/led_control', self.led_control_subscriber, 10)
    
    def actuator_control_subscriber(self, msg):
        data = struct.pack('>Bhhbb', ord('A'), msg.arm_pos, msg.bucket_pos, msg.arm_vel, msg.bucket_vel)
        self.send_serial_data(data)
    
    def motor_control_subscriber(self, msg):
        data = struct.pack('>Bbb', ord('M'), msg.left, msg.right)
        self.send_serial_data(data)
    
    def servo_control_subscriber(self, msg):
        data = struct.pack('>BB', ord('S'), 1 if msg.data else 0)
        self.send_serial_data(data)
    
    def led_control_subscriber(self, msg):
        data = struct.pack('>BBBBB', ord('L'), int(msg.red), int(msg.yellow), int(msg.green), int(msg.blue))
        self.send_serial_data(data)
    
    def send_serial_data(self, data):
        start_byte = b'\x02'
        end_byte = b'\x03'
        length_byte = struct.pack('>B', len(data))
        message = start_byte + length_byte + data + end_byte
        self.serial_write_publisher.publish(Bytes(data=message))
    
    def destroy_node(self):
        """Clean up resources when the node is shutting down."""
        self.get_logger().info('Shutting down serial_convert_node...')
        
        # Publish a message to turn off the LED
        led_off_msg = Leds(red=True, yellow=False, green=False, blue=False)
        self.serial_write_publisher.publish(led_off_msg)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialConvertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
