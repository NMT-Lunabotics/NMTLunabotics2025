import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
from serial.msg import Actuators
from serial.msg import Motors

class ConvertSerialNode(Node):
    def __init__(self):
        super().__init__('convert_serial_node')

        # Create publisher for serial_write
        self.serial_write_publisher = self.create_publisher(String, 'serial_write', 10)

        # Create subscriber for serial_read
        self.serial_read_subscriber = self.create_subscription(String, 'serial_read', self.serial_read_callback, 10)

        # Create subscribers for actuator_control and motor_control
        self.actuator_control_subscriber = self.create_subscription(Actuators, 'actuator_control', self.listener_callback, 10)
        self.motor_control_subscriber = self.create_subscription(Motors, 'motor_control', self.listener_callback, 10)

        # Create publisher for actuator_feedback
        self.actuator_feedback_publisher = self.create_publisher(Actuators, 'actuator_feedback', 10)
        
    def listener_callback(self, msg):
        # Process incoming messages and publish to serial_write
        self.get_logger().info(f'Received message: {msg.data}')
        self.serial_write_publisher.publish(msg)

    def serial_read_callback(self, msg):
        # Process incoming serial_read messages and publish to respective topics
        self.get_logger().info(f'Received serial read message: {msg.data}')
        for topic, publisher in self.publishers.items():
            publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConvertSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()