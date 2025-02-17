import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from moon_messages.msg import Actuators
from moon_messages.msg import Motors

class SerialConvertNode(Node):
    def __init__(self):
        super().__init__('serial_convert_node')

        # Create publisher for serial_write
        self.serial_write_publisher = self.create_publisher(String, 'serial_write', 10)

        # Create subscribers for actuator_control and motor_control
        self.actuator_control_subscriber = self.create_subscription(Actuators, 'actuator_control', self.actuator_control_subscriber, 10)
        self.motor_control_subscriber = self.create_subscription(Motors, 'motor_control', self.motor_control_subscriber, 10)

        self.last_act_serial_msg = String()
        self.last_motor_serial_msg = String()

    def actuator_control_subscriber(self, msg):
        actuator_control_msg = String()
        actuator_control_msg.data = f'<A,{msg.arm_pos},{msg.bucket_pos},{msg.arm_vel},{msg.bucket_vel}>'
        if actuator_control_msg.data != self.last_act_serial_msg.data:
            self.last_act_serial_msg = actuator_control_msg
            self.serial_write_publisher.publish(actuator_control_msg)
    
    def motor_control_subscriber(self, msg):
        motor_control_msg = String()
        motor_control_msg.data = f'<M,{msg.left},{msg.right}>'
        if motor_control_msg.data != self.last_motor_serial_msg.data:
            self.last_motor_serial_msg = motor_control_msg
            self.serial_write_publisher.publish(motor_control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialConvertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()