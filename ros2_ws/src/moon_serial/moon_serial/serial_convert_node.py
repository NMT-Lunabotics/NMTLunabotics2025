import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from moon_messages.msg import Actuators
from moon_messages.msg import Motors
from moon_messages.msg import Led

class SerialConvertNode(Node):
    def __init__(self):
        super().__init__('serial_convert_node')

        # Create publisher for serial_write
        self.serial_write_publisher = self.create_publisher(String, 'serial_write', 10)

        # Create subscribers for actuator_control and motor_control
        self.actuator_control_subscriber = self.create_subscription(Actuators, 'actuator_control', self.actuator_control_subscriber, 10)
        self.motor_control_subscriber = self.create_subscription(Motors, 'motor_control', self.motor_control_subscriber, 10)
        self.servo_control_subscriber = self.create_subscription(Bool, 'servo_control', self.servo_control_subscriber, 10)
        self.led_control_subscriber = self.create_subscription(Led, 'led_control', self.led_control_subscriber, 10)

        self.last_act_serial_msg = String()
        self.last_motor_serial_msg = String()
        self.last_servo_serial_msg = String()

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
    
    def servo_control_subscriber(self, msg):
        servo_control_msg = String()
        if msg.data:
            servo_control_msg.data = '<S,1>'
        else:
            servo_control_msg.data = '<S,0>'
        if servo_control_msg.data != self.last_servo_serial_msg.data:
            self.last_servo_serial_msg = servo_control_msg
            self.serial_write_publisher.publish(servo_control_msg)
    
    def led_control_subscriber(self, msg):
        led_control_msg = String()
        led_control_msg.data = f'<L,{int(msg.red)},{int(msg.yellow)},{int(msg.green)},{int(msg.blue)}>'
        self.serial_write_publisher.publish(led_control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialConvertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()