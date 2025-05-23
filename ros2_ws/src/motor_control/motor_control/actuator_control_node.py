import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from moon_messages.msg import Actuators
from std_msgs.msg import Bool
import yaml

class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
            ('joy_topic', '/joy'),
            ('actuator_max_vel', 25),
            ('bucket_axis', 3),
            ('arm_axis', 4),
            ('servo_btn', 0),
            ]
        )
        
        self.joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.actuator_max_vel = self.get_parameter('actuator_max_vel').get_parameter_value().integer_value
        self.bucket_axis = self.get_parameter('bucket_axis').get_parameter_value().integer_value
        self.arm_axis = self.get_parameter('arm_axis').get_parameter_value().integer_value
        self.servo_btn = self.get_parameter('servo_btn').get_parameter_value().integer_value

        self.actuator_min_vel = 5.0
        
        # Subscribe to the joy topic
        self.subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self.joy_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher for the actuator control messages
        self.act_pub = self.create_publisher(Actuators, '/actuator_control', 10)
        self.servo_pub = self.create_publisher(Bool, '/servo_control', 10)

    def joy_callback(self, msg):
        bucket_value = msg.axes[self.bucket_axis] * self.actuator_max_vel
        arm_value = msg.axes[self.arm_axis] * self.actuator_max_vel

        servo_msg = Bool()
        if msg.buttons[self.get_parameter('servo_btn').get_parameter_value().integer_value] == 1:
            servo_msg.data = True
        else:
            servo_msg.data = False
        self.servo_pub.publish(servo_msg)

        if abs(bucket_value) < self.actuator_min_vel:
            bucket_value = 0.0
        if abs(arm_value) < self.actuator_min_vel:
            arm_value = 0.0
        
        actuators_msg = Actuators()
        
        ## TODO: Once feedback is enabled, change the arm and bucket position messages
        actuators_msg.arm_pos = -1
        actuators_msg.bucket_pos = -1
        ##
        
        actuators_msg.bucket_vel = int(bucket_value)
        actuators_msg.arm_vel = int(arm_value)
        
        self.act_pub.publish(actuators_msg)
        

        

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
