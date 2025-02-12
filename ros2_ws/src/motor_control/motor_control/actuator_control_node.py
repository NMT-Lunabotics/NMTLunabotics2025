import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor_control.msg import Actuators
import yaml

class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')
        
        # Load parameters from YAML file
        self.declare_parameters(
            namespace='',
            parameters=[
            ('joy_topic', '/joy'),
            ('actuator_max_vel', 25),
            ('bucket_axis', 3),
            ('arm_axis', 4)
            ]
        )
        
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        
        # Subscribe to the joy topic
        self.subscription = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher for the actuator control messages
        self.publisher_ = self.create_publisher(Actuators, '/actuator_control', 10)

    def joy_callback(self, msg):
        actuator_max_vel = self.get_parameter('actuator_max_vel').get_parameter_value().integer_value
        bucket_axis = self.get_parameter('bucket_axis').get_parameter_value().integer_value
        arm_axis = self.get_parameter('arm_axis').get_parameter_value().integer_value

        bucket_value = msg.axes[bucket_axis] * actuator_max_vel
        arm_value = msg.axes[arm_axis] * actuator_max_vel

        actuators_msg = Actuators()
        actuators_msg.armPos = -1;
        actuators_msg.bucketPos = -1;
        actuators_msg.bucketVel = bucket_value
        actuators_msg.armVel = arm_value

        self.publisher_.publish(actuators_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()