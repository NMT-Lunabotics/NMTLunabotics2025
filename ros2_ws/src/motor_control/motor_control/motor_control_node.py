import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor_control.msg import Motors


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Declare parameters
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('wheel_base', 0.5)  # Default in meters
        self.declare_parameter('wheel_diameter', 0.1)  # Default in meters
        # Maximum RPM value for motor control
        self.declare_parameter('max_rpm', 100)
        # Minimum RPM value for motor control
        self.declare_parameter('min_rpm', 10)
        self.declare_parameter('arduino_serial_device', '/dev/ttyACM0')

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value
        self.wheel_base = self.get_parameter(
            'wheel_base').get_parameter_value().double_value
        self.wheel_diameter = self.get_parameter(
            'wheel_diameter').get_parameter_value().double_value
        self.max_rpm = self.get_parameter(
            'max_rpm').get_parameter_value().integer_value
        self.min_rpm = self.get_parameter(
            'min_rpm').get_parameter_value().integer_value
        self.arduino_serial_device = self.get_parameter(
            'arduino_serial_device').get_parameter_value().string_value

        self.wheel_radius = self.wheel_diameter / 2.0

        # Subscriber
        self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        
        # Publisher
        self.motor_control_publisher = self.create_publisher(Motors, '/motor_control', 10)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x  # Linear velocity in m/s
        omega = msg.angular.z  # Angular velocity in rad/s

        # Calculate left and right wheel velocities (m/s)
        v_left = 0.9*(v - omega) #(omega * self.wheel_base / 2.0)
        v_right = v + omega # (omega * self.wheel_base / 2.0)

        # Convert to RPM
        rpm_left = (v_left / self.wheel_radius) * 60 / (2 * 3.14159)  # m/s to RPM
        rpm_right = (v_right / self.wheel_radius) * 60 / (2 * 3.14159)

        # Apply limits to RPM
        if abs(rpm_left) < self.min_rpm:
            rpm_left = 0
        if abs(rpm_right) < self.min_rpm:
            rpm_right = 0

        # Cap RPM values
        if rpm_left > self.max_rpm:
            rpm_left = self.max_rpm
        elif rpm_left < -self.max_rpm:
            rpm_left = -self.max_rpm

        if rpm_right > self.max_rpm:
            rpm_right = self.max_rpm
        elif rpm_right < -self.max_rpm:
            rpm_right = -self.max_rpm

        # Publish to motors topic
        msg = Motors()
        msg.left = rpm_left
        msg.right = rpm_right
        self.motor_control_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
