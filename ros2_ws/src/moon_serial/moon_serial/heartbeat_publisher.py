import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        self.publisher_ = self.create_publisher(Bool, '/heartbeat', 10)
        
        # Uncomment this line and adjust the rate as needed
        self.timer = self.create_timer(0.2, self.publish_heartbeat)  # 0.2 seconds = 5 Hz
        
        # For different rates, change the first parameter:
        # 1.0 = 1 Hz
        # 0.1 = 10 Hz
        # 0.05 = 20 Hz
        
        self.heartbeat_state = True

    def publish_heartbeat(self):
        msg = Bool()
        msg.data = self.heartbeat_state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.heartbeat_state = not self.heartbeat_state  # Toggle state for demonstration

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()