import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

#!/usr/bin/env python3

class ScanTrimNode(Node):
    def __init__(self):
        super().__init__('scan_trim_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_trimmed',
            10)
        self.declare_parameter('trim_angle_deg', 120.0)
        self.declare_parameter('offset_angle_deg', 0.0)  # New parameter

    def scan_callback(self, msg: LaserScan):
        trim_angle_deg = self.get_parameter('trim_angle_deg').value
        offset_angle_deg = self.get_parameter('offset_angle_deg').value  # Get offset
        trim_angle_rad = trim_angle_deg * 3.141592653589793 / 180.0
        offset_angle_rad = offset_angle_deg * 3.141592653589793 / 180.0

        total_angle = msg.angle_max - msg.angle_min
        if trim_angle_rad > total_angle:
            self.get_logger().warn('Trim angle is larger than scan angle. Publishing original scan.')
            self.publisher.publish(msg)
            return

        # Center the trimmed scan with offset
        center_angle = (msg.angle_max + msg.angle_min) / 2.0 + offset_angle_rad
        new_angle_min = center_angle - trim_angle_rad / 2.0
        new_angle_max = center_angle + trim_angle_rad / 2.0

        # Calculate indices
        start_idx = int((new_angle_min - msg.angle_min) / msg.angle_increment)
        end_idx = int((new_angle_max - msg.angle_min) / msg.angle_increment)

        # Clamp indices
        start_idx = max(0, start_idx)
        end_idx = min(len(msg.ranges), end_idx)

        trimmed_scan = LaserScan()
        trimmed_scan.header = msg.header
        trimmed_scan.angle_min = msg.angle_min + start_idx * msg.angle_increment
        trimmed_scan.angle_max = msg.angle_min + (end_idx - 1) * msg.angle_increment
        trimmed_scan.angle_increment = msg.angle_increment
        trimmed_scan.time_increment = msg.time_increment
        trimmed_scan.scan_time = msg.scan_time
        trimmed_scan.range_min = msg.range_min
        trimmed_scan.range_max = msg.range_max
        trimmed_scan.ranges = msg.ranges[start_idx:end_idx]
        trimmed_scan.intensities = msg.intensities[start_idx:end_idx] if msg.intensities else []

        self.publisher.publish(trimmed_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanTrimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()