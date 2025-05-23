#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moon_services.srv import DigDump
from moon_messages.msg import Motors, Actuators  # Updated import for custom messages
import time
import signal
import sys

class DigDumpService(Node):
    def __init__(self):
        super().__init__('dig_dump_service')
        
        # Create the service
        self.srv = self.create_service(DigDump, 'dig_dump_cycle', self.dig_dump_callback)
        
        # Publishers for motor control and arm/bucket positions
        self.motor_pub = self.create_publisher(Motors, 'motor_control', 10)
        self.actuator_pub = self.create_publisher(Actuators, 'actuator_control', 10)
        
        # Flag to handle interruptions
        self.interrupted = False
        
        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('Dig-Dump Service started')

    def signal_handler(self, sig, frame):
        self.get_logger().info('Ctrl+C detected, stopping operations')
        self.interrupted = True
        self.stop_motors()
        self.stop_actuators()
        sys.exit(0)
        
    def stop_motors(self):
        """Stop the motors"""
        msg = Motors()
        msg.left = 0  # Changed from 0.0 to 0
        msg.right = 0  # Changed from 0.0 to 0
        self.motor_pub.publish(msg)
        self.get_logger().info('Motors stopped')

    def stop_actuators(self):
        """Stop the actuators"""
        msg = Actuators()
        msg.arm_pos = -1
        msg.bucket_pos = -1
        msg.arm_vel = 0
        msg.bucket_vel = 0
        self.actuator_pub.publish(msg)
        self.get_logger().info('Actuators stopped')
        
    def set_actuators(self, arm_position, bucket_position, wait_time=3.0):
        """Set arm and bucket positions and wait for them to reach the position"""
        if self.interrupted:
            return
        msg = Actuators()
        msg.arm_pos = arm_position
        msg.bucket_pos = bucket_position
        msg.arm_vel = 0
        msg.bucket_vel = 0
        self.actuator_pub.publish(msg)
        self.get_logger().info(f'Arm position set to {arm_position}, Bucket position set to {bucket_position}')
        
        # Wait for actuators to reach position
        self.get_logger().info(f'Waiting {wait_time} seconds for actuators to reach position...')
        start_time = time.time()
        while time.time() - start_time < wait_time:
            if self.interrupted:
                return
            time.sleep(0.1)
        self.get_logger().info('Wait complete')

    def run_motors(self, speed, duration):
        """Run motors at given speed for duration (seconds)"""
        if self.interrupted:
            return
        
        # Convert float to int for the Motors message
        speed_int = int(speed)  # Convert to integer
        
        msg = Motors()
        msg.left = speed_int
        msg.right = speed_int
        self.motor_pub.publish(msg)
        
        self.get_logger().info(f'Running motors at speed {speed_int} for {duration} seconds')
        
        # Wait for specified duration while checking for interrupts
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.interrupted:
                return
            time.sleep(0.1)
        
    def dig_dump_callback(self, request, response):
        """Execute the dig-dump cycle"""
        self.get_logger().info('Starting dig-dump cycle')
        
        try:
            # Run motors backwards for 3.5 seconds
            self.run_motors(-5, 3.5)
            if self.interrupted: return response
            
            # Set arm position to 10, bucket position to 50 and wait 3 seconds
            self.set_actuators(10, 50, 3.0)
            if self.interrupted: return response
            
            # Run motors forward for 1 second
            self.run_motors(5, 1)
            if self.interrupted: return response
            
            # Set bucket position to 20, arm position to 150 and wait 3 seconds
            self.set_actuators(150, 20, 3.0)
            if self.interrupted: return response
            
            # Run motors forward for 3.33 seconds
            self.run_motors(5, 3.33)
            if self.interrupted: return response
            
            # Set bucket position to 110, arm position to 100 and wait 3 seconds
            self.set_actuators(100, 110, 3.0)
            if self.interrupted: return response
            
            # Stop motors
            self.stop_motors()
            
            self.get_logger().info('Dig-dump cycle completed')

        except Exception as e:
            self.get_logger().error(f'Error in dig-dump cycle: {str(e)}')
            self.stop_motors()
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    dig_dump_service = DigDumpService()
    
    try:
        rclpy.spin(dig_dump_service)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        dig_dump_service.stop_motors()
        dig_dump_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
