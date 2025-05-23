#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moon_services.srv import DigDump
from moon_messages.msg import Motors, Actuators
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
        msg.left = 0
        msg.right = 0
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
            return False
            
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
                return False
            time.sleep(0.1)
        self.get_logger().info('Wait complete')
        return True

    def run_motors(self, speed, duration):
        """Run motors at given speed for duration (seconds)"""
        if self.interrupted:
            return False
        
        # Convert float to int for the Motors message
        speed_int = int(speed)
        
        msg = Motors()
        msg.left = speed_int
        msg.right = speed_int
        self.motor_pub.publish(msg)
        
        self.get_logger().info(f'Running motors at speed {speed_int} for {duration} seconds')
        
        # Wait for specified duration while checking for interrupts
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.interrupted:
                return False
            time.sleep(0.1)
        
        return True
        
    def execute_action(self, action_type, *args):
        """Execute a specific action and return success status"""
        if action_type == "motors":
            speed, duration = args
            success = self.run_motors(speed, duration)
            # Stop motors after each movement action
            self.stop_motors()
            return success
        elif action_type == "actuators":
            arm_pos, bucket_pos, wait_time = args
            sucess = self.set_actuators(arm_pos, bucket_pos, wait_time)
            # Stop actuators after each setting action
            # self.stop_actuators()
            return success
        else:
            self.get_logger().error(f"Unknown action type: {action_type}")
            return False
        
    def dig_dump_callback(self, request, response):
        """Execute the dig-dump cycle using a list of actions"""
        self.get_logger().info('Starting dig-dump cycle')
        
        # Define the sequence of actions
        # Each action is a tuple: (action_type, *parameters)
        actions = [
            ("motors", -5, 3.5),                    # Reverse for 3.5s
            ("actuators", 10, 50, 3.0),             # Set arm=10, bucket=50, wait 3s
            ("motors", 5, 1.0),                     # Forward for 1s
            ("actuators", 150, 20, 3.0),            # Set arm=150, bucket=20, wait 3s
            ("motors", 5, 3.33),                    # Forward for 3.33s
            ("actuators", 100, 110, 3.0),           # Set arm=100, bucket=110, wait 3s
        ]
        
        try:
            # Execute each action in sequence
            for action in actions:
                action_type = action[0]
                action_params = action[1:]
                
                self.get_logger().info(f"Executing action: {action_type} with params: {action_params}")
                success = self.execute_action(action_type, *action_params)
                
                if not success or self.interrupted:
                    self.get_logger().warn("Action interrupted or failed")
                    self.stop_motors()
                    break
            
            # Ensure motors are stopped at the end
            self.stop_motors()
            
            if not self.interrupted:
                self.get_logger().info('Dig-dump cycle completed successfully')
            
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
