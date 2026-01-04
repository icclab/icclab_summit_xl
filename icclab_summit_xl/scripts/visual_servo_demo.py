#!/usr/bin/env python3
"""
Simple demo script for visual servoing teach-and-grasp
Provides an easy interface for teaching and executing grasps
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import time


class VisualServoDemo(Node):
    """Demo node for visual servoing"""

    def __init__(self):
        super().__init__('visual_servo_demo')

        self.get_logger().info('Visual Servo Demo Node initialized')
        self.get_logger().info('Waiting for visual servo services...')

        # Wait for services
        self.teach_client = self.create_client(Trigger, '/visual_servo/teach_grasp')
        self.start_client = self.create_client(Trigger, '/visual_servo/start')
        self.stop_client = self.create_client(Trigger, '/visual_servo/stop')

        # Wait for services to be available
        while not self.teach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teach service...')

        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for start service...')

        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stop service...')

        self.get_logger().info('All services available!')

    def teach_grasp(self):
        """Call teach grasp service"""
        self.get_logger().info('Teaching grasp pose...')
        self.get_logger().info('Please select ROI around the object in the popup window')

        request = Trigger.Request()
        future = self.teach_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
                return True
            else:
                self.get_logger().error(f'Failed: {response.message}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False

    def start_servoing(self):
        """Call start servoing service"""
        self.get_logger().info('Starting visual servoing...')

        request = Trigger.Request()
        future = self.start_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
                return True
            else:
                self.get_logger().error(f'Failed: {response.message}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False

    def stop_servoing(self):
        """Call stop servoing service"""
        self.get_logger().info('Stopping visual servoing...')

        request = Trigger.Request()
        future = self.stop_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
                return True
            else:
                self.get_logger().error(f'Failed: {response.message}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False


def print_menu():
    """Print interactive menu"""
    print("\n" + "="*50)
    print("Visual Servoing Demo - Interactive Menu")
    print("="*50)
    print("1. Teach a new grasp")
    print("2. Start servoing to taught grasp")
    print("3. Stop servoing")
    print("4. Full demo (teach + servo)")
    print("5. Exit")
    print("="*50)
    print("Enter your choice: ", end='')


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoDemo()

    if len(sys.argv) > 1:
        # Command-line mode
        command = sys.argv[1].lower()

        if command == 'teach':
            node.teach_grasp()
        elif command == 'start':
            node.start_servoing()
        elif command == 'stop':
            node.stop_servoing()
        elif command == 'demo':
            # Full demo sequence
            print("\n=== Visual Servoing Full Demo ===\n")
            print("Step 1: Teaching grasp pose")
            if node.teach_grasp():
                print("\nStep 2: Starting servoing (waiting 2 seconds...)")
                time.sleep(2)
                node.start_servoing()
                print("\nServoing active! Press Ctrl+C to stop")
                try:
                    rclpy.spin(node)
                except KeyboardInterrupt:
                    print("\nStopping servoing...")
                    node.stop_servoing()
        else:
            print(f"Unknown command: {command}")
            print("Usage: visual_servo_demo.py [teach|start|stop|demo]")

    else:
        # Interactive mode
        try:
            while True:
                print_menu()
                choice = input().strip()

                if choice == '1':
                    node.teach_grasp()

                elif choice == '2':
                    node.start_servoing()
                    print("\nServoing active! Select option 3 to stop or Ctrl+C")

                elif choice == '3':
                    node.stop_servoing()

                elif choice == '4':
                    print("\n=== Starting Full Demo ===")
                    print("\nStep 1: Teaching grasp pose")
                    if node.teach_grasp():
                        print("\nStep 2: Starting servoing (waiting 2 seconds...)")
                        time.sleep(2)
                        if node.start_servoing():
                            print("\nServoing active! It will continue until you stop it (option 3)")

                elif choice == '5':
                    print("\nExiting...")
                    break

                else:
                    print(f"\nInvalid choice: {choice}")

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            node.stop_servoing()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
