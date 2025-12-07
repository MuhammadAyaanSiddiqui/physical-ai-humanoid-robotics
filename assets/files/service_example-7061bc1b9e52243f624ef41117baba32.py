#!/usr/bin/env python3
"""
ROS 2 Service Example
Physical AI & Humanoid Robotics Course - Module 1

This demonstrates a simple add_two_ints service in ROS 2.
The service takes two integers and returns their sum.

Usage:
    # Start service server
    python3 service_example.py server

    # Call service from client
    python3 service_example.py client 5 3

Prerequisites:
    - ROS 2 Humble installed
    - example_interfaces package (includes AddTwoInts service type)
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsService(Node):
    """Service server that adds two integers."""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        # Parameters: service_type, service_name, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

        self.get_logger().info('Add Two Ints Service ready')

    def add_two_ints_callback(self, request, response):
        """
        Service callback that adds two integers.

        Args:
            request (AddTwoInts.Request): Contains a and b integers
            response (AddTwoInts.Response): Will contain sum

        Returns:
            AddTwoInts.Response: Response with sum
        """
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a}, b={request.b} → sum={response.sum}')

        return response


class AddTwoIntsClient(Node):
    """Service client that requests addition of two integers."""

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        # Parameters: service_type, service_name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send request to add two integers.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            Future: Async future object for the response
        """
        self.req.a = a
        self.req.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')
        return self.cli.call_async(self.req)


def main_server():
    """Run the service server."""
    rclpy.init()
    service = AddTwoIntsService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()


def main_client(a, b):
    """Run the service client."""
    rclpy.init()
    client = AddTwoIntsClient()

    # Send request
    future = client.send_request(int(a), int(b))

    # Wait for response
    rclpy.spin_until_future_complete(client, future)

    try:
        response = future.result()
        client.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage:')
        print('  Server: python3 service_example.py server')
        print('  Client: python3 service_example.py client <a> <b>')
        sys.exit(1)

    mode = sys.argv[1]

    if mode == 'server':
        main_server()
    elif mode == 'client':
        if len(sys.argv) != 4:
            print('Client requires two integers: python3 service_example.py client <a> <b>')
            sys.exit(1)
        main_client(sys.argv[2], sys.argv[3])
    else:
        print(f'Unknown mode: {mode}. Use "server" or "client"')
        sys.exit(1)
