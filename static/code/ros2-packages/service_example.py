#!/usr/bin/env python3
"""
Service Example for ROS 2

This example demonstrates:
- Creating a service server that responds to requests
- Creating a service client that sends requests
- Request-response communication pattern
- Using standard ROS 2 service types

The AddTwoInts service:
- Request: Two integers (a and b)
- Response: Sum of the integers

Usage:
    # Terminal 1: Start service server
    ros2 run <your_package> service_example --ros-args -p mode:=server

    # Terminal 2: Start service client
    ros2 run <your_package> service_example --ros-args -p mode:=client

    # Or call service from command line:
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys


class AddTwoIntsServer(Node):
    """
    Service server that adds two integers.

    Creates a service '/add_two_ints' that accepts two integers
    and returns their sum.
    """

    def __init__(self):
        """
        Initialize the AddTwoIntsServer node.

        Sets up:
        - Node name: 'add_two_ints_server'
        - Service server for adding two integers
        """
        super().__init__('add_two_ints_server')

        # Create service
        # Parameters:
        #   - AddTwoInts: service type
        #   - 'add_two_ints': service name
        #   - self.add_two_ints_callback: function to handle requests
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('AddTwoInts service server started')
        self.get_logger().info('Service name: /add_two_ints')
        self.get_logger().info('Waiting for requests...')

    def add_two_ints_callback(self, request, response):
        """
        Service callback - called when a request is received.

        Args:
            request (AddTwoInts.Request): Contains 'a' and 'b' integers
            response (AddTwoInts.Response): Will contain 'sum' result

        Returns:
            AddTwoInts.Response: Response with sum computed
        """
        # Extract request data
        a = request.a
        b = request.b

        # Compute result
        response.sum = a + b

        # Log the operation
        self.get_logger().info(f'Request: {a} + {b} = {response.sum}')

        return response


class AddTwoIntsClient(Node):
    """
    Service client that requests addition of two integers.

    Sends requests to '/add_two_ints' service and displays results.
    """

    def __init__(self):
        """
        Initialize the AddTwoIntsClient node.

        Sets up:
        - Node name: 'add_two_ints_client'
        - Service client for calling add_two_ints service
        """
        super().__init__('add_two_ints_client')

        # Create service client
        # Parameters:
        #   - AddTwoInts: service type
        #   - 'add_two_ints': service name (must match server)
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        self.get_logger().info('Waiting for service /add_two_ints...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service found! Client ready.')

    def send_request(self, a, b):
        """
        Send a request to the service.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            Future: Future object representing the pending response
        """
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Send request asynchronously
        self.get_logger().info(f'Sending request: {a} + {b}')
        future = self.cli.call_async(request)

        return future


def main_server(args=None):
    """
    Main function for service server.

    Starts the service server and keeps it running.
    """
    rclpy.init(args=args)

    server = AddTwoIntsServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Server shutting down...')
    finally:
        server.destroy_node()
        rclpy.shutdown()


def main_client(args=None):
    """
    Main function for service client.

    Sends multiple requests to demonstrate service usage.
    """
    rclpy.init(args=args)

    client = AddTwoIntsClient()

    # Send multiple requests
    test_cases = [
        (5, 3),
        (10, 20),
        (-5, 15),
        (100, 200),
    ]

    for a, b in test_cases:
        future = client.send_request(a, b)

        # Wait for response
        rclpy.spin_until_future_complete(client, future)

        if future.result() is not None:
            result = future.result().sum
            client.get_logger().info(f'Result: {a} + {b} = {result}')
        else:
            client.get_logger().error('Service call failed')

    client.get_logger().info('All requests completed')
    client.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """
    Main entry point - determines whether to run server or client.

    Reads 'mode' parameter to decide:
    - mode='server': Run service server
    - mode='client': Run service client
    """
    # Parse arguments to determine mode
    if args is None:
        args = sys.argv

    # Simple mode detection from command line
    mode = 'server'  # Default to server

    # Check for mode parameter
    for arg in args:
        if 'mode:=client' in arg or '-p mode:=client' in arg:
            mode = 'client'
            break
        elif 'mode:=server' in arg or '-p mode:=server' in arg:
            mode = 'server'
            break

    if mode == 'server':
        print('Starting service server...')
        main_server(args)
    else:
        print('Starting service client...')
        main_client(args)


if __name__ == '__main__':
    main()
