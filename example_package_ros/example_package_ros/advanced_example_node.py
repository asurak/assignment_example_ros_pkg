'''
Simple implementation of a node that uses the action and service from ExampleNode  # noqa: E501
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_package_msgs.srv import Example as ExampleSrv
from example_package_msgs.action import Example as ExampleAction
from rclpy.action import ActionClient


class AdvancedExampleNode(Node):
    def __init__(self):
        super().__init__('advanced_example_node')

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'example_topic',
            self.listener_callback,
            10)

        # Service Client
        self.get_logger().info('Initializing service and waiting for it..."')
        self.cli = self.create_client(ExampleSrv, 'example_service')

        # Action Client
        self.get_logger().info('Initializing action and waiting for it..."')
        self.action_client = ActionClient(self, ExampleAction, 'example_action')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

    def perform_service_query(self, request_message):
        req = ExampleSrv.Request()
        req.request_message = request_message
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'Service response: "{future.result().response_message}"')
            return future.result().response_message
        else:
            self.get_logger().error('Service call failed')
            return None

    def perform_action_query(self, goal_message):
        goal_msg = ExampleAction.Goal()
        goal_msg.goal_message = goal_message
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return None

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        self.get_logger().info(f'Action result: {result.result_message}')
        return result.result_message

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: "{feedback_msg.feedback.feedback_message}"')


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedExampleNode()
    try:
        node.perform_service_query('Hello from AdvancedExampleNode')
        node.perform_action_query('Start action')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node cleanly...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
