import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from example_package.example import Example  # Import from example_package
from example_package_msgs.srv import Example as ExampleSrv
from example_package_msgs.action import Example as ExampleAction
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup


class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')

        # Declare and get parameters
        self.declare_parameter('message', 'Hello, Sunrise')
        message = self.get_parameter('message').get_parameter_value().string_value

        # Create instance of Example class from example_package
        self.example = Example(message)

        # Publisher
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)

        # Timer to publish and print the message
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Service
        self.srv = self.create_service(ExampleSrv, 'example_service', self.service_callback)

        # Action Server
        self.action_server = ActionServer(
            self,
            ExampleAction,
            'example_action',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

    def timer_callback(self):
        msg = String()
        msg.data = self.example.get_example_message()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def service_callback(self, request, response):
        response.response_message = request.request_message
        self.get_logger().info(f'Service echo: "{response.response_message}"')
        return response

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        if 'fail' in goal_handle.request.goal_message:
            self.get_logger().info('Goal contains "fail"; aborting.')
            goal_handle.abort()
            result = ExampleAction.Result()
            result.result_message = "Aborted"
            return result

        for i in range(10):
            feedback_msg = ExampleAction.Feedback()
            feedback_msg.feedback_message = f'Hello world {i+1}'
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: "{feedback_msg.feedback_message}"')
            time.sleep(1)

        goal_handle.succeed()
        result = ExampleAction.Result()
        result.result_message = "OK"
        self.get_logger().info('Goal succeeded.')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node cleanly...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
