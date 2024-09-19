import rclpy
import unittest
from example_package_ros.example_node import ExampleNode
from std_msgs.msg import String


class TestExampleNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = ExampleNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_publisher(self):
        # wait for the message to arrive.
        # Note: while rclpy.wait_for_message can be used, that does not spin.
        # Using async spinners here is maybe a bit of overkill, so adopt the
        # simple approach of spinning once until the message is received.

        msgs_received = []

        def callback(msg):
            msgs_received.append(msg.data)

        subscription = self.node.create_subscription(  # noqa: F841
            String,
            'example_topic',
            callback,
            10)

        # Spin the node to process incoming messages
        timeout = 5.0  # seconds
        end_time = self.node.get_clock().now() + rclpy.time.Duration(seconds=timeout)
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if msgs_received:
                break
            if self.node.get_clock().now() > end_time:
                break

        # Assert that the message was received
        self.assertTrue(len(msgs_received) > 0, 'Did not receive message on example_topic')
        self.assertEqual(msgs_received[0], 'Hello, Sunrise')
