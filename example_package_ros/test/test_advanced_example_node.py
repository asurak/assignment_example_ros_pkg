import rclpy
import unittest

import launch
import launch_ros.actions
import launch_testing.markers
import launch_testing.actions
import pytest

from example_package_ros.advanced_example_node import AdvancedExampleNode
from example_package_msgs.srv import Example as ExampleSrv

class TestAdvancedExampleNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = AdvancedExampleNode()
        self.node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_service(self):
        response_message = self.node.perform_service_query('Test Service')
        self.assertEqual(response_message, 'Test Service')

    def test_action(self):
        action_result = self.node.perform_action_query('Test Action')
        self.node.get_logger().info(action_result)
        self.assertEqual(action_result, 'OK')

    def test_action_fail(self):
        action_result = self.node.perform_action_query('fail')
        self.assertEqual(action_result, 'Aborted')

    # TODO: test feedback of action

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='example_package_ros',
            executable='example_node',
            name='example_node',
            output='screen'

        ),
        launch_testing.actions.ReadyToTest()
    ])
