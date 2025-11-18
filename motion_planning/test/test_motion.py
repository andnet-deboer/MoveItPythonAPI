"""Runs all tests for motion_planning."""

import os
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros import substitutions

from launch_testing.actions import ReadyToTest

from motion_planning.motionplanninginterface import MotionPlanningInterface

import pytest

import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Generate tests."""
    franka_pkgPath = substitutions.FindPackageShare(
        package='franka_fer_moveit_config'
    ).find('franka_fer_moveit_config')
    franka_launchPath = os.path.join(franka_pkgPath, 'launch/demo.launch.py')
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(franka_launchPath),
        launch_arguments={'use_rviz': 'false'}.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_rviz',
                default_value='false',
                description='launch Rviz with the file',
            ),
            included_launch,
            ReadyToTest(),
        ]
    )


class TestMotionPlanner(unittest.TestCase):
    """MotionPlanningInterface test manager."""

    @classmethod
    def setUpClass(cls):
        """Run one time, when the testcase is loaded."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Run one time, when testcase is unloaded."""
        rclpy.shutdown()

    def setUp(self):
        """Run before every test."""
        self.node = rclpy.create_node('test_node')

        self.motionplanner = MotionPlanningInterface(
            node=self.node,
            accel_factor=0.1,
            vel_factor=0.1,
            max_cart_speed=0.1,
            eename='fer_hand_tcp',
            basename='base',
            groupname='fer_arm',
        )

        # Spin to allow joint states and planning scene to initialize
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def tearDown(self):
        """Run after every test."""
        self.node.destroy_node()

    def test_move_pose(self, launch_service, proc_output):
        """Test pose movement - successful case."""
        targetpoint = [0.4, -0.2, 0.2]
        # targetOrient = [1.0, 0.0, 0.0, 0.0]

        result = self.specialawait(
            self.motionplanner.Planner.planPathToPose(targetpoint)
        )

        assert result is not None

    def test_failed_move(self, launch_service, proc_output):
        """Test pose movement - failure case."""
        targetpoint = [0.5, 0.2, -0.5]
        targetOrient = [0.0, 0.0, 0.0, 1.0]

        self.motionplanner.scene.AddBox(*targetpoint, 1.0, 1.0, 1.0, 3)

        result = self.specialawait(
            self.motionplanner.Planner.planPathToPose(
                targetpoint, targetOrient
            )
        )

        assert result is None

    # &&&&&&&&&& Begin_Citation [1] &&&&&&&&&&
    def specialawait(self, coro):
        """Allow the calling of async functions within the test."""
        while not self.motionplanner.Planner.poseplan_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.node.get_logger().debug(
                'Pose plan service not available, waiting...'
            )
            rclpy.spin_once(self.node, timeout_sec=0.1)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(self.node)
        future = ex.create_task(coro)

        rclpy.spin_until_future_complete(self.node, future, executor=ex)
        return future.result()

    # &&&&&&&&&& End_Citation [1] &&&&&&&&&&
