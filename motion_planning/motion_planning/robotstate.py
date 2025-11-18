"""
Provides synchronized access to robot states.

Access inlcudes:

Robot joint states, TF transforms, MoveIt FK/IK services.

Handles:
  - Joint state updates from `/joint_states`
  - Base => gripper TF lookup and caching
  - FK/IK requests via MoveIt
  - Frame transforms for Cartesian positions

Serves as the core state interface for planners and controllers requiring
consistent kinematic and frame information for the robot.
"""

from geometry_msgs.msg import Pose, TransformStamped

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import JointState

from tf2_geometry_msgs import do_transform_pose

import tf2_ros
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)


class RobotState:
    """
    Interface for robot forward and inverse kinematics computation.

    Maintains current joint state and provides async methods to compute FK/IK
    via MoveIt services.
    """

    def __init__(self, node: Node):
        """
        Initialize service clients and joint state subscription.

        Args:
        ----
        node: ROS2 node for creating subscriptions and service clients.

        """
        self.node = node

        self.cbgroup = MutuallyExclusiveCallbackGroup()

        # joint state
        self.joint_state = JointState()

        self.baseToGripper = TransformStamped()

        self.currentEE_Pose = np.zeros(3)

        # subscriber
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # clients
        self.fk_client = self.node.create_client(
            GetPositionFK, '/compute_fk', callback_group=self.cbgroup
        )
        self.ik_client = self.node.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cbgroup
        )

        # tf2
        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # self.timer = self.node.create_timer(0.05, self.lookup_transform)

    def joint_state_callback(self, msg):
        """
        Update internal joint state from /joint_states topic.

        Args:
        ----
        msg: Current joint state message.

        """
        self.joint_state = msg
        'string'

    def lookup_transform(self):
        """
        Periodically update transform from the robot base frame to the gripper.

        Also compute the current end-effector pose.

        This method:
        - Queries TF2 for the transform base => fer_hand_tcp.
        - Applies the TF transform to compute the pose in the gripper frame.
        - Updates `self.currentEE_Pose` with the transformed XYZ coordinates.

        If the transform is unavailable, log a warning and no update occurs.
        """
        try:
            self.baseToGripper = self.tf_buffer.lookup_transform(
                'base', 'fer_hand_tcp', rclpy.time.Time()
            )

            currentRobPose = Pose()

            currentRobPose.position.x = self.joint_state.position[0]
            currentRobPose.position.y = self.joint_state.position[1]
            currentRobPose.position.z = self.joint_state.position[2]

            transformedpose = do_transform_pose(
                currentRobPose, self.baseToGripper
            )

            self.currentEE_Pose = [
                transformedpose.position.x,
                transformedpose.position.y,
                transformedpose.position.z,
            ]

        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ) as e:
            self.node.get_logger().warn(f'TF lookup failed: {e}')

    def transform_BaseToGripperFrame(self, position):
        """
        Transform a 3D pose expressed in the base frame into the gripper frame.

        Use the most recently cached TF transform.
        Note that only translation is considered

        Args:
        ----
        position (array): [x, y, z] coordinates in the base frame.

        Returns
        -------
        geometry_msgs.msg.Pose:
            The input position expressed as a Pose transformed into the
            gripper frame.

        """
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        self.lookup_transform()
        return do_transform_pose(pose, self.baseToGripper)

    async def compute_fk(self, joint_state=None, fk_link_names=None):
        """
        Compute forward kinematics for specified links.

        Args:
        joint_state: Joint configuration. Uses current state if None.
        fk_link_names: Links to compute poses for.

        Returns
        -------
        GetPositionFK.Response containing poses and error code.

        """
        if fk_link_names is None:
            return None
        if fk_link_names is str:
            fk_link_names = [fk_link_names]
        # Wait for service
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('FK service not available, waiting...')

        if joint_state is None:
            joint_state = self.joint_state

        request = GetPositionFK.Request()
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.fk_link_names = fk_link_names
        request.robot_state.joint_state = joint_state

        future = await self.fk_client.call_async(request)
        return future

    async def compute_ik(
        self, joint_state=None, pose_stamped=None, ik_link_name=None
    ):
        """
        Compute inverse kinematics for target pose.

        Args:
        ----
            joint_state: Seed state for IK solver. Uses current state if None.
            pose_stamped: Target pose for specified link.
            ik_link_name: Link to compute IK for.

        Returns
        -------
            GetPositionIK.Response containing joint solution and error code.

        """
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('IK service not available, waiting...')

        if joint_state is None:
            joint_state = self.joint_state

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'fer_arm'
        request.ik_request.group_name = 'fer_arm'
        request.ik_request.robot_state.joint_state = joint_state
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.pose_stamped = pose_stamped

        future = await self.ik_client.call_async(request)

        if future.error_code == MoveItErrorCodes.SUCCESS:
            return future.pose_stamped[0].pose
        else:
            return None
