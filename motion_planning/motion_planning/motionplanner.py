"""Deals with functions for planning robot trajectory."""

from geometry_msgs.msg import Pose, PoseStamped, Point
from geometry_msgs.msg import Quaternion, Vector3

from motion_planning.utilities import Utilities

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotTrajectory,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetMotionSequence,
)

from rclpy.action import ActionClient
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Move


from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import JointState

from shape_msgs.msg import SolidPrimitive


class MotionPlanner:
    """Motion planner."""

    def __init__(
        self,
        node: Node,
        interface,
        eename: str,
        groupname: str,
        basename: str,
        accel_factor: float,
        vel_factor: float,
        max_cart_speed: float,
    ):
        """
        Initialize motion planner.

        Args:
        ----
        node (Node) : User node for storing state.
        interface (motionplanninginterface.MotionPlanningInterface) :
            MotionPlanningInterface instance.
        eename : the name of the end effector link
        groupname : the motion planning group being invoked
        basename : the name of the base link of the robot
        accel_factor (float) : acceleration multiplier
        vel_factor (float) : velocity multiplier
        max_cart_speed (float) : Max speed of the end effector

        """
        self.node = node

        self.interface = interface

        self.utils = Utilities(node, self.interface.state)

        self.cb = MutuallyExclusiveCallbackGroup()

        self.groupname = groupname
        self.eename = eename
        self.basename = basename

        # service clients
        self.cartesian_client = self.node.create_client(
            GetCartesianPath, '/compute_cartesian_path', callback_group=self.cb
        )

        self.poseplan_client = self.node.create_client(
            GetMotionPlan, '/plan_kinematic_path', callback_group=self.cb
        )

        self.jointplan_client = self.node.create_client(
            GetMotionSequence, '/plan_sequence_path', callback_group=self.cb
        )

        # action client
        self.execTrajClient = ActionClient(
            self.node,
            ExecuteTrajectory,
            '/execute_trajectory',
            callback_group=self.cb,
        )

        self.gripper_client = ActionClient(node, Grasp, '/fer_gripper/grasp')
        self.gripper_client2 = ActionClient(
            node, GripperCommand, '/fer_gripper/gripper_action'
        )

        self.accel_factor = accel_factor
        self.vel_factor = vel_factor
        self.max_cart_speed = max_cart_speed

        self.orientTolerance = 0.1

        self.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        self.min_corner = Vector3(x=-1.0, y=-1.0, z=0.0)

    def createMotionPlanRequest(self):
        """Create Motion Plan Request."""
        request = MotionPlanRequest()
        request.group_name = self.groupname
        request.workspace_parameters.header.stamp = self.getStamp()
        request.workspace_parameters.max_corner = self.max_corner
        request.workspace_parameters.min_corner = self.min_corner
        request.max_acceleration_scaling_factor = self.accel_factor
        request.max_velocity_scaling_factor = self.vel_factor

        request.workspace_parameters.header.stamp = self.getStamp()
        request.workspace_parameters.header.frame_id = 'base'
        request.workspace_parameters.max_corner = self.max_corner
        request.workspace_parameters.min_corner = self.min_corner

        return request

    async def planMotionSequence(
        self,
        sequence: list[MotionSequenceItem],
        blend_radius: list[int],
        execImmediately=False,
        save=False,
    ):
        """
        Plan a path given a valid starting joint configuration.

        Start from any valid joint configuration to a valid goal configuration.
        If starting configuration is not provided,
        the current robot pose is used.
        """
        sequenceItems = []

        for index, item in enumerate(sequence):
            motionseq_item = MotionSequenceItem()
            motionseq_item.blend_radius = blend_radius[index]
            motionseq_item.req = self.createMotionPlanRequest()
            sequenceItems.append(motionseq_item)

        motionsequence = GetMotionSequence()
        motionsequence.Request().request.items = sequenceItems

        future = await self.poseplan_client.call_async(motionsequence)

        if (
            future.motion_plan_response.error_code.val
            == MoveItErrorCodes.SUCCESS
        ):
            if execImmediately:
                await self.execTrajClient.send_goal_async(
                    future.motion_plan_response.trajectory
                )
            if save:
                self.utils.savePath(future.motion_plan_response.trajectory)
            return future
        else:
            return None

    async def planPathToConfig(
        self,
        end: JointState,
        start: JointState = None,
        execImmediately: bool = False,
        save: bool = False,
    ):
        """Create path to given robot configuration."""
        request = self.createMotionPlanRequest()

        if start is not None:
            request.start_state.joint_state = start

        goalconst = Constraints()

        for i in range(len(end.name)):
            goalconst.joint_constraints.append(
                JointConstraint(
                    joint_name=end.name[i],
                    position=end.position[i],
                    tolerance_above=0.01,
                    tolerance_below=0.01,
                    weight=1,
                )
            )

        request.goal_constraints = [goalconst]

        return await self.dealWithGeneratingPlan(
            request, execImmediately, save
        )

    async def operate_gripper_2(self, position: float, max_effort: float):
        goal = Grasp.Goal()

        position = float(position)
        max_effort = float(max_effort)

        MostClosed = 0.01
        MostOpen = 0.035

        amount = position * (MostOpen - MostClosed) + MostClosed
        goal.width = amount
        goal.force = max_effort  # in Newtons
        goal.speed = 0.04
        goal.epsilon.inner = 0.01
        goal.epsilon.outer = 0.01

        self.node.get_logger().info(f'Goal: {goal}')

        future = self.gripper_client.send_goal_async(
            goal, feedback_callback=self.gripperFeedbackLogger
        )
        future.add_done_callback(self.goal_response_callback)

    async def operate_gripper_3(self, position: float, max_effort: float):
        goal = GripperCommand.Goal()
        position = float(position)
        max_effort = float(max_effort)

        MostClosed = 0.01
        MostOpen = 0.035

        amount = position * (MostOpen - MostClosed) + MostClosed

        # Set the command message inside the goal
        goal.command.position = amount
        goal.command.max_effort = max_effort

        self.node.get_logger().info(f'Goal: {goal}')

        future = self.gripper_client2.send_goal_async(
            goal, feedback_callback=self.gripperFeedbackLogger
        )
        future.add_done_callback(self.goal_response_callback)

    async def operate_gripper_3(self, position: float, max_effort: float):
        goal = GripperCommand.Goal()
        position = float(position)
        max_effort = float(max_effort)

        MostClosed = 0.01
        MostOpen = 0.035

        amount = position * (MostOpen - MostClosed) + MostClosed

        # Set the command message inside the goal
        goal.command.position = amount
        goal.command.max_effort = max_effort

        self.node.get_logger().info(f'Goal: {goal}')

        future = self.gripper_client2.send_goal_async(
            goal, feedback_callback=self.gripperFeedbackLogger
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return

        self.node.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def gripperFeedbackLogger(self, feedback_msg):
        self.node.get_logger().info(f'Grip Feedback: {feedback_msg.feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Grip Result: {result}')

    async def operate_gripper(self, fraction: float):
        """Set gripper to a position based on open. 0=closed, 1=open."""
        request = self.createMotionPlanRequest()
        request.group_name = 'hand'

        MostClosed = 0.01
        MostOpen = 0.03

        ammount = fraction * (MostOpen - MostClosed) + MostClosed

        goalconst = Constraints()
        goalconst.joint_constraints.append(
            JointConstraint(
                joint_name='fer_finger_joint1',
                position=ammount,
                tolerance_above=0.0001,
                tolerance_below=0.0001,
                weight=1.0,
            )
        )
        goalconst.joint_constraints.append(
            JointConstraint(
                joint_name='fer_finger_joint2',
                position=ammount,
                tolerance_above=0.0001,
                tolerance_below=0.0001,
                weight=1.0,
            )
        )

        request.goal_constraints = [goalconst]
        self.node.get_logger().info('Sending to plan executor')

        result = await self.dealWithGeneratingPlan(
            request, execImmediately=True, save=False
        )
        self.node.get_logger().info('Executor completed')
        return result

    async def open_gripper(self):
        """Open the gripper."""
        return await self.operate_gripper(1.0)

    async def close_gripper(self):
        """Close the gripper."""
        return await self.operate_gripper(0.0)

    async def planPathToPose(
        self,
        loc: Point = None,
        orient: Quaternion = None,
        start: JointState = None,
        execImmediately: bool = False,
        save: bool = False,
    ):
        """
        Create a path to a given pose and orientation.

        If starting configuration is not provided, use the current robot pose.
        If goal pose is not specified,
        the plan should achieve the specified orientation (at any position)
        If goal orientation is not specified,
        the plan should achieve the specified position (at any orientation)
        """
        if loc is None and orient is None:
            return None  # Return nothing if no pose is provided

        request = self.createMotionPlanRequest()

        if start is not None:
            request.start_state.joint_state = start

        goalconst = Constraints()
        goalconst.name = 'Goal'

        if orient is not None:
            rotconst = OrientationConstraint()
            rotconst.orientation = orient
            rotconst.absolute_x_axis_tolerance = 0.01
            rotconst.absolute_y_axis_tolerance = 0.01
            rotconst.absolute_z_axis_tolerance = 0.01
            rotconst.weight = 1.0
            rotconst.header.stamp = self.getStamp()
            rotconst.link_name = self.eename

            goalconst.orientation_constraints = [rotconst]

        if loc is not None:
            locconst = PositionConstraint()

            locconst.header.stamp = self.getStamp()
            locconst.header.frame_id = self.basename
            locconst.link_name = self.eename
            locconst.weight = 1.0

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [
                0.001,
                0.001,
                0.001,
            ]  # 1mm tolerance in each direction

            # Define where this
            box_pose = Pose()
            box_pose.position = loc
            box_pose.orientation.w = 1.0

            locconst.constraint_region.primitives = [box]
            locconst.constraint_region.primitive_poses = [box_pose]

            self.node.get_logger().info(
                f' Goal in tcp frame {locconst.target_point_offset}'
            )

            locconst.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
            goalconst.position_constraints = [locconst]

        request.goal_constraints = [goalconst]

        self.node.get_logger().info(
            f'Goal constraint {request.goal_constraints}'
        )

        return await self.dealWithGeneratingPlan(
            request, execImmediately, save
        )

    async def planPathToPoseLists(
        self,
        loc=None,
        orient=None,
        start: JointState = None,
        execImmediately: bool = False,
        save: bool = False,
    ):
        """Wrap PlanPathToPose to take lists."""
        QOrient = Quaternion(
            x=float(orient[0]),
            y=float(orient[1]),
            z=float(orient[2]),
            w=float(orient[3]),
        )

        PLoc = Point(x=float(loc[0]), y=float(loc[1]), z=float(loc[2]))

        return await self.planPathToPose(
            PLoc, QOrient, start, execImmediately, save
        )

    async def planPathToPoseMsg(
        self,
        pose,
        start: JointState = None,
        execImmediately: bool = False,
        save: bool = False,
    ):
        """Wrap PlanPathToPose to take a Pose."""
        if pose is PoseStamped:
            pose = pose.pose

        loc = [pose.position.x, pose.position.y, pose.position.z]
        orient = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

        return await self.planPathToPose(
            loc, orient, start, execImmediately, save
        )

    async def dealWithGeneratingPlan(
        self, request: MotionPlanRequest, execImmediately: bool, save: bool
    ):
        """Boilerplate for motion plans."""
        motionrequest = GetMotionPlan.Request()
        motionrequest.motion_plan_request = request
        future = await self.poseplan_client.call_async(motionrequest)

        motplan = future.motion_plan_response

        if motplan.error_code.val == MoveItErrorCodes.SUCCESS:
            self.node.get_logger().info('Plan Success')
            if execImmediately:
                await self.executePath(motplan.trajectory)
            if save:
                self.utils.savePath(motplan.trajectory)

            return motplan
        else:
            self.node.get_logger().info(f'ERROR: f{motplan.error_code.val}')
            return None

    async def planPath_PoseToConfig(
        self,
        start=None,
        execImmediately=False,
        save=False,
        namedConfig=str,
    ):
        """
        Plan a path from any valid starting pose to a named configuration.

        Args:
        ----
        start: Starting joint configuration (JointState). If None, uses current
        robot pose.
        namedConfig: Name of the target configuration to load from YAML file.
        execImmediately: Execute the trajectory immediately if True.
        save: Save the trajectory if True.

        Returns
        -------
        Future containing the motion plan response, or None on failure.

        """
        if namedConfig is None:
            return None

        # Load the named configuration from YAML file
        goal_config = self.utils.load_joint_configuration(
            config_name=namedConfig
        )

        if goal_config is None:
            self.node.get_logger().error(
                f'Failed to load configuration: {namedConfig}'
            )

            return None

        request = self.createMotionPlanRequest()

        # Set starting state if provided
        if start is not None:
            request.start_state.joint_state = start

        # Create goal constraints from the loaded joint configuration
        goalconst = Constraints()
        goalconst.name = 'Goal'

        for i, joint_name in enumerate(goal_config['names']):
            self.node.get_logger().info(
                f'Adding constraint for {joint_name}: \
                {goal_config["positions"][i]}'
            )
            goalconst.joint_constraints.append(
                JointConstraint(
                    joint_name=joint_name,
                    position=float(
                        goal_config['positions'][i]
                    ),  # Convert to float
                    tolerance_above=0.01,
                    tolerance_below=0.01,
                    weight=1.0,
                )
            )

        request.goal_constraints = [goalconst]

        self.node.get_logger().info('Calling motion planner service...')

        # Call the service
        return await self.dealWithGeneratingPlan(
            request, execImmediately, save
        )

    async def planCartesianPath(
        self,
        waypoints,
        max_step=0.01,
        avoidcollision=True,
        execImmediately=False,
        save=False,
    ):
        """
        Plan a Cartesian path from any valid starting pose to a goal pose.

        If the starting configuration is not provided, current pose is used.
        """
        # Wait for service
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'Cartesian service not available, waiting...'
            )

        transformed_waypoints = []

        for pose in waypoints:
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            # pose_transformed = (
            #     self.interface.state.transform_BaseToGripperFrame(pose)
            # )
            transformed_waypoints.append(pose)

        request = GetCartesianPath.Request()
        request.header.stamp = self.getStamp()
        request.group_name = self.groupname
        request.header.frame_id = self.basename
        request.link_name = self.eename
        request.waypoints = transformed_waypoints
        request.max_step = max_step
        request.avoid_collisions = avoidcollision
        request.max_acceleration_scaling_factor = self.accel_factor
        request.max_velocity_scaling_factor = self.vel_factor
        request.max_cartesian_speed = self.max_cart_speed
        future = await self.cartesian_client.call_async(request)

        if future.error_code.val == MoveItErrorCodes.SUCCESS:
            if execImmediately:
                self.node.get_logger().info('EXECUTING PATH')
                await self.executePath(future.solution)
            if save:
                self.utils.savePath(
                    path=future.solution, filename='cartesianpath.txt'
                )
            return future.solution
        else:
            return None

    async def executePath(self, path: RobotTrajectory):
        """Execute provided path."""
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = path
        self.node.get_logger().info('Exec start')
        handle = await self.execTrajClient.send_goal_async(goal_msg)
        result = await handle.get_result_async()
        self.node.get_logger().info('Exec done')
        return result

    def getStamp(self):
        """Get a valid header stamp."""
        return self.node.get_clock().now().to_msg()
