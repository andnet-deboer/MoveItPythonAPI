"""Deals with functions for planning robot trajectory."""

from geometry_msgs.msg import Pose, PoseStamped, Point
from geometry_msgs.msg import Quaternion, Vector3
import math
import numpy as np
import transforms3d as t3d

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
        self.gripper_move_client = ActionClient(node, Move, '/fer_gripper/move')
        self.gripper_client2 = ActionClient(
            node, GripperCommand, '/fer_gripper/gripper_action'
        )

        self.accel_factor = accel_factor
        self.vel_factor = vel_factor
        self.max_cart_speed = max_cart_speed

        self.orientTolerance = 0.1

        self.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        self.min_corner = Vector3(x=-1.0, y=-1.0, z=0.0)

        self.down = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

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


    async def operate_gripper_3(self, width: float, speed: float):
        goal = Move.Goal()
        width = float(width)
        speed = float(speed)

        MostClosed = 0.01
        MostOpen = 0.035

        amount = width * (MostOpen - MostClosed) + MostClosed

        # Set the command message inside the goal
        goal.width = amount
        goal.speed = speed

        self.node.get_logger().info(f'Goal: {goal}')

        future = self.gripper_move_client.send_goal_async(
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

    # async def planPathToPoseMsg(
    #     self,
    #     pose,
    #     start: JointState = None,
    #     execImmediately: bool = False,
    #     save: bool = False,
    # ):
    #     """Wrap PlanPathToPose to take a Pose."""
    #     poseUnstamped: Pose = Pose()
    #     if pose is PoseStamped:
    #         poseUnstamped = pose.pose
    #     elif pose is Pose:
    #         poseUnstamped = pose

    #     loc = poseUnstamped.position
    #     orient = poseUnstamped.orientation

    #     return await self.planPathToPose(
    #         loc, orient, start, execImmediately, save
    #     )

    async def planPathToPoseMsg(
        self,
        pose,
        start: JointState = None,
        execImmediately: bool = True,
        save: bool = False,
    ):
        """Wrap PlanPathToPose to take a Pose or PoseStamped."""
        poseUnstamped: Pose = Pose()

        # Use isinstance, not identity comparison
        if isinstance(pose, PoseStamped):
            poseUnstamped = pose.pose
            # OPTIONAL: if pose.header.frame_id != self.basename:
            #    transform pose to self.basename here (use TF) before using it.
        elif isinstance(pose, Pose):
            poseUnstamped = pose
        else:
            # nothing useful passed
            return None

        loc = poseUnstamped.position
        orient = poseUnstamped.orientation

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
        orientation= False
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
            if not orientation:
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
        
    async def planCircularScanPath(
        self,
        center: Point,
        radius: float,
        height: float,
        num_waypoints: int = 12,
        start_angle: float = 0.0,
        end_angle: float = 360.0,
        execImmediately: bool = False,
        save: bool = False,
    ):
        """
        Plan a circular scanning path
        
        Args:
        ----
        center: Point at the center of the circular path (table center)
        radius: Radius of the circular path
        height: Height above the table to maintain
        num_waypoints: Number of waypoints for the circular path
        start_angle: Starting angle in degrees (0-360)
        end_angle: Ending angle in degrees
        execImmediately: Execute immediately if True
        save: Save the trajectory if True
        
        Returns
        -------
        RobotTrajectory if successful, None otherwise
        """
        
        waypoints = []
        angles = np.linspace(
            np.radians(start_angle), 
            np.radians(end_angle), 
            num_waypoints
        )
        
        for angle in angles:
            # Calculate TCP position in circular path
            x = center.x + radius * np.cos(angle)
            y = center.y + radius * np.sin(angle)
            z = center.z + height
            
            tcp_pos = Point(x=x, y=y, z=z)
            
            # Calculate orientation to maintain camera angle
            orientation = self._calculateCameraOrientation(
                tcp_pos, 
                center
            )
            
            pose = Pose()
            pose.position = tcp_pos
            pose.orientation = orientation
            waypoints.append(pose)
            self.node.get_logger().info(f"WAYPOINTS: {pose}\n")
        
        # Use Cartesian planning for smooth path following
        return await self.planCartesianPath(
            waypoints,
            max_step=0.01,
            avoidcollision=True,
            execImmediately=execImmediately,
            save=save,
            orientation=True
        )
    
    # def _calculateCameraOrientation(self,
    #     tcp_pos: Point,
    #     target_pos: Point,
    # ) -> Quaternion:
    #     """Calculate orientation pointing camera toward center."""
        
        
    #     # Direction vector
    #     dx = target_pos.x - tcp_pos.x
    #     dy = target_pos.y - tcp_pos.y
    #     dz = target_pos.z - tcp_pos.z
        
    #     # Yaw: horizontal direction
    #     yaw = np.arctan2(dy, dx)
        
    #     # Pitch: vertical tilt (negative for looking down)
    #     horizontal_distance = np.sqrt(dx**2 + dy**2)
    #     pitch = np.arctan2(-dz, horizontal_distance)
        
    #     # Roll: camera rotation (0 or π based on mounting)
    #     roll = -np.pi  # or np.pi

    #     quat= t3d.euler.euler2quat(roll, pitch, yaw, 'rxyz')

    #     quat= np.array([quat[1], quat[2], quat[3], quat[0]])
        
    #     return Quaternion(
    #         w=float(quat[0]),
    #         x=float(quat[1]),
    #         y=float(quat[2]),
    #         z=float(quat[3])
    #     )

    def _calculateCameraOrientation(
        self,
        tcp_pos: Point,
        target_center: Point,
        camera_offset_from_tcp: dict = None
    ) -> Quaternion:
        """
        Calculate orientation for camera to look at target center.
        
        Args:
            tcp_pos: Current TCP position
            target_center: Target point to look at
            camera_offset_from_tcp: Optional dict with camera offset from TCP
                                {'x': 0, 'y': 0, 'z': 0.1} for camera 10cm above TCP
        
        Returns:
            Quaternion orientation for the end effector
        """
        
        # If camera has offset from TCP, adjust the calculation point
        if camera_offset_from_tcp:
            camera_x = tcp_pos.x + camera_offset_from_tcp.get('x', 0)
            camera_y = tcp_pos.y + camera_offset_from_tcp.get('y', 0)
            camera_z = tcp_pos.z + camera_offset_from_tcp.get('z', 0)
        else:
            camera_x, camera_y, camera_z = tcp_pos.x, tcp_pos.y, tcp_pos.z
        
        # Calculate direction vector from camera to target
        dx = target_center.x - camera_x
        dy = target_center.y - camera_y
        dz = target_center.z - camera_z
        
        # Normalize the direction vector
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        if distance < 1e-6:  # Avoid division by zero
            # Return default downward orientation
            return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        
        dx /= distance
        dy /= distance
        dz /= distance
        
        # Method 1: Using rotation matrix (more reliable)
        # Create a rotation matrix where Z-axis points toward the target
        
        # Z-axis: pointing from camera to target (optical axis)
        z_axis = np.array([dx, dy, dz])
        
        # Choose an up vector (usually world Z-up)
        world_up = np.array([0, 0, 1])
        
        # X-axis: perpendicular to both Z and world_up
        x_axis = np.cross(world_up, z_axis)
        x_norm = np.linalg.norm(x_axis)
        
        # Handle special case when looking straight up or down
        if x_norm < 1e-6:
            # Use world X or Y as reference
            world_x = np.array([1, 0, 0])
            x_axis = np.cross(world_x, z_axis)
            x_norm = np.linalg.norm(x_axis)
        
        x_axis /= x_norm
        
        # Y-axis: complete the right-handed coordinate system
        y_axis = np.cross(z_axis, x_axis)
        
        # Create rotation matrix
        rotation_matrix = np.array([
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]]
        ])
        
        # Convert to quaternion
        quat = t3d.quaternions.mat2quat(rotation_matrix)
        
        # transforms3d returns [w, x, y, z], ROS expects x, y, z, w
        return Quaternion(
            x=float(quat[1]),
            y=float(quat[2]),
            z=float(quat[3]),
            w=float(quat[0])
        )

    # def _calculateCameraOrientation(
    #     self,
    #     tcp_pos: Point,
    #     target_center: Point,
    # ) -> Quaternion:
    #     """
    #     Calculate orientation pointing camera toward center.
        
    #     This function calculates the quaternion orientation needed to point
    #     the camera at the target center from the current TCP position.
    #     """
        
    #     # Calculate direction vector from TCP to target
    #     dx = target_center.x - tcp_pos.x
    #     dy = target_center.y - tcp_pos.y
    #     dz = target_center.z - tcp_pos.z
        
    #     yaw = np.arctan2(dy, dx)
        
    #     horizontal_distance = np.sqrt(dx**2 + dy**2)
    #     pitch = np.arctan2(-dz, horizontal_distance)  
        
    #     roll = np.pi 
        
    #     # Convert Euler angles to quaternion
    #     # transforms3d uses 'rxyz' convention (roll-pitch-yaw)
    #     quat = t3d.euler.euler2quat(roll, pitch, yaw, 'rxyz')
        
    #     # t3d.euler.euler2quat returns [w, x, y, z]
    #     # ROS Quaternion expects x, y, z, w
    #     return Quaternion(
    #         x=float(quat[1]),
    #         y=float(quat[2]),
    #         z=float(quat[3]),
    #         w=float(quat[0])
    #     )

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
