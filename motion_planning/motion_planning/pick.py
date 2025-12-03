"""Uses motion planning interface to pick up an object."""

from geometry_msgs.msg import Pose, Point

from motion_planning.motionplanninginterface import MotionPlanningInterface

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from std_srvs.srv import Empty


class Pick(Node):
    """User node."""

    def __init__(self):
        """Set up motion planning interface and create service."""
        super().__init__('main_robot_node')

        self.interface = MotionPlanningInterface(
            node=self,
            eename='fer_hand_tcp',
            groupname='fer_arm',
            basename='base',
            accel_factor=0.1,
            vel_factor=0.1,
            max_cart_speed=0.2,
        )
        self.callback = MutuallyExclusiveCallbackGroup()
        # Create empty service for calling the pick sequence
        self.create_service(
            Empty, '/pick', self.pick_object, callback_group=self.callback
        )
        self.create_service(
            Empty, '/test', self.pick_object_test, callback_group=self.callback
        )

        self.create_service(
            Empty,
            '/pickupspot',
            self.pickupspot,
            callback_group=self.callback,
        )

        self.create_service(
            Empty,
            '/pushtrain',
            self.pushtrain,
            callback_group=self.callback,
        )
        self.create_service(
            Empty, '/squeeze', self.squeeze, callback_group=self.callback
        )
        self.create_service(
            Empty, '/letgo', self.letgo, callback_group=self.callback
        )

        self.create_service(
            Empty,
            '/grab',
            self.grab,
            callback_group=self.callback,
        )

        self.create_service(
            Empty,
            '/scan',
            self.scan,
            callback_group=self.callback,
        )


        self.declare_parameter('scene', '')
        self.scene_file = (
            self.get_parameter('scene').get_parameter_value().string_value
        )

        self.pose1 = self.make_pose(
            0.4, -0.2, 0.2
        )  # Add a pose above the object
        self.pose2 = self.make_pose(0.4, -0.2, 0.12)  # Pose at the object
        self.pose3 = self.make_pose(
            0.4, 0.2, 0.2
        )  # Pose above the place location
        self.pose4 = self.make_pose(
            0.4, 0.2, 0.125
        )  # Pose at the place location
        self.pose5 = self.make_pose(0.5, 0.2, 0.2)
        self.pose6 = self.make_pose(0.5, 0.2, 0.125)
        self.orientation = [1.0, 0.0, 0.0, 0.0]
        # self.scene = self.get_parameter(
        #     'yaml_file').get_parameter_value().string_value
        # Might need to switch qx

    def make_pose(self, x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
        """
        Create a pose message from xyz coordinates.

        Defaults to gripper-downorientation.
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def locfrompose(self, pose: Pose):
        """Generate an array of coordinates from a pose."""
        return [pose.position.x, pose.position.y, pose.position.z]

    async def squeeze(self, req, res):
        await self.interface.Planner.operate_gripper_2(0.034, 0)

        return res

    async def letgo(self, req, res):
        await self.interface.Planner.open_gripper()
        return res

    async def pickupspot(self, req, res):
        """Send the gripper to the spot from which it will pick up the item."""
        loc1 = [
            self.pose1.position.x,
            self.pose1.position.y,
            self.pose1.position.z,
        ]

        await self.interface.Planner.planPathToPoseLists(
            loc=loc1, orient=self.orientation, execImmediately=True
        )

        # Open gripper
        await self.interface.Planner.open_gripper()

        # Move straight down to object
        await self.interface.Planner.planCartesianPath(
            waypoints=[self.pose2], execImmediately=True
        )

        return res

    async def pushtrain(self, req, res):
        """Push the train which has been set down."""
        await self.interface.Planner.planPathToPoseLists(
            loc=self.locfrompose(self.pose5),
            orient=self.orientation,
            execImmediately=True,
        )
        self.interface.scene.Attach('object')
        await self.interface.Planner.close_gripper()

        await self.interface.Planner.planCartesianPath(
            waypoints=[self.pose5, self.pose6, self.pose4, self.pose3],
            execImmediately=True,
        )
        self.interface.scene.Detach('object')

        return res
 
    async def grab(self, request, response):
        # Close gripper to 70% (more open)
        await self.interface.Planner.operate_gripper_3(0.7, 30.0)
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.1)

        # await self.interface.Planner.open_gripper()
        # for _ in range(40):
        #     rclpy.spin_once(self, timeout_sec=0.1)
        
        # Close gripper to 40% (more closed, grasping)
        await self.interface.Planner.operate_gripper_3(0.4, 30.0)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        return response

    async def scan(self, request, response):
        from geometry_msgs.msg import Point
        
        center = Point(x=0.45, y=0.0, z=0.08)
        
        await self.interface.Planner.planCircularScanPath(
            center=center,
            radius=0.07,
            height=0.25,
            num_waypoints=36,
            start_angle=0,
            end_angle=160,
            execImmediately=True,
            save=False,
        )
        
        await self.interface.Planner.planCircularScanPath(
            center=center,
            radius=0.07,
            height=0.25,
            num_waypoints=36,
            start_angle=160,
            end_angle=0,
            execImmediately=True,
            save=False,
        )

        await self.interface.Planner.planCircularScanPath(
            center=center,
            radius=0.12,
            height=0.1,
            num_waypoints=36,
            start_angle=0,
            end_angle=120,
            execImmediately=True,
            save=False,
        )

        await self.interface.Planner.planCircularScanPath(
            center=center,
            radius=0.12,
            height=0.1,
            num_waypoints=36,
            start_angle=0,
            end_angle=-120,
            execImmediately=True,
            save=False,
        )
        return response

    async def pick_object_test(self, request, response):
        """
        Run sequence to do the following.

        1. Move to object location and pose
        2. Open grippers and attach object
        3. Move to a different location
        4. Open grippers and detach object
        """
        # Load scene from yaml file
        # self.interface.scene.LoadScene(scene=self.scene_file)
        # Move robot to above object location
        # self.interface.Planner.planPathToPose(pose1)

        # Move above object
        loc1 = Point()
        loc1.x = 0.6
        loc1.y = -0.36
        loc1.z = 0.05
        await self.interface.Planner.planPathToPose(
            loc=loc1, execImmediately=True
        )
        return response
    async def pick_object(self, request, response):
        """
        Run sequence to do the following.

        1. Move to object location and pose
        2. Open grippers and attach object
        3. Move to a different location
        4. Open grippers and detach object
        """
        # Load scene from yaml file
        # self.interface.scene.LoadScene(scene=self.scene_file)
        # Move robot to above object location
        # self.interface.Planner.planPathToPose(pose1)

        # Move above object
        loc1 = [
            self.pose1.position.x,
            self.pose1.position.y,
            self.pose1.position.z,
        ]
        await self.interface.Planner.planPathToPoseLists(
            loc=loc1, orient=self.orientation, execImmediately=True
        )

        # Open gripper
        await self.interface.Planner.open_gripper()

        # Move straight down to object
        await self.interface.Planner.planCartesianPath(
            waypoints=[self.pose2], execImmediately=True
        )

        # Attach object and close gripper
        self.interface.scene.Attach('object')
        await self.interface.Planner.operate_gripper(0.5)
        # Move robot back to above the object
        await self.interface.Planner.planCartesianPath(
            waypoints=[self.pose1], execImmediately=True
        )

        # Move robot to a different location to place
        # Above the final position
        loc2 = [
            self.pose3.position.x,
            self.pose3.position.y,
            self.pose3.position.z,
        ]
        await self.interface.Planner.planPathToPoseLists(
            loc=loc2, orient=self.orientation, execImmediately=True
        )

        # Move down to place object
        await self.interface.Planner.planCartesianPath(
            waypoints=[self.pose4], execImmediately=True
        )

        # Open the gripper and detach object
        await self.interface.Planner.open_gripper()
        self.interface.scene.Detach('object')
        # response.success = True
        # response.message = 'Sequence complete'
        await self.interface.Planner.planCartesianPath(
            waypoints=[self.pose3], execImmediately=True
        )

        return response


def main(args=None):
    """User Node."""
    rclpy.init(args=args)
    node = Pick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
