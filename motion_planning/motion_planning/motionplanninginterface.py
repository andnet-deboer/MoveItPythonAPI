"""Interface file to bring all functionality together."""

from motion_planning import motionplanner, planningscene, robotstate
from rclpy.node import Node


class MotionPlanningInterface:
    """
    Python class for interacting with MoveIt2.

    Initializes all needed classes and references.
    """

    def __init__(
        self,
        node: Node,
        eename: str,
        groupname: str,
        basename: str,
        accel_factor: float,
        vel_factor: float,
        max_cart_speed: float,
    ):
        """Initialze motion planning interface."""
        self.state = robotstate.RobotState(node)
        self.scene = planningscene.PlanningScene(node)
        self.Planner = motionplanner.MotionPlanner(
            node,
            interface=self,
            eename=eename,
            groupname=groupname,
            basename=basename,
            accel_factor=accel_factor,
            vel_factor=vel_factor,
            max_cart_speed=max_cart_speed,
        )
