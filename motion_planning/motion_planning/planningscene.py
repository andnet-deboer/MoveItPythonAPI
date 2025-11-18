"""
This class allows the user to alter the planning scene.

The functions provided are as follows:
AddBox: Allows the user to create a box with specified
position and dimensions

RemoveBox: Allows the user to remove a specific box from the scene

Attach: Allows the user to attach an object to the end effector of the bot

Detach: Allows the user to detach an object from the end effector
"""

from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.msg import PlanningScene as ps

from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive

import yaml


class PlanningScene:
    """Creates the class and defines functions."""

    def __init__(self, node: Node):
        """Set up the initial parameters."""
        # Grab the node that has been passed in from usernode
        self.node = node
        # Create scene publisher
        self.node.scene_object_publisher = self.node.create_publisher(
            ps, '/planning_scene', 10
        )
        self.node.declare_parameter('objects', [])

    def AddBox(self, X, Y, Z, L, W, H, boxId):
        """Add a box to the planning scene."""
        box_pose = PoseStamped()
        box_pose.header.frame_id = f'box_{boxId}'
        box_pose.pose.position.x = X
        box_pose.pose.position.y = Y
        box_pose.pose.position.z = Z
        box_pose.pose.orientation.w = 1.0
        self.box_name = f'box_{boxId}'

        # Create a solid primitive object
        box_shape = SolidPrimitive()
        box_shape.type = 1
        box_shape.dimensions = [L, W, H]
        collision_box = CollisionObject()
        collision_box.id = f'box_{boxId}'

        collision_box.pose = box_pose.pose
        collision_box.pose.orientation.w = 1.0
        collision_box.header.frame_id = 'base'

        collision_box.primitives.append(box_shape)
        collision_box.operation = CollisionObject.ADD

        # CITATION [3]] #
        self.scene = ps()
        self.scene.world.collision_objects.append(collision_box)
        self.scene.is_diff = True

        self.node.scene_object_publisher.publish(self.scene)
        self.node.get_logger().info(f'Box {boxId} placed at {X}, {Y}, {Z}')
        # END CITATION #

    def RemoveBox(self, boxId):
        """Remove a specific box from the planning scene."""
        collision_box = CollisionObject()
        collision_box.id = f'box_{boxId}'
        collision_box.operation = CollisionObject.REMOVE

        self.scene = ps()
        self.scene.world.collision_objects = [collision_box]
        self.scene.is_diff = True

        self.node.scene_object_publisher.publish(self.scene)
        self.node.get_logger().info(f'Box {boxId} Removed')

    def Attach(self, boxId):  # attaches to end effector
        """Attach an object to the end effector."""
        self.attached_object = AttachedCollisionObject()
        self.attached_object.link_name = 'fer_hand'  # placeholder name

        self.attached_object.object.id = f'box_{boxId}'
        self.attached_object.object.operation = CollisionObject.ADD
        self.scene = ps()

        # CITATION [3]]
        self.scene.robot_state.attached_collision_objects.append(
            self.attached_object
        )
        # END CITATION

        self.scene.is_diff = True
        self.node.scene_object_publisher.publish(self.scene)
        self.node.get_logger().info(f'Box {boxId} attached to end effector')

    def Detach(self, boxId):
        """Detach an object from the end effector."""
        attached_object = AttachedCollisionObject()
        attached_object.link_name = 'fer_hand'
        attached_object.object.id = f'box_{boxId}'
        attached_object.object.operation = CollisionObject.REMOVE
        self.scene = ps()

        box = CollisionObject()
        box.id = f'box_{boxId}'
        box.operation = CollisionObject.ADD
        self.scene.world.collision_objects = [box]
        self.scene.robot_state.attached_collision_objects = [attached_object]
        self.scene.is_diff = True
        self.scene.robot_state.is_diff = True
        self.node.scene_object_publisher.publish(self.scene)
        self.node.get_logger().info(f'Box_{boxId} detached from end effector')

    def LoadScene(self, scene):  # loads existing generated scene
        """Load a full scene from a specified filepath."""
        # Citation [4]
        self.node.get_logger().info(f'Scene path is {scene}')

        with open(scene, 'r') as file:
            # End Citation
            data = yaml.safe_load(file)
            objects = data['/**']['ros__parameters']['objects']
            for dic in objects:
                self.node.get_logger().info(f'dict is {dic}')
                self.AddBox(
                    dic['object_pose']['position'][0],
                    dic['object_pose']['position'][1],
                    dic['object_pose']['position'][2],
                    dic['geometry']['size'][0],
                    dic['geometry']['size'][1],
                    dic['geometry']['size'][2],
                    dic['name'],
                )
