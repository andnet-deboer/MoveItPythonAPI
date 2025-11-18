"""Launch necessary files for motion_planning."""

import os

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    """Generate franka launch file."""
    moveit_config = MoveItConfigsBuilder(
        'fer', package_name='franka_fer_moveit_config'
    ).to_moveit_configs()
    description = generate_demo_launch(moveit_config)
    # Getting full path to the package
    pkgPath = launch_ros.substitutions.FindPackageShare(
        package='motion_planning'
    ).find('motion_planning')
    # Full path to the yaml file that has the parameters for the scene
    yamlPath = os.path.join(pkgPath, 'config/planning_scene.yaml')

    description.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    FindPackageShare('franka_gripper'),
                    'launch',
                    'gripper.launch.py',
                ]
            ),
            launch_arguments={
                'arm_id': 'fer',
                'robot_ip': 'None',
                'use_fake_hardware': 'true',
            }.items(),
        )
    )
    # We need a joint_state_publisher to unify the joint states from the
    # gripper and the arm
    description.add_action(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[
                {
                    'source_list': [
                        'joint_state_broadcaster/joint_states',
                        'fer_gripper/joint_states',
                    ],
                    'rate': 30,
                }
            ],
        )
    )
    description.add_action(
        Node(
            package='motion_planning',
            executable='pick',
            parameters=[yamlPath, {'scene': yamlPath}],
        )
    )
    return description
