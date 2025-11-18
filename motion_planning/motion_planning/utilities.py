"""
File and data utilities for motion planning.

Provides helpers for saving and loading both joint trajectories(paths)
and jointconfigurations(poses)
"""

from ament_index_python.packages import get_package_prefix

from moveit_msgs.action import ExecuteTrajectory

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import yaml


class Utilities:
    """
    Helper tools.

    Tools dealing with saving, loading, and managing robot trajectories and
    joint configurations. Provides lightweight I/O utilities used by motion
    planners and external ROS 2 nodes.
    """

    def __init__(self, node, robotstate):
        """
        Initialize utilities and resolve config file paths.

        Args:
        ----
        node :
            ROS2 node for logging and file access.
        robotstate :
            RobotState instance providing joint state data.

        """
        self.robotstate = robotstate
        self.node = node
        pkg_prefix = get_package_prefix('motion_planning')

        # Extract workspace path
        workspace_path = pkg_prefix.split('/install/')[0]
        self.config_path = (
            f'{workspace_path}'
            '/src/homework-3-part-2-actual-north-western-northwestern'
            '/motion_planning/config'
        )
        self.savedJointConfigs = f'{self.config_path}/jointConfiguration.yaml'
        self.savedPaths = f'{self.config_path}/savedPaths.txt'

    # Save path to file
    def savePath(self, path, filename: str = 'saved_path.txt'):
        """
        Save a JointTrajectory to a text file.

        Args:
        ----
        path :
            saves the ExecuteTrajectory result
        filename :
            Output file name inside the package config directory.

        """
        # Make sure there is a trajectory for the given path

        traj = getattr(path, 'trajectory', path)
        if not hasattr(traj, 'joint_trajectory'):
            self.node.get_logger().warn('No joint_trajectory field found.')
            return

        # Make sure trajectory contains joint trajectory data
        if not hasattr(traj, 'joint_trajectory'):
            self.node.get_logger().warn('No joint_trajectory field found.')
            return

        if filename is None:
            filename = self.savedPaths
        else:
            filename = self.config_path + f'/{filename}'

        jt = traj.joint_trajectory
        lines = []
        lines.append(f'Joint names: {", ".join(jt.joint_names)}\n')
        lines.append('Points:\n')

        for i, point in enumerate(jt.points):
            t = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )

            lines.append(f'  Point {i}:\n')
            lines.append(f'    time_from_start: {t:.3f} s\n')
            lines.append(f'   positions: {list(point.positions)}\n')
            if point.velocities:
                lines.append(f'    velocities: {list(point.velocities)}\n')
            if point.accelerations:
                lines.append(f'  accelerations: {list(point.accelerations)}\n')
            lines.append('\n')

        # Save to file
        with open(filename, 'w') as f:
            f.writelines(lines)

        self.node.get_logger().info(f'Trajectory saved to {filename}')

    # Load path from file
    def loadPath(self, filename=None):
        """
        Load a saved path from a text file.

        Args
        ----
        filename :
            the name of the file to save the path to

        Returns
        -------
        trajectory_msgs.msg._JointTrajectory.JointTrajectory.

        """
        jt = JointTrajectory()
        points = []

        if filename is None:
            filename = self.savedPaths
        else:
            filename = self.config_path + f'/{filename}'

        with open(filename, 'r') as f:
            lines = f.readlines()

        # Parse joint names
        for line in lines:
            if line.startswith('Joint names:'):
                jt.joint_names = [
                    name.strip() for name in line.split(':')[1].split(',')
                ]
                break

        # Parse trajectory points
        current_point = None
        for line in lines:
            line = line.strip()
            if line.startswith('Point '):
                if current_point:
                    points.append(current_point)
                current_point = JointTrajectoryPoint()
            elif line.startswith('positions:'):
                vals = line.split(':', 1)[1].strip()
                vals = vals.strip('[]')
                current_point.positions = [
                    float(v) for v in vals.split(',') if v.strip()
                ]
            elif line.startswith('velocities:'):
                vals = line.split(':')[1].strip()
                vals = vals.strip('[]')
                current_point.velocities = [
                    float(v) for v in vals.split(',') if v.strip()
                ]
            elif line.startswith('accelerations:'):
                vals = line.split(':')[1].strip()
                vals = vals.strip('[]')
                current_point.velocities = [
                    float(v) for v in vals.split(',') if v.strip()
                ]
                current_point.accelerations = eval(vals)
            elif line.startswith('time_from_start:'):
                t = float(line.split(':')[1].split()[0])
                current_point.time_from_start.sec = int(t)
                current_point.time_from_start.nanosec = int((t % 1) * 1e9)

        if current_point:
            points.append(current_point)

        jt.points = points
        self.node.get_logger().info(
            f'Loaded {len(points)} \
                                    points from {filename}'
        )

        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = jt
        return goal

    def save_joint_configuration(self, path=None, config_name='config'):
        """
        Save the current JointState to a YAML entry.

        Args:
        ----
        path :
            YAML file path. Defaults to stored jointConfiguration file.
        config_name :
            Name of the joint configuration to save

        """
        if path is None:
            path = self.savedJointConfigs
        js = self.robotstate.joint_state
        self.node.get_logger().info(f'Saving JointState: {js}')

        with open(path, 'a') as f:
            f.write(f'{config_name}:\n')
            f.write(f'  names: {js.name}\n')
            f.write(f'  positions: {js.position}\n')
            if js.velocity:
                f.write(f'  velocities: {js.velocity}\n')
            if js.effort:
                f.write(f'  efforts: {js.effort}\n')
            f.write('\n')  # Add blank line between configs

        self.node.get_logger().info(
            f'\n\n\nJointState saved to {self.savedJointConfigs}'
        )

    def load_joint_configuration(self, path=None, config_name='config'):
        """
        Load a JointState configuration from a YAML file.

        Args:
        ----
        path :
            YAML file to read from.
        config_name :
            Name of the joint configuration to load.

        Returns
        -------
        dict containing joint names, positions, and
        optional velocities/efforts.

        """
        if path is None:
            path = self.savedJointConfigs
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            if config_name in data:
                config = data[config_name]

                # Convert string representation of array to list
                for key in ['names', 'positions', 'velocities', 'efforts']:
                    if key in config:
                        value = config[key]
                        if isinstance(value, str):
                            # Extract the list
                            start = value.find('[')
                            end = value.rfind(']')
                            if start != -1 and end != -1:
                                list_str = value[start: end + 1]
                                # Use eval to parse the liist
                                config[key] = eval(list_str)
                        elif hasattr(value, '__iter__') and not isinstance(
                            value, str
                        ):
                            # convert to list
                            config[key] = list(value)

                self.node.get_logger().debug(
                    f"Loaded configuration '{config_name}': {config}"
                )
                return config
            else:
                self.node.get_logger().warn(
                    f"Configuration '{config_name}' not found"
                )
                return None

        except FileNotFoundError:
            self.node.get_logger().error(
                f'File {self.savedJointConfigs} not found'
            )
            return None
        except yaml.YAMLError as e:
            self.node.get_logger().error(f'Error parsing YAML: {e}')
            return None
