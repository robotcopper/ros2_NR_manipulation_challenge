from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Specify directory and path to file within package
    ur_simulation_gazebo_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_simulation_gazebo_launch_file_subpath = 'launch/ur_sim_control.launch.py'

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ur_simulation_gazebo_pkg_dir, ur_simulation_gazebo_launch_file_subpath)
            ),
            launch_arguments={'ur_type': 'ur5'}.items(),
        ),

        # Wait 2 seconds for the controller to become available
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros2_nr_motion_control',
                    executable='joint_angle_publisher',
                    name='joint_angle_publisher',
                    output='screen',
                ),

                Node(
                    package='ros2_nr_motion_control',
                    executable='cartesian_space_trajectory_node',
                    name='cartesian_space_trajectory_node',
                    output='screen',
                )
            ]
        )
    ])
