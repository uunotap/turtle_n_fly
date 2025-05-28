from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_ignition_bringup'),
                'launch',
                'ignition.launch.py'
            )
        )
    )

    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_drone',
            '-x', '1.0', '-y', '1.0', '-z', '1.5',
            '-file', os.path.join(
                get_package_share_directory('turtle_n_fly'),
                'models', 'simple_drone', 'model.sdf'
            )
        ],
        output='screen'
    )

    drone_controller = Node(
        package='turtle_n_fly',
        executable='drone_controller',
        name='drone_controller',
        output='screen'
    )

    return LaunchDescription([
        turtlebot_launch,
        spawn_drone,
        drone_controller
    ])

