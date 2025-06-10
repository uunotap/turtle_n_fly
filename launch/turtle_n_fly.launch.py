import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('turtle_n_fly')
    webots_dir = get_package_share_directory('webots_ros2')

    world_file = os.path.join(package_dir, 'worlds', 'empty.wbt')

    webots = WebotsLauncher(
        world=world_file
    )
    
    
    
    turtlebot_driver_node = Node(
    package='webots_ros2_driver',
    executable='driver',
    name='turtlebot_driver',
    output='screen',
    parameters=[
    {'robot_name': 'turtlebot3'},
    {'use_sim_time': True}
    ],
    namespace='turtlebot3'
    )
    
    
    
    mavic_driver_node = Node(
    package='webots_ros2_driver',
    executable='driver',
    name='mavic_driver',
    output='screen',
    parameters=[
    {'robot_name': 'mavic2pro'},
    {'use_sim_time': True}
    ],
    namespace='mavic'
    )
    
    
    
#    drone_controller_node = Node(
#	    package='turtle_n_fly',
#    	executable='drone_controller',
#    	output='screen'
#	)


    return LaunchDescription([
        webots,
	
        launch.actions.RegisterEventHandler(
        	event_handler=launch.event_handlers.OnProcessStart(
        	
        	target_action=webots,
        	on_start=[
        		turtlebot_driver_node,
        		mavic_driver_node,


		      	#drone_controller_node,
		      	]
		   )
	),
        
        
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
