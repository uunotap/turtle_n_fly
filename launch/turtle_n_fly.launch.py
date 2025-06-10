import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('turtle_n_fly')
    webots_dir = get_package_share_directory('webots_ros2')

    world_file = os.path.join(package_dir, 'worlds', 'empty.wbt')

    webots = WebotsLauncher(
        world=world_file,
        ros2_supervisor=True
        	
    )
    
    
    
    
    turtle_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    mavic_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')
        
    turtlebot_driver_node = WebotsController(
        robot_name='turtlebot3',
        namespace='turtlebot3',
        parameters=[
            {'robot_description': turtle_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
        ],
        #respawn=True
    )    

    mavic_driver_node = WebotsController(
        robot_name='mavic2pro',
        namespace='mavic',
        parameters=[
            {'robot_description': mavic_path},        
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
        ],
        #respawn=True
    )    

    
    
    
    
#    drone_controller_node = Node(
#	    package='turtle_n_fly',
#    	executable='drone_controller',
#    	output='screen'
#	)


    return LaunchDescription([
        webots,

       	turtlebot_driver_node,
	
        launch.actions.RegisterEventHandler(
        	event_handler=launch.event_handlers.OnProcessStart(
        	
        	target_action=webots,
        	on_start=[
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
