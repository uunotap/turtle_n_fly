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
            {'update_rate': 50},
            {'set_robot_state_publisher': True},
        ],
        remappings= [('/cmd_vel', 'turtlebot3/cmd_vel'),],
        respawn=True
    )    

    mavic_driver_node = WebotsController(
        robot_name='mavic2pro',
        namespace='mavic',
        parameters=[
            {'robot_description': mavic_path},        
            {'use_sim_time': True},
            {'set_robot_state_publisher': True},
        ], 
 	remappings= [('/cmd_vel', 'mavic/cmd_vel'),],
        respawn=True
    
    )
    
    
    #state publisher to make the simulation editable??
    turtle_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
        namespace='turtlebot3'
    )
    
    mavic_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
        namespace='mavic'
    )
    
    
    drone_controller_node = Node(
        package='turtle_n_fly',
        executable='drone_controller',
        namespace='mavic',
        output='screen'
    )



    return LaunchDescription([
        webots,
        webots._supervisor,
        
       	turtlebot_driver_node,
 	    mavic_driver_node,
	
        launch.actions.RegisterEventHandler(
        	event_handler=launch.event_handlers.OnProcessStart(
        	
        	target_action=webots,
        	on_start=[
			turtle_robot_state_publisher,
			mavic_robot_state_publisher,
		      	drone_controller_node,
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
