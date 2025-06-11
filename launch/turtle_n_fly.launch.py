import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection



def generate_launch_description():
    package_dir = get_package_share_directory('turtle_n_fly')
    webots_dir = get_package_share_directory('webots_ros2')

    world_file = os.path.join(package_dir, 'worlds', 'empty.wbt')

    webots = WebotsLauncher(
        world=world_file,
        ros2_supervisor=True
        	
    )
        
    #-- Mavic 2 Pro launching and config --     
    mavic_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')
    mavic_driver = WebotsController(
        robot_name='mavic2pro',
        parameters=[
            {'robot_description': mavic_description_path},
        ],
        respawn=True
    )
        
#    mavic_controller_node = Node(
#	   package='turtle_n_fly',
#    	executable='drone_controller',
#    	output='screen'
#	)
    #-- Mavic 2 Pro launching and config --



    #-- Turtlebot3 (birgir) launching and config --
    ## altered from cyberbotics turtlebot example
           
    turtle_controller_node = Node(
	    package='turtle_n_fly',
    	executable='turtle_node',
    	output='screen'
	)


    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    
    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/turtlebot3/cmd_vel'), ('/diffdrive_controller/odom', '/turtlebot3/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/turtlebot3/cmd_vel'), ('/diffdrive_controller/odom', '/turtlebot3/odom')]
    
    
    turtlebot_driver_node = WebotsController(
        robot_name='turtlebot3',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )
    
    turtle_ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver_node,
        nodes_to_start=turtle_ros_control_spawners + [turtle_controller_node] #+ navigation_nodes
		,

    )
    #-- turtle launching and config --




    #xöntröl
    controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description_path, ros2_control_params],
    output='screen',
    namespace='turtlebot3'
	)





    return LaunchDescription([
        webots,
        webots._supervisor,
        controller_manager_node,




        #Turtle
        footprint_publisher,
       	turtlebot_driver_node,
        waiting_nodes,

        #Turtle

        # Mavic
      	mavic_driver,	
 #       launch.actions.RegisterEventHandler(
 #       	event_handler=launch.event_handlers.OnProcessStart(
 #       	
 #       	target_action=mavic_driver,
 #       	on_start=[
#		      	mavic_controller_node,
#		      	]
#		   )

#    	),
        # Mavic
        
        
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
