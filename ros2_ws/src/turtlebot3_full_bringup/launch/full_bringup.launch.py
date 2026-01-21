from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os




TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    # Locate the package share directory
    pkg_bringup = FindPackageShare('turtlebot3_full_bringup').find(
        'turtlebot3_full_bringup'
    )
    
    # Enable simulation time across all nodes
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Define paths
    world_file = os.path.join(pkg_bringup, 'worlds', 'playground.world')  # Gazebo world file
    map_dir = os.path.join(pkg_bringup, 'maps', 'playground_map_hq.yaml')  # Navigation map
    
    # Gazebo Launch - locate and prepare Gazebo launch files
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch')  # Gazebo launch directory

    # Nav2 Launch - navigation stack configuration
    init_nav2_params = os.path.join(pkg_bringup, 'config', 'init_nav2_params.yaml')  # Nav2 parameters
    nav2_bringup_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_pkg, 'launch')  # Nav2 launch directory

    # RViz Config - visualization tool configuration
    rviz_config_dir = os.path.join(pkg_bringup, 'rviz', 'rviz.rviz')  # RViz configuration file

    
    # Robot State Publisher - paths for robot transform and spawn launch files
    robostate_package = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    robostate_launch = os.path.join(
        robostate_package, 'launch', 'robot_state_publisher.launch.py'  # Publishes TF transforms
    )
    robotspawn_launch = os.path.join(
        robostate_package, 'launch', 'spawn_turtlebot3.launch.py'  # Spawns robot in Gazebo
    )

    # Initial robot pose parameters (x, y coordinates in world frame)
    x_pose = LaunchConfiguration('x_pose', default='-2.0')  # Initial X position
    y_pose = LaunchConfiguration('y_pose', default='-0.5')  # Initial Y position  

    # Gazebo server - physics simulation backend
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items(),  # Pass world and time settings
    )

    # Gazebo client - graphical interface for visualization
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzclient.launch.py')
        ),
    )

    # Spawn TurtleBot3 robot in the Gazebo simulation at specified position
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robotspawn_launch)),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),  # Set initial robot position
    )

    # Robot state publisher - publishes URDF and transform tree
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robostate_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),  # Sync with simulation time
    )
    
    # Nav2 navigation stack - autonomous navigation with localization and path planning
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,  # Provide pre-built map for localization
            'use_sim_time': use_sim_time,  # Use simulated time
            'params_file': init_nav2_params,  # Nav2 configuration parameters
        }.items(),
    )

    # RViz - 3D visualization tool for monitoring sensor data and robot state
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],  # Load predefined visualization configuration
        parameters=[{'use_sim_time': use_sim_time}],  # Sync time with simulation
        output='screen',  # Display output in terminal
    )


    # Combine all launch actions - order matters for dependency resolution
    return LaunchDescription(
        [
        rviz,  # Start visualization
        gzserver,  # Start physics engine
        gzclient,  # Start Gazebo GUI
        spawn_turtlebot_cmd,  # Spawn robot in world
        robot_state_publisher_cmd,  # Publish robot transforms
        nav2_launch  # Start navigation stack
        ]
    )
