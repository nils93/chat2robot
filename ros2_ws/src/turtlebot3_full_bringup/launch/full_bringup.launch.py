from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

#from scripts.Chatbot import Chatbot


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    pkg_bringup = FindPackageShare('turtlebot3_full_bringup').find(
        'turtlebot3_full_bringup'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Define paths
    world_file = os.path.join(pkg_bringup, 'worlds', 'playground.world')
    map_dir = os.path.join(pkg_bringup, 'maps', 'playground_map_hq.yaml')
    
    # Gazebo Launch
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch')

    # Nav2 Launch
    init_nav2_params = os.path.join(pkg_bringup, 'config', 'init_nav2_params.yaml')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_bringup_launch = os.path.join(nav2_bringup_pkg, 'launch')

    # RViz Config
    rviz_config_dir = os.path.join(pkg_bringup, 'rviz', 'rviz.rviz')

    
    # Robot State Publisher
    robostate_package = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    robostate_launch = os.path.join(
        robostate_package, 'launch', 'robot_state_publisher.launch.py'
    )
    robotspawn_launch = os.path.join(
        robostate_package, 'launch', 'spawn_turtlebot3.launch.py'
    )

    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')  

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch, 'gzclient.launch.py')
        ),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robotspawn_launch)),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )


    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robostate_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': init_nav2_params,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )


    return LaunchDescription(
        [
        rviz,
        gzserver,
        gzclient,
        spawn_turtlebot_cmd,
        robot_state_publisher_cmd,
        nav2_launch
        ]
    )
