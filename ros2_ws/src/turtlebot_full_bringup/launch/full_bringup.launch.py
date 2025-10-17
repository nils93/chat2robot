from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    pkg_bringup = FindPackageShare('turtlebot3_full_bringup').find(
        'turtlebot3_full_bringup'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define paths
    world_file = os.path.join(pkg_bringup, 'worlds', 'playground.world')

    # Gazebo Launch
    gazebo_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch')

    # Nav2 Launch
    nav2_pkg = FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2')
    nav2_param = os.path.join(nav2_pkg, 'param')

    # RViz Config
    rviz_config_dir = os.path.join(pkg_bringup, 'rviz', 'rviz.rviz')
    #rviz_config_dir = os.path.join(
    #    nav2_pkg,
    #    'rviz',
    #    'tb3_navigation2.rviz')
    
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
    
    # Parameters
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                nav2_param,
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                nav2_param,
                param_file_name))
    # NODES
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
        ]
    )
