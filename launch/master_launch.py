import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction # Thêm TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'Xe'
    pkg_dir = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_dir, 'urdf', 'Xe.urdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'xe_pro.rviz')
    world_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    os.environ['QT_QPA_PLATFORM'] = 'xcb'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )),
        launch_arguments={'world': world_path, 'gui': 'true'}.items()
    )

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph'
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', 
             parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], 
             parameters=[{'use_sim_time': True}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', 
             arguments=['-entity', 'Xe', '-topic', 'robot_description', '-x', '-2.0', '-y', '0.5', '-z', '0.2']),
        Node(package='Xe', executable='mecanum_numpad.py', name='mecanum_control', 
             output='screen', emulate_tty=True, prefix='gnome-terminal --'),
        TimerAction(
            period=5.0,
            actions=[rqt_graph_node]
        )
    ])