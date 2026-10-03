import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('cmd_vel_guard'), 'config', 'params.yaml')

    bag_path = LaunchConfiguration('bag_path')
    play_bag = LaunchConfiguration('play_bag')
    rqt_graph = LaunchConfiguration('rqt_graph')

    return LaunchDescription([
        # 启动参数，可在命令行用 名字:=值 覆盖
        DeclareLaunchArgument('bag_path', default_value='/ws/ros2/src/103/bags/cmd_vel'),
        DeclareLaunchArgument('play_bag', default_value='true'),
        DeclareLaunchArgument('rqt_graph', default_value='false'),

        Node(
            package='cmd_vel_guard',
            executable='cmd_vel_filter',
            name='cmd_vel_filter',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='cmd_vel_guard',
            executable='robot_state_monitor',
            name='robot_state_monitor',
            parameters=[params],
            output='screen',
        ),

        # 延迟 2 秒播放 bag，确保两个节点已经启动并完成订阅
        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_path],
                output='screen',
                condition=IfCondition(play_bag),
            ),
        ]),

        ExecuteProcess(
            cmd=['rqt_graph'],
            condition=IfCondition(rqt_graph),
        ),
    ])
