import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    bt_server_config_path = os.path.join(package_dir, 'config', 'bt_server.yaml')

    return LaunchDescription([
        Node(
            package='mpc_rbt_student',
            executable='localization',
            name='localization',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='mpc_rbt_student',
            executable='planning',
            name='planning',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='mpc_rbt_student',
            executable='motion_control',
            name='motion_control_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='mpc_rbt_student',
            executable='warehouse_manager',
            name='warehouse_manager',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='mpc_rbt_student',
            executable='bt_server',
            name='bt_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                bt_server_config_path
            ]
        )
    ])