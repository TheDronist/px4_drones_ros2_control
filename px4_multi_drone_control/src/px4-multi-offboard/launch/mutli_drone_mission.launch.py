import os
from ament_index_python.packages import get_package_share_directory
from time import sleep
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    offboard_node_1 = Node(
        package='px4_offboard',
        executable='mission_example',
        name='offboard_control_node',
        parameters=[
            {'px4_id': 'px4_1'},
            {'target_system': 2},  # px4_instance=2
            {'takeoff_x': 3.0},
            {'takeoff_y': -9.0},
            {'takeoff_height': 2.0},
            {'takeoff_yaw_ang': 0.0},
        ],
        output='screen'
    )

    offboard_node_2 = Node(
        package='px4_offboard',
        executable='mission_example',
        name='offboard_control_node',
        parameters=[
            {'px4_id': 'px4_2'},
            {'target_system': 3},  # px4_instance=3
            {'takeoff_x': 0.0},
            {'takeoff_y': 9.0},
            {'takeoff_height': 3.0},
            {'takeoff_yaw_ang': 0.0},
        ],
        output='screen'
    )

    offboard_node_3 = Node(
        package='px4_offboard',
        executable='mission_example',
        name='offboard_control_node',
        parameters=[
            {'px4_id': 'px4_3'},
            {'target_system': 4},  # px4_instance=4
            {'takeoff_x': 3.0},
            {'takeoff_y': 0.0},
            {'takeoff_height': 6.0},
            {'takeoff_yaw_ang': 0.0},
        ],
        output='screen'
    )

    ld.add_action(offboard_node_1)
    ld.add_action(offboard_node_2)
    ld.add_action(offboard_node_3)

    return ld