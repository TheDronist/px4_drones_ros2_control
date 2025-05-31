from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='px4_offboard', # Replace with your package name 
            executable='test_nodes', # Replace with your executable name 
            name='px4_ros_node', 
            namespace='px4_1', 
            remappings=[ ('/fmu/out/vehicle_status', '/px4_1/fmu/out/vehicle_status'), # Add additional remappings as needed 
                        ], 
            ) 
        ])