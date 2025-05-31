import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adel',
    maintainer_email='adel@@',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'visualizer = px4_offboard.visualizer:main',
                'figure_8_traj_control = px4_offboard.figure_8_traj_control:main',
                'velocity_setpoint_control = px4_offboard.velocity_setpoint_control:main',   
                'test_arming = px4_offboard.test_arming:main', 
                'test_vtol = px4_offboard.test_vtol:main',   
                'test_moveto_wp = px4_offboard.test_moveto_wp:main',      
                'mission_example = px4_offboard.mission_example:main', 
        ],
    },
)
