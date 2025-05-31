# px4_drones_ros2_control
This repository contains different ROS2 nodes that control the movement of the PX4 drones in single and multiple systems.

## Single PX4 Drone Control

The package `px4_single_drone_control` contains several ROS2 nodes that can be used for arming, offboard mode switching, taking off & landing, PID trajectory tracking and offboard missions execution.

### How It Works

To build the package, first both the `px4_msgs` and `px4_ros_com` packages should be cloned inside the `src` folder. This can be done using the commands `git clone https://github.com/PX4/px4_msgs.git --branch release/1.14` and `git clone https://github.com/PX4/px4_ros_com --branch release/v1.14`. Then, the whole package must be build using `colcon build --symlink-install`. To run any node, fisrt the `PX4-Autopilot` [Link text](https://github.com/PX4/PX4-Autopilot.git) must be clone and build (I used the release/1.14 version). The MicroDDS Agent is also mandatory for running the nodes. To install `PX4-Autopilot`, run inside the `PX4-Autopilot` folder: `make px4_sitl gazebo-classic` (This may take several minutes). To run it, you may use `PX4_UXRCE_DDS_NS=px4_1 make px4_sitl gazebo-classic` where `PX4_UXRCE_DDS_NS` is the namespace attributed to the drone's topics. On the other hand, you may follow the instructions in [Link text](https://docs.px4.io/main/en/middleware/uxrce_dds.html) to install the `MicroDDS Agent`. To run it simply use the command `MicroXRCEAgent udp4 -p 8888`. After running both mentioned commands, you may check the ROS2 topics whether they are listed correctly. Now, you are ready to run any node of your choice using the command `ros2 run px4-offboard <node_name>`. You can use `QGroundControl` to visualise the movement of the drone.

## Multiple PX4 Drones Control

Just like the `px4_single_drone_control` package, the `px4_multi_drone_control` contains ROS2 nodes and launch files for multiple PX4 drones mission execution.

### How It Works

To spawn a certain number of drones in your environmen, run the command `./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 2` inside the PX4-Autopilot folder, where `-n` stands for the number of drones you want to spawn and `-m` is their types. Then run the `MicroDDS Agent` to obtain the drones' topics using `MicroXRCEAgent udp4 -p 8888`. Now, you are ready to run any node of your choice using the command `ros2 launch px4-offboard <nlaunch_file_name>`. You can use `QGroundControl` to visualise the movement of the drones.
