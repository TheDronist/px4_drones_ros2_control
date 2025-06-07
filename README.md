# Px4 Drones ROS2 Control
This repository contains different ROS2 nodes that control the movement of the PX4 drones in single and multiple systems.

## Single PX4 Drone Control

The package `px4_single_drone_control` contains several ROS2 nodes that can be used for arming, offboard mode switching, taking off & landing, PID trajectory tracking and offboard missions execution.

### How It Works

- To build the package, first both the `px4_msgs` and `px4_ros_com` packages should be cloned inside the `src` folder. This can be done using the commands `git clone https://github.com/PX4/px4_msgs.git --branch release/1.14` and `git clone https://github.com/PX4/px4_ros_com --branch release/v1.14`. 
- Then, the whole package must be build using `colcon build --symlink-install`. To run any node, fisrt the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git) must be clone and build (I used the release/1.14 version). The MicroDDS Agent is also mandatory for running the nodes. 
- To install `PX4-Autopilot`, run inside the `PX4-Autopilot` folder: `make px4_sitl gazebo-classic` (This may take several minutes). To run it, you may use `PX4_UXRCE_DDS_NS=px4_1 make px4_sitl gazebo-classic` where `PX4_UXRCE_DDS_NS` is the namespace attributed to the drone's topics. On the other hand, you may follow the instructions in [uXRCE DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) to install the `MicroDDS Agent`. To run it simply use the command `MicroXRCEAgent udp4 -p 8888`. 
- After running both mentioned commands, you may check the ROS2 topics whether they are listed correctly. Now, you are ready to run any node of your choice using the command `ros2 run px4-offboard <node_name>`. You can use `QGroundControl` to visualise the movement of the drone.

## Multiple PX4 Drones Control

Just like the `px4_single_drone_control` package, the `px4_multi_drone_control` contains ROS2 nodes and launch files for multiple PX4 drones mission execution.

### How It Works

- To spawn a certain number of drones in your environmen, run the command `./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 2` inside the PX4-Autopilot folder, where `-n` stands for the number of drones you want to spawn and `-m` is their types. 
- Then run the `MicroDDS Agent` to obtain the drones' topics using `MicroXRCEAgent udp4 -p 8888`. 
- Now, you are ready to run any node of your choice using the command `ros2 launch px4-offboard <launch_file_name>`. You can use `QGroundControl` to visualise the movement of the drones.

## Some Hardware Techniques

- In case of using single drone simulation/implementation, there would be no problems as long as you are targeting a single drone system with a already declared `self.target_system` which equals to 1. However when using multi-drone system in simulation, each drone should have its own `self.target_system` which starts from 2 by default. Meaning, the drone which is attributed with a namespace `/px4_1/` would be targeted using an instance `self.target_system = 2` and the drone which is attributed with a namespace `/px4_2/` would be targeted using an instance `self.target_system = 3`, and so on. 
- When dealing with multi-drone system in hardware, you have to follow the following steps:
1. Set each drone system ID `MAV_SYS_ID` to have a unique value for each drone. Drones with same `MAV_SYS_ID` may be seen as a single drone system by the ground station. To do so, first connect the drone to the ground station using QGroundControl. Once connected, use the `Mavlink Shell` of the `PX4-Autopilo` by running `./Tools/mavlink_shell.py 0.0.0.0:14550`. Then, set the `MAV_SYS_ID` to a defined ID using `param set MAV_SYS_ID 1` for Drone 1, for example. It is worth mentioning that in hardware, the `MAV_SYS_ID` represents `self.target_system`. So unlike in simulation, for a drone with `MAV_SYS_ID = 1`, `self.target_system = 1`
2. The drone should be set to their Station Mode, using the `adb shell` then `voxl-wifi`. The drones and the ground station should all be in the same network.
3. To connect multiple drones to the ground station, you may use QGroundControl. Turn ON the first drone and connect it to the ground station using QGroundControl. The drone is in its Station Mode. Thus to do so, first find the IP address of the drone and then use the `Comm Links` in QGroundControl to perform the connection. Once connected, go to `Analyze Tools` then `Mavlink Console`, check the status of the Microdds_Client using `uxrce_dds_client status` for simulated drone or `microdds_client status` for real drone. To run the MicroDDS Client using a predefined namespace, first stop the running MicroDDS Client using `uxrce_dds_client stop` for simulated drone or `microdds_client stop` for real drone then run `uxrce_dds_client start -t udp -p 8888 -h 127.0.0.1 -n <namespace>` for simulated drone or `microdds_client start -t udp -p 8888 -h 127.0.0.1 -n <namespace>` for real drone. Finally, check the topics using `ros2 topic list`.
4. Do the same with the second drone, follow step `3` above. To run the MicroDDS Client with a namespace, make sure that you have selected the appropriate drone using the list of `VEHICLE` in QGroundControl otherwise you would make changes to the MicroDDS Client of the previous drones.
5. Each time a drone is added, check the topics if they are published using `ros2 topic list`.
6. When you are done with all drones, now you are ready to control them using the designed ROS2 nodes. 
