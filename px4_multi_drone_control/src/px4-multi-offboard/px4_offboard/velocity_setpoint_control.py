import rclpy
import rclpy.clock
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleAttitudeSetpoint, VehicleRatesSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleStatus, VehicleLocalPositionSetpoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion

import time
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R

class GeometricControl(Node):
    def __init__(self):
        super().__init__('velocity_setpoint_control_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.state_callback,
            qos_profile)
        
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.pose_callback,
            qos_profile
        )
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.attitude_set_publisher_ = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.rates_set_publisher_ = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.publisher_ = self.create_publisher(VehicleLocalPositionSetpoint, '/fmu/in/vehicle_local_position_setpoint', qos_profile)
        
        self.position = None
        self.velocity = None
        self.acceleration = None
        self.quaternion = None
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.failsafe = False
        self.flightCheck = False
        self.offboardMode = False
        self.current_state = "IDLE"
        self.offboard_setpoint_counter_ = 0
        self.takeoff_yaw = 0.0
        self.height = -2.0

        self.start_iteration = None
        self.start_x = None
        self.start_y = None
        self.first_translation = None
        self.iter = 0

        self.dt = 0.02

        self.state_timer_ = self.create_timer(0.1, self.drone_state_callback)

        self.control_timer_ = self.create_timer(self.dt, self.control_callback)
    
    def control_callback(self):

        if(self.offboardMode == True and self.current_state == "OFFBOARD"):
            self.trajectory_profile()
            self.arm()
            self.offboard_mode()

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = self.v_ref[self.iter, 0]
            trajectory_msg.velocity[1] = self.v_ref[self.iter, 1]
            trajectory_msg.velocity[2] = self.v_ref[self.iter, 2]
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = float('nan')

            self.trajectory_setpoint_publisher_.publish(trajectory_msg)
            
            self.iter = self.iter + 1

            if self.iter == np.shape(self.v_ref)[0]:
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                trajectory_msg.velocity[0] = 0.0
                trajectory_msg.velocity[1] = 0.0
                trajectory_msg.velocity[2] = 0.0
                trajectory_msg.position[0] = float('nan')
                trajectory_msg.position[1] = float('nan')
                trajectory_msg.position[2] = float('nan')
                trajectory_msg.acceleration[0] = float('nan')
                trajectory_msg.acceleration[1] = float('nan')
                trajectory_msg.acceleration[2] = float('nan')
                trajectory_msg.yaw = float('nan')
                trajectory_msg.yawspeed = float('nan')

                self.trajectory_setpoint_publisher_.publish(trajectory_msg)
        
    def state_callback(self, msg):
        self.position = np.array([msg.x, msg.y, msg.z])
        if msg.v_xy_valid and msg.v_z_valid:
            self.velocity = np.array([msg.vx, msg.vy, msg.vz])
        self.acceleration = np.array([msg.ax, msg.ay, msg.az])
        # print("the cceleration is", self.acceleration)

    def pose_callback(self, msg):
        self.quaternion = msg.q
    
    def vehicle_status_callback(self, msg):
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")

        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass
    
    def drone_state_callback(self):
        """Callback function to publish messages every 100ms"""
        if self.current_state == "IDLE":
            if self.flightCheck == True:
                self.current_state = "ARMING"
                print("ARMING THE DRONE")

        elif self.current_state == "ARMING":
            if(not(self.flightCheck)):
                self.current_state = "IDLE"
                self.get_logger().info(f"Arming, Flight Check Failed")

            elif (self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.offboard_setpoint_counter_ == 10):
                print(self.current_state)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
                self.offboardMode = True
                self.current_state = "TAKEOFF"
            self.arm()

        # Publish offboard control mode and trajectory setpoint
        elif self.current_state == "TAKEOFF":
            if(not(self.flightCheck)):
                self.current_state = "IDLE"
                self.get_logger().info(f"Takeoff, Flight Check Failed")
            else:   
                self.offboard_mode()
                self.arm()            
                self.takeoff(self.height)
                height_error = self.position[2] - self.height
                if abs(height_error) < 0.2:
                    self.current_state = "LOITER"
        
        elif self.current_state == "LOITER":
            if(not(self.flightCheck)):
                self.current_state = "IDLE"
                self.get_logger().info(f"Loiter, Flight Check Failed")
            else:
                self.current_state = "OFFBOARD"
                self.get_logger().info(f"Loiter, Offboard")
            self.arm()
        
        elif self.current_state == "OFFBOARD":
            if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                self.current_state = "IDLE"
                self.get_logger().info(f"Offboard, Flight Check Failed")
            self.offboard_mode()
            # print("The mode", self.offboardMode)
            # print("The flightCheck", self.flightCheck)
            # print("The arming mode", self.arm_state)
            # print("The navigating mode", self.nav_state)

        # Stop the counter after reaching 11
        if self.offboard_setpoint_counter_ < 10:
            self.offboard_mode()  # Continuously send setpoints
            self.offboard_setpoint_counter_ += 1
    
    def arm(self):
        """Send a command to arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        # self.get_logger().info("Arm command sent")
    
    def offboard_mode(self):
        """Publish the offboard control mode message"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(time.time() * 1e9) // 1000  # Convert time to microseconds
        self.offboard_control_mode_publisher_.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command messages"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(time.time() * 1e9) // 1000  # Convert time to microseconds
        self.vehicle_command_publisher_.publish(msg)
    
    def takeoff(self, height):
        """Publish a trajectory setpoint to make the vehicle hover at 5 meters"""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, height]  # Hover at 5 meters altitude
        msg.yaw = self.takeoff_yaw  # Yaw set to 180 degrees
        msg.timestamp = int(time.time() * 1e9) // 1000  # Convert time to microseconds
        self.trajectory_setpoint_publisher_.publish(msg)
    
    def trajectory_profile(self):
        if not self.start_iteration:
            first_position = self.position
            self.start_x = first_position[0]
            self.start_y = first_position[1]
            self.first_translation = (self.start_x, self.start_y)
            print("the first position is ", self.first_translation)
            self.start_iteration = True

            t = np.array([0, 1/3, 2/3, 1])
            x = np.array([self.first_translation[0], self.first_translation[0] + 2.0, self.first_translation[0] + 4.0, self.first_translation[0] + 6.0])
            y = np.array([self.first_translation[1], self.first_translation[1] + 2.0, self.first_translation[1] + 2.0, self.first_translation[1] + 6.0])
            z = np.array([1, 1, 1, 1])

            cs_x = CubicSpline(t, x)
            cs_y = CubicSpline(t, y)
            cs_z = CubicSpline(t, z)

            t_traj = np.linspace(0, 1, 2000)
            self.px_des = cs_x(t_traj)
            self.py_des = cs_y(t_traj)
            self.pz_des = cs_z(t_traj)
            self.p_ref = np.vstack((self.px_des, self.py_des, self.pz_des)).T

            self.v_ref = np.diff(self.p_ref, axis=0) / 0.02

            # fig = plt.figure()
            # ax = fig.add_subplot(111, projection='3d')
            # ax.plot(self.x_new, self.y_new, self.y_new, label='Cubic Trajectory')
            # ax.scatter(x ,y, z, c='b', marker='o')
            # ax.legend()
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')
            # ax.set_zlabel('Z')
            # plt.show()
    


    
def main(args=None):
    rclpy.init(args=args)
    offboard_control_node = GeometricControl()
    rclpy.spin(offboard_control_node)
    offboard_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

