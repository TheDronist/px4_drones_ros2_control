import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand 
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool

import time
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from tf_transformations import euler_from_quaternion

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers for OffboardControlMode, TrajectorySetpoint, and VehicleCommand
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)
        self.goal_reached_pub = self.create_publisher(Bool, "/goal_reached", qos_profile)

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/px4_1/fmu/out/vehicle_odometry',
            self.pose_callback,
            qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
        )

        self.offboard_setpoint_counter_ = 0
        self.height = -2.0
        self.takeoff_yaw = 0.0
        self.position = None
        self.quaternion = None

        self.path = False
        self.current_state = "IDLE"

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.failsafe = False
        self.flightCheck = False
        self.offboardMode = False

        self.start_x = None
        self.start_y = None
        self.start_iteration = False
        self.traj_index = 0
        self.time_limit = 1.0
        self.Kp = 1.5
        self.Ki = 0.001
        self.Kd = 1.0
        self.prev_dist_error = 0
        self.integral_dist_error = 0
        self.goal_reached = False
        self.holonomic = True
        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        self.state_timer_ = self.create_timer(0.1, self.drone_state_callback)
        self.control_timer_ = self.create_timer(0.001, self.control_callback)
    
    def pose_callback(self, msg):

        self.position = msg.position
        self.quaternion = msg.q  # [qw, qx, qy, qz]
        
        # Convert quaternion to yaw (assuming FRD frame)
        yaw = np.arctan2(2.0 * (self.quaternion[0] * self.quaternion[3] + self.quaternion[1] * self.quaternion[2]), 
                        1.0 - 2.0 * (self.quaternion[2] * self.quaternion[2] + self.quaternion[3] * self.quaternion[3]))
        
        # Print position and orientation
        # print(f"Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        # print(f"Orientation (Yaw): {yaw:.2f} rad")
    
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

    def arm(self):
        """Send a command to arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        # self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a command to disarm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")
    
    def loiter(self):
        """Send a command to disarm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

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

    def takeoff(self, height):
        """Publish a trajectory setpoint to make the vehicle hover at 5 meters"""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, height]  # Hover at 5 meters altitude
        msg.yaw = self.takeoff_yaw  # Yaw set to 180 degrees
        msg.timestamp = int(time.time() * 1e9) // 1000  # Convert time to microseconds
        self.trajectory_setpoint_publisher_.publish(msg)

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
                print(height_error)
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
            self.path = True
            # print("The mode", self.offboardMode)
            # print("The flightCheck", self.flightCheck)
            # print("The arming mode", self.arm_state)
            # print("The navigating mode", self.nav_state)

        # Stop the counter after reaching 11
        if self.offboard_setpoint_counter_ < 10:
            self.offboard_mode()  # Continuously send setpoints
            self.offboard_setpoint_counter_ += 1
    
    def control_callback(self):

        if(self.offboardMode == True and self.current_state == "OFFBOARD"):
            if self.path == True:
                # print("Iam publishing trajectory setpoints")
                if not self.start_iteration:
                    first_position = self.position
                    self.start_x = first_position[0]
                    self.start_y = first_position[1]
                    self.first_translation = (self.start_x, self.start_y)
                    self.start_iteration = True

                    t = np.array([0, 1/3, 2/3, 1])
                    # x = np.array([self.first_translation[0], self.first_translation[0] + 0.5, self.first_translation[0] + 1.0, self.first_translation[0] + 1.5])
                    # y = np.array([self.first_translation[1], self.first_translation[1] + 0.0, self.first_translation[1] + 0.0, self.first_translation[1] + 0.0])

                    x = np.array([self.first_translation[0], self.first_translation[0] + 0.5, self.first_translation[0] + 1.0, self.first_translation[0] + 1.5])
                    y = np.array([self.first_translation[1], self.first_translation[1] + 0.5, self.first_translation[1] + 1.0, self.first_translation[1] + 1.5])
                    z = np.array([1, 1, 1, 1])

                    cs_x = CubicSpline(t, x)
                    cs_y = CubicSpline(t, y)
                    cs_z = CubicSpline(t, z)

                    t_fine = np.linspace(0, 1, 100)
                    self.x_new = cs_x(t_fine)
                    self.y_new = cs_y(t_fine)
                    z_fine = cs_z(t_fine)

                    # fig = plt.figure()
                    # ax = fig.add_subplot(111, projection='3d')
                    # ax.plot(self.x_new, self.y_new, z_fine, label='Cubic Trajectory')
                    # ax.scatter(x ,y, z, c='b', marker='o')
                    # ax.legend()
                    # ax.set_xlabel('X')
                    # ax.set_ylabel('Y')
                    # ax.set_zlabel('Z')
                    # plt.show()
                current_time = self.get_clock().now()
                dt = (current_time - self.start_time).nanoseconds / 1e9

                if len(self.x_new) == 0:
                    self.get_logger().info("No trajectory has been published.")
                    return
                
                current_pose = self.get_robot_pose()
                if current_pose is None:
                    return

                current_x, current_y, current_theta = current_pose
                target_x, target_y = self.x_new[self.traj_index], self.y_new[self.traj_index]
                # Compute errors in position
                dx = target_x - current_x
                dy = target_y - current_y
                dist_error = math.sqrt(dx**2 + dy**2)

                if dist_error < 0.2:
                    if dist_error < 0.2 or dt > self.time_limit:
                        if self.traj_index < len(self.x_new) - 1:
                            self.traj_index += 1
                        else:
                            if (dist_error < 0.2 or self.traj_index == np.shape(self.x_new)):
                                print("I am publishing zero velocities ====== 0.0")
                                self.send_trajectory_setpoint_position(0.0, 0.0, 0.0, 0.0)
                                self.path = False
                                if not self.goal_reached:
                                    goal_reached_msg = Bool()
                                    goal_reached_msg.data = True
                                    self.goal_reached_pub.publish(goal_reached_msg)
                                    self.goal_reached = True
                                return
                        self.start_time = self.get_clock().now()

                self.integral_dist_error += dist_error * dt
                dist_derivative = (dist_error - self.prev_dist_error) /dt if dt > 0 else 0.0
                self.prev_dist_error = dist_error
                linear_vel = (self.Kp * dist_error + self.Ki * self.integral_dist_error + self.Kd * dist_derivative)
                print("The velocity is at ====", linear_vel)
                linear_vel = self.limit_trans_velocity(linear_vel)
                
                if self.holonomic:
                    # Holonomic movement, control both x and y velocities
                    velocity_x = linear_vel * (dx / dist_error)
                    velocity_y = linear_vel * (dy / dist_error)
                    velocity_z = 0.0
                    # print("The velocity along x is", velocity_world_x)
                    # print("The velocity along y is", velocity_world_y)

                print("dist_error", dist_error)
                print("elapsed_time", dt)
                print("self.traj_index", self.traj_index)
                print("the velocity", velocity_x)

                self.send_trajectory_setpoint_position(velocity_x, velocity_y, velocity_z, 0.0)
                self.start_time = current_time

                # print("The mode", self.offboardMode)
                # print("The flightCheck", self.flightCheck)
                # print("The arming mode", self.arm_state)
                # print("The navigating mode", self.nav_state)
                
            else:
                print("I am loitering")
    
    def send_trajectory_setpoint_position(self, velocity_x, velocity_y , velocity_z, velocity_yaw):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = velocity_x
        trajectory_msg.velocity[1] = velocity_y
        trajectory_msg.velocity[2] = velocity_z
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = velocity_yaw
        self.trajectory_setpoint_publisher.publish(trajectory_msg)
    
    def get_timestamp(self):
        return self.get_clock().now().nanoseconds // 1
    
    def get_robot_pose(self):
        trans_x = self.position[0]
        trans_y = self.position[1]
        rot = self.quaternion
        euler = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
        return trans_x, trans_y, euler[2]
    
    def limit_trans_velocity(self, velocity):
        max_speed = 0.1
        return max(min(velocity, max_speed), -max_speed)

def main(args=None):
    rclpy.init(args=args)

    offboard_control_node = OffboardControl()

    rclpy.spin(offboard_control_node)

    offboard_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()