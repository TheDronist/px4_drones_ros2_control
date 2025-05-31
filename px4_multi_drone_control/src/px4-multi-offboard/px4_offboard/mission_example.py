import rclpy
import rclpy.clock
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleAttitudeSetpoint, VehicleRatesSetpoint
from px4_msgs.msg import VehicleLocalPosition, GotoSetpoint, VehicleStatus, VehicleGlobalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
from codrones_msgs.msg import ThreatDistance

import time
import numpy as np
import math
from geopy.distance import geodesic
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R

EARTH_RADIUS = 6378137.0

def gps_to_ned(reference_lat, reference_lon, reference_alt, lat, lon, alt):
    ref_lat_rad = math.radians(reference_lat)
    ref_lon_rad = math.radians(reference_lon)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    delta_lat = lat_rad - ref_lat_rad
    delta_lon = lon_rad - ref_lon_rad

    # Convert to meters
    north = EARTH_RADIUS * delta_lat
    east = EARTH_RADIUS * math.cos(ref_lat_rad) * delta_lon
    down = reference_alt - alt

    return north, east, down

def ned_to_gps(reference_lat, reference_lon, reference_alt, north, east, down):
    ref_lat_rad = math.radians(reference_lat)
    ref_lon_rad = math.radians(reference_lon)

    delta_lat = north / EARTH_RADIUS
    delta_lon = east / (EARTH_RADIUS * math.cos(ref_lat_rad))

    lat = math.degrees(ref_lat_rad + delta_lat)
    lon = math.degrees(ref_lon_rad + delta_lon)

    alt = reference_alt + down

    return lat, lon, alt

class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('mission_example_3')

        # Parameters
        self.declare_parameter('px4_id', 'px4_1')
        self.declare_parameter('target_system', 2)
        self.declare_parameter('takeoff_x', 0.0)
        self.declare_parameter('takeoff_y', 0.0)
        self.declare_parameter('takeoff_height', 2.0)
        self.declare_parameter('takeoff_yaw_ang', 0.0)

        self.px4_id = self.get_parameter('px4_id').get_parameter_value().string_value
        self.target_system = self.get_parameter('target_system').get_parameter_value().integer_value
        self.takeoff_x = self.get_parameter('takeoff_x').get_parameter_value().double_value
        self.takeoff_y = self.get_parameter('takeoff_y').get_parameter_value().double_value
        self.takeoff_height = self.get_parameter('takeoff_height').get_parameter_value().double_value
        self.takeoff_yaw_ang = self.get_parameter('takeoff_yaw_ang').get_parameter_value().double_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        vehicle_status_topic = f'{self.px4_id}/fmu/out/vehicle_status_v1'
        vehicle_local_position_topic = f'{self.px4_id}/fmu/out/vehicle_local_position'
        vehicle_global_position_topic = f'{self.px4_id}/fmu/out/vehicle_global_position'
        vehicle_offboard_topic = f'{self.px4_id}/fmu/in/offboard_control_mode'
        vehicle_command_topic = f'{self.px4_id}/fmu/in/vehicle_command'
        vehicle_trajectory_set_topic = f'{self.px4_id}/fmu/in/trajectory_setpoint'
        vehicle_attitude_setpoint_topic = f'{self.px4_id}/fmu/in/vehicle_attitude_setpoint'
        vehicle_rates_setpoint_topic = f'{self.px4_id}/fmu/in/vehicle_rates_setpoint'
        vehicle_gps_threat_position_topic = f'{self.px4_id}/threat_detected'
        vehicle_gps_goto_topic = f'{self.px4_id}/fmu/in/aux_global_position'

        self.status_subscriber_ = self.create_subscription(
            VehicleStatus,
            vehicle_status_topic ,
            self.vehicle_status_callback,
            qos_profile
        )

        self.position_subscriber_ = self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self.state_callback,
            qos_profile)
        
        self.gps_position_subscriber_ = self.create_subscription(
            VehicleGlobalPosition,
            vehicle_global_position_topic,
            self.gps_position_callback,
            qos_profile)
        
        self.threat_distance_subscriber_ = self.create_subscription(
            ThreatDistance,
            '/drone_threat_distance',
            self.threat_distance_callback,
            qos_profile)
        
        self.gps_threat_subscriber_ = self.create_subscription(
            NavSatFix,
            vehicle_gps_threat_position_topic,
            self.threat_position_callback,
            qos_profile)
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, vehicle_offboard_topic, qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, vehicle_command_topic, qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, vehicle_trajectory_set_topic, qos_profile)
        self.attitude_set_publisher_ = self.create_publisher(VehicleAttitudeSetpoint, vehicle_attitude_setpoint_topic, qos_profile)
        self.rates_set_publisher_ = self.create_publisher(VehicleRatesSetpoint, vehicle_rates_setpoint_topic, qos_profile)
        self.gps_publisher_ = self.create_publisher(NavSatFix, vehicle_gps_threat_position_topic, qos_profile)
        self.moveto_pub = self.create_publisher(VehicleGlobalPosition, '/px4_1/fmu/in/aux_global_position', qos_profile)
        self.threat_distance_publisher_ = self.create_publisher(ThreatDistance, '/drone_threat_distance',qos_profile)

        # Parameters
        self.declare_parameter('radius', 2.0)

        self.radius = self.get_parameter('radius').value
        
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
        self.start_time = time.time()

        self.rate = 20
        # self.radius = 2
        self.cycle_s = 20
        self.steps = self.cycle_s * self.rate
        self.path = []
        self.path_generated = False
        self.mission_completed = False
        self.step_index = 0
        self.loop_count = 0
        self.max_loops = 3
        self.gps_counter = 0
        self.is_hovering = False
        self.latitude = False
        self.longitude = False
        self.altitude = False
        self.get_position = False
        self.curr_pos_x = False
        self.curr_pos_y = False
        self.curr_pos_z = False
        self.home_lat = False
        self.home_lon = False
        self.home_alt = False
        self.home_position = False
        self.takeoff_pos_x = False
        self.takeoff_pos_y = False
        self.takeoff_pos_z = False
        self.takeoff_flag = False
        self.center = (2.0, 2.0)
        self.target_position = [2.0, 5.0, -3.0]
        self.gps_threat_position = [47.398742, 8.543577, 10.0]

        self.drone_distances = {}
        self.my_distance = None
        self.total_drones = 2
        self.moving_to_threat = False

        self.state_timer_ = self.create_timer(0.1, self.drone_state_callback)
        self.gps_timer = self.create_timer(0.1, self.publish_gps_point)

        self.get_logger().info(f"Started OffboardControlNode for {self.px4_id} with target_system {self.target_system}")
    
    def gps_position_callback(self, msg):
        if self.home_position:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_alt = msg.alt
            self.home_position = True
            # print("The home position is", self.home_lon)
        
        self.latitude = msg.lat
        self.longitude = msg.lon
    
    def threat_distance_callback(self, msg):
        # Save all distances, including own (optional)
        self.drone_distances[msg.px4_id] = msg.distance_to_threat
    
    def threat_position_callback(self, msg):
        self.threat_detected_latitude = msg.latitude
        self.threat_detected_longitude = msg.longitude
        self.threat_detected_altitude = msg.altitude
        
    def state_callback(self, msg):
        if self.takeoff_flag:
            self.takeoff_pos_x = msg.x
            self.takeoff_pos_y = msg.y
            self.takeoff_pos_z = msg.z
            self.takeoff_flag = True

        if self.get_position:
            self.curr_pos_x = msg.x
            self.curr_pos_y = msg.y
            self.curr_pos_z = msg.z
            self.get_position = True

        self.position = np.array([msg.x, msg.y, msg.z])
        if msg.v_xy_valid and msg.v_z_valid:
            self.velocity = np.array([msg.vx, msg.vy, msg.vz])
        self.acceleration = np.array([msg.ax, msg.ay, msg.az])
        # print("the cceleration is", self.acceleration)
    
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
        if self.current_state == "IDLE":
            if self.flightCheck == True:
                self.current_state = "ARMING"
                self.get_logger().info(f"ARMING THE DRONE")

        elif self.current_state == "ARMING":
            if(not(self.flightCheck)):
                self.current_state = "IDLE"
                self.get_logger().info(f"ARMING, Flight Check FAILED")

            elif (self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.offboard_setpoint_counter_ == 10):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1., param2=6.)
                self.offboardMode = True
                self.current_state = "TAKEOFF"
            self.arm()

        # Publish offboard control mode and trajectory setpoint
        elif self.current_state == "TAKEOFF":
            if(not(self.flightCheck) and not self.mission_completed):
                self.current_state = "IDLE"
                self.get_logger().info(f"TAKEOFF, Flight Check FAILED")
            else:   
                self.offboard_mode()
                self.arm()
                self.home_position = True
                self.takeoff_flag = True            
                self.takeoff(self.takeoff_x, self.takeoff_y, -self.takeoff_height, self.takeoff_yaw_ang)
                height_error = self.position[2] - self.height
                if abs(height_error) < 0.2:
                    self.current_state = "LOITER"
        
        elif self.current_state == "LOITER":
            if(not(self.flightCheck)):
                self.current_state = "IDLE"
                self.get_logger().info(f"LOITER, Flight Check FAILED")
            else:
                self.current_state = "OFFBOARD"
                self.get_logger().info(f"LOITER, OFFBOARD")
            self.arm()
        
        # elif self.current_state == "OFFBOARD":
        #     if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
        #         self.current_state = "IDLE"
        #         self.get_logger().info(f"Offboard, Flight Check Failed")
        #     self.offboard_mode()
        
        elif self.current_state == "OFFBOARD":
            if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                self.current_state = "IDLE"
                self.get_logger().info(f"OFFBOARD, Flight Check FAILED")
            else:
                self.offboard_mode()
                if(not self.path_generated):
                    self.get_logger().info("I AM EXECUTING THE PATH") 
                    self.trajectory_profile(8.0)
                    self.iter = 0               
                    self.trajectory_timer_ = self.create_timer(1 / self.rate, self.offboard_trajectory_callback)
                    self.path_generated = True

            self.mission_completed = True

        # Stop the counter after reaching 11
        if self.offboard_setpoint_counter_ < 10 and not self.mission_completed:
            self.offboard_mode()
            self.offboard_setpoint_counter_ += 1
    
    def offboard_trajectory_callback(self):
        if self.is_hovering:
            self.get_logger().info("Evaluating distance to threat...")

            pos_x, pos_y, pos_z = gps_to_ned(
                self.home_lat, self.home_lon, self.home_alt,
                self.threat_detected_latitude,
                self.threat_detected_longitude,
                self.home_alt + self.threat_detected_altitude
            )

            print("The position is at", pos_x)
            print("The position is at", pos_y)
            print("The position is at", pos_z)

            my_distance = math.sqrt(
                (pos_x - self.curr_pos_x)**2 +
                (pos_y - self.curr_pos_y)**2 +
                (pos_z - self.curr_pos_z)**2
            )
            self.my_distance = my_distance
            self.drone_distances[self.px4_id] = my_distance

            msg = ThreatDistance()
            msg.px4_id = self.px4_id
            msg.distance_to_threat = my_distance
            self.threat_distance_publisher_.publish(msg)

            if len(self.drone_distances) < self.total_drones:
                self.get_logger().info("Waiting for all drones to report distance...")
                return

            closest_drone = min(self.drone_distances, key=self.drone_distances.get)

            if closest_drone == self.px4_id:
                self.moving_to_threat = True
                self.get_logger().info("I'm the closest to the threat. Moving to target...")
            else:
                self.moving_to_threat = False
                self.get_logger().info(f"{closest_drone} is closer. Continuing normal mission.")

            self.is_hovering = False  # Exit hovering mode, resume normal callback
            return

        # === Handle threat interception if assigned ===
        # if self.moving_to_threat:
        #     self.get_logger().info("I am moving to the target point.")
        #     pos_x, pos_y, pos_z = gps_to_ned(
        #         self.home_lat, self.home_lon, self.home_alt,
        #         self.threat_detected_latitude,
        #         self.threat_detected_longitude,
        #         self.home_alt + self.threat_detected_altitude
        #     )

        #     yaw = math.atan2(pos_y - self.curr_pos_y, pos_x - self.curr_pos_x)

        #     # 🟡 Directly publish a setpoint (safe even if threat_detection also does it)
        #     setpoint = TrajectorySetpoint()
        #     setpoint.position = [pos_x, pos_y, pos_z]
        #     setpoint.yaw = yaw
        #     self.trajectory_setpoint_publisher_.publish(setpoint)

        #     # You can still call your helper if needed
        #     # self.threat_detection(pos_x, pos_y, pos_z, yaw)

        #     pos_error = math.sqrt(
        #         (self.curr_pos_x - pos_x)**2 +
        #         (self.curr_pos_y - pos_y)**2 +
        #         (self.curr_pos_z - pos_z)**2
        #     )
        #     print("The error is at", pos_error)
        #     if pos_error < 0.2:
        #         self.get_logger().info("Reached threat position. Switching to LOITER.")
        #         self.current_state = "LOITER"
        #         self.moving_to_threat = False
        #     return

        # === Handle threat interception if assigned ===
        if self.moving_to_threat:
            self.home_position = True
            print("The home altitude is ", self.home_alt)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_ORBIT, param1=8.0, param2=1.0, param3=0.0, param5=47.398742, param6=8.543577, param7 = 500.0)
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LOITER_TIME, param1=30.0, param2=0.0, param3=15.0, param5=47.398742, param6=8.543577, param7 = 500.0)
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

        # === Continue sinusoidal mission ===
        if self.iter < len(self.path):
            self.trajectory_setpoint_publisher_.publish(self.path[self.iter])
        self.iter += 1

        if self.iter >= len(self.path):
            self.get_logger().info("All loops completed. Preparing to land.")
            self.trajectory_timer_.cancel()
            self.mission_completed = True
            self.current_state = "COMPLETED"
            self.land()
            self.disarm()
    
    def arm(self):
        """Send a command to arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # self.get_logger().info("Arm command sent")
    
    def disarm(self):
        """Send a command to arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        # self.get_logger().info("Arm command sent")
    
    def offboard_mode(self):
        """Publish the offboard control mode message"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(time.time() * 1e9) // 1000
        self.offboard_control_mode_publisher_.publish(msg)
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
    
    def takeoff(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
    
    def publish_gps_point(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'
    
        # Example GPS coordinates
        msg.latitude = self.gps_threat_position[0]
        msg.longitude = self.gps_threat_position[1] 
        msg.altitude = self.gps_threat_position[2]

        msg.status.status = 0
        msg.status.service = 1 

        self.gps_publisher_.publish(msg)
        self.gps_counter += 1
        if self.gps_counter == 400:
            self.is_hovering = True
        # self.get_logger().info(f'Published GPS point: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')
    
    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("I AM LANDING")
    
    def trajectory_profile(self, A):
        # A = 8.0           # Amplitude (radius)
        L = 100.0         # Total x-axis path length
        N = 50            # Number of sine wave periods
        v_max = 0.5       # Max speed in m/s

        dt = 1.0 / self.rate
        step_distance = v_max * dt  # = 0.025 m
        self.steps = int(L / step_distance)

        k = 2 * math.pi * N / L     # Wave number
        dx = L / self.steps         # Step size in x

        for i in range(self.steps):
            msg = TrajectorySetpoint()

            # Shift x and y by starting point
            x_local = i * dx
            x = self.takeoff_x + x_local
            y = self.takeoff_y + A * math.sin(k * x_local)

            dy_dx = A * k * math.cos(k * x_local)
            d2y_dx2 = -A * k * k * math.sin(k * x_local)

            # Position
            msg.position = [x, y, -self.takeoff_height]

            # Velocity vector
            direction_norm = math.sqrt(1 + dy_dx**2)
            vx = v_max / direction_norm
            vy = dy_dx * vx
            msg.velocity = [vx, vy, 0.0]

            # Acceleration
            ax = 0.0
            ay = d2y_dx2 * vx * vx
            msg.acceleration = [ax, ay, 0.0]

            msg.yaw = 0.0

            self.path.append(msg)
        
            # self.get_logger().info("Generating trajectory path...")
    
    def threat_detection(self, pos_x, pos_y, pos_z, yaw):
        new_setpoint = TrajectorySetpoint()
        new_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        new_setpoint.position = [pos_x, pos_y, pos_z]
        new_setpoint.velocity = [0.0, 0.0, 0.0]
        new_setpoint.acceleration = [0.0, 0.0, 0.0]
        new_setpoint.yaw = yaw

        self.trajectory_setpoint_publisher_.publish(new_setpoint)
        # self.get_logger().info('Sending TrajectorySetpoint: %s' % str([pos_x, pos_y, pos_z]))
    
    def threat_detection_gps_point(self):
        msg = VehicleGlobalPosition()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.lat = self.gps_threat_position[0]
        msg.lon = self.gps_threat_position[1]
        # msg.alt = self.altitude + self.gps_threat_position[2]

        self.moveto_pub.publish(msg)
        self.get_logger().info("Sent GOTO GPS position")


    
def main(args=None):
    rclpy.init(args=args)
    offboard_control_node = TrajectoryControl()
    rclpy.spin(offboard_control_node)
    offboard_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()