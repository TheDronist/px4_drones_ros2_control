import rclpy
import rclpy.clock
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleAttitudeSetpoint, VehicleRatesSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition, VehicleStatus, VehicleLocalPositionSetpoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion

import time
import numpy as np
import math
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
        super().__init__('mission_example_2')

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
            self.pose_state_callback,
            qos_profile)
        
        self.gps_position_subscriber_ = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.gps_position_callback,
            qos_profile)
        
        self.gps_threat_subscriber_ = self.create_subscription(
            NavSatFix,
            '/threat_detected',
            self.threat_position_callback,
            qos_profile)
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.attitude_set_publisher_ = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.rates_set_publisher_ = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.publisher_ = self.create_publisher(VehicleLocalPositionSetpoint, '/fmu/in/vehicle_local_position_setpoint', qos_profile)
        self.gps_publisher_ = self.create_publisher(NavSatFix, '/threat_detected', qos_profile)
        
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
        self.takeoff_yaw = (90.0) * math.pi / 180.0
        self.altitude = -2.0

        self.start_iteration = None
        self.start_x = None
        self.start_y = None
        self.first_translation = None
        self.iter = 0
        self.start_time = time.time()

        self.rate = 20
        self.periods = 3
        self.cycle_s = 8
        self.radius = 2
        self.cycle_s = 8
        self.gps_counter = 0
        # self.steps = self.cycle_s * 20
        self.path = []
        self.heartbeat_sent = False
        self.path_generated = False
        self.mission_completed = False
        self.get_position = False
        self.gps_threat_position = [47.398742, 8.543577, 10.0]
        self.is_hovering = False
        self.home_lat = False
        self.home_lon = False
        self.home_alt = False
        self.home_position = False
        self.curr_pos_x = False
        self.curr_pos_y = False
        self.curr_pos_z = False

        self.state_timer_ = self.create_timer(0.1, self.drone_state_callback)
        self.gps_timer = self.create_timer(0.1, self.publish_gps_point)
    
    def gps_position_callback(self, msg):
        if self.home_position:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_alt = msg.alt
            self.home_position = True
            # print("The home position is", self.home_lon)
        
        self.latitude = msg.lat
        self.longitude = msg.lon
    
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
    
    def threat_position_callback(self, msg):
        self.threat_detected_latitude = msg.latitude
        self.threat_detected_longitude = msg.longitude
        self.threat_detected_altitude = msg.altitude
    
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
        if self.gps_counter == 600:
            self.is_hovering = True
        # self.get_logger().info(f'Published GPS point: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')
    
    def pose_state_callback(self, msg):
        if self.get_position:
            self.curr_pos_x = msg.x
            self.curr_pos_y = msg.y
            self.curr_pos_z = msg.z
            self.get_position = True

        self.position = np.array([msg.x, msg.y, msg.z])
        if msg.v_xy_valid and msg.v_z_valid:
            self.velocity = np.array([msg.vx, msg.vy, msg.vz])
        self.acceleration = np.array([msg.ax, msg.ay, msg.az])
    
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)
        self.get_logger().info(f"HEARTBEAT: I AM ALIVE")
    
    def drone_state_callback(self):
        if not self.heartbeat_sent:
            self.publish_offboard_control_heartbeat_signal()
            self.heartbeat_sent = True
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
                self.takeoff(0.0, 0.0, self.altitude, self.takeoff_yaw)
                height_error = self.position[2] - self.altitude
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
        
        elif self.current_state == "OFFBOARD":
            if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                self.current_state = "IDLE"
                self.get_logger().info(f"OFFBOARD, Flight Check FAILED")
            else:
                self.offboard_mode()
                if(not self.path_generated):
                    self.get_logger().info("I AM EXECUTING THE PATH") 
                    self.trajectory_profile()
                    self.iter = 0               
                    self.trajectory_timer_ = self.create_timer(1 / self.rate, self.offboard_trajectory_callback)
                    self.path_generated = True
                
            self.mission_completed = True
            # print("The mode", self.offboardMode)
            # print("The flightCheck", self.flightCheck)
            # print("The arming mode", self.arm_state)
            # print("The navigating mode", self.nav_state)

        # Stop the counter after reaching 11
        if self.offboard_setpoint_counter_ < 10 and not self.mission_completed:
            self.offboard_mode()
            self.offboard_setpoint_counter_ += 1
    
    def offboard_trajectory_callback(self):
        if self.is_hovering:
            self.get_logger().info("Move to the threat GPS position .")
            self.trajectory_timer_.cancel()
            self.get_position = True
            pos_x, pos_y, pos_z = gps_to_ned(self.home_lat, self.home_lon, self.home_alt, self.threat_detected_latitude,
                                             self.threat_detected_longitude, self.home_alt + self.threat_detected_altitude)
            
            yaw = math.atan2(pos_y - self.curr_pos_y, pos_x - self.curr_pos_x)
            self.threat_detection(pos_x, pos_y, pos_z, yaw)
            pos_error = self.position[0] - pos_x
            if abs(pos_error) < 0.2:
                self.current_state = "LOITER"
            return

        if(self.iter < len(self.path)):
            self.trajectory_setpoint_publisher_.publish(self.path[self.iter])

        self.iter += 1

        if(self.iter >= len(self.path)):
            self.takeoff(0.0, 0.0, self.altitude, self.takeoff_yaw)      
        
        if(self.iter == len(self.path) + 100):
            self.trajectory_timer_.cancel()
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
        msg.timestamp = int(time.time() * 1e9) // 1000  # Convert time to microseconds
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
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
    
    def takeoff(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
    
    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("I AM LANDING")
    
    def trajectory_profile(self):
        A = 8.0           # Amplitude (radius)
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

            x = i * dx
            y = A * math.sin(k * x)

            dy_dx = A * k * math.cos(k * x)
            d2y_dx2 = -A * k * k * math.sin(k * x)

            # Position
            msg.position = [x, y, self.altitude]

            # Correct speed vector to match v_max
            direction_norm = math.sqrt(1 + dy_dx**2)
            vx = v_max / direction_norm
            vy = dy_dx * vx
            msg.velocity = [vx, vy, 0.0]

            ax = 0.0
            ay = d2y_dx2 * vx * vx
            msg.acceleration = [ax, ay, 0.0]

            msg.yaw = 0.0    # OR math.atan2(vy, vx)

            self.path.append(msg)

        # # Yaw speed calculation
        # for i in range(self.steps):
        #     next_yaw = self.path[(i + 1) % self.steps].yaw
        #     curr = self.path[i].yaw
        #     if next_yaw - curr < -math.pi:
        #         next_yaw += 2.0 * math.pi
        #     if next_yaw - curr > math.pi:
        #         next_yaw -= 2.0 * math.pi

        #     self.path[i].yawspeed = (next_yaw - curr) * self.rate

        self.get_logger().info(
            f"Generated sinusoidal path: {N} periods, {L}m long, radius {A}m, v_max {v_max} m/s, fixed rate {self.rate} Hz"
        )
    
    def threat_detection(self, pos_x, pos_y, pos_z, yaw):
        new_setpoint = TrajectorySetpoint()
        new_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        new_setpoint.position = [pos_x, pos_y, pos_z]
        new_setpoint.velocity = [0.0, 0.0, 0.0]
        new_setpoint.acceleration = [0.0, 0.0, 0.0]
        new_setpoint.yaw = yaw

        self.trajectory_setpoint_publisher_.publish(new_setpoint)
        self.get_logger().info('Sending TrajectorySetpoint: %s' % str([pos_x, pos_y, pos_z]))


    
def main(args=None):
    rclpy.init(args=args)
    offboard_control_node = TrajectoryControl()
    rclpy.spin(offboard_control_node)
    offboard_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
