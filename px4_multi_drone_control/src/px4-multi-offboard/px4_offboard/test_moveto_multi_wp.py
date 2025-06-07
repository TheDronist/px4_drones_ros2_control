import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, TrajectorySetpoint
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import time

class VOXL_Offboard(Node):
    def __init__(self):
        super().__init__('test_moveto_wp')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

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


        vehicle_local_position_topic = f'{self.px4_id}/fmu/out/vehicle_local_position'

        vehicle_offboard_topic = f'{self.px4_id}/fmu/in/offboard_control_mode'
        vehicle_command_topic = f'{self.px4_id}/fmu/in/vehicle_command'
        vehicle_trajectory_set_topic = f'{self.px4_id}/fmu/in/trajectory_setpoint'

        # Subscribers
        self.position_subscriber_ = self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self.position_state_callback,
            qos_profile)

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, vehicle_offboard_topic, qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, vehicle_command_topic, qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,vehicle_trajectory_set_topic, qos_profile)

        # Timer for periodic publishing (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Initial positions and flags
        self.offboard_setpoint_counter = 0
        self.armed = False
        self.loiter = False
        self.disarmed = False
        self.landed = False

        # Waypoint management
        self.waypoints = [
            [1.0, 1.0, self.takeoff_height],
            [2.0, 0.0, self.takeoff_height],
            [1.0, -1.0, self.takeoff_height],
            [0.0, 0.0, self.takeoff_height]
        ]
        self.current_wp_index = 0
        self.wp_reached = False

    def timer_callback(self):

        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 99:
           self.set_offboard_mode()
           self.arm()
           self.armed = True

        # First 100 iterations for mode switching
        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1
        
        if self.armed == True:
            self.takeoff_setpoint(self.takeoff_x, self.takeoff_y, self.takeoff_height, self.takeoff_yaw_ang)
            height_error = self.position[2] - self.takeoff_height
            if abs(height_error) < 0.2:
                self.loiter = True
        
        if self.loiter:
            if self.current_wp_index >= len(self.waypoints):
                self.get_logger().info('All waypoints reached.')
                self.land()
                self.disarm()
                return

            # Get current waypoint
            target_wp = self.waypoints[self.current_wp_index]
            pos_x, pos_y, pos_z = target_wp
            yaw = self.takeoff_yaw_ang

            # Move to the waypoint
            self.moveto(pos_x, pos_y, pos_z, yaw)

            # Compute distance to current waypoint
            wp_pos = np.array([pos_x, pos_y, pos_z])
            distance = np.linalg.norm(self.position - wp_pos)

            self.get_logger().info(f'Moving to waypoint {self.current_wp_index}: {target_wp} (distance {distance:.2f})')

            if distance < 0.3:  # Reached
                self.get_logger().info(f'Waypoint {self.current_wp_index} reached.')
                self.current_wp_index += 1
    
    def position_state_callback(self, msg):
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
        self.offboard_mode_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
    
    def takeoff_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
    
    def moveto(self, pos_x, pos_y, pos_z, yaw):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        setpoint.position = [pos_x, pos_y, pos_z]
        setpoint.velocity = [0.0, 0.0, 0.0]
        setpoint.acceleration = [0.0, 0.0, 0.0]
        setpoint.yaw = yaw

        self.trajectory_setpoint_publisher_.publish(setpoint)
        self.get_logger().info('Sending TrajectorySetpoint: %s' % str([pos_x, pos_y, pos_z]))

def main(args=None):
    rclpy.init(args=args)
    node = VOXL_Offboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()