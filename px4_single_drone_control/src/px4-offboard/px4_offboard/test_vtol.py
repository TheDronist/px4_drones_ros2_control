import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, TrajectorySetpoint
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import time

class VOXL_Offboard(Node):
    def __init__(self):
        super().__init__('test_vtol')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.position_subscriber_ = self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self.position_state_callback,
            qos_profile)

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,'/px4_1/fmu/in/trajectory_setpoint', 10)

        # Timer for periodic publishing (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Initial positions and flags
        self.offboard_setpoint_counter = 0
        self.armed = False
        self.loiter = False
        self.disarmed = False
        self.landed = False

        self.height = -1.0
        self.takeoff_yaw = 0.0

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
            self.takeoff_setpoint(0.0, 0.0, self.height, self.takeoff_yaw)
            height_error = self.position[2] - self.height
            if abs(height_error) < 0.2:
                self.loiter = True
        
        # Land once target height is reached
        if self.loiter == True:
            self.land()
            self.arm()
    
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
        msg.target_system = 1
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

def main(args=None):
    rclpy.init(args=args)
    node = VOXL_Offboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()