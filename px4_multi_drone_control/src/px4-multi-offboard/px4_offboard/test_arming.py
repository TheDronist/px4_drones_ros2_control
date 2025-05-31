import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class VOXL_Offboard(Node):
    def __init__(self):
        super().__init__('voxl_offboard')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parameters
        self.declare_parameter('px4_id', 'px4_1')
        self.declare_parameter('target_system', 2)

        self.px4_id = self.get_parameter('px4_id').get_parameter_value().string_value
        self.target_system = self.get_parameter('target_system').get_parameter_value().integer_value

        vehicle_offboard_topic = f'{self.px4_id}/fmu/in/offboard_control_mode'
        vehicle_command_topic = f'{self.px4_id}/fmu/in/vehicle_command'

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, vehicle_offboard_topic, 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, vehicle_command_topic, 10)

        # Timer for periodic publishing (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Initial positions and flags
        self.offboard_setpoint_counter = 0
        self.is_armed = False

    def timer_callback(self):

        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 99:
           self.set_offboard_mode()
           self.arm()
           self.armed = True

        # First 100 iterations for mode switching
        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1
    
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

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # PX4 offboard mode

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

def main(args=None):
    rclpy.init(args=args)
    node = VOXL_Offboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()