#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import can
import cantools
import os
from ament_index_python.packages import get_package_share_directory

class HardwareAbstractionLayer(Node):
    def __init__(self):
        super().__init__('hal_node')
        self.get_logger().info("✅ Darkhorse HAL Node Started")

        # Declare a parameter to control if the radar bus should be used
        self.declare_parameter('dbw_only', False)
        self.dbw_only_mode = self.get_parameter('dbw_only').get_parameter_value().bool_value

        # --- Locate and Load the DBC File ---
        if not self.dbw_only_mode:
            try:
                db_path = os.path.join(
                    get_package_share_directory('darkhorse_dbw'),
                    'config',
                    'Starkenn_Orion_Radar_x_aBAJA.dbc'
                )
                self.radar_db = cantools.database.load_file(db_path)
                self.get_logger().info("DBC file loaded successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to load DBC file: {e}")
                self.radar_db = None
                # This should not shut down the node, as we need it for DBW
                # rclpy.shutdown()
                # return
        else:
            self.get_logger().info("Running in DBW-only mode. Skipping DBC loading.")
            self.radar_db = None

        # --- Initialize CAN Buses with error handling ---
        self.can_bus_available = False
        try:
            self.dbw_bus = can.interface.Bus(channel='can1', bustype='socketcan')
            if not self.dbw_only_mode:
                self.radar_bus = can.interface.Bus(channel='can2', bustype='socketcan')
                self.get_logger().info("SocketCAN Buses Initialized (can1 for DBW, can2 for Radar)")
            else:
                self.get_logger().info("SocketCAN Bus Initialized (can1 for DBW only)")
            self.can_bus_available = True
        except Exception as e:
            self.get_logger().warn(f"CAN buses not available: {e}. Running in a non-functional mode.")
            self.can_bus_available = False

        # === Publishers ===
        self.speed_pub = self.create_publisher(Float32, '/VehicleSpeed', 10)
        self.raw_can_pub = self.create_publisher(String, '/from_can_bus', 10)

        # === Subscribers ===
        self.create_subscription(Twist, '/vehicle_control', self.control_callback, 10)
        
        # === Timers ===
        if self.can_bus_available:
            self.can_read_timer = self.create_timer(0.01, self.can_read_callback)
            if not self.dbw_only_mode:
                self.radar_feed_timer = self.create_timer(0.1, self.feed_radar_callback)
        else:
            self.get_logger().warn("CAN bus timers disabled. Connect hardware to enable.")

        self.current_speed_mps = 0.0

    def control_callback(self, msg: Twist):
        speed_cmd = msg.linear.x
        turn_cmd = msg.angular.z
        
        throttle, brake, steer = self.translate_command(speed_cmd, turn_cmd)
        
        if self.can_bus_available:
            self.send_dbw_commands(throttle, steer, brake)
        else:
            self.get_logger().info(f"Throttle: {throttle}%, Brake: {brake}%, Steering: {steer}")

    def can_read_callback(self):
        if self.can_bus_available:
            dbw_msg = self.dbw_bus.recv(timeout=0.0)
            
            if not self.dbw_only_mode:
                radar_msg = self.radar_bus.recv(timeout=0.0)
                if radar_msg is not None and self.radar_db:
                    can_id = radar_msg.arbitration_id
                    can_data = radar_msg.data.hex()
                    string_msg = String()
                    string_msg.data = f"{can_id}:{can_data}"
                    self.raw_can_pub.publish(string_msg)
            
            if dbw_msg is not None:
                # TODO: Add your DBW feedback decoding and publishing logic here
                pass

    def feed_radar_callback(self):
        if self.can_bus_available and not self.dbw_only_mode and self.radar_db:
            try:
                speed_kmph = self.current_speed_mps * 3.6
                msg_def = self.radar_db.get_message_by_name('Vehicle_Data_Speed')
                data = {'Vehicle_Data_Speed': int(speed_kmph / msg_def.signals[0].scale)}
                encoded_data = msg_def.encode(data)
                can_msg = can.Message(arbitration_id=msg_def.frame_id, data=encoded_data, is_extended_id=False)
                self.radar_bus.send(can_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to send speed to radar: {e}")
    
    def translate_command(self, speed, turn):
        throttle = int(max(0, speed) * 100)
        brake = int(max(0, -speed) * 100)
        steer = int(turn * 700)
        return throttle, brake, steer

    def send_dbw_commands(self, throttle, steer, brake):
        try:
            throttle_msg = can.Message(arbitration_id=0x100, data=[throttle])
            self.dbw_bus.send(throttle_msg)
            
            brake_msg = can.Message(arbitration_id=0x101, data=[brake])
            self.dbw_bus.send(brake_msg)
            
            steer_msg = can.Message(arbitration_id=0x102, data=[(steer >> 8) & 0xFF, steer & 0xFF])
            self.dbw_bus.send(steer_msg)

        except can.CanError as e:
            self.get_logger().error(f"Failed to send DBW command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HardwareAbstractionLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

