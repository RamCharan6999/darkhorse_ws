#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

# Optional: use your custom message if available
try:
    from vehicle_msgs.msg import ControlCmd  # fields: throttle_pct, brake_pct, steering_pct
    HAS_CONTROLCMD = True
except Exception:
    HAS_CONTROLCMD = False

# CAN (python-can) + SocketCAN
import can

def clamp(v, lo, hi): return max(lo, min(hi, v))

class HALNode(Node):
    def __init__(self):
        super().__init__('hal_node')

        # ----------------- Parameters -----------------
        # CAN
        self.declare_parameter('can_channel', 'can1')
        self.declare_parameter('ros_only', False)
        self.declare_parameter('send_hz', 50.0)
        self.declare_parameter('heartbeat_hz', 5.0)
        self.declare_parameter('failsafe_ms', 250)

        # Joystick mapping (change to match your gamepad)
        # axes[throttle_axis] in [-1..+1]; axes[steer_axis] in [-1..+1]
        self.declare_parameter('throttle_axis', 1)  # forward on +1? (TeleopTwistJoy uses 1 by default)
        self.declare_parameter('steer_axis',    2)  # yaw stick
        self.declare_parameter('brake_axis',   -1)  # -1 = disabled (use trigger differently if you have one)

        # deadzones and scales
        self.declare_parameter('axis_deadzone', 0.05)     # ignore tiny stick noise
        self.declare_parameter('invert_throttle', False)  # set True if stick forward is -1
        self.declare_parameter('invert_steer',    False)
        self.declare_parameter('invert_brake',    False)

        # read params
        self.can_channel = self.get_parameter('can_channel').get_parameter_value().string_value
        self.ros_only    = self.get_parameter('ros_only').get_parameter_value().bool_value
        self.send_dt     = 1.0 / self.get_parameter('send_hz').get_parameter_value().double_value
        self.hb_dt       = 1.0 / self.get_parameter('heartbeat_hz').get_parameter_value().double_value
        self.failsafe_ms = int(self.get_parameter('failsafe_ms').get_parameter_value().integer_value)

        self.throttle_axis = int(self.get_parameter('throttle_axis').get_parameter_value().integer_value)
        self.steer_axis    = int(self.get_parameter('steer_axis').get_parameter_value().integer_value)
        self.brake_axis    = int(self.get_parameter('brake_axis').get_parameter_value().integer_value)
        self.deadzone      = float(self.get_parameter('axis_deadzone').get_parameter_value().double_value)
        self.inv_T         = bool(self.get_parameter('invert_throttle').get_parameter_value().bool_value)
        self.inv_S         = bool(self.get_parameter('invert_steer').get_parameter_value().bool_value)
        self.inv_B         = bool(self.get_parameter('invert_brake').get_parameter_value().bool_value)

        # ----------------- State -----------------
        self.cmd = {'throttle': 0.0, 'brake': 100.0, 'steering': 50.0}  # safe defaults (idle + brake)
        self.last_input_time = time.monotonic()
        self.hb_counter = 0

        # ----------------- ROS I/O -----------------
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)

        # Monitor pubs (and optional ControlCmd)
        self.pub_throttle = self.create_publisher(Float32, '/throttle_pct', 10)
        self.pub_brake    = self.create_publisher(Float32, '/brake_pct', 10)
        self.pub_steering = self.create_publisher(Float32, '/steering_pct', 10)
        if HAS_CONTROLCMD:
            self.pub_control = self.create_publisher(ControlCmd, '/vehicle_control', 10)
        else:
            self.pub_control = None
            self.get_logger().warn("vehicle_msgs/ControlCmd not found. Publishing only /throttle_pct, /brake_pct, /steering_pct.")

        # ----------------- CAN -----------------
        self.bus: Optional[can.Bus] = None
        if not self.ros_only:
            try:
                self.bus = can.interface.Bus(channel=self.can_channel, bustype='socketcan')
                self.get_logger().info(f"✅ CAN open on {self.can_channel}")
            except Exception as e:
                self.get_logger().warn(f"⚠️ CAN not available ({e}). Running ROS-only.")
                self.ros_only = True
        else:
            self.get_logger().warn("ROS-only mode: not opening CAN.")

        # ----------------- Timers -----------------
        self.send_timer = self.create_timer(self.send_dt, self.on_send_timer)
        self.hb_timer   = self.create_timer(self.hb_dt,   self.on_heartbeat)

        self.get_logger().info("✅ HAL (direct joystick → 0–100% → CAN) started")

    # --------- Joystick callback ---------
    def cb_joy(self, msg: Joy):
        ax = msg.axes

        # Helper to fetch axis safely
        def get_axis(i, invert=False):
            if i < 0 or i >= len(ax): 
                return 0.0
            v = ax[i]
            if abs(v) < self.deadzone:
                v = 0.0
            if invert:
                v = -v
            return clamp(v, -1.0, 1.0)

        # Throttle: map [0..+1] to 0..100% (ignore negative; braking handles that)
        t_raw = get_axis(self.throttle_axis, self.inv_T)
        throttle_pct = clamp(max(0.0, t_raw) * 100.0, 0.0, 100.0)

        # Brake: if you have a trigger axis 0..1, change mapping accordingly.
        if self.brake_axis >= 0:
            b_raw = get_axis(self.brake_axis, self.inv_B)  # in [-1..+1]
            # Example: use positive side as brake (adjust if your trigger is different)
            brake_pct = clamp(max(0.0, b_raw) * 100.0, 0.0, 100.0)
        else:
            # No dedicated brake axis → treat negative throttle as brake
            brake_pct = clamp(max(0.0, -t_raw) * 100.0, 0.0, 100.0)

        # Steering: map [-1..+1] to 0..100%, center 50%
        s_raw = get_axis(self.steer_axis, self.inv_S)     # [-1..+1]
        steering_pct = clamp((s_raw + 1.0) * 50.0, 0.0, 100.0)

        # Update state
        self.cmd['throttle'] = throttle_pct
        self.cmd['brake']    = brake_pct
        self.cmd['steering'] = steering_pct
        self.last_input_time = time.monotonic()

        # Publish for visibility each time we get joystick input
        self.publish_cmds(throttle_pct, brake_pct, steering_pct)

    # --------- Periodic send to CAN (and failsafe) ---------
    def on_send_timer(self):
        # Failsafe if joystick stale
        age_ms = (time.monotonic() - self.last_input_time) * 1000.0
        if age_ms > self.failsafe_ms:
            t, b, s = 0.0, 100.0, 50.0
        else:
            t = self.cmd['throttle']; b = self.cmd['brake']; s = self.cmd['steering']

        # Re-publish for monitors (even if no new joy)
        self.publish_cmds(t, b, s)

        # Send 3 CAN IDs (0–100 encoded as u8)
        if self.bus:
            try:
                self.bus.send(can.Message(arbitration_id=0x100, is_extended_id=False, data=bytes([int(round(clamp(t,0,100)))])))
                self.bus.send(can.Message(arbitration_id=0x101, is_extended_id=False, data=bytes([int(round(clamp(s,0,100)))])))
                self.bus.send(can.Message(arbitration_id=0x102, is_extended_id=False, data=bytes([int(round(clamp(b,0,100)))])))
            except Exception as e:
                self.get_logger().error(f"CAN send failed: {e}")

    def publish_cmds(self, t, b, s):
        self.pub_throttle.publish(Float32(data=float(t)))
        self.pub_brake.publish(Float32(data=float(b)))
        self.pub_steering.publish(Float32(data=float(s)))
        if self.pub_control:
            msg = ControlCmd()
            msg.throttle_pct = float(t)
            msg.brake_pct    = float(b)
            msg.steering_pct = float(s)
            self.pub_control.publish(msg)

    # --------- Heartbeat ---------
    def on_heartbeat(self):
        if self.bus:
            try:
                self.bus.send(can.Message(arbitration_id=0x1FF, is_extended_id=False, data=bytes([self.hb_counter & 0xFF])))
                self.hb_counter = (self.hb_counter + 1) & 0xFF
            except Exception as e:
                self.get_logger().warn(f"HB send failed: {e}")

def main():
    rclpy.init()
    node = HALNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

