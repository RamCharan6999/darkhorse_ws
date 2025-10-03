#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Bool, String


class AEBController(Node):
    """
    Automatic Emergency Braking (AEB) controller.

    Inputs
    ------
    /radar/ttc : Float32         (seconds; +inf when safe)
    /VehicleSpeed : Float32      (m/s; from HAL)

    Outputs
    -------
    /brake_pct : Float32         (0.0 .. 1.0)
    /throttle_pct : Float32      (0.0 .. 1.0; set to 0.0 while braking)
    /aeb/active : Bool
    /aeb/status : String         (human-readable state/debug)
    """

    def __init__(self):
        super().__init__('aeb_controller')

        # ---------------- Parameters (tune if needed)
        self.declare_parameter('ttc_trigger_s', 2.0)     # trigger when TTC < this
        self.declare_parameter('ttc_hard_s', 1.0)        # full-brake when TTC < this
        self.declare_parameter('speed_trigger_mps', 1.0) # ignore AEB when practically stopped
        self.declare_parameter('target_decel_mps2', 6.0) # desired decel during AEB
        self.declare_parameter('brake_k_per_mps2', 0.12) # linear map: brake = k * decel
        self.declare_parameter('min_brake_pct', 0.15)    # floor once triggered
        self.declare_parameter('ramp_up_pct_per_s', 1.5) # slew limit up (0..1 per second)
        self.declare_parameter('ramp_down_pct_per_s', 2.0)# slew limit down
        self.declare_parameter('hold_brake_at_stop', True)
        self.declare_parameter('stop_speed_mps', 0.3)    # consider stopped below this
        self.declare_parameter('release_after_stop_s', 2.0)  # keep holding after stop
        self.declare_parameter('ttc_fresh_timeout_s', 0.5)   # if no fresh TTC, don’t re-trigger
        self.declare_parameter('speed_fresh_timeout_s', 0.5) # if no fresh speed, be conservative
        self.declare_parameter('ttc_topic', '/radar/ttc')
        self.declare_parameter('speed_topic', '/VehicleSpeed')
        self.declare_parameter('brake_topic', '/brake_pct')
        self.declare_parameter('throttle_topic', '/throttle_pct')
        self.declare_parameter('control_rate_hz', 50.0)

        # Pull params
        self.ttc_trigger_s = float(self.get_parameter('ttc_trigger_s').value)
        self.ttc_hard_s = float(self.get_parameter('ttc_hard_s').value)
        self.speed_trigger_mps = float(self.get_parameter('speed_trigger_mps').value)
        self.target_decel = float(self.get_parameter('target_decel_mps2').value)
        self.brake_k = float(self.get_parameter('brake_k_per_mps2').value)
        self.min_brake = float(self.get_parameter('min_brake_pct').value)
        self.ramp_up = float(self.get_parameter('ramp_up_pct_per_s').value)
        self.ramp_down = float(self.get_parameter('ramp_down_pct_per_s').value)
        self.hold_at_stop = bool(self.get_parameter('hold_brake_at_stop').value)
        self.stop_speed = float(self.get_parameter('stop_speed_mps').value)
        self.release_after_stop_s = float(self.get_parameter('release_after_stop_s').value)
        self.ttc_timeout = float(self.get_parameter('ttc_fresh_timeout_s').value)
        self.speed_timeout = float(self.get_parameter('speed_fresh_timeout_s').value)

        self.ttc_topic = self.get_parameter('ttc_topic').value
        self.speed_topic = self.get_parameter('speed_topic').value
        self.brake_topic = self.get_parameter('brake_topic').value
        self.throttle_topic = self.get_parameter('throttle_topic').value
        self.dt = 1.0 / float(self.get_parameter('control_rate_hz').value)

        # ---------------- Subscriptions
        self._ttc = math.inf
        self._ttc_stamp = self.get_clock().now()
        self._speed = 0.0
        self._speed_stamp = self.get_clock().now()

        self.sub_ttc = self.create_subscription(
            Float32, self.ttc_topic, self._on_ttc, qos_profile_sensor_data
        )
        self.sub_speed = self.create_subscription(
            Float32, self.speed_topic, self._on_speed, qos_profile_sensor_data
        )

        # ---------------- Publishers
        self.pub_brake = self.create_publisher(Float32, self.brake_topic, 10)
        self.pub_throttle = self.create_publisher(Float32, self.throttle_topic, 10)
        self.pub_active = self.create_publisher(Bool, '/aeb/active', 10)
        self.pub_status = self.create_publisher(String, '/aeb/status', 10)

        # ---------------- Internal state
        self.state = 'IDLE'             # IDLE, BRAKING, HOLD
        self.brake_cmd = 0.0            # last command (0..1)
        self.last_change_time = time.time()
        self.time_stopped = None

        # Control loop
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f'AEB ready | ttc<{self.ttc_trigger_s}s; decel~{self.target_decel}m/s²; '
            f'publish→ {self.brake_topic}, {self.throttle_topic}'
        )

    # ---------------- Callbacks
    def _on_ttc(self, msg: Float32):
        self._ttc = float(msg.data)
        self._ttc_stamp = self.get_clock().now()

    def _on_speed(self, msg: Float32):
        self._speed = float(msg.data)
        self._speed_stamp = self.get_clock().now()

    # ---------------- Helpers
    def _fresh(self, stamp, timeout_s) -> bool:
        return (self.get_clock().now() - stamp) < Duration(seconds=timeout_s)

    def _slew(self, target: float, rate_up: float, rate_down: float) -> float:
        """Rate-limit brake command per step."""
        max_up = rate_up * self.dt
        max_dn = rate_down * self.dt
        if target > self.brake_cmd:
            return min(target, self.brake_cmd + max_up)
        else:
            return max(target, self.brake_cmd - max_dn)

    # ---------------- Main loop
    def _tick(self):
        # Freshness checks
        ttc_fresh = self._fresh(self._ttc_stamp, self.ttc_timeout)
        spd_fresh = self._fresh(self._speed_stamp, self.speed_timeout)

        ttc = self._ttc if ttc_fresh else math.inf
        v = self._speed if spd_fresh else 0.0

        # Decide desired brake level
        desired_brake = 0.0
        next_state = self.state

        if self.state == 'IDLE':
            if (v > self.speed_trigger_mps) and (ttc < self.ttc_trigger_s):
                # Trigger AEB
                desired_brake = max(self.min_brake, min(1.0, self.brake_k * self.target_decel))
                # If TTC extremely small, go full
                if ttc <= self.ttc_hard_s:
                    desired_brake = 1.0
                next_state = 'BRAKING'
                self.last_change_time = time.time()
            else:
                desired_brake = 0.0

        elif self.state == 'BRAKING':
            # Maintain/deepen braking
            desired_brake = max(self.min_brake, min(1.0, self.brake_k * self.target_decel))
            if ttc <= self.ttc_hard_s:
                desired_brake = 1.0

            # Transition to HOLD when we’re basically stopped
            if v <= self.stop_speed:
                if self.hold_at_stop:
                    desired_brake = 1.0
                    next_state = 'HOLD'
                    self.time_stopped = time.time()
                else:
                    next_state = 'IDLE'

        elif self.state == 'HOLD':
            # Keep full brake at standstill for a bit, then release when clearly safe
            desired_brake = 1.0
            stopped_long_enough = (
                self.time_stopped is not None
                and (time.time() - self.time_stopped) >= self.release_after_stop_s
            )
            safe_now = (v <= self.stop_speed) and (ttc > 5.0 or (not ttc_fresh))

            if stopped_long_enough and safe_now:
                desired_brake = 0.0
                next_state = 'IDLE'
                self.time_stopped = None

        # Rate-limit the brake command
        self.brake_cmd = self._slew(desired_brake, self.ramp_up, self.ramp_down)
        self.brake_cmd = max(0.0, min(1.0, self.brake_cmd))

        # If braking, kill throttle; otherwise do nothing (another node can manage throttle)
        throttle_cmd = 0.0 if self.brake_cmd > 0.01 else 0.0

        # Publish commands
        self.pub_brake.publish(Float32(data=float(self.brake_cmd)))
        self.pub_throttle.publish(Float32(data=float(throttle_cmd)))
        self.pub_active.publish(Bool(data=(self.state != 'IDLE' or self.brake_cmd > 0.01)))

        # Status line
        status = (
            f'state={self.state} v={v:.2f}m/s ttc={"inf" if math.isinf(ttc) else f"{ttc:.2f}s"} '
            f'brk={self.brake_cmd:.2f}'
        )
        self.pub_status.publish(String(data=status))

        # Move to next state
        self.state = next_state


def main(args=None):
    rclpy.init(args=args)
    node = AEBController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
