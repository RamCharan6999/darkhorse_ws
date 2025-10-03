#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def rate_limit(prev, target, rate_per_s, dt):
    step = clamp(target - prev, -rate_per_s*dt, rate_per_s*dt)
    return prev + step

class AEBNode(Node):
    """
    Self-contained AEB:
    - Reaches & holds ~20 km/h (5.56 m/s) using /VehicleSpeed
    - Brakes when obstacle detected from /radar/ttc (or /radar/closest fallback)
    - Publishes /acc_cmd (throttle %) and /aeb_cmd (brake %)
    """

    def __init__(self):
        super().__init__('aeb_node')

        # --- Parameters ---
        self.declare_parameter('target_speed_mps', 5.56)     # 20 km/h
        self.declare_parameter('speed_kp', 22.0)
        self.declare_parameter('speed_ki', 2.0)
        self.declare_parameter('throttle_max_pct', 60.0)
        self.declare_parameter('throttle_slew_pct_s', 60.0)
        self.declare_parameter('brake_slew_pct_s', 150.0)

        self.declare_parameter('warn_ttc_s', 2.5)
        self.declare_parameter('brake_ttc_s', 1.8)
        self.declare_parameter('full_ttc_s', 1.0)

        self.declare_parameter('min_brake_pct', 20.0)
        self.declare_parameter('brake_gain_pct_per_mps2', 14.0)
        self.declare_parameter('hold_full_s', 1.5)
        self.declare_parameter('stop_speed_eps_mps', 0.2)
        self.declare_parameter('fallback_stop_dist_m', 6.0)

        # --- Config ---
        self.v_ref = float(self.get_parameter('target_speed_mps').value)
        self.kp = float(self.get_parameter('speed_kp').value)
        self.ki = float(self.get_parameter('speed_ki').value)
        self.u_max = float(self.get_parameter('throttle_max_pct').value)
        self.thr_slew = float(self.get_parameter('throttle_slew_pct_s').value)
        self.brk_slew = float(self.get_parameter('brake_slew_pct_s').value)

        self.warn_ttc = float(self.get_parameter('warn_ttc_s').value)
        self.brk_ttc  = float(self.get_parameter('brake_ttc_s').value)
        self.full_ttc = float(self.get_parameter('full_ttc_s').value)

        self.min_brk = float(self.get_parameter('min_brake_pct').value)
        self.brk_gain = float(self.get_parameter('brake_gain_pct_per_mps2').value)
        self.hold_s = float(self.get_parameter('hold_full_s').value)
        self.v_stop = float(self.get_parameter('stop_speed_eps_mps').value)
        self.fallback_stop = float(self.get_parameter('fallback_stop_dist_m').value)

        # --- State ---
        self.v = 0.0
        self.ttc = float('inf')
        self.range_x = float('inf')
        self.acc_cmd = 0.0
        self.brk_cmd = 0.0
        self.int_err = 0.0
        self.last_t = time.time()
        self.hold_until = 0.0

        # --- Publishers ---
        self.pub_acc  = self.create_publisher(Float32, 'acc_cmd', 10)
        self.pub_brk  = self.create_publisher(Float32, 'aeb_cmd', 10)
        self.pub_dbg  = self.create_publisher(String,  'aeb_debug', 10)

        # --- Subscriptions ---
        self.create_subscription(Float32, 'VehicleSpeed', self._on_speed, 10)   # m/s
        self.create_subscription(Float32, 'radar/ttc',    self._on_ttc,   10)
        self.create_subscription(String,  'radar/closest', self._on_closest, 10)

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info("✅ AEBNode ready: speed-hold + AEB → /acc_cmd & /aeb_cmd")

    # --- Callbacks ---
    def _on_speed(self, msg: Float32):  self.v = float(msg.data)
    def _on_ttc(self, msg: Float32):    self.ttc = float(msg.data)
    def _on_closest(self, msg: String):
        try:
            if "first=(" in msg.data:
                s = msg.data.split("first=(")[1].split(")")[0]
                x_str, _y = s.split(',')
                self.range_x = float(x_str)
        except Exception:
            self.range_x = float('inf')

    # --- Control loop ---
    def _tick(self):
        t = time.time()
        dt = max(1e-3, t - self.last_t)
        self.last_t = t

        state = "SPEED_HOLD"
        des_thr, des_brk = self.acc_cmd, 0.0

        # 1) SPEED HOLD (PI control) if no risk
        if not (math.isfinite(self.ttc) and self.ttc <= self.warn_ttc):
            err = self.v_ref - self.v
            self.int_err += err * dt
            des_thr = clamp(self.kp*err + self.ki*self.int_err, 0.0, self.u_max)
            des_brk = 0.0

        # 2) TTC-based braking
        if math.isfinite(self.ttc) and self.ttc < 10.0:
            if self.ttc <= self.full_ttc:
                des_thr, des_brk = 0.0, 100.0
                self.hold_until = t + self.hold_s
                state = "FULL"
            elif self.ttc <= self.brk_ttc:
                desired_decel = max(0.0, self.v / max(1e-3, self.ttc))
                brk_from_decel = self.min_brk + self.brk_gain * desired_decel
                des_brk = clamp(brk_from_decel, self.min_brk, 100.0)
                des_thr = 0.0
                state = "BRAKE"
            elif self.ttc <= self.warn_ttc:
                state = "WARN"

        # 3) Fallback physics (range-based)
        elif math.isfinite(self.range_x) and self.v > 0.5:
            denom = max(1e-3, (self.range_x - self.fallback_stop))
            desired_decel = max(0.0, (self.v * self.v) / denom)
            if desired_decel > 0.0:
                des_brk = clamp(self.min_brk + self.brk_gain * desired_decel, self.min_brk, 100.0)
                des_thr = 0.0
                state = "PHYS"

        # 4) Hold full after stop
        if self.v <= self.v_stop and t < self.hold_until:
            des_thr, des_brk = 0.0, 100.0
            state = "HOLD"

        # 5) Slew + clamp
        self.acc_cmd = rate_limit(self.acc_cmd, des_thr, self.thr_slew, dt)
        self.brk_cmd = rate_limit(self.brk_cmd, des_brk, self.brk_slew, dt)
        self.acc_cmd = clamp(self.acc_cmd, 0.0, 100.0)
        self.brk_cmd = clamp(self.brk_cmd, 0.0, 100.0)
        if self.brk_cmd > 1.0:
            self.acc_cmd = 0.0  # safety

        # --- Publish ---
        self.pub_acc.publish(Float32(data=self.acc_cmd))
        self.pub_brk.publish(Float32(data=self.brk_cmd))
        dbg = (f"[{state}] v={self.v:.2f}m/s (→{self.v_ref:.2f}) "
               f"TTC={self.ttc:.2f}s rng={self.range_x:.1f}m | "
               f"ACC={self.acc_cmd:.1f}% BRK={self.brk_cmd:.1f}%")
        self.pub_dbg.publish(String(data=dbg))
        self.get_logger().info(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

