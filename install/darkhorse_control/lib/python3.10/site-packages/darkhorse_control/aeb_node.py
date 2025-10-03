#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, re, time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String

_NUM_RE = re.compile(r'[-+]?\d*\.?\d+')

def _parse_first_xy(txt: str):
    # expects: "detections=N first=(x,y) m"
    try:
        parts = _NUM_RE.findall(txt)
        if len(parts) >= 2:
            return float(parts[-2]), float(parts[-1])
    except Exception:
        pass
    return None, None

class AEBNode(Node):
    def __init__(self):
        super().__init__('aeb_node')

        # -------- params (good for bench testing)
        self.declare_parameter('warn_ttc_s', 2.5)     # just print WARN
        self.declare_parameter('brake_ttc_s', 1.8)    # start braking
        self.declare_parameter('full_ttc_s', 1.0)     # go full brake
        self.declare_parameter('min_brake_pct', 50.0) # your hardware needs >=50% to move
        self.declare_parameter('hold_full_s', 2.0)    # hold 100% after trigger (video demo)
        self.declare_parameter('hz', 20.0)            # publish rate
        self.declare_parameter('lat_fov_m', 8.0)      # unused here, kept for future
        self.declare_parameter('use_radar_ttc', True) # use /radar/ttc if available

        self.warn_ttc = float(self.get_parameter('warn_ttc_s').value)
        self.brk_ttc  = float(self.get_parameter('brake_ttc_s').value)
        self.full_ttc = float(self.get_parameter('full_ttc_s').value)
        self.min_brk  = float(self.get_parameter('min_brake_pct').value)
        self.hold_s   = float(self.get_parameter('hold_full_s').value)
        self.dt       = 1.0/float(self.get_parameter('hz').value)
        self.use_ttc_topic = bool(self.get_parameter('use_radar_ttc').value)

        # state
        self.last_ttc = float('inf')
        self.last_x   = float('nan')
        self.brake_pct_cmd = 0.0
        self.hold_until = 0.0

        # pubs
        self.pub_brake = self.create_publisher(Float32, 'brake_pct', 10)
        self.pub_dbg   = self.create_publisher(String,  'aeb_debug', 10)

        # subs
        if self.use_ttc_topic:
            self.create_subscription(Float32, 'radar/ttc', self._on_ttc, 10)
        self.create_subscription(String,  'radar/closest', self._on_closest, 10)

        # timer
        self.create_timer(self.dt, self._tick)
        self.get_logger().info('AEB ready: /radar/ttc + /radar/closest -> /brake_pct')

    # --- callbacks
    def _on_ttc(self, msg: Float32):
        self.last_ttc = float(msg.data)

    def _on_closest(self, msg: String):
        x, _y = _parse_first_xy(msg.data)
        if x is not None:
            self.last_x = x

    # --- main loop
    def _tick(self):
        now = time.time()
        ttc = self.last_ttc

        # compute brake %
        if math.isfinite(ttc):
            if ttc <= self.full_ttc:
                # slam + latch
                self.brake_pct_cmd = 100.0
                self.hold_until = max(self.hold_until, now + self.hold_s)
                state = 'HOLD_FULL'
            elif ttc <= self.brk_ttc:
                # smooth ramp from min_brk @ brk_ttc to 100 @ full_ttc
                span = max(1e-3, self.brk_ttc - self.full_ttc)
                alpha = (self.brk_ttc - ttc)/span  # 0..1
                pct = self.min_brk + alpha*(100.0 - self.min_brk)
                self.brake_pct_cmd = max(self.min_brk, min(100.0, pct))
                state = 'BRAKE'
            elif ttc <= self.warn_ttc:
                self.brake_pct_cmd = 0.0
                state = 'WARN'
            else:
                self.brake_pct_cmd = 0.0
                state = 'IDLE'
        else:
            # no TTC / clear
            if now < self.hold_until:
                self.brake_pct_cmd = 100.0
                state = 'HOLD_FULL'
            else:
                self.brake_pct_cmd = 0.0
                state = 'IDLE'

        # publish every tick (so HAL always has fresh value)
        self.pub_brake.publish(Float32(data=float(self.brake_pct_cmd)))

        # debug line
        x_txt = f"{self.last_x:.1f}m" if math.isfinite(self.last_x) else "nan"
        ttc_txt = f"{ttc:.2f}s" if math.isfinite(ttc) else "inf"
        dbg = f"state={state} x={x_txt} ttc={ttc_txt} brk={int(round(self.brake_pct_cmd))}%"
        self.pub_dbg.publish(String(data=dbg))

def main(argv=None):
    rclpy.init(args=argv)
    node = AEBNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

