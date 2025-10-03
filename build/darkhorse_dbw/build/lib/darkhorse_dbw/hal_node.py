#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import can, math

class HALNode(Node):
    def __init__(self):
        super().__init__('hal_node')

        # ---- params
        self.declare_parameter('can_interface', 'can1')
        self.declare_parameter('hz', 50.0)          # CAN send rate
        self.declare_parameter('tx_id_brake', 0x102)
        self.declare_parameter('use_joy', True)     # listen to /brake_pct_joy
        self.declare_parameter('arb_mode', 'max')   # 'max' | 'aeb' | 'joy'

        self.iface = self.get_parameter('can_interface').value
        self.dt    = 1.0/float(self.get_parameter('hz').value)
        self.tx_id = int(self.get_parameter('tx_id_brake').value)
        self.use_joy = bool(self.get_parameter('use_joy').value)
        self.arb_mode = str(self.get_parameter('arb_mode').value).lower()

        # ---- CAN
        try:
            self.bus = can.interface.Bus(channel=self.iface, bustype='socketcan')
            self.get_logger().info(f'✅ CAN open on {self.iface}')
        except Exception as e:
            self.bus = None
            self.get_logger().error(f'❌ CAN open failed on {self.iface}: {e}')

        # ---- subs
        self.last_aeb = 0.0
        self.last_joy = 0.0

        self.create_subscription(Float32, 'brake_pct', self._on_brake_aeb, 10)
        if self.use_joy:
            self.create_subscription(Float32, 'brake_pct_joy', self._on_brake_joy, 10)

        # feedback/debug
        self.pub_tx  = self.create_publisher(Float32, 'brake_pct_tx', 10)
        self.pub_dbg = self.create_publisher(String,  'hal_debug', 10)

        # ---- timer (ALWAYS send)
        self.create_timer(self.dt, self._tick)

    def _on_brake_aeb(self, msg: Float32):
        self.last_aeb = float(msg.data)

    def _on_brake_joy(self, msg: Float32):
        self.last_joy = float(msg.data)

    def _arb(self) -> float:
        a = max(0.0, min(100.0, self.last_aeb))
        j = max(0.0, min(100.0, self.last_joy))
        if self.arb_mode == 'aeb':
            return a
        if self.arb_mode == 'joy':
            return j
        # default: max
        return max(a, j)

    def _tick(self):
        if not self.bus:
            return

        cmd = self._arb()
        # 1 byte 0..100
        val = int(round(cmd))
        if val < 0: val = 0
        if val > 100: val = 100

        frame = can.Message(arbitration_id=self.tx_id,
                            is_extended_id=False,
                            data=[val])

        try:
            self.bus.send(frame)
        except can.CanError as e:
            self.get_logger().error(f'CAN send failed: {e}')
            return

        # echo + debug
        self.pub_tx.publish(Float32(data=float(val)))
        self.pub_dbg.publish(String(
            data=f"sent id=0x{self.tx_id:X} brake={val}% (aeb={self.last_aeb:.1f}, joy={self.last_joy:.1f}, mode={self.arb_mode})"
        ))

def main(argv=None):
    rclpy.init(args=argv)
    node = HALNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

