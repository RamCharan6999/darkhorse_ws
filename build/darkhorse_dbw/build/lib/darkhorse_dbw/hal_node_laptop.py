#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy
import can


class HALNode(Node):
    def __init__(self, default_iface='can1', name='hal_node_laptop'):
        super().__init__(name)

        # --- Parameters ---
        self.declare_parameter('can_interface', default_iface)
        self.declare_parameter('hz', 50.0)

        self.iface = self.get_parameter('can_interface').value
        self.dt = 1.0 / float(self.get_parameter('hz').value)

        # --- CAN setup ---
        self.bus = None
        try:
            self.bus = can.interface.Bus(channel=self.iface, bustype='socketcan')
            self.get_logger().info(f'✅ CAN open on {self.iface}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ CAN not available on {self.iface}: {e}. Running ROS-only mode.')

        # --- Last commands from control nodes ---
        self.last_aeb = 0.0
        self.last_acc = 0.0
        self.last_lka = 0.0
        self.last_tsd = None

        # --- Last joystick axes ---
        self.joy_throttle = 0.0
        self.joy_brake = 0.0
        self.joy_steer = 0.0

        # --- Subscriptions ---
        self.create_subscription(Float32, 'aeb_cmd', self._on_aeb, 10)
        self.create_subscription(Float32, 'acc_cmd', self._on_acc, 10)
        self.create_subscription(Float32, 'lka_cmd', self._on_lka, 10)
        self.create_subscription(Float32, 'tsd_cmd', self._on_tsd, 10)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)  # ✅ Direct joystick

        # --- Publishers ---
        self.pub_throttle = self.create_publisher(Float32, 'throttle_pct', 10)
        self.pub_brake    = self.create_publisher(Float32, 'brake_pct', 10)
        self.pub_steer    = self.create_publisher(Float32, 'steering_deg', 10)
        self.pub_debug    = self.create_publisher(String, 'hal_debug', 10)

        # --- Timer ---
        self.create_timer(self.dt, self._tick)

    # --- Callbacks ---
    def _on_aeb(self, msg): self.last_aeb = float(msg.data)
    def _on_acc(self, msg): self.last_acc = float(msg.data)
    def _on_lka(self, msg): self.last_lka = float(msg.data)
    def _on_tsd(self, msg): self.last_tsd = float(msg.data)

    def _on_joy(self, msg: Joy):
        # 🎮 Example mapping (adjust as per your joystick)
        # msg.axes[1] → left stick vertical (up=+1, down=-1)
        # msg.axes[3] → right stick horizontal (left=-1, right=+1)

        # Throttle from pushing stick UP
        self.joy_throttle = max(0.0, msg.axes[1]) * 100.0  # scale 0–100

        # Brake from pushing stick DOWN
        self.joy_brake = max(0.0, -msg.axes[1]) * 100.0   # scale 0–100

        # Steering from right stick
        self.joy_steer = msg.axes[3] * 30.0  # -30°..+30°

    # --- Arbitration + Output ---
    def _tick(self):
        # Priority: joystick > AEB > ACC/LKA
        throttle = self.joy_throttle if self.joy_throttle > 0 else self.last_acc
        brake    = self.joy_brake if self.joy_brake > 0 else self.last_aeb
        steer    = self.joy_steer if abs(self.joy_steer) > 1e-3 else self.last_lka

        # AEB override → kill throttle
        if brake > 1.0:
            throttle = 0.0

        # TSD cap
        if self.last_tsd is not None:
            max_throttle_pct = (self.last_tsd / 30.0) * 100.0
            throttle = min(throttle, max_throttle_pct)

        # Clamp
        throttle = max(0.0, min(100.0, throttle))
        brake    = max(0.0, min(100.0, brake))
        steer    = max(-30.0, min(30.0, steer))

        # Publish ROS outputs
        self.pub_throttle.publish(Float32(data=throttle))
        self.pub_brake.publish(Float32(data=brake))
        self.pub_steer.publish(Float32(data=steer))

        dbg = f"[HAL] th={throttle:.1f}% br={brake:.1f}% st={steer:.1f}°"
        if self.last_tsd is not None:
            dbg += f" | limit={self.last_tsd:.1f} km/h"
        dbg += " | CAN=OK" if self.bus else " | CAN=OFF"
        self.pub_debug.publish(String(data=dbg))

        # Try CAN send
        if self.bus:
            self._send(0x100, int(round(throttle)))
            self._send(0x102, int(round(brake)))
            self._send(0x101, int(round(steer)))

    def _send(self, tx_id, value):
        try:
            frame = can.Message(arbitration_id=tx_id,
                                is_extended_id=False,
                                data=[int(value) & 0xFF])
            self.bus.send(frame)
        except Exception as e:
            self.get_logger().warn(f"⚠️ CAN send failed: {e}")


def main(argv=None):
    rclpy.init(args=argv)
    node = HALNode(default_iface='can1', name='hal_node_laptop')  # Jetson → default_iface='can2'
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

