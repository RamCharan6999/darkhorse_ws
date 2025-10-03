#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can
import time

class VehicleSpeedNode(Node):
    def __init__(self):
        super().__init__('vehicle_speed_node')

        # Parameters
        self.declare_parameter('can_interface', 'can1')    # DBW bus
        self.declare_parameter('can_id', 0x120)            # speed feedback ID
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('timeout_s', 0.8)           # warn if no frames

        self.iface = self.get_parameter('can_interface').value
        self.can_id = int(self.get_parameter('can_id').value)
        self.dt = 1.0 / float(self.get_parameter('publish_hz').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        # Publisher
        self.pub_speed = self.create_publisher(Float32, '/VehicleSpeed', 10)

        # Open CAN interface
        try:
            self.bus = can.interface.Bus(channel=self.iface, bustype='socketcan')
            try:
                self.bus.set_filters([{"can_id": self.can_id, "can_mask": 0x7FF, "extended": False}])
            except Exception:
                pass
            self.get_logger().info(f'✅ Listening on {self.iface} for ID=0x{self.can_id:X}')
        except Exception as e:
            self.bus = None
            self.get_logger().error(f'❌ CAN open failed on {self.iface}: {e}')

        # State
        self.last_rx_ts = 0.0
        self.last_speed_ms = 0.0

        # Timer loop
        self.create_timer(self.dt, self._tick)

    def _tick(self):
        if not self.bus:
            return

        # Drain available frames
        while True:
            try:
                msg = self.bus.recv(timeout=0.0)  # non-blocking
            except Exception as e:
                self.get_logger().error(f'CAN recv error: {e}')
                break
            if msg is None:
                break

            if msg.arbitration_id == self.can_id and len(msg.data) >= 4:
                motor_rpm = (msg.data[0] << 8) | msg.data[1]
                speed_tenths_kmh = (msg.data[2] << 8) | msg.data[3]
                speed_kmh = speed_tenths_kmh / 10.0
                speed_ms = speed_kmh / 3.6

                self.last_rx_ts = time.time()
                self.last_speed_ms = float(speed_ms)

                # Debug log for each valid frame
                self.get_logger().info(
                    f"📥 CAN RX [0x{self.can_id:X}] | Motor={motor_rpm} RPM | "
                    f"Speed={speed_kmh:.1f} km/h ({speed_ms:.2f} m/s)"
                )

        # Always publish the last known speed (even if no new frames this tick)
        self.pub_speed.publish(Float32(data=self.last_speed_ms))

        # Warn if no frames for too long
        if (time.time() - self.last_rx_ts) > self.timeout_s:
            if int(time.time()) % 2 == 0:  # warn roughly every 2s
                self.get_logger().warn(
                    f'⚠️ No speed frames (0x{self.can_id:X}) on {self.iface} '
                    f'for {time.time()-self.last_rx_ts:.1f}s'
                )

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

