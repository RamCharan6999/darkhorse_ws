#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import struct
import time

# optional imports (only used if use_dbc is true)
try:
    import cantools
    import can
except Exception:
    cantools = None
    can = None

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header


class RadarRxNode(Node):
    """
    Radar receiver node that:
      - optionally decodes via DBC (if configured and cantools available)
      - otherwise decodes raw bytes using configurable byte offsets/scales
    Publishes:
      - /radar/points (geometry_msgs/PointStamped) for visualization
      - /radar/arrays (std_msgs/Float32MultiArray) where each object is [id,x,y,vx,vy,rcs]
    """

    def __init__(self):
        super().__init__('radar_rx_node')

        # --- parameters (tune these) ---
        self.declare_parameter('use_dbc', False)
        self.declare_parameter('dbc_file', '/home/ram_charan/DH_ws/src/darkhorse_dbw/config/Starkenn_Orion_Radar_x_aBAJA.dbc')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('poll_hz', 20.0)

        # Raw parsing params (defaults chosen to match common radar layouts; tweak as needed)
        self.declare_parameter('msg_ids', [0x590, 0x710])  # IDs to parse if not using DBC
        self.declare_parameter('endianness', 'big')        # 'big' or 'little' for multi-byte fields
        self.declare_parameter('dist_scale', 0.01)         # raw units -> meters
        self.declare_parameter('lat_scale', 0.01)          # lateral units -> meters
        self.declare_parameter('vel_scale', 0.01)          # velocity units -> m/s
        self.declare_parameter('rcs_scale', 1.0)           # RCS scale

        # Read parameters
        self.use_dbc = self.get_parameter('use_dbc').value
        self.dbc_file = self.get_parameter('dbc_file').value
        self.can_interface = self.get_parameter('can_interface').value
        self.poll_hz = float(self.get_parameter('poll_hz').value)

        self.msg_ids = self.get_parameter('msg_ids').value
        self.endianness = self.get_parameter('endianness').value
        self.dist_scale = float(self.get_parameter('dist_scale').value)
        self.lat_scale = float(self.get_parameter('lat_scale').value)
        self.vel_scale = float(self.get_parameter('vel_scale').value)
        self.rcs_scale = float(self.get_parameter('rcs_scale').value)

        self.get_logger().info(f"radar_rx_node starting (use_dbc={self.use_dbc})")

        # Publishers
        self.pub_point = self.create_publisher(PointStamped, '/radar/points', 10)
        self.pub_array = self.create_publisher(Float32MultiArray, '/radar/arrays', 10)

        # DBC setup (optional)
        self.db = None
        if self.use_dbc:
            if cantools is None:
                self.get_logger().error("cantools or python-can not installed; set use_dbc=false to bypass DBC.")
                raise RuntimeError("cantools missing")
            try:
                self.db = cantools.database.load_file(self.dbc_file)
                self.get_logger().info(f"Loaded DBC {self.dbc_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to load DBC: {e}. Set use_dbc=false to parse raw frames.")
                raise

        # CAN interface setup
        if can is None:
            self.get_logger().error("python-can package not installed (required for socketcan).")
            raise RuntimeError("python-can missing")

        try:
            self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
            self.get_logger().info(f"Connected to CAN interface {self.can_interface}")
        except OSError as e:
            self.get_logger().error(f"Could not access SocketCAN device {self.can_interface} ({e})")
            raise

        # Timer for polling
        period = 1.0 / max(1.0, self.poll_hz)
        self.timer = self.create_timer(period, self.poll_can)

        # debugging counters
        self.frame_count = 0

    def poll_can(self):
        """Called periodically to read one CAN frame (non-blocking) and process."""
        try:
            msg = self.bus.recv(timeout=0.005)  # short timeout
        except Exception as e:
            self.get_logger().error(f"CAN recv error: {e}")
            return

        if msg is None:
            return

        self.frame_count += 1

        # If using DBC, try decode and publish important signals
        if self.use_dbc and self.db is not None:
            try:
                decoded = self.db.decode_message(msg.arbitration_id, msg.data)
            except Exception:
                # unknown message id for DBC
                return

            # Attempt to map common signal names to outputs (these names depend on your DBC)
            # This is heuristic — tweak to match your dbc's signal names
            obj_id = decoded.get('Object_ID') or decoded.get('ObjID') or decoded.get('ObjectID') or 0
            x = decoded.get('Object_DistX') or decoded.get('DistX') or decoded.get('Range') or 0.0
            y = decoded.get('Object_DistY') or decoded.get('DistY') or decoded.get('Lat') or 0.0
            vx = decoded.get('Object_VrelX') or decoded.get('Vrel') or decoded.get('RelVel') or 0.0
            vy = decoded.get('Object_VrelY') or 0.0
            rcs = decoded.get('Object_RCS') or 0.0

            self._publish_object(int(obj_id), float(x), float(y), float(vx), float(vy), float(rcs))
            return

        # ELSE: raw parsing (no DBC). We'll attempt to parse a few common radar message formats.
        # Default raw parser: assume:
        #   bytes 0-1 -> dist (unsigned 16)
        #   bytes 2-3 -> lat   (signed 16)  (or unsigned depending on radar)
        #   byte 4    -> vrel  (signed 8)
        #   byte 5    -> obj id (unsigned 8)
        # This is configurable using parameters (scale, endianness)
        can_id = msg.arbitration_id
        data = msg.data

        if can_id not in self.msg_ids:
            return  # ignore unlisted IDs

        # ensure minimum length
        if len(data) < 6:
            self.get_logger().debug(f"Ignoring short frame id=0x{can_id:X} len={len(data)}")
            return

        # Choose struct formats based on endianness
        endian_char = '>' if self.endianness == 'big' else '<'
        try:
            # unsigned short (H) for dist, signed short (h) for lat, signed char (b) for vel, unsigned char (B) for id
            # We use slices: bytes 0-2, 2-4, 4, 5
            dist_raw = struct.unpack(endian_char + 'H', data[0:2])[0]
            lat_raw = struct.unpack(endian_char + 'h', data[2:4])[0]   # lateral often signed
            vel_raw = struct.unpack(endian_char + 'b', data[4:5])[0]
            obj_id = struct.unpack(endian_char + 'B', data[5:6])[0]
        except Exception as e:
            self.get_logger().debug(f"Raw parse error for id=0x{can_id:X}: {e}")
            return

        x_m = dist_raw * self.dist_scale
        y_m = lat_raw * self.lat_scale
        vx_m = vel_raw * self.vel_scale
        vy_m = 0.0
        rcs = 0.0

        # Some radars pack more fields — attempt best-effort extra parsing for bytes 6-7
        if len(data) >= 8:
            try:
                # example: byte6 = rcs, byte7 maybe flags
                rcs = float(data[6]) * self.rcs_scale
            except Exception:
                rcs = 0.0

        self._publish_object(int(obj_id), float(x_m), float(y_m), float(vx_m), float(vy_m), float(rcs))

    def _publish_object(self, obj_id, x, y, vx, vy, rcs):
        # publish PointStamped for quick visualization
        pt = PointStamped()
        pt.header = Header()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'radar'
        pt.point.x = x
        pt.point.y = y
        pt.point.z = 0.0
        self.pub_point.publish(pt)

        # publish details in a flat float array [id, x, y, vx, vy, rcs]
        arr = Float32MultiArray()
        arr.data = [float(obj_id), float(x), float(y), float(vx), float(vy), float(rcs)]
        self.pub_array.publish(arr)

        self.get_logger().debug(f"Published radar obj id={obj_id} x={x:.2f} y={y:.2f} vx={vx:.2f} rcs={rcs:.1f}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RadarRxNode()
    except Exception as e:
        # initialization failure (e.g. socketcan issue)
        print(f"Failed to start radar_rx_node: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

