#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, math, time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import can
import cantools

from std_msgs.msg import String, Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray


# -------- helpers -------------------------------------------------------------

def _norm_key(s: str) -> str:
    return s.lower().replace(' ', '').replace('_', '')

def find_key(d: Dict, candidates) -> Optional[str]:
    if not d:
        return None
    norm = { _norm_key(k): k for k in d.keys() }
    for c in candidates:
        c2 = _norm_key(c)
        if c2 in norm:
            return norm[c2]
        for nk, orig in norm.items():
            if c2 in nk or nk in c2:
                return orig
    return None

def meters_auto(v):
    if v is None:
        return None
    v = float(v)
    if abs(v) > 1e6:
        return float('nan')
    if v > 250.0:  # looks like cm
        return v / 100.0
    return v


# -------- main node -----------------------------------------------------------

class RadarRxNode(Node):
    def __init__(self):
        super().__init__('radar_rx_node')

        # parameters with defaults so CLI args aren’t needed each run
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter(
            'dbc_path',
            os.path.expanduser(
                '~/DH_ws/install/darkhorse_dbw/share/darkhorse_dbw/config/Starkenn_Orion_Radar_x_aBAJA.dbc'
            )
        )
        self.declare_parameter('poll_hz', 25.0)
        self.declare_parameter('frame_id', 'base_link')   # RViz-friendly
        self.declare_parameter('track_ttl_sec', 0.35)
        self.declare_parameter('lat_fov_m', 8.0)
        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('max_range_m', 200.0)
        self.declare_parameter('vx_ema', 0.30)

        can_iface   = self.get_parameter('can_interface').value
        self.dbc    = self.get_parameter('dbc_path').value
        self.dt     = 1.0 / float(self.get_parameter('poll_hz').value)
        self.frame  = self.get_parameter('frame_id').value
        self.ttl    = float(self.get_parameter('track_ttl_sec').value)
        self.latwin = float(self.get_parameter('lat_fov_m').value)
        self.rmin   = float(self.get_parameter('min_range_m').value)
        self.rmax   = float(self.get_parameter('max_range_m').value)
        self.vx_ema = float(self.get_parameter('vx_ema').value)

        # -------- DBC
        self.db = None
        try:
            self.db = cantools.database.load_file(self.dbc)
            self.get_logger().info(f'✅ Loaded DBC: {self.dbc}')
        except Exception as e:
            self.get_logger().warn(f'DBC load failed ({e}). Raw decode only.')

        # -------- CAN
        self.bus = None
        try:
            self.bus = can.interface.Bus(channel=can_iface, bustype='socketcan')
            self.get_logger().info(f'✅ Connected to CAN interface {can_iface}')
        except Exception as e:
            self.get_logger().error(f'❌ CAN open failed: {e}')

        # -------- QoS for RViz (RELIABLE)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # -------- pubs
        self.pub_debug   = self.create_publisher(String,     'radar/debug',   10)
        self.pub_points  = self.create_publisher(PointCloud, 'radar/points',  qos_reliable)
        self.pub_markers = self.create_publisher(MarkerArray,'radar/markers', qos_reliable)
        self.pub_closest = self.create_publisher(String,     'radar/closest', 10)
        self.pub_ttc     = self.create_publisher(Float32,    'radar/ttc',     10)

        # track state: id -> {x,y,vx,vy,rcs,t, vx_est, x_prev, t_prev}
        self.tracks: Dict[int, Dict[str, float]] = {}

        self.create_timer(self.dt, self._tick)

    # -------- polling
    def _tick(self):
        if not self.bus:
            return
        for _ in range(20):  # drain a few frames
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            self._handle_can(msg)
        self._publish()

    # -------- handle one CAN frame
    def _handle_can(self, msg):
        now = time.time()

        if not self.db:
            self.pub_debug.publish(String(data=f'RAW id=0x{msg.arbitration_id:X} data={msg.data.hex()}'))
            return

        try:
            dec = self.db.decode_message(msg.arbitration_id, msg.data)
        except Exception:
            return

        # summary/debug
        if msg.arbitration_id in (0x501, 0x581):
            self.pub_debug.publish(String(data=f"0x{msg.arbitration_id:X} {dec}"))
        elif 'Obj' in ''.join(dec.keys()) or 'Object' in ''.join(dec.keys()):
            self.pub_debug.publish(String(data=f"0x{msg.arbitration_id:X} OK"))

        # find keys
        kx  = find_key(dec, ['Obj_DistLong','Object_DistLong','X','RangeLong'])
        ky  = find_key(dec, ['Obj_DistLat','Object_DistLat','Y','RangeLat'])
        kvx = find_key(dec, ['Obj_VrelLong','VrelLong','RelVelLong','VelX','RelVx'])
        kvy = find_key(dec, ['Obj_VrelLat','VrelLat','RelVelLat','VelY','RelVy'])
        kid = find_key(dec, ['Obj_ID','Object_ID','Track_ID','ID'])
        krc = find_key(dec, ['RCS','Obj_RCS'])

        if not (kx or ky):
            return

        x   = meters_auto(dec.get(kx, 0.0)) if kx else 0.0
        y   = meters_auto(dec.get(ky, 0.0)) if ky else 0.0
        vx  = float(dec.get(kvx, 0.0)) if kvx else 0.0
        vy  = float(dec.get(kvy, 0.0)) if kvy else 0.0
        rcs = float(dec.get(krc, 0.0)) if krc else 0.0

        if x is None or y is None or math.isnan(x) or math.isnan(y):
            return
        if x < 0.0 or x < self.rmin or x > self.rmax or abs(y) > 1000.0:
            return

        if kid and isinstance(dec.get(kid), (int, float)):
            obj_id = int(dec[kid])
        else:
            obj_id = (msg.arbitration_id & 0x7FF) * 100 + int((y + 50.0) * 2.0)

        tr = self.tracks.get(obj_id, None)
        if tr is None:
            tr = {'x_prev': x, 't_prev': now, 'vx_est': 0.0}

        # derive vx if DBC not useful
        dt = max(1e-3, now - tr['t_prev'])
        vx_num = (x - tr['x_prev']) / dt   # approach → negative
        vx_est = (1.0 - self.vx_ema) * tr['vx_est'] + self.vx_ema * vx_num

        vx_use = vx if abs(vx) >= 0.05 else vx_est

        tr.update({'x': x, 'y': y, 'vx': vx_use, 'vy': vy, 'rcs': rcs,
                   't': now, 'x_prev': x, 't_prev': now, 'vx_est': vx_est})
        self.tracks[obj_id] = tr

    # -------- publish outputs
    def _publish(self):
        now = time.time()
        # drop stale
        self.tracks = {tid: tr for tid, tr in self.tracks.items() if now - tr.get('t', 0.0) <= self.ttl}

        # PointCloud
        pc = PointCloud()
        pc.header.frame_id = self.frame
        for tr in self.tracks.values():
            pc.points.append(Point32(x=float(tr['x']), y=float(tr['y']), z=0.0))
        self.pub_points.publish(pc)

        # Markers
        marr = MarkerArray()
        mid = 0
        for tid, tr in self.tracks.items():
            x, y, vx = tr['x'], tr['y'], tr['vx']

            m = Marker()
            m.header.frame_id = self.frame
            m.ns = 'radar_obj'
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            if vx < -0.1:  # approaching
                m.color.r, m.color.g, m.color.b = 1.0, 0.3, 0.3
            else:
                m.color.r, m.color.g, m.color.b = 0.3, 1.0, 0.3
            m.color.a = 0.9
            m.lifetime = rclpy.duration.Duration(seconds=self.ttl).to_msg()
            marr.markers.append(m)

            t = Marker()
            t.header.frame_id = self.frame
            t.ns = 'radar_text'
            t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.8
            t.scale.z = 0.4
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = f'id={tid} x={x:.1f} y={y:.1f} vx={vx:.1f}'
            t.lifetime = rclpy.duration.Duration(seconds=self.ttl).to_msg()
            marr.markers.append(t)

        self.pub_markers.publish(marr)

        # Closest + TTC
        closest = None
        for tr in self.tracks.values():
            if abs(tr['y']) <= self.latwin:
                if closest is None or tr['x'] < closest['x']:
                    closest = tr

        if closest is None:
            self.pub_closest.publish(String(data='detections=0'))
            self.pub_ttc.publish(Float32(data=float('inf')))
        else:
            x, y, vx = closest['x'], closest['y'], closest['vx']
            self.pub_closest.publish(String(data=f"detections={len(self.tracks)} first=({x:.1f},{y:.1f}) m"))
            ttc = float('inf')
            if vx < -0.05 and x > 0.0:
                ttc = max(0.0, x / (-vx))
            self.pub_ttc.publish(Float32(data=ttc))


def main(argv=None):
    rclpy.init(args=argv)
    node = RadarRxNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

