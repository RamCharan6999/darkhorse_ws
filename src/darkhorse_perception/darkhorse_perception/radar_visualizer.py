import rclpy
from rclpy.node import Node
import cantools
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import os
from ament_index_python.packages import get_package_share_directory

class RadarVisualizer(Node):
    def __init__(self):
        super().__init__('radar_visualizer')
        
        try:
            db_path = os.path.join(
                get_package_share_directory('darkhorse_dbw'),
                'config',
                'Starkenn_Orion_Radar_x_aBAJA.dbc'
            )
            self.radar_db = cantools.database.load_file(db_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load DBC file: {e}")
            return

        self.marker_pub = self.create_publisher(MarkerArray, '/radar/markers', 10)
        self.can_sub = self.create_subscription(String, '/from_can_bus', self.can_callback, 10)
        
        self.get_logger().info("✅ Radar Visualizer Node Started (String Workaround)")

    def can_callback(self, msg: String):
        try:
            id_str, data_str = msg.data.split(':')
            can_id = int(id_str)
            can_data = bytes.fromhex(data_str)
        except ValueError:
            self.get_logger().warn(f"Received malformed CAN string: {msg.data}")
            return
            
        try:
            decoded = self.radar_db.decode_message(can_id, can_data)
        except:
            return

        marker_array = MarkerArray()
        
        try:
            msg_name = self.radar_db.get_message_by_frame_id(can_id).name
            if 'Tracked_obj' in msg_name:
                x_dist_name = next(s.name for s in self.radar_db.get_message_by_frame_id(can_id).signals if 'x_distance' in s.name)
                y_dist_name = next(s.name for s in self.radar_db.get_message_by_frame_id(can_id).signals if 'Y_distance' in s.name)
                
                x_dist = decoded.get(x_dist_name)
                y_dist = decoded.get(y_dist_name)
                
                if x_dist is not None and y_dist is not None:
                    pos_marker = Marker()
                    pos_marker.header.frame_id = "base_link"
                    pos_marker.header.stamp = self.get_clock().now().to_msg()
                    pos_marker.ns = "radar_objects"
                    pos_marker.id = can_id
                    pos_marker.type = Marker.SPHERE
                    pos_marker.action = Marker.ADD
                    pos_marker.pose.position.x = float(x_dist)
                    pos_marker.pose.position.y = float(y_dist)
                    pos_marker.pose.position.z = 0.5
                    pos_marker.scale.x = 0.8
                    pos_marker.scale.y = 0.8
                    pos_marker.scale.z = 0.8
                    pos_marker.color.a = 0.8
                    pos_marker.color.r = 1.0
                    pos_marker.color.g = 0.2
                    pos_marker.color.b = 0.0
                    marker_array.markers.append(pos_marker)
        except:
            pass # Message not found, etc.

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
