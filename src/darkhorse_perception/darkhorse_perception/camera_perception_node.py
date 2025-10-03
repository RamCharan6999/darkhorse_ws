#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class CameraPerception(Node):
    def __init__(self):
        super().__init__('camera_perception_node')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('model_path', 'my_traffic_model.pt')
        model_path = self.get_parameter('model_path').value

        # Load YOLO traffic model
        self.model = YOLO(model_path)
        self.get_logger().info(f"✅ Loaded YOLO model from {model_path}")

        # ROS pubs
        self.lane_pub = self.create_publisher(Float32, '/lane_offset', 10)
        self.signal_pub = self.create_publisher(String, '/signals', 10)
        self.obj_pub = self.create_publisher(Float32MultiArray, '/camera_objects', 10)  # [class_id, distance]
        self.debug_pub = self.create_publisher(Image, '/camera/debug', 10)

        # Sub camera
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Class mapping (adapt to your YOLO training labels)
        self.classes = ["car", "person", "stop", "traffic_light", "speed25", "red", "green"]

        # Known average widths (meters)
        self.KNOWN_WIDTHS = {
            "car": 1.8,
            "person": 0.5,
            "stop": 0.75,
            "traffic_light": 0.3
        }

        # Approx camera focal length in pixels (calibrate for real camera!)
        self.FOCAL_LENGTH = 700

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # --- Lane detection ---
        offset, annotated = self.detect_and_annotate(frame)
        self.lane_pub.publish(Float32(data=float(offset)))

        # --- Object & traffic detection (YOLO) ---
        results = self.model(frame, verbose=False)

        traffic_label = "NONE"
        obj_msg = Float32MultiArray()
        obj_list = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls)
                label = self.classes[cls_id] if cls_id < len(self.classes) else "unknown"

                # If it's a traffic signal
                if label in ["stop", "red", "green", "speed25"]:
                    traffic_label = label.upper()

                # If known-width object, estimate depth
                if label in self.KNOWN_WIDTHS:
                    pixel_width = (box.xywh[0][2]).item()
                    if pixel_width > 0:
                        distance = (self.KNOWN_WIDTHS[label] * self.FOCAL_LENGTH) / pixel_width
                        obj_list.extend([float(cls_id), float(distance)])

                        # Draw on debug image
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        cv2.rectangle(annotated, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0,255,0), 2)
                        cv2.putText(annotated, f"{label} {distance:.1f}m",
                                    (xyxy[0], xyxy[1]-10), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (0,255,0), 2)

        # Publish traffic signal
        self.signal_pub.publish(String(data=traffic_label))

        # Publish objects (flattened list of [cls_id, dist, cls_id, dist, ...])
        obj_msg.data = obj_list
        self.obj_pub.publish(obj_msg)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        self.debug_pub.publish(debug_msg)

        self.get_logger().info(f"Lane offset={offset:.2f}, Signal={traffic_label}, Objects={obj_list}")

    def detect_and_annotate(self, frame):
        """Very simple lane detector (Canny + Hough) returning lateral offset."""
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        mask = np.zeros_like(edges)
        polygon = np.array([[(0,h),(w,h),(w,int(h*0.6)),(0,int(h*0.6))]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        roi = cv2.bitwise_and(edges, mask)
        lines = cv2.HoughLinesP(roi, 1, np.pi/180, 50, minLineLength=40, maxLineGap=120)

        offset = 0.0
        if lines is not None:
            xs = []
            for l in lines:
                x1,y1,x2,y2 = l[0]
                xs.extend([x1,x2])
            if xs:
                lane_center = sum(xs)/len(xs)
                offset = (lane_center - w/2.0) / (w/2.0)
                cv2.line(frame, (int(lane_center), h), (int(lane_center), int(h*0.6)), (0,255,255), 2)

        return offset, frame

def main(args=None):
    rclpy.init(args=args)
    node = CameraPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

