#!/usr/bin/env python3
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

try:
    import onnxruntime as ort
except Exception:
    ort = None

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

def now_sec():
    return time.time()

def xywh2xyxy(x):
    y = x.copy()
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y

class OrtYolo:
    def __init__(self, model_path, img_size=640, conf=0.25):
        if ort is None:
            raise RuntimeError("onnxruntime not available.")
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"❌ Model not found: {model_path}")

        print(f"🎯 Loading YOLO model: {model_path}")
        self.sess = ort.InferenceSession(model_path, providers=ort.get_available_providers())
        self.img_size = img_size
        self.conf = conf
        self.input_name = self.sess.get_inputs()[0].name
        self.out_names = [o.name for o in self.sess.get_outputs()]
        print(f"✅ Model ready: {model_path} | providers={ort.get_available_providers()}")

    def infer(self, img_bgr):
        img0 = cv2.resize(img_bgr, (self.img_size, self.img_size))
        img_rgb = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
        x = img_rgb.astype(np.float32) / 255.0
        x = np.transpose(x, (2, 0, 1))[None, ...]

        outputs = self.sess.run(self.out_names, {self.input_name: x})
        preds = outputs[0]
        if preds.ndim == 3:
            preds = preds[0]

        if preds.shape[-1] < 6:
            return np.zeros((0, 4)), np.zeros((0,)), np.zeros((0,))

        boxes_xywh = preds[:, :4]
        obj_conf = preds[:, 4:5]
        cls_conf = preds[:, 5:]
        cls_ids = np.argmax(cls_conf, axis=1)
        cls_scores = cls_conf[np.arange(cls_conf.shape[0]), cls_ids]
        scores = obj_conf.reshape(-1) * cls_scores

        keep = scores >= self.conf
        if not np.any(keep):
            return np.zeros((0, 4)), np.zeros((0,)), np.zeros((0,))

        boxes_xywh = boxes_xywh[keep]
        scores = scores[keep]
        cls_ids = cls_ids[keep]
        boxes_xyxy = xywh2xyxy(boxes_xywh)

        return boxes_xyxy.astype(np.float32), scores.astype(np.float32), cls_ids.astype(np.int32)

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # --- Parameters ---
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('frame_id', 'camera_optical')
        self.declare_parameter('tsd_model_path', 'models/ts_yolov8n.onnx')
        self.declare_parameter('tld_model_path', 'models/tl_yolov8n.onnx')
        self.declare_parameter('img_size', 640)
        self.declare_parameter('conf_thres', 0.30)

        # --- Get parameters ---
        self.camera_topic = self.get_parameter('camera_topic').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.frame_id = self.get_parameter('frame_id').value

        tsd_path = self.get_parameter('tsd_model_path').value
        tld_path = self.get_parameter('tld_model_path').value

        # --- Load models ---
        try:
            self.tsd = OrtYolo(tsd_path, conf=0.30)
            self.get_logger().info(f"TSD model loaded: {tsd_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Could not load TSD model: {e}")
            self.tsd = None

        try:
            self.tld = OrtYolo(tld_path, conf=0.30)
            self.get_logger().info(f"TLD model loaded: {tld_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Could not load TLD model: {e}")
            self.tld = None

        # ROS pubs/subs
        self.pub_signs = self.create_publisher(Detection2DArray, '/camera/traffic_signs', 10)
        self.pub_lights = self.create_publisher(Detection2DArray, '/camera/traffic_lights', 10)
        self.pub_lane_offset = self.create_publisher(Float32, '/lane_offset', 10)
        self.pub_lane_yaw = self.create_publisher(Float32, '/lane_yaw_error', 10)
        self.pub_debug = self.create_publisher(Image, '/camera/debug', 10)

        self.bridge = CvBridge()
        self.create_subscription(Image, self.camera_topic, self.on_image, SENSOR_QOS)

        self.last_t = now_sec()
        self.get_logger().info('✅ camera_node started with TSD+TLD+Lane in one process.')

    def build_detection_array(self, header_stamp, frame_id, boxes, scores, classes):
        arr = Detection2DArray()
        arr.header.stamp = header_stamp
        arr.header.frame_id = frame_id
        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes[i]
            det = Detection2D()
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.id = str(int(classes[i]))
            hyp.score = float(scores[i])
            det.results.append(hyp)
            arr.detections.append(det)
        return arr

    def compute_lane(self, img):
        h, w = img.shape[:2]
        roi_ymin = int(0.55 * h)
        roi = img[roi_ymin:h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 70, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=40, minLineLength=40, maxLineGap=20)

        overlay = img.copy()
        offset_m, yaw_err = 0.0, 0.0
        if lines is not None:
            xs, angles = [], []
            for l in lines:
                x1, y1, x2, y2 = l[0]
                cv2.line(overlay[roi_ymin:h, :], (x1, y1), (x2, y2), (0, 255, 0), 2)
                xs.extend([x1, x2])
                ang = np.arctan2((y2 - y1), (x2 - x1) + 1e-6)
                angles.append(ang)

            if len(xs) >= 2:
                lane_center_px = np.median(xs)
                offset_px = (lane_center_px - (w / 2.0))
                offset_m = offset_px * (3.7 / 640.0)

            if angles:
                yaw_err = -float(np.median(angles))

        return offset_m, yaw_err, overlay

    def on_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dbg = cv_image.copy()

        # --- Lane ---
        offset_m, yaw_err, lane_overlay = self.compute_lane(cv_image)
        self.pub_lane_offset.publish(Float32(data=offset_m))
        self.pub_lane_yaw.publish(Float32(data=yaw_err))
        dbg = cv2.addWeighted(dbg, 0.7, lane_overlay, 0.3, 0)

        # --- Traffic Signs ---
        if self.tsd:
            boxes, scores, cls_ids = self.tsd.infer(cv_image)
            self.get_logger().info(f"Signs detected: {len(boxes)}")
            arr = self.build_detection_array(msg.header.stamp, self.frame_id, boxes, scores, cls_ids)
            self.pub_signs.publish(arr)
            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, boxes[i])
                cv2.rectangle(dbg, (x1, y1), (x2, y2), (0, 140, 255), 2)

        # --- Traffic Lights ---
        if self.tld:
            boxes, scores, cls_ids = self.tld.infer(cv_image)
            self.get_logger().info(f"Lights detected: {len(boxes)}")
            arr = self.build_detection_array(msg.header.stamp, self.frame_id, boxes, scores, cls_ids)
            self.pub_lights.publish(arr)
            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, boxes[i])
                cv2.rectangle(dbg, (x1, y1), (x2, y2), (60, 255, 80), 2)

        # --- Debug Info ---
        fps = 1.0 / max(1e-3, (now_sec() - self.last_t))
        self.last_t = now_sec()
        cv2.putText(dbg, f"FPS: {fps:.1f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        if self.publish_debug:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

