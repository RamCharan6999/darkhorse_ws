import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
from collections import deque

# =============================
# PID Controller
# =============================
class PIDController:
    def __init__(self, Kp=1.6, Ki=0.0, Kd=0.2):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def step(self, error):
        now = time.time()
        dt = (now - self.prev_time) if self.prev_time else 0.033
        self.prev_time = now
        dt = max(1e-3, dt)
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        out = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.prev_error = error
        return out

# =============================
# Lane Detector (polynomial fit)
# =============================
class LaneDetector:
    def __init__(self, smoothing=6, min_points_for_fit=6):
        self.left_coeffs = deque(maxlen=smoothing)
        self.right_coeffs = deque(maxlen=smoothing)
        self.min_points_for_fit = min_points_for_fit

    def detect(self, frame):
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)

        mask = np.zeros_like(edges)
        poly = np.array([[  # trapezoid ROI
            (int(0.08*w), h),
            (int(0.45*w), int(0.6*h)),
            (int(0.55*w), int(0.6*h)),
            (int(0.92*w), h)
        ]], dtype=np.int32)
        cv2.fillPoly(mask, poly, 255)
        roi = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(roi, 1, np.pi/180, threshold=40,
                                minLineLength=30, maxLineGap=120)

        left_pts, right_pts = [], []
        if lines is not None:
            for l in lines:
                x1,y1,x2,y2 = l[0]
                if abs(x2-x1) < 3: 
                    continue
                slope = (y2-y1) / (x2-x1 + 1e-6)
                if slope < -0.2:
                    left_pts += [(x1,y1),(x2,y2)]
                elif slope > 0.2:
                    right_pts += [(x1,y1),(x2,y2)]

        def fit_poly(points, history):
            if len(points) < self.min_points_for_fit:
                return np.mean(np.array(history), axis=0) if history else None
            ys = np.array([p[1] for p in points], dtype=np.float64)
            xs = np.array([p[0] for p in points], dtype=np.float64)
            try:
                coeffs = np.polyfit(ys, xs, 2)
            except Exception:
                return None
            history.append(coeffs)
            return np.mean(np.array(history), axis=0)

        left_coeff = fit_poly(left_pts, self.left_coeffs)
        right_coeff = fit_poly(right_pts, self.right_coeffs)

        center_coeff = None
        if left_coeff is not None and right_coeff is not None:
            center_coeff = (left_coeff + right_coeff) / 2.0
        elif left_coeff is not None:
            center_coeff = left_coeff.copy()
            center_coeff[2] += 200
        elif right_coeff is not None:
            center_coeff = right_coeff.copy()
            center_coeff[2] -= 200

        # Draw overlay
        overlay = np.zeros_like(frame)
        def draw_poly(coeffs, color):
            if coeffs is None:
                return
            y_vals = np.linspace(int(0.6*h), h-1, num=60)
            x_vals = np.polyval(coeffs, y_vals)
            pts = np.vstack((x_vals, y_vals)).T
            pts[:,0] = np.clip(pts[:,0], 0, w-1)
            pts = pts.astype(np.int32)
            cv2.polylines(overlay, [pts], isClosed=False, color=color, thickness=6)

        draw_poly(left_coeff, (255,0,0))
        draw_poly(right_coeff, (0,0,255))
        annotated = cv2.addWeighted(frame, 0.8, overlay, 1.0, 0)

        return center_coeff, annotated

# =============================
# Steering computation
# =============================
def compute_steer(center_coeff, frame, pid, LOOKAHEAD_RATIO=0.55):
    h, w = frame.shape[:2]
    cx = w/2.0
    if center_coeff is None:
        return 0.0, 0.0

    y_look = int(h * LOOKAHEAD_RATIO)
    Ld = max(1.0, h - y_look)
    x_look = np.polyval(center_coeff, y_look)
    dx = x_look - cx
    heading_error = np.arctan2(dx, Ld)
    steer_cmd = pid.step(heading_error)
    return steer_cmd, heading_error

# =============================
# ROS2 Node
# =============================
class LaneNode(Node):
    def __init__(self):
        super().__init__("lane_node")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/RGBImage", self.callback, 10)
        self.pid = PIDController()
        self.detector = LaneDetector()

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        center_coeff, annotated = self.detector.detect(frame)
        steer_cmd, heading_error = compute_steer(center_coeff, annotated, self.pid)

        cv2.putText(annotated, f"Steer: {steer_cmd:+.3f}", (30,50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow("Lane Detection", annotated)
        cv2.waitKey(1)

# =============================
# Main
# =============================
def main(args=None):
    rclpy.init(args=args)
    node = LaneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
