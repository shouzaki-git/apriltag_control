import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from move_interfaces.action import MoveToGoal
import tf2_ros
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # カメラ設定
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # AprilTag 検出器の設定
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # 画像およびアクションクライアントの設定
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.action_client = ActionClient(self, MoveToGoal, 'move_to_goal')

        # カメラのキャリブレーションパラメータ
        self.cx = 640
        self.cy = 360
        self.fx = 1000
        self.fy = 1000

    def send_goal(self, x, y, theta):
        goal_msg = MoveToGoal.Goal()
        goal_msg.target_x = float(x)
        goal_msg.target_y = float(y)
        goal_msg.target_theta = float(theta)
        self.action_client.send_goal_async(goal_msg)

    def estimate_position(self, circle, h, r, theta):
        x_i, y_i, _ = circle
        x_i -= self.cx
        y_i -= self.cy

        y_w = (h - r) / np.tan(theta - np.arctan(y_i / self.fy))
        x_w = (x_i * np.sqrt((h - r)**2 + y_w**2)) / np.sqrt(y_i**2 + self.fx**2)
        return x_w, y_w

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Failed to capture image')
            return

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            gray_frame,
            estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy),
            tag_size=0.162
        )

        for tag in tags:
            position = tag.pose_t
            rotation_matrix = np.array(tag.pose_R)
            euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)
            
            x_w, y_w = self.estimate_position(position, 0.5, 0.1, np.deg2rad(euler_angles[2]))
            if self.action_client.wait_for_server(timeout_sec=1.0):
                self.send_goal(x_w, y_w, euler_angles[2])
            else:
                self.get_logger().error("Action server not available")

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
