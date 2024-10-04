import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import tf2_ros
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
from filterpy.kalman import KalmanFilter

# AprilTag の角度検出ノード
class AprilTagDetector(Node):
    def __init__(self, robot_controller):
        super().__init__('apriltag_detector')
        self.robot_controller = robot_controller  # RobotController インスタンスを保持
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # AprilTag 検出器の初期化
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)  # AprilTagの位置情報をブロードキャスト
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Failed to capture image')
            return
        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            gray_frame,
            estimate_tag_pose=True,
            camera_params=(1280, 720, 640, 360),  # 仮のカメラパラメータ、実際のキャリブレーション値を使用
            tag_size=0.162
        )
        
        if tags:
            tag = tags[0]  # 複数タグがあれば最初のタグを使用
            rotation_matrix = np.array(tag.pose_R)
            euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)

            # 現在のヨー角をロボットコントローラに更新
            yaw_angle = euler_angles[2]  # Z軸周りの回転角（ヨー角）
            self.robot_controller.update_current_angle(yaw_angle)

        # 検出した AprilTag をフレームに描画
        for tag in tags:
            center = (int(tag.center[0]), int(tag.center[1]))
            cv2.circle(frame, center, 5, (0, 0, 255), 2)
        
        # 画像メッセージをパブリッシュ
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.get_logger().info("Image published.")

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()


# ロボットの制御ノード
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_callback)
        self.target_angle = 0.0
        self.current_angle = 0.0

    def control_callback(self):
        angle_diff = self.target_angle - self.current_angle
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angle_diff * 0.1
        self.cmd_vel_publisher.publish(twist)

    def update_current_angle(self, new_angle):
        self.current_angle = new_angle
        self.get_logger().info(f'Updated current angle: {self.current_angle}')


def main(args=None):
    rclpy.init(args=args)

    # RobotController のインスタンスを作成
    robot_controller = RobotController()

    # AprilTagDetector のインスタンスを作成し、RobotController を渡す
    apriltag_detector = AprilTagDetector(robot_controller)

    # 複数ノードを同時に実行するためのマルチスレッドExecutorを使用
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_controller)
    executor.add_node(apriltag_detector)

    try:
        executor.spin()
    finally:
        # ノードのクリーンアップ
        apriltag_detector.destroy()
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
