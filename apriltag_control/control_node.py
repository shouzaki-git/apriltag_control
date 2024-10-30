import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
from filterpy.kalman import KalmanFilter
import math

T = 0.01

# AprilTag の角度検出ノード
class AprilTagDetector(Node):
    def __init__(self, robot_controller):
        super().__init__('apriltag_detector')
        self.robot_controller = robot_controller  # RobotController インスタンスを保持
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # カメラパラメータ（仮の値なのでキャリブレーション値に置き換えてください）
        self.fx = 496.62  # 焦点距離（X軸方向）
        self.fy = 499.1   # 焦点距離（Y軸方向）
        self.cx = 429.43  # 画像中心のX座標
        self.cy = 233.57  # 画像中心のY座標

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
        self.image_publisher = self.create_publisher(Image, 'apriltag_image', 10)  # 画像トピックをパブリッシュ
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
            camera_params=(self.fx, self.fy, self.cx, self.cy),  # カメラパラメータ
            tag_size=0.122  # Tag size in meters
        )
        
        if tags:
            tag = tags[0]  # 複数タグがあれば最初のタグを使用
            rotation_matrix = np.array(tag.pose_R)
            euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)

            # 現在のヨー角をロボットコントローラに更新
            yaw_angle = euler_angles[2]  # Z軸周りの回転角（ヨー角）
            self.robot_controller.update_current_angle(yaw_angle)

            # タグの中心をカメラ座標系で取得
            circle = tag.center

            self.get_logger().info(f'Tag center: {circle}')
            h = 810  # カメラの高さ (mm)
            r = 300   # ボールの半径 (mm)
            theta = np.radians(40)  # 仮の角度

            # 位置を推定
            x_w, y_w = self.estimate_position(circle, h, r, theta)
            self.get_logger().info(f'Estimated Position (World Coordinates): x={x_w}, y={y_w}')

            # 目標位置を自分で設定（例として(x_target, y_target)を設定）
            x_target = 0  # 目標のx座標
            y_target = 500  # 目標のy座標
            self.robot_controller.move_towards_target(x_target, y_target, x_w, y_w)

        # 検出した AprilTag をフレームに描画
        for tag in tags:
            center = (int(tag.center[0]), int(tag.center[1]))
            cv2.circle(frame, center, 5, (0, 0, 255), 2)
        
        # 画像メッセージをパブリッシュ
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(img_msg)  # 画像をパブリッシュ
        self.get_logger().info("Image published.")

    def estimate_position(self, circle, h, r, theta):
        x_i, y_i = circle
        x_i = x_i - self.cx
        y_i = y_i - self.cy

        y_w = (h - r) / np.tan(theta + np.arctan(y_i / self.fy))
        x_w = (x_i * np.sqrt((h - r)**2 + y_w**2)) / np.sqrt(y_i**2 + self.fx**2)
        return x_w, y_w

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

        # Kalmanフィルタの初期化
        self.kalman = KalmanFilter(dim_x=1, dim_z=1)
        self.kalman.x = np.array([[0.]])  # 初期状態
        self.kalman.F = np.array([[1.]])  # 状態遷移行列
        self.kalman.H = np.array([[1.]])  # 観測行列
        self.kalman.P *= 1000.  # 共分散行列
        self.kalman.R = np.array([[5.]])  # 観測ノイズ共分散
        self.kalman.Q = np.array([[0.1]])  # プロセスノイズ共分散
        self.prev_error = 0

    def control_callback(self):
        # コールバックで実際にロボットを制御する処理は必要ありません
        pass

    def move_towards_target(self, x_target, y_target, x_current, y_current):
        # 目標位置と現在位置の誤差を計算
        xerror = 0.0 * (x_target - x_current)
        yerror = y_target - y_current
        errordistance = math.sqrt(yerror ** 2 + xerror ** 2)
        if errordistance > 100:
            errordistance = 100
        derivative = (errordistance - self.prev_error) / T        

        
        # 誤差角度を計算
        errorangle = math.atan2(yerror, xerror) - self.current_angle
        errorangle = self.constrain_angle(errorangle)  # エラー角度を制限

        # 各モーターの速度を計算
        linearVelocity = errordistance * 0.008 + derivative * 0.008 # 誤差に基づいて速度を調整

        if yerror < 0:
            linearVelocity = -1.0 * linearVelocity

        angularVelocity = errorangle * 0.0    # 角度誤差に基づいて速度を調整
        self.prev_error = errordistance

        # 速度制限
        linearVelocity = max(min(linearVelocity, 1.0), -1.0)  # 最大速度制限
        angularVelocity = max(min(angularVelocity, 1.0), -1.0)  # 最大回転速度制限

        twist = Twist()
        twist.linear.x = linearVelocity
        twist.angular.z = angularVelocity
        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info(f'Moving towards target: linear={linearVelocity}, angular={angularVelocity}')

    def update_current_angle(self, new_angle):
        # Kalmanフィルタで角度を更新
        self.kalman.predict()
        self.kalman.update(np.array([[new_angle]]))
        self.current_angle = self.kalman.x[0, 0]
        self.get_logger().info(f'Updated current angle (Kalman filtered): {self.current_angle}')

    def constrain_angle(self, angle):
        # 角度を-πからπの範囲に制限
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


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
        executor.shutdown()
        robot_controller.destroy_node()
        apriltag_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 