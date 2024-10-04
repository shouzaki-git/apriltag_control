import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import serial


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.declare_parameter('topic', 'image_raw')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.bridge = CvBridge()
        self.create_subscription(Image, self.topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        # ROS2のImageメッセージをOpenCVの画像に変換
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 画像をウィンドウに表示
        cv.imshow("AprilTag Detection", cv_img)
        cv.waitKey(1)


class DirectionSubscriber(Node):
    def __init__(self):
        super().__init__('direction_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        # シリアルポートの設定 (例: Arduinoと通信する場合)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info('Serial connection established.')
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to establish serial connection: {e}")

    def listener_callback(self, twist):
        # Twistメッセージから線形速度と角速度を取得
        linear_x = twist.linear.x
        angular_z = twist.angular.z

        # 速度データをArduinoに送信 (例: "L:0.5,A:0.2\n" のようなフォーマットで送信)
        command = f'L:{linear_x:.2f},A:{angular_z:.2f}\n'
        try:
            self.ser.write(command.encode())  # シリアル通信で送信
            self.get_logger().info(f'Sent to Arduino: {command}')
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send data to Arduino: {e}")


def main(args=None):
    rclpy.init(args=args)

    # ノードの初期化
    image_subscriber = ImageSubscriber()
    direction_subscriber = DirectionSubscriber()

    # 複数のノードを動作させるためにExecutorを使用
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(image_subscriber)
    executor.add_node(direction_subscriber)

    try:
        executor.spin()
    finally:
        # ノードのクリーンアップ
        image_subscriber.destroy_node()
        direction_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
