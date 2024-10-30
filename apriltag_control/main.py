import rclpy
from rclpy.executors import MultiThreadedExecutor
from motion_server import MotionActionServer
from apriltag_server import AprilTagDetector
import cv2

def main(args=None):
    rclpy.init(args=args)
    
    # ノードのインスタンス化
    motion_server = MotionActionServer()
    apriltag_server = AprilTagDetector()

    # マルチスレッドエグゼキューターを使用してノードを同時に実行
    executor = MultiThreadedExecutor()
    executor.add_node(motion_server)
    executor.add_node(apriltag_server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        apriltag_server.cap.release()
        cv2.destroyAllWindows()
        motion_server.destroy_node()
        apriltag_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
