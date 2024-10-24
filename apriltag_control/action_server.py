import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from move_interfaces.action import MoveToGoal
import serial

class MotionActionServer(Node):
    def __init__(self):
        super().__init__('motion_action_server')
        self._action_server = ActionServer(
            self,
            MoveToGoal,
            'move_to_goal',
            self.execute_callback)
        
        self.ser = serial.Serial('/dev/ttyACM0', 9600)  # ポート名を適切に設定
        self.get_logger().info('Serial connection established.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal: x=%.2f, y=%.2f, theta=%.2f' % (goal_handle.request.target_x, goal_handle.request.target_y, goal_handle.request.target_theta))
        
        # Arduinoに送信するフォーマット: "X:<target_x>,Y:<target_y>,Theta:<target_theta>\n"
        command = f'X:{goal_handle.request.target_x:.2f},Y:{goal_handle.request.target_y:.2f},Theta:{goal_handle.request.target_theta:.2f}\n'
        self.ser.write(command.encode())
        self.get_logger().info(f'Sent to Arduino: {command}')
        
        # 実行結果を設定
        goal_handle.succeed()
        result = MoveToGoal.Result()
        result.success = True
        result.message = 'Command sent to Arduino.'
        return result

def main(args=None):
    rclpy.init(args=args)
    motion_action_server = MotionActionServer()
    rclpy.spin(motion_action_server)
    motion_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
