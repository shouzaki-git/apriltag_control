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
            self.execute_callback
        )
        
        # シリアルポート設定
        self.ser = serial.Serial('/dev/ttyACM0', 9600)  
        self.get_logger().info('Serial connection established.')

    def execute_callback(self, goal_handle):
        # 目標位置の取得
        self.get_logger().info(f'Received goal: x={goal_handle.request.target_x}, y={goal_handle.request.target_y}, theta={goal_handle.request.target_theta}')
        
        # コマンド生成と送信
        command = f'X:{goal_handle.request.target_x:.2f},Y:{goal_handle.request.target_y:.2f},Theta:{goal_handle.request.target_theta:.2f}\n'
        self.ser.write(command.encode())
        self.get_logger().info(f'Sent to Arduino: {command}')
        
        # 結果の生成と送信
        goal_handle.succeed()
        result = MoveToGoal.Result()
        result.success = True
        result.message = 'Command sent to Arduino.'
        return result
