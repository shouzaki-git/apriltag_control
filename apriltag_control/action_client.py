import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from move_interfaces.action import MoveToGoal

class MotionActionClient(Node):
    def __init__(self):
        super().__init__('motion_action_client')
        self._action_client = ActionClient(self, MoveToGoal, 'move_to_goal')

    def send_goal(self, target_x, target_y, target_theta):
        goal_msg = MoveToGoal.Goal()
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.target_theta = target_theta
        
        self._action_client.wait_for_server()
        self.send_goal_async(goal_msg)

    def send_goal_async(self, goal_msg):
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

def main(args=None):
    rclpy.init(args=args)
    motion_action_client = MotionActionClient()
    motion_action_client.send_goal(1.0, 2.0, 3.14)  # 目標位置と角度を指定
    rclpy.spin(motion_action_client)
    motion_action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
