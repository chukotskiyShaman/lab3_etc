import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from actions.action import MessageTurtleCommands
from action_msgs.msg import GoalStatus

class TurtleActionClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'turtle_command')
        self.goals = []
        self.current_goal_index = 0

    def send_goal(self, command, s, angle):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = s
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.odom}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'Goal succeeded! Result: {result}')
            self.send_next_goal()
        else:
            self.get_logger().info(f'Goal failed with status: {status}')

        # После завершения текущей цели, отправляем следующую
        

    def send_next_goal(self):
        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
            # print(goal)
            self.current_goal_index += 1
            future = self.send_goal(goal['command'], goal['s'], goal['angle'])
            future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info('All goals completed')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = TurtleActionClient()

    # Список целей для последовательного выполнения
    action_client.goals = [
        {'command': 'forward', 's': 2, 'angle': 0},
        {'command': 'turn', 's': 0, 'angle': 90},  # 90 degrees in radians
        {'command': 'forward', 's': 1, 'angle': 0},
    ]
    action_client.current_goal_index = 0

    # Начинаем с первой цели
    action_client.send_next_goal()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
