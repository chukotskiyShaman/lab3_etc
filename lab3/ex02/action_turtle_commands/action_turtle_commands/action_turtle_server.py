import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from actions.action import MessageTurtleCommands
import math
import threading

class TurtleActionServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'turtle_command',
            self.execute_callback)
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.current_pose = Pose()
        self.start_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        command = goal_handle.request.command
        s = goal_handle.request.s
        angle = goal_handle.request.angle
        # print(goal_handle.request)

        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0.0

        twist = Twist()
        self.start_pose = self.current_pose

        if command == 'forward':
            twist.linear.x = float(s)
            print('in forward')
            while self.calculate_distance() < s:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return MessageTurtleCommands.Result()

                self.publisher_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.1)
                feedback_msg.odom = self.calculate_distance()
                goal_handle.publish_feedback(feedback_msg)

        elif command == 'turn':
            
            while abs(abs(self.current_pose.theta - self.start_pose.theta)-math.radians(abs(angle))) > 0.005 and abs(abs(self.current_pose.theta) - self.start_pose.theta)-math.radians(abs(angle)) < math.radians(abs(angle*2)):
                twist.angular.z = (abs((self.current_pose.theta) - (self.start_pose.theta)) - math.radians(angle))/2
                print(self.current_pose.theta, self.start_pose.theta, math.radians(angle))
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return MessageTurtleCommands.Result()

                self.publisher_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.001)
                feedback_msg.odom = abs(self.current_pose.theta - self.start_pose.theta)
                goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = MessageTurtleCommands.Result()
        result.result = True
        return result

    def calculate_distance(self):
        return math.sqrt((self.current_pose.x - self.start_pose.x)**2 + (self.current_pose.y - self.start_pose.y)**2)

def main(args=None): 
    rclpy.init(args=args) 
    turtle_action_server = TurtleActionServer() 
 
    executor = rclpy.executors.SingleThreadedExecutor() 
    executor.add_node(turtle_action_server) 
 
    # Запуск executor в отдельном потоке 
    executor_thread = threading.Thread(target=executor.spin) 
    executor_thread.start() 
 
    try: 
        while rclpy.ok(): 
            rclpy.spin_once(turtle_action_server, timeout_sec=0.1) 
    except KeyboardInterrupt: 
        pass 
    finally: 
        executor.shutdown() 
        turtle_action_server.destroy_node() 
        rclpy.shutdown() 
        executor_thread.join() 
 
if __name__ == '__main__': 
    main()

