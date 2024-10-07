import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class MoveToGoalNode(Node):
    def __init__(self):
        super().__init__('move_to_goal_node')
        
        # Получение параметров из командной строки
        self.declare_parameter('x', 5.0)
        self.declare_parameter('y', 5.0)
        self.declare_parameter('theta', 0.0)
        
        self.goal_x = self.get_parameter('x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('y').get_parameter_value().double_value
        self.goal_theta = self.get_parameter('theta').get_parameter_value().double_value
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.current_pose = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.goal_reached = False

    def pose_callback(self, msg):
        self.current_pose = msg

    def timer_callback(self):
        if self.current_pose is None:
            return
        
        # Расчет ошибки положения
        error_x = self.goal_x - self.current_pose.x
        error_y = self.goal_y - self.current_pose.y
        error_theta = self.goal_theta - self.current_pose.theta
        
        # Расчет расстояния до цели
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # Расчет угла до цели
        angle_to_goal = math.atan2(error_y, error_x)
        
        # Создание сообщения Twist
        twist_msg = Twist()
        
        # Если расстояние больше порога, двигаемся вперед
        if distance > 0.1:
            twist_msg.linear.x = 1.5 * distance
            twist_msg.angular.z = 6.0 * (angle_to_goal - self.current_pose.theta)
        else:
            # Если расстояние мало, поворачиваемся на нужный угол
            twist_msg.angular.z = 6.0 * error_theta
        
        self.publisher_.publish(twist_msg)
        
        # Проверка, достиг ли робот цели
        if distance < 0.1 and abs(error_theta) < 0.1:
            self.goal_reached = True
            self.get_logger().info("Goal reached! Stopping the node.")
            self.timer.cancel()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.goal_reached:
            break
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
