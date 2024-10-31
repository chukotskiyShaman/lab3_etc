#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovementsNode(Node):
    def __init__(self):
        super().__init__('movements_node')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.drift_factor = 4.0  # Коэффициент заноса (чем больше, тем сильнее занос)
        self.direction = 1  # Направление угловой скорости: 1 - вправо, -1 - влево
        self.change_direction_time = 1.0  # Время до смены направления
        self.last_change_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        if current_time - self.last_change_time > self.change_direction_time:
            self.direction *= -1  # Меняем направление угловой скорости
            self.last_change_time = current_time

        twist_msg = Twist()
        twist_msg.linear.x = 2.0  # Линейная скорость вперед
        twist_msg.angular.z = self.direction * self.drift_factor * twist_msg.linear.x  # Угловая скорость для заноса
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovementsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
