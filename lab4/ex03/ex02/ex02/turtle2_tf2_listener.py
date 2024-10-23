import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist
import math

class Turtle2TFListener(Node):
    def __init__(self):
        super().__init__('turtle2_tf2_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('turtle2', 'carrot', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info('Transform not found')
            return

        msg = Twist()
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        msg.angular.z = 4.0 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Turtle2TFListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
