import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class CarrotTFBroadcaster(Node):
    def __init__(self):
        super().__init__('carrot_tf2_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('direction_of_rotation', 1)
        self.radius = self.get_parameter('radius').value
        self.direction = self.get_parameter('direction_of_rotation').value
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot'
        angle = self.get_clock().now().to_msg().sec * 0.1 * self.direction
        t.transform.translation.x = self.radius * math.cos(angle)
        t.transform.translation.y = self.radius * math.sin(angle)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CarrotTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
