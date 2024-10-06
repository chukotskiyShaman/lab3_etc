# client_name.py
import sys
import rclpy
from rclpy.node import Node
from fullnamemessage.srv import FullNameSumService

class SummFullNameClient(Node):
    def __init__(self, last_name, first_name, middle_name):
        super().__init__('summ_full_name_client')
        self.cli = self.create_client(FullNameSumService, 'summ_full_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FullNameSumService.Request()
        self.req.last_name = last_name
        self.req.first_name = first_name
        self.req.middle_name = middle_name

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    if len(sys.argv) < 4:
        print("Usage: client_name.py last_name first_name middle_name")
        return

    last_name = sys.argv[1]
    first_name = sys.argv[2]
    middle_name = sys.argv[3]

    rclpy.init(args=args)
    client = SummFullNameClient(last_name, first_name, middle_name)
    response = client.send_request()
    client.get_logger().info(f"Result of summ_full_name: {response.full_name}")
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
