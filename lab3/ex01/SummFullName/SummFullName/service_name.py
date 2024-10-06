# service_name.py
import rclpy
from rclpy.node import Node
from fullnamemessage.srv import FullNameSumService

class SummFullNameService(Node):
    def __init__(self):
        super().__init__('summ_full_name_service')
        self.srv = self.create_service(FullNameSumService, 'summ_full_name', self.summ_full_name_callback)

    def summ_full_name_callback(self, request, response):
        full_name = f"{request.last_name} {request.first_name} {request.middle_name}"
        response.full_name = full_name
        self.get_logger().info(f"Received request: {request.last_name}, {request.first_name}, {request.middle_name}")
        self.get_logger().info(f"Sending back response: {response.full_name}")
        return response

def main(args=None):
    rclpy.init(args=args)
    service = SummFullNameService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

