import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # 假设服务接口在 example_interfaces 包中

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        # 创建服务，指定服务类型为 AddTwoInts，服务名为 add_two_ints
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts server is ready.')

    def add_two_ints_callback(self, request, response):
        # 计算两个整数的和
        response.sum = request.a + request.b
        self.get_logger().info(f'Received request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS2
    server = AddTwoIntsServer()  # 创建服务端节点
    rclpy.spin(server)  # 保持节点运行
    rclpy.shutdown()  # 关闭 ROS2

if __name__ == '__main__':
    main()