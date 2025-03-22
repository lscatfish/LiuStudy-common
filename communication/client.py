import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        # 创建客户端，指定服务类型为 AddTwoInts，服务名为 add_two_ints
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, a, b):
        # 创建请求消息
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.get_logger().info(f'Sending request: a={a}, b={b}')
        # 发送请求并等待响应
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Received response: sum={future.result().sum}')
        else:
            self.get_logger().info('Service call failed')

def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS2
    client = AddTwoIntsClient()  # 创建客户端节点
    client.send_request(2, 3)  # 发送请求
    rclpy.shutdown()  # 关闭 ROS2

if __name__ == '__main__':
    main()