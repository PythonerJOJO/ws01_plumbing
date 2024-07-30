"""  
    需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__("listener_node_py")
        
        # 3-1.创建订阅方
        self.subscription_ = self.create_subscription(
            String, 'chatter', self.do_callback, 10)
        
        self.get_logger().info('订阅方创建(python)！')
    
    def do_callback(self,msg):
        # 3-2.处理订阅到的消息
        self.get_logger().info('订阅的数据：%s'%msg.data)

        

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Listener())  # 调用spin函数，并传入节点对象
    rclpy.shutdown()

if __name__ == '__main__':
    main()
