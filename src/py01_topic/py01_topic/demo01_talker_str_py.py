"""  
    需求：以某个固定频率发送文本“hello world!”，文本后缀编号，每发送一条消息，编号递增1。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建发布方；
            3-2.创建定时器；
            3-3.组织消息并发布。
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super.__init__("talker_node_py")
        self.get_logger().info("发布方创建(python)!")
        # 3-1.创建发布方
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # 3-2.创建定时器
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World(py):%d'%self.i
        self.publisher_.publish(msg)
        self.get_logger().info('发布的消息："%s"'% msg.data)
        self.i +=1




def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Talker)  # 调用spin函数，并传入节点对象
    rclpy.shutdown()

if __name__ == '__main__':
    main()




