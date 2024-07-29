/*
  需求：以某个固定频率发送文本“hello world!”，文本后缀编号，每发送一条消息，编号递增1。
  步骤：
	1.包含头文件；
	2.初始化 ROS2 客户端
	3.定义节点类：
	  3-1.创建发布方
	  3-2.创建定时器
	  3-3.组织消息并发布
	4.调用spin函数，并传入节点对象指针；	执行到此函数会返回并调用回调函数
	5.释放资源
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals; // 用于设置定时器时间，可直接用后缀 s 或 ms

// 3.定义节点类
class Talker : public rclcpp::Node
{
public:
	// 初始化节点；	并给 count 初始化赋值
	Talker() : Node("talker_node_cpp"), count(0)
	{
		RCLCPP_INFO(this->get_logger(), "发布节点创建成功！");
		// 3-1.创建发布方
		/**
		 * 模板：被发布的消息类型
		 * 参数：
		 * 	1.话题名称
		 * 	2.队列长度
		 * @return	发布类型指针
		 */
		publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10); // 发布消息的队列为10
		// 3-2.创建定时器	实现按频率发送
		/**
		 * 参数：
		 * 	1.时间间隔
		 * 	2.回调函数
		 * @return	定时器指针
		 */
		timer_ = this->create_wall_timer(1s, std::bind(&Talker::on_timer, this));
	}

private:
	/**
	 * 创建发布者类型
	 * 模板：被发布的消息类型
	 * ::SharedPtr 表示这个发布者对象是以智能指针的形式管理的
	 */
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	size_t count;
	void on_timer()
	{
		// 3-3.组织消息并发布
		auto message = std_msgs::msg::String();										  // 创建String对象
		message.data = "hello world!" + std::to_string(count++);					  // 存入需要发布的消息
		RCLCPP_INFO(this->get_logger(), "发布方发布的消息:%s", message.data.c_str()); // 输出到控制台
		publisher_->publish(message);												  // 发布到dds
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);				  // 2.初始化 ROS2 客户端；
	rclcpp::spin(std::make_shared<Talker>()); // 4.调用spin函数，并传入节点对象指针(执行该对象中的回调函数)；
	rclcpp::shutdown();
	return 0;
}
