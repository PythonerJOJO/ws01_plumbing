/**
 * 需求：以某个固定频率发送文本“hello world！”，文本后缀编号，每发送一条消息，编号递增 1
 * 步骤：
 *  1.包含头文件
 *  2.初始化 ROS2 客户端
 *  3.定义节点类:
 *      3-1.创建发布方
 *      3-2.创建定时器
 *      3-3.组织消息并发布
 *  4.调用spin函数，并传入节点对象指针；    spin是回旋函数，执行到spin便调用回调函数
 */
// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// 3.定义节点类
class MinimalPublisher : public rclcpp::Node
{
private:
    /* data */
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        // 3-1.创建发布方

        // 3-2.创建定时器
    }
    ~MinimalPublisher() {};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared</* namespace_name::ClassName */>());

    rclcpp::shutdown();
    return 0;
}
