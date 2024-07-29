/*
    需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数，并传入节点对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 定义节点类
class Listener : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void on_callback()
    {
    }

public:
    Listener() : Node("listener_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "订阅方创建");
        // subscription_ = this->create_subscription<std_msgs::msg::String, this->on_callback()>("talker", 10, this->on_callback());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
