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
    void do_callback(const std_msgs::msg::String &msg)
    {
        // 3-2.处理订阅到的消息
        RCLCPP_INFO(this->get_logger(), "订阅到的消息是：%s", msg.data.c_str());
    }

public:
    Listener() : Node("listener_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "订阅方创建");
        // 3-1.创建订阅方
        /**
         * @note    std::bind()可以将一个函数或者函数对象与其部分参数绑定在一起,生成一个新的可调用对象
         * 模板：消息类型
         * @param
         *      1.话题名
         *      2.QOS,队列长度
         *      3.回调函数，回调函数中要处理订阅到的数据(要绑定的成员函数指针,要绑定的对象指针:也就是当前 Listener 类的实例,哪一个参数会被传递给 do_callback 函数)
         * @return  订阅对象指针
         */
        subscription_ = this->create_subscription<std_msgs::msg::String>("chatter", 10,
                                                                         std::bind(&Listener::do_callback, this, std::placeholders::_1));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
