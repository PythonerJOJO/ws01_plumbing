/*
    需求：订阅发布方发布的学生消息，并输出到终端。
    步骤：
        1.包含头文件
        2.初始化 ROS2 服务端
        3.定义节点类
            3.1.创建订阅方
            3.2.回调函数订阅并处理Student类型消息

        4.调用spin()函数，并传入节点对象指针
        5.释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;

// 节点对象
class ListenerStu : public rclcpp::Node
{
private:
    rclcpp::Subscription<Student>::SharedPtr subscription_;

    void do_callback(const Student &stu)
    {
        // 3.2.回调函数订阅并处理Student类型消息
        RCLCPP_INFO(this->get_logger(), "订阅的学生信息：name = %s，age = %d，height = %.2f", stu.name.c_str(), stu.age, stu.height);
    }

public:
    ListenerStu() : Node("listener_stu_node")
    {
        // 1.创建订阅方
        using std::placeholders::_1;
        subscription_ = this->create_subscription<Student>(
            "chatter_stu", 10, std::bind(&ListenerStu::do_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "订阅方创建成功！");
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化 ROS2 客户端
    rclcpp::spin(std::make_shared<ListenerStu>());
    rclcpp::shutdown(); // 资源释放
    return 0;
}