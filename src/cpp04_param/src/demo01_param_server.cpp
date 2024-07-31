/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"

// 定义节点类
class MyNode : public rclcpp::Node
{
private:
public:
    MyNode() : Node("node_name")
    {
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化 ROS2 客户端
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown(); // 资源释放
    return 0;
}