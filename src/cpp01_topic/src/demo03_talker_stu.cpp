/*
  需求：以某个固定频率发送文本学生信息，包含学生的姓名、年龄、身高等数据。
  流程：
    1.包含头文件；
    2.初始化 ROS2 客户端
    3.定义节点类：
      3-1.创建发布方
      3-2.创建定时器
      3-3.组织并发布学生消息
    4.调用spin函数，并传入节点对象指针；	执行到此函数会返回并调用回调函数
    5.释放资源
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;

// 节点对象
class TalkerStu : public rclcpp::Node
{
private:
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int32_t age;
    void on_timer()
    {
        // 3-3.组织并发布学生消息
        auto stu = Student();
        stu.name = "大娃";
        stu.age = age;
        stu.height = 2.20;
        RCLCPP_INFO(this->get_logger(), "发布的消息：(%s,%d,%.2f)", stu.name.c_str(), stu.age, stu.height);
        publisher_->publish(stu);
        age++;
    }

public:
    TalkerStu() : Node("talker_stu_node"), age(0)
    {
        // 3-1.创建发布方
        publisher_ = this->create_publisher<Student>("chatter_stu", 10);
        RCLCPP_INFO(this->get_logger(), "发布方创建完成！");
        // 3-2.创建定时器
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&TalkerStu::on_timer, this));
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化 ROS2 客户端
    rclcpp::spin(std::make_shared<TalkerStu>());
    rclcpp::shutdown(); // 资源释放
    return 0;
}