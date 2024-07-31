#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
// 定义节点类
class Time : public rclcpp::Node
{
private:
  void demo_rate()
  {
    rclcpp::Rate loop_rate1(500ms); // 创建 Rate 对象,使用毫秒、秒或纳秒  休眠500ms
    rclcpp::Rate loop_rate2(1.0);   // 创建 Rate 对象，使用频率
                                    // 调用 Rate 的sleep函数
    while (rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "-------------------");
      // loop_rate1.sleep();
      loop_rate2.sleep();
    }
  }

public:
  Time() : Node("time_node")
  {
    demo_rate();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); // 初始化 ROS2 客户端
  rclcpp::spin(std::make_shared<Time>());
  rclcpp::shutdown(); // 资源释放
  return 0;
}
