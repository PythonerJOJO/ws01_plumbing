/**
 * 需求：演示参数API使用
 * 1.包含头文件
 * 2.初始化 ROS2 客户端
 * 3.定义节点类：
 *    3-1.参数对象的创建
 *    3-2.参数对象的解析  (获取键、值、将获取的值转换成字符串等···)
 *    3-3.
 * 4.调用 spin() 函数，并传入节点对象指针
 * 5.释放资源
 */
#include "rclcpp/rclcpp.hpp"

// 定义节点类
class MyParam : public rclcpp::Node
{
private:
public:
  MyParam() : Node("my_param_node_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "演示参数API使用");
    // 3-1.参数对象的创建
    rclcpp::Parameter p1("car_name", "tiger");
    rclcpp::Parameter p2("height", 1.68);
    rclcpp::Parameter p3("wheels", 4);

    // 3-2.参数对象的解析  (获取键、值、将获取的值转换成字符串等···)
    // 获取值
    RCLCPP_INFO(this->get_logger(), "car_name = %s,", p1.as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "height = %.3f,", p2.as_double());
    RCLCPP_INFO(this->get_logger(), "wheels = %ld,", p3.as_int());
    // 获取键
    RCLCPP_INFO(this->get_logger(), "name = %s", p1.get_name().c_str());
    // 获取数据类型
    RCLCPP_INFO(this->get_logger(), "type = %s", p1.get_type_name().c_str());
    // 将值转换为字符串类型
    RCLCPP_INFO(this->get_logger(), "value2string = %s", p2.value_to_string().c_str());
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); // 初始化 ROS2 客户端
  rclcpp::spin(std::make_shared<MyParam>());
  rclcpp::shutdown(); // 资源释放
  return 0;
}