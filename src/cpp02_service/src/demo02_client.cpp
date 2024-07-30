/*
  需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
  步骤：
    前提：main函数中判断提交的参数个数
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建客户端；
      3-2.等待服务连接（对于服务通信而言，如果客户端连接不到服务器，那么不能发送请求）；
      3-3.组织请求数据并发送；
    4.创建客户端对象指针调用其功能,
      根据连接结果并处理响应；
    5.释放资源。

*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

// 3.定义节点类
class AddIntsClient : public rclcpp::Node
{
private:
  rclcpp::Client<AddInts>::SharedPtr client_;

public:
  AddIntsClient() : Node("add_ints_client_node")
  {
    // 3-1.创建客户端
    client_ = this->create_client<AddInts>("add_ints");

    RCLCPP_INFO(this->get_logger(), "客户端创建成功，等待连接服务器");
  }
  // 3-2.等待服务连接（对于服务通信而言，如果客户端连接不到服务器，那么不能发送请求）
  bool connect_server()
  {
    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) // 判断程序是否在正常执行中
      {
        // RCLCPP_ERROR(this->get_logger(), "强行终止客户端！"); 此处用this->，那么使用ctrl+c终止程序时会有问题
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "强行终止客户端！");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接中！");
    };
    return true;
  }

  // 3-3.组织请求数据并发送
  /**
   * 编写发送请求函数
   * 参数     两个整形数据
   * 返回值   服务器返回的结果
   */
  rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2)
  {
    // 组织请求数据

    // 发送
    using std::placeholders::_1;
    auto request = std::make_shared<AddInts::Request>();
    request->num1 = num1;
    request->num2 = num2;

    return client_->async_send_request(
        request);
  }
};

int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两整形数据！");
    return 1;
  }
  rclcpp::init(argc, argv); // 初始化 ROS2 客户端

  // 4.创建客户端对象指针调用其功能,并处理响应   调用代码片段 ros2_client_async_send_request
  using std::placeholders::_1;
  // 创建一个 AddIntsClient 类型的智能指针对象,表示一个 ROS2 的服务客户端
  auto client = std::make_shared<AddIntsClient>();
  // 调用客户端对象的连接服务器功能
  bool flag = client->connect_server();
  // 根据连接结果做进一步处理
  if (!flag)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "服务器连接失败，程序退出");
    return 0;
  }

  // 调用请求提交函数，接收并处理响应结果
  auto future = client->send_request(atoi(argv[1]), atoi(argv[2]));
  // 处理响应
  if (!(rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)) // 失败
  {
    RCLCPP_ERROR(client->get_logger(), "响应失败!");
  }
  else // 成功
  {
    RCLCPP_INFO(client->get_logger(), "响应成功! sum = %d", future.get()->sum);
  }

  // rclcpp::spin(std::make_shared<AddIntsClient>());     客户端无需挂起
  rclcpp::shutdown(); // 资源释放
  return 0;
}