/*
    需求：编写动作客户端实现，可以提交一个整型数据到服务端，并处理服务端的连续反馈以及最终返回结果。
        附加：可以解析终端下动态传入的参数
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建动作客户端；
            3-2.发送请求；
            3-3.处理目标发送后的反馈；      回调函数实现
            3-4.处理连续反馈；              回调函数实现
            3-5.处理最终响应。              回调函数实现
        4.调用spin函数，并传入节点对象指针；
        5.释放资源。
*/
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/action/progress.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using base_interfaces_demo::action::Progress;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Progress>;

// 定义节点类
class ProgressActionClient : public rclcpp::Node
{
private:
    rclcpp_action::Client<Progress>::SharedPtr client_;

    // 3-3.处理目标发送后的反馈；      回调函数实现
    void goal_response_callback(const ClientGoalHandle::SharedPtr &future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) // 判断 goal_handle 是否为空指针
        {
            RCLCPP_ERROR(this->get_logger(), "目标请求被服务端拒绝！");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "目标处理中");
        }
    }

    // 3-4.处理连续反馈；              回调函数实现
    void feedback_callback(
        ClientGoalHandle::SharedPtr,
        const std::shared_ptr<const Progress::Feedback> feedback)
    {
        double progress = feedback->progress;
        int pro = (int)(progress * 100);
        RCLCPP_INFO(this->get_logger(), "接收到的反馈：%d%%", pro);
    }

    // 3-5.处理最终响应。              回调函数实现
    void result_callback(const ClientGoalHandle::WrappedResult &result)
    {
        switch (result.code) // 通过状态码判断当前响应结果的状态
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "最终结果：%d", result.result->sum);
            break;
        case rclcpp_action::ResultCode::ABORTED: // 被强行终止
            RCLCPP_ERROR(this->get_logger(), "被中断");
            return;
        case rclcpp_action::ResultCode::CANCELED: // 被取消
            RCLCPP_ERROR(this->get_logger(), "被取消");
            return;
        default: // 未知异常
            RCLCPP_ERROR(this->get_logger(), "未知异常");
            return;
        }
    }

public:
    ProgressActionClient() : Node("progress_action_client")
    {
        RCLCPP_INFO(this->get_logger(), "action客户端创建！");
        client_ = rclcpp_action::create_client<Progress>(
            this, "get_sum");
    }
    void send_goal(int num)
    {

        while (!client_->wait_for_action_server(1s)) // 连接服务端
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the action server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Action server not available, waiting again...");
        }

        // 发送具体请求
        auto goal = Progress::Goal();
        goal.num = num;
        auto send_goal_options =
            rclcpp_action::Client<Progress>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ProgressActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ProgressActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&ProgressActionClient::result_callback, this, _1);
        RCLCPP_INFO(this->get_logger(), "发送目标");
        // 异步发送目标
        auto future = client_->async_send_goal(goal, send_goal_options);
    }
};

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整形数据！");
        return 1;
    }
    rclcpp::init(argc, argv); // 初始化 ROS2 客户端

    auto node = std::make_shared<ProgressActionClient>();
    node->send_goal(std::stoi(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown(); // 资源释放
    return 0;
}