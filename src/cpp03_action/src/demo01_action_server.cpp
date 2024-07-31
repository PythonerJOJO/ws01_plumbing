/*
	需求：编写动作服务端实习，可以提取客户端请求提交的整型数据，并累加从1到该数据之间的所有整数以求和，
		每累加一次都计算当前运算进度并连续反馈回客户端，最后，在将求和结果返回给客户端。
		附加：可以解析终端下动态传入的参数
	分析：
		1.创建动作服务器对象；
		2.处理提交的目标值
		3.生成连续反馈
		4.响应最终结果
		5.处理取消请求
	步骤：
		1.包含头文件；
		2.初始化 ROS2 客户端；
		3.定义节点类；
			3-1.创建动作服务端；
			3-2.处理请求数据；				通过回调函数
			3-3.处理取消任务请求。			通过回调函数
			3-4.生成连续反馈与最终响应		通过回调函数
		4.调用spin函数，并传入节点对象指针；
		5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using namespace std::placeholders;
using base_interfaces_demo::action::Progress;

using std::placeholders::_1;
using std::placeholders::_2;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Progress>;

// 定义节点类
class ProgressActionServer : public rclcpp::Node
{
private:
	// GoalHandleProgress::SharedPtr action_server_;
	rclcpp_action::Server<Progress>::SharedPtr server_;

	/**3-2.处理请求数据；				通过回调函数
	 * std::function<GoalResponse(
		const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
	 */
	rclcpp_action::GoalResponse goal_callback(
		const rclcpp_action::GoalUUID &uuid,
		std::shared_ptr<const Progress::Goal> goal)
	{
		(void)uuid; // 防止编译器警告
		// 业务逻辑		判断提交的数字是否>1
		if (!(goal->num > 1))
		{
			RCLCPP_ERROR(this->get_logger(), "提交的目标值必须 大于 1 !");
			// 拒绝了
			return rclcpp_action::GoalResponse::REJECT;
		}
		RCLCPP_INFO(this->get_logger(), "提交的目标值合法，接收请求");
		// 接收并执行了，		rclcpp_action::GoalResponse::ACCEPT_AND_DEFER为推迟执行
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**3-3.处理取消任务请求。			通过回调函数
	 * std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
	 */
	rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<ServerGoalHandle> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "接收到取消请求！");
		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	/**3-4.生成连续反馈与最终响应		通过回调函数
	 * std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>
	 */
	void accepted_callback(const std::shared_ptr<ServerGoalHandle> goal_handle)
	{
		/**耗时操作，单独开一个线程处理响应
		 *
		 */
		std::thread(std::bind(&ProgressActionServer::execute, this, _1), goal_handle).detach();
	}

	void execute(const std::shared_ptr<ServerGoalHandle> goal_handle)
	{

		RCLCPP_INFO(this->get_logger(), "Executing goal");
		// 生成连续反馈返回给客户端
		const auto goal = goal_handle->get_goal(); // 获取目标对象指针
		int num = goal->num;
		int sum = 0;
		auto feedback = std::make_shared<Progress::Feedback>(); // 反馈指针
		rclcpp::Rate loop_rate(1);								// 用于对耗时操作休眠,每隔 1s 执行一次
		auto result = std::make_shared<Progress::Result>();
		for (int i = 1; i <= num; i++)
		{
			if (goal_handle->is_canceling())
			{
				result->sum = sum;
				goal_handle->canceled(result);
				RCLCPP_ERROR(this->get_logger(), "任务被取消！当前结果：%d", sum);
				return;
			}
			sum += i;
			double progress = i / (double)num; // 计算进度
			feedback->progress = progress;
			goal_handle->publish_feedback(feedback);
			RCLCPP_INFO(this->get_logger(), "连续反馈中，进度：%.2f", progress);
			loop_rate.sleep();
		}

		// 生成响应结果
		result->sum = sum;
		goal_handle->succeed(result); // 将 result 响应回去
		RCLCPP_INFO(this->get_logger(), "最终响应结果：%d", sum);
	}

public:
	ProgressActionServer() : Node("progress_action_server")
	{
		RCLCPP_INFO(this->get_logger(), "action服务端创建");
		// 3-1.创建动作服务端；
		/**
		 *
		 */
		server_ = rclcpp_action::create_server<Progress>(
			this,
			"get_sum",
			std::bind(&ProgressActionServer::goal_callback, this, _1, _2),
			std::bind(&ProgressActionServer::cancel_callback, this, _1),
			std::bind(&ProgressActionServer::accepted_callback, this, _1));
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv); // 初始化 ROS2 客户端
	rclcpp::spin(std::make_shared<ProgressActionServer>());
	rclcpp::shutdown(); // 资源释放
	return 0;
}