#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::placeholders;

class GripperClient : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

  GripperClient() : Node("gripper_client") {
    // Create an action client
    client_ = rclcpp_action::create_client<GripperCommand>(this, "gripper_controller/gripper_cmd");

    // Wait for the action server to become available
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Action server is available.");

    // Send a goal
    send_goal(0.6, 5.0); // Example: Position = 0.05, Max Effort = 5.0
  }

  
  void send_goal(double position, double max_effort) {
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = max_effort;

    RCLCPP_INFO(this->get_logger(), "Sending goal: position = %f, max_effort = %f", position, max_effort);

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(&GripperClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&GripperClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&GripperClient::result_callback, this, _1);

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GripperCommand>::SharedPtr client_;

  void goal_response_callback(const GoalHandleGripperCommand::SharedPtr &goal_handle)
  {
    if (goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Goal accepted by server!");
    }
  }
  void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback)
  {
    (void)feedback;
    RCLCPP_INFO(get_logger(), "feedback :");
  }
  void result_callback(const GoalHandleGripperCommand::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "result :");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(), "canceled!!");
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(get_logger(), "aborted!!");
      return;
    case rclcpp_action::ResultCode::UNKNOWN:
      RCLCPP_INFO(get_logger(), "unknown!!");
      return;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
