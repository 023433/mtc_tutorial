#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.15;
    msg.position.y = -0.1;
    msg.position.z = 0.2;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setPlanningPipelineId("ompl");

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  RCLCPP_INFO(node->get_logger(), "Planning succeeded!");

  // Print the trajectory details (joint names and positions)
  const auto& trajectory = plan.trajectory_;
  RCLCPP_INFO(node->get_logger(), "Number of points in the trajectory: %zu", trajectory.joint_trajectory.points.size());

  // Print the joint names (if available)
  if (!trajectory.joint_trajectory.joint_names.empty()) {
    RCLCPP_INFO(node->get_logger(), "Joint names: ");
    for (const auto& joint_name : trajectory.joint_trajectory.joint_names) {
      RCLCPP_INFO(node->get_logger(), "%s", joint_name.c_str());
    }
  }

  // Print each trajectory point (joint positions)
  for (const auto& point : trajectory.joint_trajectory.points) {
    for (size_t i = 0; i < point.positions.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "Joint[%zu] position: %.2f", i, point.positions[i]);
    }
  }

  // start_state_는 moveit_msgs::msg::RobotState 형식입니다.
  const moveit_msgs::msg::RobotState& msg_start_state = plan.start_state_;

  for (size_t i = 0; i < msg_start_state.joint_state.name.size(); ++i) {
    RCLCPP_INFO(
      node->get_logger(), 
      "Joint[%zu]: %s = %.2f", i,
      msg_start_state.joint_state.name[i].c_str(),
      msg_start_state.joint_state.position[i]
    );
  }

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Next step goes here

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}