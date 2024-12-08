#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


// plan 출력 함수
void printPlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Plan Details:");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning Time: %.2f seconds", plan.planning_time_);

  const auto& points = plan.trajectory_.joint_trajectory.points;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of Trajectory Points: %zu", points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    const auto& point = points[i];
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point %zu:", i + 1);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Positions: ");
    for (const auto& position : point.positions) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%.2f ", position);
    }
  }
}

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

  const std::string PLANNING_GROUP = "arm";

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);


  double x_ = node->get_parameter("x").as_double();
  double y_ = node->get_parameter("y").as_double();
  double z_ = node->get_parameter("z").as_double();
  double ow_ = node->get_parameter("ow").as_double();
  double ox_ = node->get_parameter("ox").as_double();
  double oz_ = node->get_parameter("oz").as_double();
  double oy_ = node->get_parameter("oy").as_double();
  // double joint1_ = node->get_parameter("joint1").as_double();
  // double joint2_ = node->get_parameter("joint2").as_double();
  // double joint3_ = node->get_parameter("joint3").as_double();
  // double joint4_ = node->get_parameter("joint4").as_double();

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = ox_;
  target_pose.orientation.y = oy_;
  target_pose.orientation.z = oz_;
  target_pose.orientation.w = ow_;
  target_pose.position.x = x_;
  target_pose.position.y = y_;
  target_pose.position.z = z_;


  move_group_interface.setPoseTarget(target_pose);
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    printPlan(plan); 
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}