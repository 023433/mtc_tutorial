#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>

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


  auto move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm"); // 플래닝 그룹 설정
  auto joint_model_group = move_group_->getRobotModel()->getJointModelGroup("arm");

  // 카르테시안 목표 정의
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.5;

  // 현재 로봇 상태 가져오기
  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(10); // 10ms 대기
  std::vector<double> joint_group_positions;

  // 역기구학 계산
  bool found_ik = current_state->setFromIK(joint_model_group, target_pose);


  if (found_ik) {
    // IK 결과를 관절 공간 목표로 설정
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group_->setJointValueTarget(joint_group_positions);

    // 플래닝 요청
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(node->get_logger(), "Planning successful. Executing the plan...");
        move_group_->execute(my_plan);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to find IK solution for the given Cartesian target.");
  }






























  // // Create the MoveIt MoveGroup Interface
  // using moveit::planning_interface::MoveGroupInterface;
  // auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // double x_ = node->get_parameter("x").as_double();
  // double y_ = node->get_parameter("y").as_double();
  // double z_ = node->get_parameter("z").as_double();
  // double joint1_ = node->get_parameter("joint1").as_double();
  // double joint2_ = node->get_parameter("joint2").as_double();
  // double joint3_ = node->get_parameter("joint3").as_double();
  // double joint4_ = node->get_parameter("joint4").as_double();


  // geometry_msgs::msg::Pose target_pose;
  // target_pose.orientation.w = 1.0;
  // target_pose.position.x = x_;
  // target_pose.position.y = y_;
  // target_pose.position.z = z_;


  // // move_group_interface.setPlanningPipelineId("ompl");
  // // move_group_interface.setPlannerId("RRTConnect");
  // // move_group_interface.setPoseTarget(target_pose);

  // std::map<std::string, double> joint_goal;
  // joint_goal["joint1"] = joint1_;  // Replace with valid joint names and values
  // joint_goal["joint2"] = joint2_;  // Example values
  // joint_goal["joint3"] = joint3_;
  // joint_goal["joint4"] = joint4_;

  // // move_group_interface.setJointValueTarget(joint_goal);
  // geometry_msgs::msg::PoseStamped current_pose;
  // current_pose = move_group_interface.getCurrentPose("end_effector");
  // geometry_msgs::msg::Pose target_pose1;

  // target_pose1.orientation = current_pose.pose.orientation;
  // target_pose1.position.x = 0.1;
  // target_pose1.position.y = 0.1;
  // target_pose1.position.z = 0.1;
  // move_group_interface.setPoseTarget(target_pose1);

  // // move_group_interface.setJointValueTarget(move_group_interface.getNamedTargetValues("init"));



  // // Create a plan to that target pose
  // auto const [success, plan] = [&move_group_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();

  // // Execute the plan
  // if(success) {
  //   move_group_interface.execute(plan);
  // } else {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }
  






  // RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  // RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // auto robot_state = move_group_interface.getCurrentState();

  // if (robot_state == nullptr) {
  //   RCLCPP_ERROR(node->get_logger(), "Failed to get current robot state.");
  // } else {
  //   RCLCPP_INFO(node->get_logger(), "Successfully retrieved current robot state.");
  // }

  // // Extract joint values
  // std::vector<double> joint_values;
  // robot_state->copyJointGroupPositions("arm", joint_values);

  // // Set joint-space target
  // move_group_interface.setJointValueTarget(joint_values);

  // RCLCPP_INFO(node->get_logger(), "Planning succeeded!");

  // // Print the trajectory details (joint names and positions)
  // const auto& trajectory = plan.trajectory_;
  // RCLCPP_INFO(node->get_logger(), "Number of points in the trajectory: %zu", trajectory.joint_trajectory.points.size());

  // // Print the joint names (if available)
  // if (!trajectory.joint_trajectory.joint_names.empty()) {
  //   RCLCPP_INFO(node->get_logger(), "Joint names: ");
  //   for (const auto& joint_name : trajectory.joint_trajectory.joint_names) {
  //     RCLCPP_INFO(node->get_logger(), "%s", joint_name.c_str());
  //   }
  // }

  // // Print each trajectory point (joint positions)
  // for (const auto& point : trajectory.joint_trajectory.points) {
  //   for (size_t i = 0; i < point.positions.size(); ++i) {
  //     RCLCPP_INFO(node->get_logger(), "Joint[%zu] position: %.2f", i, point.positions[i]);
  //   }
  // }

  // // start_state_는 moveit_msgs::msg::RobotState 형식입니다.
  // const moveit_msgs::msg::RobotState& msg_start_state = plan.start_state_;

  // for (size_t i = 0; i < msg_start_state.joint_state.name.size(); ++i) {
  //   RCLCPP_INFO(
  //     node->get_logger(), 
  //     "Joint[%zu]: %s = %.2f", i,
  //     msg_start_state.joint_state.name[i].c_str(),
  //     msg_start_state.joint_state.position[i]
  //   );
  // }

  // Next step goes here

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}