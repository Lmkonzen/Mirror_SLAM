#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <gap_explorer_interfaces/action/probe_arm.hpp>

class ArmProbeServer
{
public:
  using ProbeArm = gap_explorer_interfaces::action::ProbeArm;
  using GoalHandleProbeArm = rclcpp_action::ServerGoalHandle<ProbeArm>;

  explicit ArmProbeServer(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    logger_(node_->get_logger()),
    is_moving_(false),
    startup_done_(false)
  {
    move_group_interface_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, "manipulator");

    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setNumPlanningAttempts(10);
    move_group_interface_->setMaxVelocityScalingFactor(0.2);
    move_group_interface_->setMaxAccelerationScalingFactor(0.2);

    action_server_ = rclcpp_action::create_server<ProbeArm>(
      node_,
      "probe_arm",
      std::bind(&ArmProbeServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmProbeServer::handleCancel, this, std::placeholders::_1),
      std::bind(&ArmProbeServer::handleAccepted, this, std::placeholders::_1)
    );

    startup_timer_ = node_->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ArmProbeServer::startupMove, this));

    RCLCPP_INFO(logger_, "arm_probe_cpp action server started: probe_arm");
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ProbeArm::Goal> goal)
  {
    RCLCPP_INFO(
      logger_,
      "Received probe goal: extend_distance_m=%.3f timeout_sec=%.3f",
      goal->extend_distance_m,
      goal->timeout_sec);

    if (!startup_done_) {
      RCLCPP_WARN(logger_, "Rejecting probe goal: startup move not finished yet");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (is_moving_) {
      RCLCPP_WARN(logger_, "Rejecting probe goal: arm already moving");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (std::abs(goal->extend_distance_m) < 1e-6) {
      RCLCPP_WARN(logger_, "Rejecting probe goal: extend distance is zero");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleProbeArm>)
  {
    RCLCPP_WARN(logger_, "Received request to cancel probe");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleProbeArm> goal_handle)
  {
    std::thread{std::bind(&ArmProbeServer::executeProbe, this, goal_handle)}.detach();
  }

  void startupMove()
  {
    if (startup_done_) {
      return;
    }

    startup_timer_->cancel();

    RCLCPP_INFO(logger_, "Moving to initial cocked-forward pose...");

    // Tune these values as needed for your specific Kinova model/config.
    // The intent is a ready-to-poke posture with the wrist pitched forward.
    std::vector<double> joint_targets = {
      0.0,   // joint 1: base
      0.75,  // joint 2: shoulder slightly forward
      3.14,  // joint 3
      1.10,  // joint 4: elbow bend
      0.0,   // joint 5
      1.00,  // joint 6: wrist pitched forward
      1.57   // joint 7
    };

    bool success = moveToJointPositions(joint_targets);

    if (success) {
      startup_done_ = true;
      RCLCPP_INFO(logger_, "Initial cocked-forward pose reached");
    } else {
      RCLCPP_ERROR(logger_, "Failed to reach initial cocked-forward pose");
    }
  }

  void executeProbe(const std::shared_ptr<GoalHandleProbeArm> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<ProbeArm::Feedback>();
    auto result = std::make_shared<ProbeArm::Result>();

    if (goal_handle->is_canceling()) {
      feedback->state = "canceled";
      goal_handle->publish_feedback(feedback);

      result->success = false;
      result->object_detected = false;
      result->message = "Probe canceled before start";
      goal_handle->canceled(result);
      return;
    }

    if (is_moving_) {
      result->success = false;
      result->object_detected = true;
      result->message = "Arm busy";
      goal_handle->abort(result);
      return;
    }

    is_moving_ = true;

    feedback->state = "reading_current_pose";
    goal_handle->publish_feedback(feedback);

    auto current_pose_stamped = move_group_interface_->getCurrentPose();
    auto target_pose = current_pose_stamped.pose;

    target_pose.position.x += goal->extend_distance_m;

    RCLCPP_INFO(
      logger_,
      "Current pose: x=%.3f y=%.3f z=%.3f",
      current_pose_stamped.pose.position.x,
      current_pose_stamped.pose.position.y,
      current_pose_stamped.pose.position.z);

    RCLCPP_INFO(
      logger_,
      "Probe target: x=%.3f y=%.3f z=%.3f",
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z);

    if (goal_handle->is_canceling()) {
      move_group_interface_->stop();
      move_group_interface_->clearPoseTargets();
      is_moving_ = false;

      feedback->state = "canceled";
      goal_handle->publish_feedback(feedback);

      result->success = false;
      result->object_detected = false;
      result->message = "Probe canceled";
      goal_handle->canceled(result);
      return;
    }

    feedback->state = "planning";
    goal_handle->publish_feedback(feedback);

    move_group_interface_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = static_cast<bool>(move_group_interface_->plan(plan));

    if (!planned) {
      move_group_interface_->clearPoseTargets();
      is_moving_ = false;

      feedback->state = "blocked";
      goal_handle->publish_feedback(feedback);

      result->success = false;
      result->object_detected = true;
      result->message = "Planning failed; path appears blocked";
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      move_group_interface_->stop();
      move_group_interface_->clearPoseTargets();
      is_moving_ = false;

      feedback->state = "canceled";
      goal_handle->publish_feedback(feedback);

      result->success = false;
      result->object_detected = false;
      result->message = "Probe canceled after planning";
      goal_handle->canceled(result);
      return;
    }

    feedback->state = "executing";
    goal_handle->publish_feedback(feedback);

    auto exec_code = move_group_interface_->execute(plan);

    move_group_interface_->stop();
    move_group_interface_->clearPoseTargets();
    is_moving_ = false;

    if (exec_code == moveit::core::MoveItErrorCode::SUCCESS) {
      feedback->state = "success";
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->object_detected = false;
      result->message = "Probe completed successfully";
      goal_handle->succeed(result);
      return;
    }

    feedback->state = "blocked";
    goal_handle->publish_feedback(feedback);

    result->success = false;
    result->object_detected = true;
    result->message = "Execution failed; probe blocked";
    goal_handle->abort(result);
  }

  bool moveToPose(const geometry_msgs::msg::Pose & target_pose)
  {
    if (is_moving_) {
      return false;
    }

    is_moving_ = true;

    move_group_interface_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = static_cast<bool>(move_group_interface_->plan(plan));

    if (!planned) {
      move_group_interface_->clearPoseTargets();
      is_moving_ = false;
      return false;
    }

    auto exec_code = move_group_interface_->execute(plan);

    move_group_interface_->stop();
    move_group_interface_->clearPoseTargets();
    is_moving_ = false;

    return exec_code == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool moveToJointPositions(const std::vector<double> & joint_targets)
  {
    if (is_moving_) {
      return false;
    }

    is_moving_ = true;

    move_group_interface_->setJointValueTarget(joint_targets);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = static_cast<bool>(move_group_interface_->plan(plan));

    if (!planned) {
      move_group_interface_->clearPoseTargets();
      is_moving_ = false;
      return false;
    }

    auto exec_code = move_group_interface_->execute(plan);

    move_group_interface_->stop();
    move_group_interface_->clearPoseTargets();
    is_moving_ = false;

    return exec_code == moveit::core::MoveItErrorCode::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp_action::Server<ProbeArm>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::atomic<bool> is_moving_;
  bool startup_done_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "arm_probe_cpp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  auto server = std::make_shared<ArmProbeServer>(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}