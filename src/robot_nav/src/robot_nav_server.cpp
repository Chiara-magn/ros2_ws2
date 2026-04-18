#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_nav/action/move_to_pose.hpp"

namespace robot_nav
{

class NavigationServer : public rclcpp::Node
{
public:
  using MoveToPose = robot_nav::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

  explicit NavigationServer(const rclcpp::NodeOptions & options)
  : Node("navigation_server", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "NavigationServer active");

    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void execute(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
};


// callbacks

rclcpp_action::GoalResponse NavigationServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(),
    "Goal received: x=%.2f y=%.2f",
    goal->target_pose.pose.position.x,
    goal->target_pose.pose.position.y);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse NavigationServer::handle_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}


void NavigationServer::handle_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  std::thread{
    std::bind(&NavigationServer::execute, this, goal_handle)
  }.detach();
}


// execute

void NavigationServer::execute(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal execution started");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<MoveToPose::Feedback>();
  auto result = std::make_shared<MoveToPose::Result>();

  geometry_msgs::msg::PoseStamped current_pose;

  for (int i = 0; i <= 10; ++i)
  {
    if (goal_handle->is_canceling())
    {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // tf2
    try
    {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

      current_pose.header.stamp = this->now();
      current_pose.header.frame_id = "map";

      current_pose.pose.position.x = transform.transform.translation.x;
      current_pose.pose.position.y = transform.transform.translation.y;
      current_pose.pose.position.z = transform.transform.translation.z;

      current_pose.pose.orientation = transform.transform.rotation;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF2 error: %s", ex.what());
      return;
    }

    // feedback
    feedback->current_pose = current_pose;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(),
      "Robot position: x=%.2f y=%.2f",
      current_pose.pose.position.x,
      current_pose.pose.position.y);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  result->success = true;
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Goal accomplished.");
}

} // namespace robot_nav


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_nav::NavigationServer) //component