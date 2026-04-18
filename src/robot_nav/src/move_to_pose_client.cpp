#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_nav/action/move_to_pose.hpp"

namespace robot_nav
{

class MoveToPoseClient : public rclcpp::Node
{
public:
  using MoveToPose = robot_nav::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;

  explicit MoveToPoseClient(const rclcpp::NodeOptions & options)
  : Node("move_to_pose_client", options)
  {
    client_ = rclcpp_action::create_client<MoveToPose>(
      this,
      "move_to_pose");

    RCLCPP_INFO(this->get_logger(), "MoveToPose client ready");

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&MoveToPoseClient::send_goal, this));
  }

private:
  rclcpp_action::Client<MoveToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_goal()
  {
    timer_->cancel();

    if (!client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    MoveToPose::Goal goal_msg;

    // UI still has to be implemented
    goal_msg.target_pose.header.frame_id = "map";
    goal_msg.target_pose.header.stamp = this->now();

    goal_msg.target_pose.pose.position.x = 2.0;
    goal_msg.target_pose.pose.position.y = 1.0;
    goal_msg.target_pose.pose.position.z = 0.0;

    goal_msg.target_pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(),
      "Sending goal: x=2.0 y=1.0");

    auto options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();

    options.goal_response_callback =
      std::bind(&MoveToPoseClient::goal_response_callback, this, std::placeholders::_1);

    options.feedback_callback =
      std::bind(&MoveToPoseClient::feedback_callback, this,
        std::placeholders::_1, std::placeholders::_2);

    options.result_callback =
      std::bind(&MoveToPoseClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, options);
  }

  // callbacks

  void goal_response_callback(
    std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }
  }

  void feedback_callback(
    GoalHandleMoveToPose::SharedPtr,
    const std::shared_ptr<const MoveToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(),
      "Robot position x=%.2f y=%.2f",
      feedback->current_pose.pose.position.x,
      feedback->current_pose.pose.position.y);
  }

  void result_callback(
    const GoalHandleMoveToPose::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal aborted");
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_WARN(this->get_logger(), "Goal canceled");
    }
  }
};

} // namespace robot_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_nav::MoveToPoseClient)