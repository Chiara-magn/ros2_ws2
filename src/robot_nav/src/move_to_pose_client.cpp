#include <memory>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
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

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      10,
      std::bind(&MoveToPoseClient::goal_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "MoveToPose client ready");
  }

private:
  rclcpp_action::Client<MoveToPose>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  std::shared_ptr<GoalHandleMoveToPose> current_goal_handle_;

  void send_goal(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    MoveToPose::Goal goal_msg;
    goal_msg.target_pose = pose;

    auto options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();

    options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleMoveToPose> goal_handle)
      {
        if (!goal_handle)
        {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        }
        else
        {
          current_goal_handle_ = goal_handle;
          RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
      };

    options.feedback_callback =
      [this](GoalHandleMoveToPose::SharedPtr,
             const std::shared_ptr<const MoveToPose::Feedback> feedback)
      {
        RCLCPP_INFO(this->get_logger(),
          "Robot position x=%.2f y=%.2f",
          feedback->current_pose.pose.position.x,
          feedback->current_pose.pose.position.y);
      };

    options.result_callback =
      [this](const GoalHandleMoveToPose::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
          RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        else if (result.code == rclcpp_action::ResultCode::ABORTED)
          RCLCPP_ERROR(this->get_logger(), "Goal aborted");
        else if (result.code == rclcpp_action::ResultCode::CANCELED)
          RCLCPP_WARN(this->get_logger(), "Goal canceled");
      };

    client_->async_send_goal(goal_msg, options);
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id == "STOP")
    {
      RCLCPP_WARN(this->get_logger(), "STOP received");

      if (current_goal_handle_)
        client_->async_cancel_goal(current_goal_handle_);

      return;
    }

    send_goal(*msg);
  }
};

} // namespace robot_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_nav::MoveToPoseClient)

