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

  void send_goal();
};

} // namespace robot_nav