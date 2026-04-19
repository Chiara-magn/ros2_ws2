#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"   

#include "robot_nav/action/move_to_pose.hpp"

namespace robot_nav
{

class NavigationServer : public rclcpp::Node
{
public:
  using MoveToPose = robot_nav::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

  explicit NavigationServer(const rclcpp::NodeOptions & options)
  : Node("navigation_server", options)
  {
    RCLCPP_INFO(this->get_logger(), "NavigationServer active");

    // cmd_vel publisher to control robot velocity
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // added: subscription /odom 
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&NavigationServer::odom_callback, this, std::placeholders::_1)
    );

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

  // Track current active goal for preemption
  std::shared_ptr<GoalHandleMoveToPose> current_goal_handle_;

  // added: current pose variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // added: callback odom 
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // estrai yaw dal quaternion
    auto q = msg->pose.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  rclcpp_action::GoalResponse handle_goal(
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

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    (void)goal_handle;

    RCLCPP_INFO(this->get_logger(), "Cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

// modification to handle goal preemption
void handle_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  // abort active goal if a new one arrives
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    auto result = std::make_shared<MoveToPose::Result>();
    result->success = false;
    current_goal_handle_->abort(result);

    RCLCPP_WARN(this->get_logger(),
      "Previous goal aborted due to preemption");
  }

  // Update current goal
  current_goal_handle_ = goal_handle;

  // Execute new goal in a separate thread
  std::thread{
    std::bind(&NavigationServer::execute, this, goal_handle)
  }.detach();
}

  void execute(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal execution started");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    rclcpp::Rate rate(10);

    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);

        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      // preemption control, stop robot motion
      if (current_goal_handle_.get() != goal_handle.get()) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
      }

      // removed lookupTransform("map", "base_link") 
      // inserted/odom 
      double x = current_x_;
      double y = current_y_;

      double goal_x = goal->target_pose.pose.position.x;
      double goal_y = goal->target_pose.pose.position.y;

      double dx = goal_x - x;
      double dy = goal_y - y;

      double distance = std::sqrt(dx*dx + dy*dy);

      // feedback
      feedback->current_pose.header.frame_id = "odom";   // *** CAMBIATO ***
      feedback->current_pose.pose.position.x = x;
      feedback->current_pose.pose.position.y = y;
      goal_handle->publish_feedback(feedback);

      // stop condition
      if (distance < 0.1)
      {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);

        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
      }

      // proportional controller
      double angle_to_goal = std::atan2(dy, dx);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.5 * distance;
      cmd.angular.z = 1.0 * (angle_to_goal - current_yaw_);

      cmd_vel_pub_->publish(cmd);

      rate.sleep();
    }
  }
};

} // namespace robot_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_nav::NavigationServer)
