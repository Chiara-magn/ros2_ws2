#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"   

#include "robot_nav/action/move_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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

    RCLCPP_INFO(this->get_logger(), "\033[1;32mNavigationServer active\033[0m");
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleMoveToPose> current_goal_handle_;

  std::atomic<bool> running_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
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

    running_ = false;  // stop thread
    stop_robot();

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    // stop previous goal thread
    running_ = false;
    stop_robot();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    current_goal_handle_ = goal_handle;
    running_ = true;

    std::thread{
      std::bind(&NavigationServer::execute, this, goal_handle)
    }.detach();
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }

  void execute(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();

    rclcpp::Rate rate(10);

    while (rclcpp::ok() && running_)
    {
      if (goal_handle->is_canceling())
      {
        running_ = false;
        stop_robot();
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      double dx = goal->target_pose.pose.position.x - current_x_;
      double dy = goal->target_pose.pose.position.y - current_y_;
      double distance = std::sqrt(dx*dx + dy*dy);

      feedback->current_pose.pose.position.x = current_x_;
      feedback->current_pose.pose.position.y = current_y_;
      goal_handle->publish_feedback(feedback);

      if (distance < 0.1)
      {
        // ORIENTAZIONE FINALE
        tf2::Quaternion q_goal(
          goal->target_pose.pose.orientation.x,
          goal->target_pose.pose.orientation.y,
          goal->target_pose.pose.orientation.z,
          goal->target_pose.pose.orientation.w
        );

        double roll_g, pitch_g, goal_yaw;
        tf2::Matrix3x3(q_goal).getRPY(roll_g, pitch_g, goal_yaw);

        double yaw_error = goal_yaw - current_yaw_;

        while (yaw_error > M_PI) yaw_error -= 2*M_PI;
        while (yaw_error < -M_PI) yaw_error += 2*M_PI;

        rclcpp::Rate rate_yaw(20);

        while (std::abs(yaw_error) > 0.05 && running_)
        {
          geometry_msgs::msg::Twist cmd;
          cmd.angular.z = std::clamp(1.0 * yaw_error, -1.0, 1.0);
          cmd_vel_pub_->publish(cmd);

          yaw_error = goal_yaw - current_yaw_;
          while (yaw_error > M_PI) yaw_error -= 2*M_PI;
          while (yaw_error < -M_PI) yaw_error += 2*M_PI;

          rate_yaw.sleep();
        }

        stop_robot();

        if (running_)
        {
          result->success = true;
          goal_handle->succeed(result);
        }

        running_ = false;
        return;
      }

      double angle_to_goal = std::atan2(dy, dx);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = std::clamp(0.5 * distance, -0.5, 0.5);
      cmd.angular.z = std::clamp(1.0 * (angle_to_goal - current_yaw_), -1.0, 1.0);

      cmd_vel_pub_->publish(cmd);

      rate.sleep();
    }

    stop_robot();
  }
};

} // namespace robot_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_nav::NavigationServer)


