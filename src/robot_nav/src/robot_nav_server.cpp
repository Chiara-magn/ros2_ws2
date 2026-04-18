#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

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

            //action server
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

        // callback (li implementeremo dopo)
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MoveToPose::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

        void handle_accepted(
            const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
        
        void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);      
    };

    rclcpp_action::GoalResponse robot_nav::NavigationServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPose::Goal> goal)
    {
    (void)uuid;

    RCLCPP_INFO(this->get_logger(),
        "Goal recived: x=%.2f y=%.2f",
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse robot_nav::NavigationServer::handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void robot_nav::NavigationServer::handle_accepted(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        std::thread{std::bind(&NavigationServer::execute, this, goal_handle)}.detach();

    }  
} // namespace robot_nav