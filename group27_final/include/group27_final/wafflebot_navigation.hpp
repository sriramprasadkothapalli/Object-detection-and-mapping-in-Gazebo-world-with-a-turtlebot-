#pragma once
/**
 * @file wafflebot_navigation.hpp
 * @author group27
 * @brief Navigation by follow way points
 * @date 2023-12-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <chrono>

/**
 * @brief navigation namespace
 *
 */
namespace navigation
{
    /**
     * @brief class for navigation
     *
     */
    class WaffleNavigation : public rclcpp::Node
    {
    public:
        using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
        using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
        /**
         * @brief Construct a new Waffle Navigation class
         * Initializes the action client, publisher  and the subscription
         *
         * @param node_name  name of the node
         */
        WaffleNavigation(std::string node_name) : Node(node_name)
        {
            // initialize the client
            client_ =
                rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
            // initialize the publisher
            initial_pose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "initialpose", 10);

            // subscriber to read the waypoints data from the topic published by the path planning node(waffle_broadcaster)
            get_waypoints_ = this->create_subscription<geometry_msgs::msg::PoseArray>("wafflebot/waypoints", 10, std::bind(&WaffleNavigation::waypoints_cb, this, std::placeholders::_1));
        }

    private:
        /**
         * @brief Publisher to the topic /initialpose
         *
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            initial_pose_pub_;
        /**
         * @brief Action client for the action server navigate_to_pose
         *
         */
        rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
        /**
         * @brief subscription to the topic /wafflebot/waypoints
         *
         */
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr get_waypoints_;
        /**
         * @brief Response from the server after sending the goal
         */
        void goal_response_callback(
            std::shared_future<GoalHandleNavigation::SharedPtr> future);
        /**
         * @brief Feedback received while the robot is driving towards the goal
         *
         * @param feedback
         */
        void feedback_callback(
            GoalHandleNavigation::SharedPtr,
            const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
        /**
         * @brief Result after the action has completed
         *
         * @param result
         */
        void result_callback(const GoalHandleNavigation::WrappedResult &result);
        /**
         * @brief Method to build and send a goal using the client
         *
         */
        /**
         * @brief Method to build and send a goal using the client
         */
        void send_goal();
        /**
         * @brief Method to initialize the pose of the robot
         *
         */
        void set_initial_pose();
        /**
         * @brief Callback function to read the waypoints data from the topic published by the path planning node(waffle_broadcaster)
         *
         * @param msg
         */
        void waypoints_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        /**
         * @brief variable to store the waypoints data
         *
         */
        geometry_msgs::msg::PoseArray waypoint_data;
    }; // class WaffleNavigation
} // namespace navigation