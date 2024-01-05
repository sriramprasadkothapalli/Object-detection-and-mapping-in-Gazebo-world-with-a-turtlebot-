#pragma once

/**
 * @file waffle_bot.hpp
 * @author group27
 * @brief publishing way points to the navigator
 * @date 2023-12-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
using namespace std::chrono_literals;

/**
 * @brief  This class is used to broadcast the transform between the map and the wafflebot
 *
 */
class WaffleBot : public rclcpp::Node
{
public:
   /**
    * @brief Construct a new Waffle Bot object
    * 
    * @param waffle_broadcaster  name of the node
    */
    WaffleBot(std::string waffle_broadcaster) : Node(waffle_broadcaster)
    {
        // initialize the transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // load  buffers of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        // load the transform listener
        transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        waypoints.poses.resize(5);
        // timer to publish the transform
        static_broadcast_timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&WaffleBot::waffle_timer_cb_, this));
        // subscribe to the aruco markers
        aruco_subscriber_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", rclcpp::SensorDataQoS(), std::bind(&WaffleBot::arucocallback_, this, std::placeholders::_1));
        // subscribe to the camera data
        camera1_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera1/image", rclcpp::SensorDataQoS(), std::bind(&WaffleBot::camera1_cb_, this, std::placeholders::_1));
        camera2_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera2/image", rclcpp::SensorDataQoS(), std::bind(&WaffleBot::camera2_cb_, this, std::placeholders::_1));
        camera3_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera3/image", rclcpp::SensorDataQoS(), std::bind(&WaffleBot::camera3_cb_, this, std::placeholders::_1));
        camera4_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera4/image", rclcpp::SensorDataQoS(), std::bind(&WaffleBot::camera4_cb_, this, std::placeholders::_1));
        camera5_subscriber_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/camera5/image", rclcpp::SensorDataQoS(), std::bind(&WaffleBot::camera5_cb_, this, std::placeholders::_1));
        // publish the waypoints
        publish_waypoints_ = this->create_publisher<geometry_msgs::msg::PoseArray>("wafflebot/waypoints", 10);
        // declare the parameters of aruco from params file
        this->declare_parameter("aruco_0.wp1.type", "battery");
        this->declare_parameter("aruco_0.wp1.color", "green");
        this->declare_parameter("aruco_0.wp2.type", "battery");
        this->declare_parameter("aruco_0.wp2.color", "red");
        this->declare_parameter("aruco_0.wp3.type", "battery");
        this->declare_parameter("aruco_0.wp3.color", "orange");
        this->declare_parameter("aruco_0.wp4.type", "battery");
        this->declare_parameter("aruco_0.wp4.color", "purple");
        this->declare_parameter("aruco_0.wp5.type", "battery");
        this->declare_parameter("aruco_0.wp5.color", "blue");
        this->declare_parameter("aruco_1.wp1.type", "battery");
        this->declare_parameter("aruco_1.wp1.color", "green");
        this->declare_parameter("aruco_1.wp2.type", "battery");
        this->declare_parameter("aruco_1.wp2.color", "red");
        this->declare_parameter("aruco_1.wp3.type", "battery");
        this->declare_parameter("aruco_1.wp3.color", "orange");
        this->declare_parameter("aruco_1.wp4.type", "battery");
        this->declare_parameter("aruco_1.wp4.color", "purple");
        this->declare_parameter("aruco_1.wp5.type", "battery");
        this->declare_parameter("aruco_1.wp5.color", "blue");
        // using a for loop to get the latest parameters
        for (int i = 0; i < 5; i++)
        {
            std::string aruco_0_param_prefix = "aruco_0.wp" + std::to_string(i + 1) + ".";
            std::string aruco_1_param_prefix = "aruco_1.wp" + std::to_string(i + 1) + ".";
            aruco_0.wp[i].type = this->get_parameter(aruco_0_param_prefix + "type").as_string();
            aruco_0.wp[i].color = this->get_parameter(aruco_0_param_prefix + "color").as_string();
            aruco_1.wp[i].type = this->get_parameter(aruco_1_param_prefix + "type").as_string();
            aruco_1.wp[i].color = this->get_parameter(aruco_1_param_prefix + "color").as_string();
        }
    }

private:
   /**
    * @brief boolean to check if the transform has been published
    * 
    */
    bool publish = false;
    /**
     * @brief variables to read and store the part color data from the cameras
     * 
     */
    int c1; 
    int c2;
    int c3;
    int c4;
    int c5;
    /**
     * @brief tf buffer to store the transforms
     * 
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    /**
     * @brief transform  listener to listen to the transforms
     * 
     */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    /**
     * @brief transform broadcaster to broadcast the transform
     * 
     */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    /**
     * @brief vector to store the data of camera1 to camera5
     * 
     */
    std::vector<mage_msgs::msg::AdvancedLogicalCameraImage> cam_vec;
    /**
     * @brief pose array to store the waypoints
     * 
     */
    geometry_msgs::msg::PoseArray waypoints;
    /**
     * @brief structure to store the aruco data of params file
     * 
     */
    struct aruco_waypoints
    {
        struct waypoint_data
        {
            std::string type;
            std::string color;
        } wp[5];
    } aruco_0, aruco_1;
    /**
     * @brief shared pointer to the timer to publish the transform
     * 
     */
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;
    /**
     * @brief subscriber to the aruco markers
     * 
     */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
    /**
     * @brief subscriber to the camera1
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_subscriber_;
    /**
     * @brief subscriber to the camera2
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera2_subscriber_;
    /**
     * @brief subscriber to the camera3
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera3_subscriber_;
    /**
     * @brief subscriber to the camera4
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera4_subscriber_;
    /**
     * @brief subscriber to the camera5
     * 
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera5_subscriber_;
    /**
     * @brief publisher to the waypoints
     * 
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publish_waypoints_;
    /**
     * @brief variable to store the aruco data
     * 
     */
    ros2_aruco_interfaces::msg::ArucoMarkers aruco_pos_data;
    /**
     * @brief variables to store the camera1 data
     * 
     */
    mage_msgs::msg::AdvancedLogicalCameraImage camera1_data;
    mage_msgs::msg::AdvancedLogicalCameraImage camera2_data;
    mage_msgs::msg::AdvancedLogicalCameraImage camera3_data;
    mage_msgs::msg::AdvancedLogicalCameraImage camera4_data;
    mage_msgs::msg::AdvancedLogicalCameraImage camera5_data;
    /**
     * @brief to listen to the transform between source and target frames
     * 
     * @param source_frame 
     * @param target_frame 
     * @return geometry_msgs::msg::Pose 
     */
    geometry_msgs::msg::Pose waffle_listen_transform(const std::string &source_frame, const std::string &target_frame);
    /**
     * @brief call back function for the aruco markers
     * 
     * @param msg 
     */
    void arucocallback_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    /**
     * @brief broadcast the transform between the map and sensor frames and from sensor frames to part frames as well as the part data
     * 
     * @param map 
     * @param camera_frame 
     * @param part_frame 
     * @param camera_data 
     */
    void static_broadcast_data_(std::string map, std::string camera_frame, std::string part_frame, mage_msgs::msg::AdvancedLogicalCameraImage camera_data);
    /**
     * @brief timer callback function to broadcast the transform
     * 
     */
    void waffle_timer_cb_();
    /**
     * @brief call back function for the camera1
     * 
     * @param msg 
     */
    void camera1_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief call back function for the camera2
     * 
     * @param msg 
     */
    void camera2_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief call back function for the camera3
     * 
     * @param msg 
     */
    void camera3_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief call back function for the camera4
     * 
     * @param msg 
     */
    void camera4_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief call back function for the camera5
     * 
     * @param msg 
     */
    void camera5_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief function to publish the waypoints
     * 
     */
    void publish_waypoints();
}; // class WaffleBot
