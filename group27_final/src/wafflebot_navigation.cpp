#include "wafflebot_navigation.hpp"

//===============================================
// set the initial pose of the robot
void navigation::WaffleNavigation::set_initial_pose()
{
  // create a PoseWithCovarianceStamped message
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
  // st the frame id to map
  message.header.frame_id = "map";
  // set the position and orientation of the robot
  message.pose.pose.position.x = 1.000;
  message.pose.pose.position.y = -1.59;
  message.pose.pose.position.z = 0.0078;
  message.pose.pose.orientation.x = 0.0;
  message.pose.pose.orientation.y = 0.0;
  message.pose.pose.orientation.z = -0.707;
  message.pose.pose.orientation.w = 0.707;
  // publish the initial pose
  initial_pose_pub_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Initial pose set");
}
//===============================================
// send the goal to the follow way points action server
void navigation::WaffleNavigation::send_goal()
{
  using namespace std::placeholders;
  // wait for the action server to be available
  if (!this->client_->wait_for_action_server())
  {
    // if the action server is not available,log an error
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }
  // create a followwaypoints goal message
  auto goal_msg = FollowWaypoints::Goal();
  // Resize the poses array in the goal message to hold 5 poses
  goal_msg.poses.resize(5);
  // create a vector to hold posestamped message
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  // create a pose stamped message
  geometry_msgs::msg::PoseStamped pose;
  // loop over waypoints
  for (int i = 0; i < 5; i++)
  {
    // set the frame_id of each pose in the goal message to map
    goal_msg.poses[i].header.frame_id = "map";
    // copy the position and orientation from the waypoint data to goal message
    goal_msg.poses[i].pose.position.x = waypoint_data.poses[i].position.x;
    goal_msg.poses[i].pose.position.y = waypoint_data.poses[i].position.y;
    goal_msg.poses[i].pose.position.z = waypoint_data.poses[i].position.z;
    goal_msg.poses[i].pose.orientation.x = waypoint_data.poses[i].orientation.x;
    goal_msg.poses[i].pose.orientation.y = waypoint_data.poses[i].orientation.y;
    goal_msg.poses[i].pose.orientation.z = waypoint_data.poses[i].orientation.z;
    goal_msg.poses[i].pose.orientation.w = waypoint_data.poses[i].orientation.w;
    // add the pose to the poses vector
    poses.push_back(pose);
  }
  // log that goal is being sent
  RCLCPP_INFO(this->get_logger(), "Sending goal");
  // create sendgoaloptions for the action client
  auto send_goal_options =
      rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  // set the callbacks for the goal response,feedback,and result
  send_goal_options.goal_response_callback =
      std::bind(&WaffleNavigation::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&WaffleNavigation::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&WaffleNavigation::result_callback, this, _1);
  // send the goal asynchronously
  client_->async_send_goal(goal_msg, send_goal_options);
}

// way points callback function to store the subscribed data
void navigation::WaffleNavigation::waypoints_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Waypoint received");
  waypoint_data = *msg;
  set_initial_pose();
  // pause for 5 seconds
  std::this_thread::sleep_for(std::chrono::seconds(5));
  send_goal();
  // reset the subscription to prevent duplicate messages
  get_waypoints_.reset();
}

//=====================================================
//  callback function for goal response
void navigation::WaffleNavigation::goal_response_callback(
    std::shared_future<GoalHandleNavigation::SharedPtr> future)
{
  // get the goal handle for the future
  auto goal_handle = future.get();
  // If the goal handle is null, the goal was rejected by the server

  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  }
  else
  {
    // If the goal handle is not null, the goal was accepted by the server
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

//===============================================
// Callback function for feedback

void navigation::WaffleNavigation::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  // Log an info message indicating that the robot is driving towards the goal
  RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

//===============================================
// Callback function for result
void navigation::WaffleNavigation::result_callback(
    const GoalHandleNavigation::WrappedResult &result)
{
  // Switch on the result code
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  rclcpp::shutdown();
}
int main(int argc, char **argv)
{
  //intialis ros
  rclcpp::init(argc, argv);
  // create wafflebot_navigation
  auto node = std::make_shared<navigation::WaffleNavigation>("wafflebot_navigation");
  //spin the node
  rclcpp::spin(node);
  //shutdown ROS
  rclcpp::shutdown();
}