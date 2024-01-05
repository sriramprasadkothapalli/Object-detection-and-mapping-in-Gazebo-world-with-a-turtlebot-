#include <rclcpp/rclcpp.hpp>
#include <waffle_bot.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

// fucntion to broadcast the static transform
void WaffleBot::static_broadcast_data_(std::string map, std::string camera_frame, std::string part_frame, mage_msgs::msg::AdvancedLogicalCameraImage camera_data)
{
  geometry_msgs::msg::TransformStamped static_transform_stamped;
  // time stamp
  double broadcast_time = aruco_pos_data.header.stamp.sec;
  rclcpp::Time broadcast_time_rclcpp(broadcast_time, aruco_pos_data.header.stamp.nanosec);
  static_transform_stamped.header.stamp = broadcast_time_rclcpp;

  /////////////////////////////////////////////////
  // frame1 to broadcast transform between map and camera frame
  /////////////////////////////////////////////////

  static_transform_stamped.header.frame_id = map;
  static_transform_stamped.child_frame_id = camera_frame;
  static_transform_stamped.transform.translation.x = camera_data.sensor_pose.position.x;
  static_transform_stamped.transform.translation.y = camera_data.sensor_pose.position.y;
  static_transform_stamped.transform.translation.z = camera_data.sensor_pose.position.z;
  static_transform_stamped.transform.rotation.x = camera_data.sensor_pose.orientation.x;
  static_transform_stamped.transform.rotation.y = camera_data.sensor_pose.orientation.y;
  static_transform_stamped.transform.rotation.z = camera_data.sensor_pose.orientation.z;
  static_transform_stamped.transform.rotation.w = camera_data.sensor_pose.orientation.w;
  // send the tranform
  tf_static_broadcaster_->sendTransform(static_transform_stamped);

  ////////////////////////////////////////////////////////////////////
  // frame2 to broadcast transform between camera frame and part frame
  ////////////////////////////////////////////////////////////////////
  static_transform_stamped.header.frame_id = camera_frame;
  static_transform_stamped.child_frame_id = part_frame;
  static_transform_stamped.transform.translation.x = camera_data.part_poses[0].pose.position.x;
  static_transform_stamped.transform.translation.y = camera_data.part_poses[0].pose.position.y;
  static_transform_stamped.transform.translation.z = camera_data.part_poses[0].pose.position.z;
  static_transform_stamped.transform.rotation.x = camera_data.part_poses[0].pose.orientation.x;
  static_transform_stamped.transform.rotation.y = camera_data.part_poses[0].pose.orientation.y;
  static_transform_stamped.transform.rotation.z = camera_data.part_poses[0].pose.orientation.z;
  static_transform_stamped.transform.rotation.w = camera_data.part_poses[0].pose.orientation.w;
  tf_static_broadcaster_->sendTransform(static_transform_stamped);
}

// function to listen to the transform between map and part frame
geometry_msgs::msg::Pose WaffleBot::waffle_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
  geometry_msgs::msg::TransformStamped t_stamped;
  geometry_msgs::msg::Pose pose_stamped;
  try
  {
    t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 100ms);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
    return pose_stamped;
  }
  // convert the transform to pose
  pose_stamped.position.x = t_stamped.transform.translation.x;
  pose_stamped.position.y = t_stamped.transform.translation.y;
  // sending pose z as 0.0 as the part is in air and robot have to navigate on ground
  pose_stamped.position.z = 0.0;
  pose_stamped.orientation = t_stamped.transform.rotation;
  // return the pose stamped
  return pose_stamped;
  // RCLCPP_INFO_STREAM(this->get_logger(), target_frame << " in " << source_frame << ":\n"
  //                                                     << "x: " << pose_stamped.position.x << "\t"
  //                                                     << "y: " << pose_stamped.position.y << "\t"
  //                                                     << "z: " << pose_stamped.position.z << "\n"
  //                                                     << "qx: " << pose_stamped.orientation.x << "\t"
  //                                                     << "qy: " << pose_stamped.orientation.y << "\t"
  //                                                     << "qz: " << pose_stamped.orientation.z << "\t"
  //                                                     << "qw: " << pose_stamped.orientation.w << "\n");
}

// timer call back function to broadcast the static transforms
void WaffleBot::waffle_timer_cb_()
{
  static_broadcast_data_("map", "camera1_frame", "part" + std::to_string(c1) + "_frame", camera1_data);
  static_broadcast_data_("map", "camera2_frame", "part" + std::to_string(c2) + "_frame", camera2_data);
  static_broadcast_data_("map", "camera3_frame", "part" + std::to_string(c3) + "_frame", camera3_data);
  static_broadcast_data_("map", "camera4_frame", "part" + std::to_string(c4) + "_frame", camera4_data);
  static_broadcast_data_("map", "camera5_frame", "part" + std::to_string(c5) + "_frame", camera5_data);
  // to publish way points
  publish_waypoints();
}

// aruco call back function to read the aruco data and store the id
void WaffleBot::arucocallback_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  // Assign the message data to aruco_pos_data
  aruco_pos_data = *msg;
  // Get the first marker id from the aruco_pos_data
  int id = aruco_pos_data.marker_ids[0];
  // Log the marker id
  RCLCPP_INFO(this->get_logger(), "markerid: '%d'", id);
  // Reset the aruco subscriber to release memory
  aruco_subscriber_.reset();
}

// camera1 call back function to read the camera data and store the color and pose of part
void WaffleBot::camera1_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // Assign the message data to camera1_data
  camera1_data = *msg;
  // get the color of the part
  c1 = camera1_data.part_poses[0].part.color;
  // Add the camera1_data to the cam_vec vector
  cam_vec.push_back(camera1_data);
  // Reset the camera1 subscriber
  camera1_subscriber_.reset();
}
// camera2 call back function to read the camera data and store the color and pose of part
void WaffleBot::camera2_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // assign the message to camera2 data
  camera2_data = *msg;
  // get the color of the part
  c2 = camera2_data.part_poses[0].part.color;
  // add the camera2 data to camera vector
  cam_vec.push_back(camera2_data);
  // Reset the subscriber to release the memory
  camera2_subscriber_.reset();
}
// camera3 call back function to read the camera data and store the color and pose of part
void WaffleBot::camera3_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // assign the message to camera3 data
  camera3_data = *msg;
  // get the color of the part
  c3 = camera3_data.part_poses[0].part.color;
  // add data to camera vector
  cam_vec.push_back(camera3_data);
  // Reset the subscriber to release the memory
  camera3_subscriber_.reset();
}

// camera4 call back function to read the camera data and store the color and pose of part
void WaffleBot::camera4_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // Assign the message data to camera4_data
  camera4_data = *msg;
  // Get the color of the first part from the camera4_data
  c4 = camera4_data.part_poses[0].part.color;
  // Add the camera4_data to the cam_vec vector
  cam_vec.push_back(camera4_data);
  // Reset the subscriber to release the memory
  camera4_subscriber_.reset();
}
// camera5 call back function to read the camera data and store the color and pose of part
void WaffleBot::camera5_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // Assign the message data to camera5_data
  camera5_data = *msg;
  // Get the color of the first part from the camera5_data
  c5 = camera5_data.part_poses[0].part.color;
  // Add the camera5_data to the cam_vec vector
  cam_vec.push_back(camera5_data);
  // Reset the subscriber to release the memory
  camera5_subscriber_.reset();
}

// function to publish the waypoints by listening to the transform between map and part frame
void WaffleBot::publish_waypoints()
{
  if (!publish)
  {
    // checks the aruco id and publish the waypoints
    if (aruco_pos_data.marker_ids[0] == 0)
    {
      // checks the color of part and publish the waypoints
      for (int i = 0; i < 5; i++)
      {
        // checks color of the part in camera frame
        if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::BLUE))
        {
          // finds the matching waypoint in the aruco data
          for (int j = 0; j < 5; j++)
          {
            if (aruco_0.wp[j].type == "battery" && aruco_0.wp[j].color == "blue")
            {
              // publish the waypoints by listening to the transform between map and part frame
              waypoints.poses[j] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // checks color of the part in camera frame
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::RED))
        {
          for (int k = 0; k < 5; k++)
          {
            if (aruco_0.wp[k].type == "battery" && aruco_0.wp[k].color == "red")
            {
              // gets the pose of part and appends to waypoints publisher to give map
              waypoints.poses[k] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // checks color of the part in camera frame
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::GREEN))
        {
          for (int l = 0; l < 5; l++)
          {
            // checks for green color part in aruco data
            if (aruco_0.wp[l].type == "battery" && aruco_0.wp[l].color == "green")
            {
              // gets the pose of part and appends to waypoints publisher to give map
              waypoints.poses[l] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // checks color of the part in camera frame
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::ORANGE))
        {
          // checks for orange color part in aruco data
          for (int m = 0; m < 5; m++)
          {
            if (aruco_0.wp[m].type == "battery" && aruco_0.wp[m].color == "orange")
            {
              // gets pose of part and appends to waypoint array
              waypoints.poses[m] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // checks for purple color part in camera frame
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::PURPLE))
        {
          // checks for purple color part in aruco data
          for (int n = 0; n < 5; n++)
          {
            if (aruco_0.wp[n].type == "battery" && aruco_0.wp[n].color == "purple")
            {
              waypoints.poses[n] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
      }
    }
    // checks the aruco id and publish the waypoints
    else if (aruco_pos_data.marker_ids[0] == 1)
    {
      // checks the color of part and publish the waypoints
      for (int i = 0; i < 5; i++)
      {
        if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::BLUE))
        {
          for (int j = 0; j < 5; j++)
          {

            if (aruco_1.wp[j].type == "battery" && aruco_1.wp[j].color == "blue")
            {
              waypoints.poses[j] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // this will check for aruco id 1 and color red and matches to waypoint data and publish the waypoints
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::RED))
        {
          for (int k = 0; k < 5; k++)
          {
            if (aruco_1.wp[k].type == "battery" && aruco_1.wp[k].color == "red")
            {
              waypoints.poses[k] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // this will check for aruco id 1 and color green to waypoint data and publish the way points
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::GREEN))
        {
          for (int l = 0; l < 5; l++)
          {
            if (aruco_1.wp[l].type == "battery" && aruco_1.wp[l].color == "green")
            {
              waypoints.poses[l] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // this will check for aruco id 1 and color orange to waypoint data and publish the way points
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::ORANGE))
        {
          for (int m = 0; m < 5; m++)
          {
            if (aruco_1.wp[m].type == "battery" && aruco_1.wp[m].color == "orange")
            {

              waypoints.poses[m] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
        // this will check for aruco id 1 and color purple to waypoint data and publish the way points
        else if (cam_vec[i].part_poses[0].part.color == int(mage_msgs::msg::Part::PURPLE))
        {
          for (int n = 0; n < 5; n++)
          {
            if (aruco_1.wp[n].type == "battery" && aruco_1.wp[n].color == "purple")
            {
              waypoints.poses[n] = waffle_listen_transform("map", "part" + std::to_string(cam_vec[i].part_poses[0].part.color) + "_frame");
            }
          }
        }
      }
    }
    // sets bool to true
    publish = true;
  }
  else
  {
    // publishes the waypoints
    publish_waypoints_->publish(waypoints);
  }
} // end of publish_waypoints

int main(int argc, char **argv)
{
  // intialise ROS
  rclcpp::init(argc, argv);
  // create node waffle_broadcaster
  auto node = std::make_shared<WaffleBot>("waffle_broadcaster");
  // spin the node
  rclcpp::spin(node);
  // shutdown ROS
  rclcpp::shutdown();
}