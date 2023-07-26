#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

/// \brief Odometry node tracks the odometry data and odom transform in the simulation
///
/// PARAMETERS:
///     body_id (string): the child id for the odom message and tf
///     odom_id (string): the parent id for the odom message and tf
///     wheel_right (string): name of right wheel joint
///     wheel_left (string): name of left wheel joint
///
/// PUBLISHES:
///     /odom (Odometry): Publishes the odometry data
///
/// SUBSCRIBES:
///     /joint_states (JointState): the joint states of the wheels
///
/// SERVERS:
///     /initial_pose_srv (InitialPose): Sets the robots configuration
///
/// CLIENTS:
///     None

/// @brief Odometry class that initalizes the odometry node
class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_left", "");

    body_id = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right = get_parameter("wheel_right").get_parameter_value().get<std::string>();

    if (body_id == "" || odom_id == "" || \
      wheel_left == "" || wheel_right == "")
    {
      throw (std::runtime_error("odometry node died: Param values not set from yaml \n"));
    }

    path_counter = 0;

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::js_callback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    blue_path_pub_ = create_publisher<nav_msgs::msg::Path>("blue/blue_path", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    init_pose_service_ = create_service<nuturtle_control::srv::InitialPose>(
      "intial_pose_srv", std::bind(
        &Odometry::init_pose_cb, \
        this, std::placeholders::_1, std::placeholders::_2));

    prev_wheel_pos.position = {0.0, 0.0};

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blue_path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr init_pose_service_;
  std::string body_id, odom_id, wheel_left, wheel_right;
  nav_msgs::msg::Odometry odom;
  turtlelib::DiffDrive ddrive_odom;
  nav_msgs::msg::Path path_msg;
  geometry_msgs::msg::PoseStamped path_poses;
  tf2::Quaternion quats;
  sensor_msgs::msg::JointState prev_wheel_pos;
  turtlelib::Twist2D Vb;
  geometry_msgs::msg::TransformStamped odom_body;
  int path_counter;


  /// @brief Callback for joint_state subscription.
  /// @param js - the received joint states of the wheels
  /// Publishes the odometry message and broadcasts the transform
  /// between the odom and base_footprint frames
  void js_callback(sensor_msgs::msg::JointState js)
  {
    double right_wheel = js.position.at(0) - prev_wheel_pos.position.at(0);
    double left_wheel = js.position.at(1) - prev_wheel_pos.position.at(1);

    path_counter++;
    turtlelib::WheelCmds new_wheel_pos{right_wheel, left_wheel};
    Vb = ddrive_odom.getBodyTwist(new_wheel_pos);
    ddrive_odom.forward_kinematics(new_wheel_pos);

    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;
    odom.pose.pose.position.x = ddrive_odom.configuration().x;
    odom.pose.pose.position.y = ddrive_odom.configuration().y;
    odom.pose.pose.position.z = 0.0;

    quats.setRPY(0, 0, ddrive_odom.configuration().theta);
    odom.pose.pose.orientation.x = quats.x();
    odom.pose.pose.orientation.y = quats.y();
    odom.pose.pose.orientation.z = quats.z();
    odom.pose.pose.orientation.w = quats.w();

    odom.twist.twist.linear.x = Vb.x;
    odom.twist.twist.angular.z = Vb.w;

    odom_body.header.stamp = get_clock()->now();
    odom_body.header.frame_id = odom_id;
    odom_body.child_frame_id = body_id;
    odom_body.transform.translation.x = ddrive_odom.configuration().x;
    odom_body.transform.translation.y = ddrive_odom.configuration().y;
    odom_body.transform.translation.z = 0.0;

    odom_body.transform.rotation.x = quats.x();
    odom_body.transform.rotation.y = quats.y();
    odom_body.transform.rotation.z = quats.z();
    odom_body.transform.rotation.w = quats.w();
    tf_broadcaster_->sendTransform(odom_body);
    odom_pub_->publish(odom);

    if (path_counter % 100 == 0) {
      path_msg.header.stamp = get_clock()->now();
      path_msg.header.frame_id = odom_id;
      path_poses.pose.position.x = ddrive_odom.configuration().x;
      path_poses.pose.position.y = ddrive_odom.configuration().y;
      path_poses.pose.orientation.x = quats.x();
      path_poses.pose.orientation.y = quats.y();
      path_poses.pose.orientation.z = quats.z();
      path_poses.pose.orientation.w = quats.w();
      path_msg.poses.push_back(path_poses);
    }
    blue_path_pub_->publish(path_msg);
    prev_wheel_pos.position = {js.position.at(0), js.position.at(1)};
  }

  /// @brief Callback function for initial pose service
  /// @param request takes in desired x,y,theta position for the robot config
  void init_pose_cb(
    nuturtle_control::srv::InitialPose::Request::SharedPtr request,
    nuturtle_control::srv::InitialPose::Response::SharedPtr)
  {
    ddrive_odom.setConfig(request->x, request->y, request->theta);
    quats.setRPY(0, 0, ddrive_odom.configuration().theta);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Odometry>());
  } catch (std::runtime_error & error) {
    std::cerr << error.what();
  }
  rclcpp::shutdown();
  return 0;
}
