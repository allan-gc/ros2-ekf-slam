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
#include "turtlelib/ekf.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <armadillo>
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

/// \brief Simulator node that can track and manipulate the position of
/// the turtlebot3 models in rviz

/// PARAMETERS:
///     green_body_id (string): the child id for the green odom message and tf
///     green_odom_id (string): the parent id for the green odom message and tf
///     wheel_right (string): name of right wheel joint
///     wheel_left (string): name of left wheel joint

/// PUBLISHES:
///      /green_odom (Odometry): Publishes the odometry data for green robot
///
///      green/green_path (Path): Publishes the path for green robot in rviz
///
///     ~/obstacle_map (MarkerArray): Publishes the obstacle markers detected by the green robot to rviz

/// SUBSCRIBES:
///     /joint_states (JointState): Joint states of the blue robot wheels
///
///     /fake_sensor (MarkerArray): Detected location of obstacles

/// SERVERS:
///     None

/// CLIENTS:
///     None

/// @brief Slam class that initalizes the slam node
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    declare_parameter("green_body_id", "");
    declare_parameter("green_odom_id", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_left", "");
    wheel_left = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    green_body_id = get_parameter("green_body_id").get_parameter_value().get<std::string>();
    green_odom_id = get_parameter("green_odom_id").get_parameter_value().get<std::string>();
    path_counter = 0;
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Slam::js_callback, this, std::placeholders::_1));
    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, std::placeholders::_1));
    fitted_circles_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fitted_circles", 10, std::bind(&Slam::circles_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("green_odom", 10);
    green_path_pub_ = create_publisher<nav_msgs::msg::Path>("green/green_path", 10);
    obstacle_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacle_map",
      10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_map_broad_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    prev_wheel_pos.position = {0.0, 0.0};
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fitted_circles_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_, tf_map_broad_;
  std::string green_odom_id, green_body_id, wheel_left, wheel_right;
  nav_msgs::msg::Odometry odom;
  turtlelib::DiffDrive ddrive_odom;
  turtlelib::EKF ekf;
  nav_msgs::msg::Path path_msg;
  geometry_msgs::msg::PoseStamped path_poses;
  tf2::Quaternion quats, quats_map;
  sensor_msgs::msg::JointState prev_wheel_pos;
  turtlelib::Twist2D Vb;
  geometry_msgs::msg::TransformStamped odom_body, map_odom;
  turtlelib::RobotConfig updated_config;
  turtlelib::Transform2D Tor, Tmr, Tmo;
  turtlelib::Vector2D vec_or, vec_mr;
  arma::vec temp_state;
  std_msgs::msg::Header time;
  int path_counter;

  void circles_callback(visualization_msgs::msg::MarkerArray circle_msg)
  {
    ekf.setEKF_config(
      ddrive_odom.configuration().x,
      ddrive_odom.configuration().y, ddrive_odom.configuration().theta);
    size_t num_obstacles = circle_msg.markers.size();
    ekf.predict_state();

    for (size_t i = 0; i < num_obstacles; i++) {
      int index = ekf.data_association(
        circle_msg.markers.at(
          i).pose.position.x, circle_msg.markers.at(i).pose.position.y);
      RCLCPP_INFO_STREAM(get_logger(), "INDEX: " << index << " ");

      ekf.update_state(
        circle_msg.markers.at(i).pose.position.x, circle_msg.markers.at(
          i).pose.position.y, index);
    }


    temp_state = ekf.getState();
    vec_mr = turtlelib::Vector2D{temp_state(1), temp_state(2)};
    Tmr = turtlelib::Transform2D{vec_mr, temp_state(0)};

  }
  /// @brief Callback for /fake_sensor topic, gets MarkerArray of detected obstacles
  /// @param sensor_msg the MarkerArray of the obstacles
  void fake_sensor_callback(visualization_msgs::msg::MarkerArray sensor_msg)
  {
    // ekf.setEKF_config(
    //   ddrive_odom.configuration().x,
    //   ddrive_odom.configuration().y, ddrive_odom.configuration().theta);
    // double num_obstacles = sensor_msg.markers.size();
    // ekf.predict_state();
    // for (int i = 0; i < num_obstacles; i++) {
    //   if (sensor_msg.markers.at(i).action == 0) {
    //     ekf.update_state(
    //       sensor_msg.markers.at(i).pose.position.x, sensor_msg.markers.at(
    //         i).pose.position.y, i);
    //   }
    // }
    // temp_state = ekf.getState();
    // vec_mr = turtlelib::Vector2D{temp_state(1), temp_state(2)};
    // Tmr = turtlelib::Transform2D{vec_mr, temp_state(0)};

    // // Map obstacle map tp keep track of seen obstacles
    // visualization_msgs::msg::MarkerArray map_mark_arr;
    // for (size_t i = 0; i < num_obstacles; i++) {
    //   visualization_msgs::msg::Marker obstacle;
    //   obstacle.header.frame_id = "nusim/world";
    //   obstacle.header.stamp = get_clock()->now();
    //   obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
    //   obstacle.id = sensor_msg.markers.at(i).id + 100;
    //   obstacle.pose.position.x = temp_state(3 + 2 * i);
    //   obstacle.pose.position.y = temp_state(4 + 2 * i);
    //   if (temp_state(3 + 2 * i) == 0.0 && temp_state(4 + 2 * i) == 0.0) {
    //     obstacle.action = visualization_msgs::msg::Marker::DELETE;
    //   } else {
    //     obstacle.action = visualization_msgs::msg::Marker::ADD;
    //   }
    //   obstacle.pose.position.x = temp_state(3 + 2 * i);
    //   obstacle.pose.position.y = temp_state(4 + 2 * i);
    //   obstacle.pose.position.z = sensor_msg.markers.at(i).pose.position.z;
    //   obstacle.color.g = 1.0;
    //   obstacle.color.a = 1.0;
    //   obstacle.scale.x = sensor_msg.markers.at(i).scale.x;
    //   obstacle.scale.y = sensor_msg.markers.at(i).scale.y;
    //   obstacle.scale.z = sensor_msg.markers.at(i).scale.z;
    //   map_mark_arr.markers.push_back(obstacle);
    // }
    // obstacle_publisher_->publish(map_mark_arr);
  }

  /// @brief Callback for joint_state subscription.
  /// @param js - the received joint states of the wheels
  /// Publishes the odometry message and broadcasts the transform
  /// between the green/odom and green/base_footprint frames
  void js_callback(sensor_msgs::msg::JointState js)
  {
    double right_wheel = js.position.at(0) - prev_wheel_pos.position.at(0);
    double left_wheel = js.position.at(1) - prev_wheel_pos.position.at(1);
    path_counter++;

    turtlelib::WheelCmds new_wheel_pos{right_wheel, left_wheel};
    Vb = ddrive_odom.getBodyTwist(new_wheel_pos);
    ddrive_odom.forward_kinematics(new_wheel_pos);

    time.stamp = get_clock()->now();

    odom.header.stamp = time.stamp;
    odom.header.frame_id = green_odom_id;
    odom.child_frame_id = green_body_id;
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
    odom_pub_->publish(odom);

    // Green/odom to green/base_footprint TF
    odom_body.header.stamp = time.stamp;
    odom_body.header.frame_id = green_odom_id;
    odom_body.child_frame_id = green_body_id;
    odom_body.transform.translation.x = ddrive_odom.configuration().x;
    odom_body.transform.translation.y = ddrive_odom.configuration().y;
    odom_body.transform.translation.z = 0.0;

    odom_body.transform.rotation.x = quats.x();
    odom_body.transform.rotation.y = quats.y();
    odom_body.transform.rotation.z = quats.z();
    odom_body.transform.rotation.w = quats.w();
    tf_broadcaster_->sendTransform(odom_body);

    vec_or = turtlelib::Vector2D{ddrive_odom.configuration().x, ddrive_odom.configuration().y};
    Tor = turtlelib::Transform2D{vec_or, ddrive_odom.configuration().theta};
    Tmo = Tmr * Tor.inv();

    // Map tp green/odom broadcaster
    map_odom.header.stamp = time.stamp;
    map_odom.header.frame_id = "map";
    map_odom.child_frame_id = green_odom_id;
    map_odom.transform.translation.x = Tmo.translation().x;
    map_odom.transform.translation.y = Tmo.translation().y;
    map_odom.transform.translation.z = 0.0;
    quats_map.setRPY(0, 0, Tmo.rotation());
    map_odom.transform.rotation.x = quats_map.x();
    map_odom.transform.rotation.y = quats_map.y();
    map_odom.transform.rotation.z = quats_map.z();
    map_odom.transform.rotation.w = quats_map.w();
    tf_map_broad_->sendTransform(map_odom);

    // Laser path

    if (path_counter % 100 == 0) {
      path_msg.header.stamp = time.stamp;
      path_msg.header.frame_id = green_odom_id;
      path_poses.header.stamp = time.stamp;
      path_poses.pose.position.x = ddrive_odom.configuration().x;
      path_poses.pose.position.y = ddrive_odom.configuration().y;
      path_poses.pose.orientation.x = quats.x();
      path_poses.pose.orientation.y = quats.y();
      path_poses.pose.orientation.z = quats.z();
      path_poses.pose.orientation.w = quats.w();
      path_msg.poses.push_back(path_poses);
    }
    green_path_pub_->publish(path_msg);

    prev_wheel_pos.position = {js.position.at(0), js.position.at(1)};
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Slam>());
  } catch (std::runtime_error & error) {
    std::cerr << error.what();
  }
  rclcpp::shutdown();
  return 0;
}
