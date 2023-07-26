#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "turtlelib/ekf.hpp"
#include "turtlelib/circle_fit.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <armadillo>
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

/// \brief Landmarks node that detects obstacles in map using circle fitting and data association

/// PARAMETERS:
///     samples (int): samples in lidar message

/// PUBLISHES:
///
///     /fitted_circles (MarkerArray): Publishes the obstacles detected by circle fitting and data association

/// SUBSCRIBES:
///     /laser_scan (LaserScan): Laser data from sensor

/// @brief Landmarks class that initalizes the slam node
class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    declare_parameter("samples", 0);
    samples = get_parameter("samples").get_parameter_value().get<int>();
    laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "laser_scan", 10, std::bind(&Landmarks::laser_scan_callback, this, std::placeholders::_1));
    circle_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fitted_circles",
      10);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr circle_publisher_;
  // visualization_msgs::msg::MarkerArray landmarks_arr, fitted_circle_arr;
  turtlelib::DiffDrive ddrive;
  turtlelib::EKF ekf;
  tf2::Quaternion quats, quats_map;
  // std::vector<turtlelib::Circle> fitted_circle_list;
  int samples;
  double cluster_dist = 0.03;


  void laser_scan_callback(sensor_msgs::msg::LaserScan laser_data)
  {

    turtlelib::Vector2D prev_range_coords{0.0, 0.0};
    std::vector<std::vector<turtlelib::Vector2D>> clusters;
    std::vector<turtlelib::Vector2D> curr_cluster;

    for (size_t i = 0; i < laser_data.ranges.size(); i++) {

      if (laser_data.ranges.at(i) != 0.0) {
        double range_dist = laser_data.ranges.at(i);
        turtlelib::Vector2D range_coords{range_dist * cos(turtlelib::deg2rad(i)), range_dist * sin(
            turtlelib::deg2rad(
              i))};
        if (clusters.empty() && curr_cluster.empty() ) {
          // RCLCPP_INFO_STREAM(get_logger(), "CHECKING FIRST POINT OF FIRST CLUSTER: " << laser_data.ranges.at(i) << " ");
          curr_cluster.push_back(range_coords);
          continue;
        }

        prev_range_coords.x = laser_data.ranges.at(i - 1) * cos(turtlelib::deg2rad(i - 1));
        prev_range_coords.y = laser_data.ranges.at(i - 1) * sin(turtlelib::deg2rad(i - 1));
        double temp_dist = turtlelib::calcDistance(
          prev_range_coords.x, prev_range_coords.y,
          range_coords.x, range_coords.y);

        if (temp_dist <= cluster_dist) {
          curr_cluster.push_back(range_coords);
        } else {
          if (curr_cluster.size() >= 5) {
            clusters.push_back(curr_cluster);
          }
          curr_cluster = {};
          curr_cluster.push_back(range_coords);
        }

      }
    }

    if (clusters.size() >= 2) {
      turtlelib::Vector2D first_cluster{clusters.at(0).at(0).x, clusters.at(0).at(0).y};
      turtlelib::Vector2D last_cluster{clusters.back().back().x, clusters.back().back().y};
      double wrap_dist = turtlelib::calcDistance(
        first_cluster.x, first_cluster.y, last_cluster.x,
        last_cluster.y);

      if (wrap_dist <= cluster_dist) {
        clusters.back().insert(clusters.back().end(), clusters.at(0).begin(), clusters.at(0).end());
        clusters.erase(clusters.begin());
      }
    }

    std::vector<turtlelib::Circle> fitted_circle_list;
    for (size_t i = 0; i < clusters.size(); i++) {
      turtlelib::Circle fitted_circle = turtlelib::circle_fit(clusters.at(i));
      if (fitted_circle.radius > 0.01 && fitted_circle.radius < 0.06) {
        // RCLCPP_INFO_STREAM(get_logger(), "CHECKING FIRST POINT OF FIRST CLUSTER: " << laser_data.ranges.at(i) << " ");
        fitted_circle_list.push_back(fitted_circle);
      }
    }

    visualization_msgs::msg::MarkerArray fitted_circle_arr;
    int id_counter = 0;
    for (size_t i = 0; i < fitted_circle_list.size(); i++) {
      visualization_msgs::msg::Marker circle;
      circle.header.frame_id = "green/base_scan";
      circle.header.stamp = get_clock()->now();
      circle.type = visualization_msgs::msg::Marker::CYLINDER;
      circle.id = id_counter;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.pose.position.x = fitted_circle_list.at(i).center.x;
      circle.pose.position.y = fitted_circle_list.at(i).center.y;
      circle.pose.position.z = 0.0;
      // circle.color.r = 1.0;
      circle.color.b = 1.0;
      circle.color.a = 1.0;
      circle.scale.x = 2 * fitted_circle_list.at(i).radius;
      circle.scale.y = 2 * fitted_circle_list.at(i).radius;
      circle.scale.z = 0.5;
      fitted_circle_arr.markers.push_back(circle);
      id_counter++;
      // RCLCPP_INFO_STREAM(get_logger(), "MARKER LIST SIZE " << fitted_circle_arr.markers.size() << " ");
    }
    circle_publisher_->publish(fitted_circle_arr);

    //   visualization_msgs::msg::MarkerArray landmarks_mark_arr;
    //   int id_counter = 0;
    //   for (size_t i = 0; i < clusters.size(); i++)
    //   {
    //     for (size_t j = 0; j < clusters.at(i).size(); j++)
    //     {
    //       visualization_msgs::msg::Marker cluster;
    //       cluster.header.frame_id = "green/base_scan";
    //       cluster.header.stamp = get_clock()->now();
    //       cluster.type = visualization_msgs::msg::Marker::CYLINDER;
    //       cluster.id = id_counter;
    //       cluster.action = visualization_msgs::msg::Marker::ADD;
    //       cluster.pose.position.x = clusters.at(i).at(j).x;
    //       cluster.pose.position.y = clusters.at(i).at(j).y;
    //       cluster.pose.position.z = 0.0;
    //       // cluster.color.r = 1.0;
    //       cluster.color.b = 1.0;
    //       cluster.color.a = 1.0;
    //       cluster.scale.x = 0.05;
    //       cluster.scale.y = 0.05;
    //       cluster.scale.z = 0.5;
    //       landmarks_mark_arr.markers.push_back(cluster);
    //       id_counter++;
    //     }
    //   }
    // landmark_publisher_->publish(landmarks_mark_arr);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Landmarks>());
  } catch (std::runtime_error & error) {
    std::cerr << error.what();
  }
  rclcpp::shutdown();
  return 0;
}
