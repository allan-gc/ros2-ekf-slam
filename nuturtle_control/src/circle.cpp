#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/// \brief Circle node that sends controls for the tutlebot to move along an arc
///
/// PARAMETERS:
///     frequency (double): controls the frequency of the main timer callback
///
/// PUBLISHES:
///     /cmd_vel (Twist): Publishes twist commands that control robots movement
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     /control_srv (Control): Sets the radius of the circle for the robot to follow
//                              and the velocity of the robot
///
///     /reverse_srv (Empty): Reverses the robots direction along the arc
///
///     /stop_srv (Empty): Stops the robot along the arc
///
/// CLIENTS:
///     None


/// @brief  Creates the circle node with its publishers and services
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("Circle")
  {
    declare_parameter("frequency", 100.0);
    auto frequency = get_parameter("frequency").get_parameter_value().get<double>();

    timer_ = create_wall_timer(1000ms / frequency, std::bind(&Circle::timer_callback, this));
    cmds_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    control_service_ = create_service<nuturtle_control::srv::Control>(
      "control_srv", std::bind(
        &Circle::control_cb, \
        this, std::placeholders::_1, std::placeholders::_2));

    reverse_service_ = create_service<std_srvs::srv::Empty>(
      "reverse_srv", std::bind(
        &Circle::reverse_cb, \
        this, std::placeholders::_1, std::placeholders::_2));

    stop_service_ = create_service<std_srvs::srv::Empty>(
      "stop_srv", std::bind(
        &Circle::stop_cb, \
        this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmds_pub_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  geometry_msgs::msg::Twist Vb;
  int counter = 0;

  /// @brief Main timer callback for circle node
  void timer_callback()
  {
    if (Vb.linear.x != 0.0 && Vb.angular.z != 0.0) {
      cmds_pub_->publish(Vb);
    } else if (counter == 1) {
      cmds_pub_->publish(Vb);
      counter = 0;
    }
  }

  /// @brief  Callback function for control service. Sends commands for
  ///         robot to move along circle.
  /// @param request - the radius of the arc and velocity of the robot
  /// Returns a twist command for the robot
  void control_cb(
    nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr)
  {
    Vb.angular.z = request->velocity;
    Vb.linear.x = (request->velocity * request->radius);
    Vb.linear.y = 0.0;
  }

  /// @brief Callback function for reverse service
  ///        that reverses the direction of robot along the arc
  ///        Request and response are empty
  void reverse_cb(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    Vb.angular.z *= -1;
    Vb.linear.x *= -1;
    Vb.linear.y = 0.0;
  }

  /// @brief Callback function for stop service
  ///        that stops the robot.
  ///        Request and response are empty
  void stop_cb(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    Vb.angular.z = 0.0;
    Vb.linear.x = 0.0;
    Vb.linear.y = 0.0;
    counter += 1;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
