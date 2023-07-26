#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
/// \brief Robot control node that uses inputted twists and encoder data to
/// calculate the positions and velocities of the robot wheels
///
/// PARAMETERS:
///     wheel_radius (double): the radius of the robots wheels
///     track_width (double): the distance between the center of one wheel to the other
///     motor_cmd_max (int): the max wheel command for the wheel encoders
///     motor_cmd_per_rad_sec (double): the velocity of the wheels (rad/s) per 1 tick
///     encoder_ticks_per_rad (double): the number of encoder ticks per radius that the wheel moves
///     collision_radius (double): distance from the robot chassis that is used to detect a collision
///
/// PUBLISHES:
///     /wheel_cmd (WheelCommands): Publishes the wheel speeds that will make the robot
///                                follow an inputted twist
///
///     /joint_states (JointState): Publishes the joint states of the wheels, position (rad) and velocity (rad/s)
/// SUBSCRIBES:
///     /cmd_vel (Twist): Listens for specified twists for robot control
///
///     /sensor_data (SensorData): Grabs wheel encoder data from the robot that is used to calculate joint states


/// @brief  Creates TurtleControl node
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);
    declare_parameter("motor_cmd_max", 0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    declare_parameter("collision_radius", 0.0);

    wheel_rad = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    motor_cmd_per_rad_sec =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius = get_parameter("collision_radius").get_parameter_value().get<double>();

    if (wheel_rad == 0.0 || track_width == 0.0 || motor_cmd_max == 0 || \
      motor_cmd_per_rad_sec == 0.0 || encoder_ticks_per_rad == 0.0 || collision_radius == 0.0)
    {
      throw (std::runtime_error(" turtle_control node died: Param values not set from yaml \n"));
    }

    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmdvel_callback, this, std::placeholders::_1));

    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &TurtleControl::sensor_data_callback, this,
        std::placeholders::_1));

    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  sensor_msgs::msg::JointState js;
  nuturtlebot_msgs::msg::WheelCommands wheel_cmds;
  double wheel_rad, track_width, motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius, t0;
  int motor_cmd_max;
  double dt;
  turtlelib::DiffDrive ddrive;
  std_msgs::msg::Header prev;


  /// @brief Callback function for the cmd_vel topic. Listens to a
  /// twist and returns the wheel commands needed to follow the twist
  /// @param t - the twist for the robot to follow
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr t)
  {
    turtlelib::Twist2D twist{t->angular.z, t->linear.x, t->linear.y};
    turtlelib::WheelCmds wheels = ddrive.inverse_kinematics(twist);
    wheels.r_wheel /= motor_cmd_per_rad_sec;
    wheels.l_wheel /= motor_cmd_per_rad_sec;

    if (wheels.r_wheel > motor_cmd_max) {
      wheels.r_wheel = motor_cmd_max;
    } else if (wheels.r_wheel < -motor_cmd_max) {
      wheels.r_wheel = -motor_cmd_max;
    }

    if (wheels.l_wheel > motor_cmd_max) {
      wheels.l_wheel = motor_cmd_max;
    } else if (wheels.l_wheel < -motor_cmd_max) {
      wheels.l_wheel = -motor_cmd_max;
    }

    wheel_cmds.right_velocity = wheels.r_wheel;
    wheel_cmds.left_velocity = wheels.l_wheel;
    wheel_cmd_pub_->publish(wheel_cmds);
  }

  /// @brief Callback function for the sensor_data topic. Listens to
  /// the encoder data from the robot wheels and converts that
  /// to the angle (rad) and velocity(rad/s) of the wheels that is sent as joint_states.
  /// @param sensor_data - motor encoder data in ticks
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & sensor_data)
  {

    if (t0 == 0.0) {
      prev.stamp = get_clock()->now();
      t0 = 1;
    } else {
      dt = (sensor_data.stamp.sec + sensor_data.stamp.nanosec * 1e-9) -
        (prev.stamp.sec + prev.stamp.nanosec * 1e-9);
      double right_pos = static_cast<double>(sensor_data.right_encoder) / encoder_ticks_per_rad;
      double left_pos = static_cast<double>(sensor_data.left_encoder) / encoder_ticks_per_rad;

      js.header.stamp = get_clock()->now();
      js.name = {"wheel_right_joint", "wheel_left_joint"};
      js.position = {right_pos, left_pos};
      js.velocity = {right_pos / dt, left_pos / dt};
      joint_states_pub_->publish(js);
      prev.stamp = sensor_data.stamp;
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TurtleControl>());
  } catch (std::runtime_error & error) {
    std::cerr << error.what();
  }
  rclcpp::shutdown();
  return 0;
}
