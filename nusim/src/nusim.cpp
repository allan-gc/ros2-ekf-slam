#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nusim/srv/teleport.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

/// \brief Simulator node that can track and manipulate the position of
/// the turtlebot3 models in rviz

/// PARAMETERS:
///     draw_only (bool): Param that determines if the fake sensor data is published
///     rate (double): controls the frequency of the main timer callback
///     max_range (double): Max distance used to detect if the robot is near the obstacle
///     x0, y0, theta0 (double): used to set the initial position of the turtlebot3 in the simulator
///     obstacles.x (vector<double>): a list of the x-coordinates of the obstacles
///     obstacles.y (vector<double>): a list of the y-coordinates of the obstacles
///     obstacles.h (double): the height of the obstacles
///     obstacles.r (double): the radius of the obstacles
///     basic_sensor_variance (double): Variance used for zero mean Gaussian noise that is added to obstacle positions
///     motor_cmd_per_rad_sec (double): Conversion for motor command ticks to rad/s (the velocity of each motor command tick)
///     encoder_ticks_per_rad (double): Encoder ticks per radian
///     collision_radius (double): Collision circle around the turtlebot
///     input_noise (double): Variance used for zero mean Gaussian noise that is added to wheel commands
///     slipping_fraction (double): Slip added to wheel commands
///     laser_max_range (double): Max distance that the lidar laser can detect (meters)
///     laser_min_range (double): Min distance that the liadr laser can detect (meters)
///     angle_increment (double): Increment of lidar angle
///     resolution (int): Resolution of lidar data
///     samples (int): Amount of lidar data collected per reading
///     noise_level (double): Variance used for zero mean Gaussian noise that is added to lidar ranges

/// PUBLISHES:
///     ~/timestep (UInt64): Publishes the current timestep of the simulation
///                          based off of the timer callback frequency
///
///     ~/obstacles (MarkerArray): Publishes the obstacle markers to rviz
///
///     ~/walls (MarkerArray): Publishes the wall markers to rviz
///
///     /fake_sensor (MarkerArray): Publishes the detected locations of the obstacles with noise
///
///     /red/sensor_data (SensorData): Publishes encoder data
///
///     /red/red_path (Path): Publishes the path of the red robot in rviz
///
///     laser_scan (LaserScan): Publishes the laser scan information collected by the Lidar

/// SUBSCRIBES:
///     /red/wheel_cmd (WheelCommands): Wheel speeds for robot control

/// SERVERS:
///     /reset_srv (Empty): Resets the timestep and robots position to
///                         the intial conditions (timestep=0.0, position=x0,y0,theta0)
///
///     /teleport_srv (Teleport): Places the robots frame at the given position

/// CLIENTS:
///     None


/// @brief  This creates the nusim node that publishes to /~teleport and ~/obstacles topics
class NuSim : public rclcpp::Node
{
public:
  NuSim()
  : Node("nusim"), timestep_(0)
  {
    declare_parameter("draw_only", false);
    declare_parameter("rate", 200.0);
    declare_parameter("max_range", 0.0);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.h", 0.0);
    declare_parameter("obstacles.r", 0.0);
    declare_parameter("arena.x_length", 0.0);
    declare_parameter("arena.y_length", 0.0);
    declare_parameter("arena.wall_height", 0.0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    declare_parameter("collision_radius", 0.0);
    declare_parameter("input_noise", 0.0);
    declare_parameter("slipping_fraction", 0.0);
    declare_parameter("laser_max_range", 0.0);
    declare_parameter("laser_min_range", 0.0);
    declare_parameter("angle_increment", 0.0);
    declare_parameter("resolution", 0);
    declare_parameter("samples", 0);
    declare_parameter("noise_level", 0.0);

    auto rate = get_parameter("rate").get_parameter_value().get<double>();
    max_range = get_parameter("max_range").get_parameter_value().get<double>();
    basic_sensor_variance =
      get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    x0 = get_parameter("x0").get_parameter_value().get<double>();
    y0 = get_parameter("y0").get_parameter_value().get<double>();
    theta0 = get_parameter("theta0").get_parameter_value().get<double>();
    obstacle_x = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacle_y = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacle_h = get_parameter("obstacles.h").get_parameter_value().get<double>();
    obstacle_r = get_parameter("obstacles.r").get_parameter_value().get<double>();
    arena_x_length = get_parameter("arena.x_length").get_parameter_value().get<double>();
    arena_y_length = get_parameter("arena.y_length").get_parameter_value().get<double>();
    arena_wall_height = get_parameter("arena.wall_height").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius = get_parameter("collision_radius").get_parameter_value().get<double>();
    input_noise = get_parameter("input_noise").get_parameter_value().get<double>();
    slipping_fraction = get_parameter("slipping_fraction").get_parameter_value().get<double>();
    laser_max_range = get_parameter("laser_max_range").get_parameter_value().get<double>();
    laser_min_range = get_parameter("laser_min_range").get_parameter_value().get<double>();
    angle_increment = get_parameter("angle_increment").get_parameter_value().get<double>();
    resolution = get_parameter("resolution").get_parameter_value().get<int>();
    samples = get_parameter("samples").get_parameter_value().get<int>();
    noise_level = get_parameter("noise_level").get_parameter_value().get<double>();
    draw = get_parameter("draw_only").get_parameter_value().get<bool>();

    // Max, min angles for lidar scan
    angle_min = 0.0;
    angle_max = 6.2657318115234375;
    scan_time = 0.20066890120506287;

    x = x0;
    y = y0;
    theta = theta0;
    dt = 1.0 / rate;
    mark_arr = make_obstacles();
    wall_marker_arr = make_walls();
    noise_dist = std::normal_distribution<>(0.0, pow(input_noise, 0.5));
    position_dist = std::normal_distribution<>(0.0, basic_sensor_variance);
    laser_noise = std::normal_distribution<>(0.0, pow(noise_level, 0.5));
    slip_dist = std::uniform_real_distribution<>(-slipping_fraction, slipping_fraction);
    path_counter = 0;
    timer_ = create_wall_timer(1s / rate, std::bind(&NuSim::timer_callback, this));
    fake_sensor_timer_ =
      create_wall_timer(1s / 5.0, std::bind(&NuSim::fake_sensor_timer_callback, this));
    publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    obstacle_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    wall_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    fake_sensor_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor",
      10);
    sensor_data_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    red_path_pub_ = create_publisher<nav_msgs::msg::Path>("red/red_path", 10);
    laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
    wheel_cmds_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&NuSim::wheel_cmd_callback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "reset_srv", std::bind(
        &NuSim::reset_cb, \
        this, std::placeholders::_1, std::placeholders::_2));
    teleport_service_ = create_service<nusim::srv::Teleport>(
      "teleport_srv", std::bind(
        &NuSim::tele_cb, \
        this, std::placeholders::_1, std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_sensor_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_path_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmds_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  uint64_t timestep_;
  double x0, y0, theta0, x, y, theta, obstacle_r, obstacle_h, dt;
  double arena_wall_height, arena_x_length, arena_y_length;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  double circle_intersect;
  double input_noise, slipping_fraction;
  double max_range, basic_sensor_variance;
  double laser_max_range, laser_min_range, angle_increment;
  double angle_min, angle_max, scan_time, noise_level;
  int resolution, samples;
  bool draw;
  std::vector<double> obstacle_x, obstacle_y;
  std::normal_distribution<> noise_dist, position_dist, laser_noise;
  std::uniform_real_distribution<> slip_dist;
  visualization_msgs::msg::MarkerArray mark_arr, wall_marker_arr, fake_sensor_marker_arr;
  nuturtlebot_msgs::msg::SensorData sensor_data;
  nav_msgs::msg::Path path_msg;
  sensor_msgs::msg::LaserScan laser_msg;
  geometry_msgs::msg::PoseStamped path_poses;
  tf2::Quaternion quats;
  geometry_msgs::msg::TransformStamped world_red;
  turtlelib::DiffDrive ddrive;
  turtlelib::WheelCmds new_wheel_pos_fk;
  turtlelib::WheelCmds prev_wheel_pos{0.0, 0.0};
  double new_right_wheel_vel = 0.0;
  double new_left_wheel_vel = 0.0;
  int path_counter;

  /// @brief Main timer callback for nusim node
  void timer_callback()
  {
    /// Calculates new wheel positions using FK
    path_counter++;
    double new_right_wheel_pos = new_right_wheel_vel * dt;
    double new_left_wheel_pos = new_left_wheel_vel * dt;
    new_wheel_pos_fk = {new_right_wheel_pos, new_left_wheel_pos};
    ddrive.forward_kinematics(new_wheel_pos_fk);
    x = ddrive.configuration().x;
    y = ddrive.configuration().y;
    theta = ddrive.configuration().theta;
    collisionDetection();
    double updated_wheel_pos_right = prev_wheel_pos.r_wheel + (new_right_wheel_pos);
    double updated_wheel_pos_left = prev_wheel_pos.l_wheel + ( new_left_wheel_pos);
    prev_wheel_pos.r_wheel = updated_wheel_pos_right;
    prev_wheel_pos.l_wheel = updated_wheel_pos_left;

    sensor_data.right_encoder = updated_wheel_pos_right * encoder_ticks_per_rad;
    sensor_data.left_encoder = updated_wheel_pos_left * encoder_ticks_per_rad;
    sensor_data.stamp = get_clock()->now();
    sensor_data_pub_->publish(sensor_data);

    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    publisher_->publish(message);
    timestep_++;

    world_red.header.stamp = get_clock()->now();
    world_red.header.frame_id = "nusim/world";
    world_red.child_frame_id = "red/base_footprint";
    world_red.transform.translation.x = x;
    world_red.transform.translation.y = y;
    world_red.transform.translation.z = 0.0;

    quats.setRPY(0, 0, theta);
    world_red.transform.rotation.x = quats.x();
    world_red.transform.rotation.y = quats.y();
    world_red.transform.rotation.z = quats.z();
    world_red.transform.rotation.w = quats.w();
    tf_broadcaster_->sendTransform(world_red);

    if (path_counter % 100 == 0) {
      path_msg.header.stamp = get_clock()->now();
      path_msg.header.frame_id = "nusim/world";
      path_poses.pose.position.x = x;
      path_poses.pose.position.y = y;
      path_poses.pose.orientation.x = quats.x();
      path_poses.pose.orientation.y = quats.y();
      path_poses.pose.orientation.z = quats.z();
      path_poses.pose.orientation.w = quats.w();
      path_msg.poses.push_back(path_poses);
    }

    red_path_pub_->publish(path_msg);

    // Publish rviz markers
    obstacle_publisher_->publish(mark_arr);
    wall_publisher_->publish(wall_marker_arr);
  }

  /// @brief Timer for simulated obstacle markers, running at 5Hz
  void fake_sensor_timer_callback()
  {
    if (draw == false) {
      fake_sensor_marker_arr = fake_sensor_obstacles();
      fake_sensor_publisher_->publish(fake_sensor_marker_arr);
      laserObstacleIntersect();
      laser_pub_->publish(laser_msg);
    }
  }

  /// @brief Checks if laser scan intersects with any obstacles or walls
  void laserObstacleIntersect()
  {
    laser_msg.header.stamp = get_clock()->now();
    laser_msg.header.frame_id = "red/base_scan";
    laser_msg.angle_min = angle_min;
    laser_msg.angle_max = angle_max;
    laser_msg.angle_increment = angle_increment;
    laser_msg.time_increment = 0.0;
    laser_msg.scan_time = scan_time;
    laser_msg.range_min = laser_min_range;
    laser_msg.range_max = laser_max_range;
    std::vector<float> ranges(samples, 0.0);
    laser_msg.ranges = ranges;

    double wall_pos_x[4] =
    {-arena_x_length / 2, arena_x_length / 2, arena_x_length / 2, -arena_x_length / 2};
    double wall_pos_y[4] =
    {-arena_y_length / 2, -arena_y_length / 2, arena_y_length / 2, arena_y_length / 2};
    turtlelib::RobotConfig config = ddrive.configuration();

    for (int i = 0; i < samples; i++) {
      double max_x = config.x + (cos(config.theta + (i * angle_increment)) * laser_max_range);
      double max_y = config.y + (sin(config.theta + (i * angle_increment)) * laser_max_range);
      double slope = (max_y - config.y) / (max_x - config.x);
      double x_sol = 0.0;
      double y_sol = 0.0;
      double min_distance = max_range * 2;
      double temp_dist = 0.0;
      for (size_t j = 0; j < obstacle_x.size(); j++) {
        double phi = config.y - (slope * config.x) - obstacle_y.at(j);
        double a = 1 + (slope * slope);
        double b = 2 * ((phi * slope) - obstacle_x.at(j));
        double c = (obstacle_x.at(j) * obstacle_x.at(j)) + (phi * phi) - (obstacle_r * obstacle_r);
        double det = (b * b) - 4 * (a * c);

        if (det == 0) { // Found obstacle intersection that is tangent to obstacle
          x_sol = -b / (2 * a);
          y_sol = slope * (x_sol - config.x) + config.y;
          if (((x_sol - config.x) / (max_x - config.x) > 0) && \
            ((y_sol - config.y) / (max_y - config.y) > 0))
          {
            temp_dist = turtlelib::calcDistance(config.x, config.y, x_sol, y_sol);
            if (temp_dist < min_distance) {
              min_distance = temp_dist;
              laser_msg.ranges.at(i) = min_distance + laser_noise(get_random());
            }
          }
        }

        if (det > 0) { // Found obstacle intersection that is not tangent to obstacle, so choose intersection with closest distance
          double x_sol1 = (-b + sqrt(det)) / (2 * a);
          double y_sol1 = slope * (x_sol1 - config.x) + config.y;
          double x_sol2 = (-b - sqrt(det)) / (2 * a);
          double y_sol2 = slope * (x_sol2 - config.x) + config.y;
          double dist1 = turtlelib::calcDistance(config.x, config.y, x_sol1, y_sol1);
          double dist2 = turtlelib::calcDistance(config.x, config.y, x_sol2, y_sol2);

          if ((dist1 < dist2) && ((x_sol1 - config.x) / (max_x - config.x) > 0) && \
            ((y_sol1 - config.y) / (max_y - config.y) > 0))
          {
            temp_dist = dist1;
            if (temp_dist < min_distance) {
              min_distance = temp_dist;
              laser_msg.ranges.at(i) = min_distance + laser_noise(get_random());
            }
          } else if (((x_sol2 - config.x) / (max_x - config.x) > 0) && \
            ((y_sol2 - config.y) / (max_y - config.y) > 0))
          {
            temp_dist = dist2;
            if (temp_dist < min_distance) {
              min_distance = temp_dist;
              laser_msg.ranges.at(i) = min_distance + laser_noise(get_random());
            }
          }
        }


        if (det < 0) { // If determinant = 0, no obstacle intersections are found, so check for wall intersections

          double wall1_y = wall_pos_y[3] - 0.1;
          double wall1_x = ((wall1_y - config.y) / slope) + config.x;

          double wall2_x = wall_pos_x[2] - 0.1;
          double wall2_y = slope * (wall2_x - config.x) + config.y;

          double wall3_y = wall_pos_y[1] + 0.1;
          double wall3_x = ((wall3_y - config.y) / slope) + config.x;

          double wall4_x = wall_pos_x[4] + 0.1;
          double wall4_y = slope * (wall4_x - config.x) + config.y;


          if (((wall1_x - config.x) / (max_x - config.x) > 0) && \
            ((wall1_y - config.y) / (max_y - config.y) > 0))
          {
            temp_dist = turtlelib::calcDistance(config.x, config.y, wall1_x, wall1_y);
            if (temp_dist < min_distance) {
              min_distance = temp_dist;
              laser_msg.ranges.at(i) = min_distance + laser_noise(get_random());
            }
          }

          if (((wall2_x - config.x) / (max_x - config.x) > 0) && \
            ((wall2_y - config.y) / (max_y - config.y) > 0))
          {
            temp_dist = turtlelib::calcDistance(config.x, config.y, wall2_x, wall2_y);
            if (temp_dist < min_distance) {
              min_distance = temp_dist;
              laser_msg.ranges.at(i) = min_distance + laser_noise(get_random());
            }
          }

          if (((wall3_x - config.x) / (max_x - config.x) > 0) && \
            ((wall3_y - config.y) / (max_y - config.y) > 0))
          {
            temp_dist = turtlelib::calcDistance(config.x, config.y, wall3_x, wall3_y);
            if (temp_dist < min_distance) {
              min_distance = temp_dist;
              laser_msg.ranges.at(i) = min_distance + laser_noise(get_random());
            }
          }

          if (((wall4_x - config.x) / (max_x - config.x) > 0) && \
            ((wall4_y - config.y) / (max_y - config.y) > 0))
          {
            temp_dist = turtlelib::calcDistance(config.x, config.y, wall4_x, wall4_y);
            if (temp_dist < min_distance) {
              min_distance = temp_dist;
              laser_msg.ranges.at(i) = min_distance + laser_noise(get_random());
            }
          }
        }
      }
    }
  }


  /// @brief Function for handling random seeding
  /// @return random number object
  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }

  /// @brief Callback function for reset service
  ///        that resets robots position.
  ///        Request and response are empty
  void reset_cb(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    timestep_ = 0.0;
    x = x0;
    y = y0;
    theta = theta0;
  }

  /// @brief Callback function for teleport service that
  ///       sets the robots position to the inputted position
  /// @param request takes in desired x,y,theta position for the robot
  void tele_cb(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr)
  {
    x = request->x;
    y = request->y;
    theta = request->theta;
  }

  /// @brief Creates the marker for each obstacle and adds them to
  ///        a marker array
  /// @return mark_arr, the marker array of the obstacles
  visualization_msgs::msg::MarkerArray make_obstacles()
  {
    if (obstacle_x.size() != obstacle_y.size()) {
      throw (std::runtime_error(" x and y coordinate lists must be same size \n"));
    }
    visualization_msgs::msg::MarkerArray mark_arr;
    std_msgs::msg::Header time_obstacles;
    time_obstacles.stamp = get_clock()->now();
    for (size_t i = 0; i < obstacle_x.size(); i++) {
      visualization_msgs::msg::Marker obstacle;
      obstacle.header.frame_id = "nusim/world";
      obstacle.header.stamp = time_obstacles.stamp;
      obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle.id = i;
      obstacle.action = visualization_msgs::msg::Marker::ADD;
      obstacle.pose.position.x = obstacle_x.at(i);
      obstacle.pose.position.y = obstacle_y.at(i);
      obstacle.pose.position.z = obstacle_h / 2;
      obstacle.color.r = 0.5;
      obstacle.color.a = 1.0;
      obstacle.scale.x = 2 * obstacle_r;
      obstacle.scale.y = 2 * obstacle_r;
      obstacle.scale.z = obstacle_h;
      mark_arr.markers.push_back(obstacle);
    }
    return mark_arr;
  }

  /// @brief Creates marker array for walls placed
  ///       around the environment
  /// @return wall_mark_arr, the marker array of the walls
  visualization_msgs::msg::MarkerArray make_walls()
  {
    double scale_x[4] = {arena_x_length, 0.2, 0.2, arena_x_length};
    double scale_y[4] = {0.2, arena_y_length, arena_y_length, 0.2};

    double pos_x[4] = {0.0, -arena_x_length / 2, arena_x_length / 2, 0.0};
    double pos_y[4] = {-arena_y_length / 2, 0.0, 0.0, arena_y_length / 2};

    std_msgs::msg::Header time_walls;
    time_walls.stamp = get_clock()->now();

    visualization_msgs::msg::MarkerArray wall_mark_arr;
    for (size_t i = 0; i < 4; i++) {
      visualization_msgs::msg::Marker wall;
      wall.header.frame_id = "nusim/world";
      wall.header.stamp = time_walls.stamp;
      wall.type = visualization_msgs::msg::Marker::CUBE;
      wall.id = i;
      wall.action = visualization_msgs::msg::Marker::ADD;
      wall.color.r = 0.5;
      wall.color.a = 1.0;
      wall.scale.x = scale_x[i];
      wall.scale.y = scale_y[i];
      wall.scale.z = arena_wall_height;
      wall.pose.position.x = pos_x[i];
      wall.pose.position.y = pos_y[i];
      wall.pose.position.z = wall.scale.z / 2;
      wall_mark_arr.markers.push_back(wall);
    }
    return wall_mark_arr;
  }

  /// @brief Creates simulated obstacles for the fake sensor
  /// @return A marker array with fake sensor obstacles
  visualization_msgs::msg::MarkerArray fake_sensor_obstacles()
  {
    turtlelib::Vector2D robot_xy{ddrive.configuration().x, ddrive.configuration().y};
    turtlelib::Transform2D T_wr{robot_xy, ddrive.configuration().theta}; // Tf for world to robot
    turtlelib::Transform2D T_rw = T_wr.inv(); // Tf for robot to world

    visualization_msgs::msg::MarkerArray obstacle_mark_arr;
    std_msgs::msg::Header time_fake_sensor;
    time_fake_sensor.stamp = get_clock()->now();

    for (size_t i = 0; i < obstacle_x.size(); i++) {

      turtlelib::Vector2D obstacle_xy{obstacle_x.at(i), obstacle_y.at(i)};
      turtlelib::Transform2D T_wo{obstacle_xy}; // Tf for world to obstacles
      turtlelib::Transform2D T_ro = T_rw * T_wo;   // Tf for robot to obstacles
      turtlelib::Vector2D obstacle_positions = T_ro.translation();  // Get the x and y coorindates of the obstacles relative to the robot
      double distance_mag = sqrt(pow(obstacle_positions.x, 2) + pow(obstacle_positions.y, 2));

      visualization_msgs::msg::Marker obstacle;
      obstacle.header.frame_id = "red/base_footprint";
      obstacle.header.stamp = time_fake_sensor.stamp;
      obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle.id = i;

      if (distance_mag > max_range) {
        obstacle.action = visualization_msgs::msg::Marker::DELETE;
      } else {
        obstacle.action = visualization_msgs::msg::Marker::ADD;
      }
      obstacle.pose.position.x = obstacle_positions.x + position_dist(get_random());
      obstacle.pose.position.y = obstacle_positions.y + position_dist(get_random());
      obstacle.pose.position.z = obstacle_h / 2;
      obstacle.color.r = 1.0;
      obstacle.color.g = 1.0;
      obstacle.color.a = 1.0;
      obstacle.scale.x = 2 * obstacle_r;
      obstacle.scale.y = 2 * obstacle_r;
      obstacle.scale.z = obstacle_h;
      obstacle_mark_arr.markers.push_back(obstacle);
    }
    return obstacle_mark_arr;
  }


  /// @brief Function that moves the robot if a collision is detected
  ///         by correcting its current config
  void collisionDetection()
  {
    for (size_t i = 0; i < obstacle_x.size(); i++) {
      circle_intersect =
        sqrt(
        (x - obstacle_x.at(i)) * (x - obstacle_x.at(i)) + (y - obstacle_y.at(i)) *
        (y - obstacle_y.at(i)));
      if (circle_intersect < collision_radius + obstacle_r) {
        turtlelib::Vector2D robot_pos{x, y};  // Current position of the robot
        turtlelib::Vector2D Vc{x - obstacle_x.at(i), y - obstacle_y.at(i)}; // Line from robot center to obstacle center
        turtlelib::Vector2D Vc_unit = turtlelib::norm(Vc);
        double dist = turtlelib::magnitude(Vc); // Calculate the distance betwen the robot center and obstacle center
        double intersect_d = (collision_radius + obstacle_r) - dist; // Distance the robot must move back so that it is not intersecting
        turtlelib::Vector2D new_pos = robot_pos + (intersect_d * Vc_unit);
        x = new_pos.x;
        y = new_pos.y;
        ddrive.setConfig(x, y, theta);
      }
    }
  }

  /// @brief Callback for wheel_cmd topic. Converts recieved wheel commands to wheel positions
  /// @param wheel_commands - wheel commands recieved from topic
  void wheel_cmd_callback(nuturtlebot_msgs::msg::WheelCommands wheel_commands)
  {
    double noise_right = noise_dist(get_random());
    double noise_left = noise_dist(get_random());
    double slip_right = slip_dist(get_random());
    double slip_left = slip_dist(get_random());

    new_right_wheel_vel = (static_cast<double>(wheel_commands.right_velocity)) *
      motor_cmd_per_rad_sec;
    new_left_wheel_vel = (static_cast<double>(wheel_commands.left_velocity)) *
      motor_cmd_per_rad_sec;

    if (new_right_wheel_vel != 0.0) {
      new_right_wheel_vel += noise_right;
      new_right_wheel_vel *= (1 + slip_right);
    }

    if (new_left_wheel_vel != 0.0) {
      new_left_wheel_vel += noise_left;
      new_left_wheel_vel *= (1 + slip_left);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<NuSim>());
  } catch (std::runtime_error & error) {
    std::cerr << error.what();
  }
  rclcpp::shutdown();
  return 0;
}
