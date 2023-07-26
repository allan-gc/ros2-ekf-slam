cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv "srv/InitialPose.srv" "srv/Control.srv" LIBRARY_NAME ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")


add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)


ament_target_dependencies(
  turtle_control
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nuturtlebot_msgs
  sensor_msgs)

ament_target_dependencies(
  odometry
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nuturtlebot_msgs
  nav_msgs
  tf2_ros
  tf2
  sensor_msgs)

ament_target_dependencies(
  circle
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nuturtlebot_msgs)

target_link_libraries(turtle_control turtlelib::turtlelib)
target_link_libraries(odometry turtlelib::turtlelib)
target_link_libraries(circle turtlelib::turtlelib)

target_link_libraries(odometry "${cpp_typesupport_target}")
target_link_libraries(circle "${cpp_typesupport_target}")



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  turtle_control
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/nuturtle_control/
)

ament_package()
