cmake_minimum_required(VERSION 3.8)
project(learn_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(learn_ros2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/RobotStatus.msg"
  "srv/MoveRobot.srv"
  DEPENDENCIES geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
# node 
add_executable(first_node src/node.cpp)
ament_target_dependencies(first_node rclcpp)
install(TARGETS
  first_node
  DESTINATION lib/${PROJECT_NAME}
)

# publisher
add_executable(pub_node src/publisher.cpp)
ament_target_dependencies(pub_node rclcpp std_msgs)


# subscriber
add_executable(sub_node src/subscriber.cpp)
ament_target_dependencies(sub_node rclcpp std_msgs)


# server
add_executable(server_node src/server.cpp)
ament_target_dependencies(server_node rclcpp example_interfaces)
install(TARGETS
  server_node
  DESTINATION lib/${PROJECT_NAME}
)
# client
add_executable(client_node src/client.cpp)
ament_target_dependencies(client_node rclcpp example_interfaces)
install(TARGETS
  client_node
  DESTINATION lib/${PROJECT_NAME}
)


# robot
add_executable(robot_node src/robot.cpp)
ament_target_dependencies(robot_node rclcpp learn_ros2)
install(TARGETS
  robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# controller
add_executable(controller_node src/controller.cpp)
ament_target_dependencies(controller_node rclcpp learn_ros2)
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)