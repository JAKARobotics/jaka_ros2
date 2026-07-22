cmake_minimum_required(VERSION 3.8)
project(jaka_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotMsg.msg"

  "srv/Move.srv"
  "srv/ServoMoveEnable.srv"
  "srv/ServoMove.srv"
  "srv/SetTcpFrame.srv"
  "srv/SetUserFrame.srv"
  "srv/SetPayload.srv"
  "srv/SetCollision.srv"
  "srv/SetIO.srv"
  "srv/GetIO.srv"
  "srv/ClearError.srv"
  "srv/GetFK.srv"
  "srv/GetIK.srv"
   DEPENDENCIES std_msgs std_srvs
)

ament_package()


# install(
  # DIRECTORY msg srv
  # DESTINATION share/${PROJECT_NAME}
# S)


# if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
# endif()

