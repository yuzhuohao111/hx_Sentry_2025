cmake_minimum_required(VERSION 3.10)
project(rm_decision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/Armor.msg"
  "msg/Flag.msg"
  "msg/NavFlag.msg"
  "msg/Mode.msg"
  "msg/Yaw.msg"
  "msg/RadarMode.msg"
  "srv/PidPlannerStatus.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_package()
