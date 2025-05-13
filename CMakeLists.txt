cmake_minimum_required(VERSION 3.5)
project(waterlinked_dvl)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Find dependencies
find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PkgConfig REQUIRED)
find_package(builtin_interfaces REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
# Message and service files
set(msg_files
  "msg/PositionReport.msg"
  "msg/PositionReportStamped.msg"
  "msg/Transducer.msg"
  "msg/TransducerStamped.msg"
  "msg/TransducerReport.msg"
  "msg/TransducerReportStamped.msg"
  "msg/Config.msg"
)

set(srv_files
  "srv/GetConfig.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

# Dynamic Reconfigure equivalent in ROS 2 would typically be parameter support in rclcpp
# If still using dynamic_reconfigure-like .cfg file, external tools are needed

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# Source files
add_executable(${PROJECT_NAME}_tcp_node
  src/waterlinked_dvl_tcp_node.cpp
  src/waterlinked_dvl/waterlinked_dvl_tcp.cpp
)

ament_target_dependencies(${PROJECT_NAME}_tcp_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  tf2
  Boost
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(${PROJECT_NAME}_tcp_node PRIVATE
  ${JSONCPP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_tcp_node
  ${JSONCPP_LIBRARIES}
  ${cpp_typesupport_target}
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_tcp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
