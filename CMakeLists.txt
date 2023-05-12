cmake_minimum_required(VERSION 3.5)
project(rclcpp_mqtt_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  /usr/local/include
  /usr/local/include/mqtt
)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_library(PAHO_MQTT_CPP_LIB paho-mqttpp3 PATHS /usr/local/lib REQUIRED)

add_executable(ros_connection_bridge src/ros_connection_bridge/ros_connection_bridge.cpp)
ament_target_dependencies(ros_connection_bridge rcl rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs nav2_msgs tf2_msgs)

add_executable(ros_mqtt_bridge src/ros_mqtt_bridge/ros_mqtt_bridge.cpp src/ros_mqtt_bridge/connections/ros_mqtt_message_converter.cpp)
target_link_libraries(ros_mqtt_bridge ${PAHO_MQTT_CPP_LIB} -lpaho-mqtt3as jsoncpp)
ament_target_dependencies(ros_mqtt_bridge rcl rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs nav2_msgs tf2_msgs)

install(TARGETS
  ros_connection_bridge
  ros_mqtt_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROEJCT_NAME}
  OPTIONAL
)

ament_package()