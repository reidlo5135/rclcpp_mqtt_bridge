cmake_minimum_required(VERSION 3.5)
project(rclcpp_mqtt_client)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_library(PAHO_MQTT_C_LIB paho-mqtt3c PATHS /usr/local/lib)

include_directories(
  include
  include/${PROJECT_NAME}
  /usr/local/include
)

add_executable(ros_mqtt_bridge src/ros_mqtt_bridge/ros_mqtt_bridge.cpp src/mqtt/mqtt.cpp)
target_link_libraries(ros_mqtt_bridge ${PAHO_MQTT_C_LIB})
ament_target_dependencies(ros_mqtt_bridge rcl rclcpp std_msgs)

install(TARGETS
  ros_mqtt_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
