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
  include/ros_mqtt_bridge/
  /usr/local/include
  /usr/local/include/mqtt
)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_library(PAHO_MQTT_CPP_LIB paho-mqttpp3 PATHS /usr/local/lib)

add_executable(ros_mqtt_bridge src/ros_mqtt_bridge/ros_mqtt_bridge.cpp src/mqtt/mqtt.cpp src/main.cpp)
target_link_libraries(ros_mqtt_bridge ${PAHO_MQTT_CPP_LIB} -lpaho-mqtt3as)

add_executable(ros_mqtt_publisher src/ros_mqtt_bridge/publisher/ros_mqtt_publisher.cpp src/mqtt/mqtt.cpp)
target_link_libraries(ros_mqtt_publisher ${PAHO_MQTT_CPP_LIB} -lpaho-mqtt3as)

add_executable(ros_mqtt_subscription src/ros_mqtt_bridge/subscription/ros_mqtt_subscription.cpp src/mqtt/mqtt.cpp)
target_link_libraries(ros_mqtt_subscription ${PAHO_MQTT_CPP_LIB} -lpaho-mqtt3as)

ament_target_dependencies(ros_mqtt_bridge rcl rclcpp std_msgs nav_msgs)
ament_target_dependencies(ros_mqtt_publisher rcl rclcpp std_msgs nav_msgs)
ament_target_dependencies(ros_mqtt_subscription rcl rclcpp std_msgs nav_msgs)

install(TARGETS
  ros_mqtt_bridge
  ros_mqtt_publisher
  ros_mqtt_subscription
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()