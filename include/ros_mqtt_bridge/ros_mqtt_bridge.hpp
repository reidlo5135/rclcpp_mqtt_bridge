// Copyright [2023] [wavem-reidlo]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS_MQTT_BRIDGE
#define ROS_MQTT_BRIDGE

/**
 * include cpp header files
 * @see iostream
 * @see math.h
 * @see signal.h
 * @see functional
*/
#include <iostream>
#include <map>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>
#include <jsoncpp/json/json.h>

/**
 * include rclcpp header files
 * @see rclcpp/rclcpp.hpp
 * @see ros_mqtt_bridge/connections/ros_mqtt_connections.hpp
*/
#include "rclcpp/rclcpp.hpp"
#include "ros_mqtt_bridge/connections/ros_mqtt_connections.hpp"

// define common log for ros-mqtt-bridge
#define LOG_ROS_MQTT_BRIDGE "[ROS-MQTT-BRIDGE]"

/**
 * @brief Class for initialize rclcpp::Node & ros_mqtt_connections::to_ros::Bridge / ros_mqtt_connections::to_mqtt::Bridge classes' instances
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @see std::shared_ptr<rclcpp::Node>
*/
class RosMqttBridge : public rclcpp::Node {
    private :
        const std::string& log_ros_mqtt_bridge_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        ros_mqtt_connections::to_ros::Bridge * ros_mqtt_connections_to_ros_bridge_ptr_;
        ros_mqtt_connections::to_mqtt::Bridge * ros_mqtt_conenction_to_mqtt_bridge_ptr_;
    public :
        RosMqttBridge();
        virtual ~RosMqttBridge();
};

// function for check rclcpp status
void check_rclcpp_status();

#endif