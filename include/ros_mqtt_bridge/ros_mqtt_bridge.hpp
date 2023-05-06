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
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>
#include <jsoncpp/json/json.h>

/**
 * include mqtt header file
 * @see mqtt/mqtt.hpp
*/
#include "mqtt/mqtt.hpp"

/**
 * include rclcpp header files
 * @see rclcpp/rclcpp.hpp
 * @see std_msgs/msgs/string.hpp
 * @see nav_msgs/msg/odometry.hpp
 * @see ros_mqtt_bridge/connections/ros_mqtt_connections.hpp
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_mqtt_bridge/connections/ros_mqtt_connections.hpp"
#include "ros_mqtt_bridge/connections/ros_message_converter.hpp"

// define common log string
#define LOG_ROS_SUBSCRITPION "[RosSubscription]"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Class for establish ros2 mqtt connections
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosMqttSubscription {
    private :
        const std::string& log_ros_subscription_;
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        ros_message_converter::ros_std_msgs::StdMessageConverter * std_msgs_converter_ptr_;
        ros_message_converter::ros_nav_msgs::NavMessageConverter * nav_msgs_converter_ptr_;
    public :
        RosMqttSubscription(MqttMgr * mqtt_ptr, std::shared_ptr<rclcpp::Node> ros_node_ptr_);
        virtual ~RosMqttSubscription();
        void create_ros_mqtt_bridge();
};

/**
 * @brief Class for initialize rclcpp::Node & bridge connections between ros2 - mqtt
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosMqttBridge : public rclcpp::Node {
    private :
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosMqttSubscription * ros_subscription_ptr_;
    public :
        RosMqttBridge(MqttMgr * mqtt_ptr);
        virtual ~RosMqttBridge();
};

// function for check rclcpp status
void check_rclcpp_status();

#endif