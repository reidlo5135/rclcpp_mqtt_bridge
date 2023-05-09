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
 * @see std_msgs/msgs/string.hpp
 * @see nav_msgs/msg/odometry.hpp
 * @see ros_mqtt_bridge/connections/ros_mqtt_connections.hpp
*/
#include "mqtt/async_client.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_mqtt_bridge/connections/ros_mqtt_connections.hpp"
#include "ros_mqtt_bridge/connections/ros_message_converter.hpp"

// define common log string
#define LOG_ROS_MQTT_BRIDGE "[ROS-MQTT-BRIDGE]"
// define mqtt address
#define MQTT_ADDRESS    "tcp://localhost:1883"
// define mqtt client id
#define MQTT_CLIENT_ID    "ros_mqtt_bridge"
// define mqtt qos
#define MQTT_QOS         0
// define mqtt retry attempts
#define MQTT_N_RETRY_ATTEMPTS 5

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Class for manage connections between ros - mqtt
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @see mqtt::callback
 * @see std::shared_ptr<rclcpp::Node>
*/
class RosMqttConnectionManager : public virtual mqtt::callback {
    private :
        const std::string& log_ros_mqtt_bridge_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        ros_message_converter::ros_std_msgs::StdMessageConverter * std_msgs_converter_ptr_;
        ros_message_converter::ros_nav_msgs::NavMessageConverter * nav_msgs_converter_ptr_;
        mqtt::async_client mqtt_async_client_;
        const int mqtt_qos_;
		const int mqtt_is_success_;
        void mqtt_connect();
        void connection_lost(const std::string& mqtt_connection_lost_cause) override;
		void message_arrived(mqtt::const_message_ptr mqtt_message) override;
		void delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) override;
        void mqtt_publish(char * topic, std::string payload);
		void mqtt_subscribe(char * topic);
        void create_ros_publishers();
        void create_ros_subscriptions();
        void create_ros_bridge();
    public :
        RosMqttConnectionManager(std::shared_ptr<rclcpp::Node> ros_node_ptr);
        virtual ~RosMqttConnectionManager();
};

/**
 * @brief Class for initialize rclcpp::Node & RosMqttConnectionManager instance
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @see std::shared_ptr<rclcpp::Node>
*/
class RosMqttBridge : public rclcpp::Node {
    private :
        const std::string& log_ros_mqtt_bridge_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosMqttConnectionManager * ros_mqtt_connection_manager_ptr_;
    public :
        RosMqttBridge();
        virtual ~RosMqttBridge();
};

// function for check rclcpp status
void check_rclcpp_status();

#endif