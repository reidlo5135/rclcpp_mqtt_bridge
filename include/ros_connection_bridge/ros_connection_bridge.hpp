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

#ifndef ROS_CONNECTION_BRIDGE
#define ROS_CONNECTION_BRIDGE

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

/**
 * include rclcpp header files
 * @see rclcpp/rclcpp.hpp
 * @see std_msgs/msg/string.hpp
 * @see nav_msgs/msg/odemtery.hpp
 * @see ros_connection_bridge/connections/ros_connections.hpp
*/
#include "rclcpp/rclcpp.hpp"
#include "ros_connection_bridge/connections/ros_connections.hpp"

#define LOG_ROS_CONNECTION_BRIDGE "[ROS-CONNECTION-BRIDGE]"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Class for initialize rclcpp::Node & register ros2 connections into node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosConnectionBridge : public rclcpp::Node {
    private :
        const std::string& log_ros_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        ros_connections::ros_connections_to_mqtt::Bridge * ros_connections_to_mqtt_bridge_ptr_;
        ros_connections::ros_connections_from_mqtt::Bridge * ros_connections_from_mqtt_bridge_ptr_;
        void check_current_topics_and_types();
    public :
        RosConnectionBridge();
        virtual ~RosConnectionBridge();
};

// function for check rclcpp status
void check_rclcpp_status();

#endif