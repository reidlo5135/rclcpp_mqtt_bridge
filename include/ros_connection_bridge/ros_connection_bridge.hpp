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
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_connection_bridge/connections/ros_connections.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Class for establish ros2 publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosConnectionPublisher {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
    public :
        RosConnectionPublisher(std::shared_ptr<rclcpp::Node> ros_node_ptr);
        virtual ~RosConnectionPublisher();
        void create_publishers();
};

/**
 * @brief Class for establish ros2 subscriptions
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosConnectionSubscription {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
    public :
        RosConnectionSubscription(std::shared_ptr<rclcpp::Node> ros_node_ptr);
        virtual ~RosConnectionSubscription();
        void create_subscriptions();
};

/**
 * @brief Class for initialize rclcpp::Node & register ros2 connections into node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
class RosConnectionBridge : public rclcpp::Node {
    private :
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosConnectionPublisher * ros_connection_publisher_ptr_;
        RosConnectionSubscription * ros_connection_subscription_ptr_;
    public :
        RosConnectionBridge();
        virtual ~RosConnectionBridge();
};

// function for check rclcpp status
void check_rclcpp_status();

#endif