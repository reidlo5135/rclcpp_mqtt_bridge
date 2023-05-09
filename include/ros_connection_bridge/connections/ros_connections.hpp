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

#ifndef ROS_CONNECTIONS
#define ROS_CONNECTIONS

/**
 * include cpp header files
 * @see iostream
 * @see math.h
 * @see unistd.h
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
 * @see std_msgs/msgs/string.hpp
 * @see nav_msgs/msg/odometry.hpp
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

/**
 * @brief namespace for declare ros connections to mqtt
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @see rclcpp::Publisher
 * @see rclcpp::Subscription
*/
namespace ros_connections_to_mqtt {
    namespace publisher {
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_chatter_publisher_ptr_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_publisher_ptr_;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr ros_tf_publisher_ptr_;
    }
    namespace subscription {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_chatter_subscription_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_ptr_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ros_tf_subscription_ptr_;
    }
}

/**
 * @brief namespace for declare ros connections from mqtt
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @see rclcpp::Publisher
 * @see rclcpp::Subscription
*/
namespace ros_connections_from_mqtt {
    namespace publisher {
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_chatter_publisher_ptr_;
    }
    namespace subscription {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_chatter_subscription_ptr_;
    }
}

/**
 * @brief namespace for declare ros topics
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
namespace ros_topics {
    namespace to_mqtt {
        const char * origin_chatter = "/chatter";
        const char * bridge_chatter = "connection_bridge/chatter";
        const char * origin_odometry = "/odom";
        const char * bridge_odometry = "connection_bridge/odom";
        const char * origin_scan = "/scan";
        const char * bridge_scan = "connection_bridge/scan";
        const char * origin_tf = "/tf";
        const char * bridge_tf = "connection_bridge/tf";
    }
    namespace from_mqtt {
        const char * bridge_chatter = "mqtt_bridge/chatter";
        const char * origin_chatter = "/chatter";
    }
}

#endif