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

#ifndef ROS_MQTT_CONNECTIONS
#define ROS_MQTT_CONNECTIONS

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

/**
 * include std_msgs::msg::String header file
 * @see std_msgs::msg::String
*/
#include "std_msgs/msg/string.hpp"

/**
 * include geometry_msgs' header files
 * @see geometry_msgs::msg::Twsit
 * @see geometry_msgs::msg::PoseWithCovarianceStamped
*/
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

/**
 * include sensor_msgs' header files
 * @see sensor_msgs::msg::LaserScan
*/
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * include nav_msgs' header files
 * @see nav_msgs::msg::path
 * @see nav_msgs::srv::GetMap
 * @see nav_msgs::msg::Odometry
 * @see nav2_msgs::action::Navigate_To_Pose
*/
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

/**
 * include tf2_msgs' header files
 * @see tf2_msgs::msg::TFMessage
*/
#include "tf2_msgs/msg/tf_message.hpp"

/**
 * @brief namespace for declare ros connections
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @see rclcpp::Publisher
 * @see rclcpp::Subscription
*/
namespace ros_mqtt_connections {
    namespace publisher {
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_chatter_publisher_ptr_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_publisher_ptr_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ros_initial_pose_publisher_ptr_;
    }
    namespace subscription {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_chatter_subscription_ptr_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ros_robot_pose_subscription_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ros_scan_subscription_ptr_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ros_tf_subscription_ptr_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ros_tf_static_subscription_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ros_global_plan_subscription_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ros_local_plan_subscription_ptr_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_subscription_ptr_;
    }
}

/**
 * @brief namespace for declare ros topics
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
namespace ros_topics {
    namespace to_connection {
        const char * chatter = "mqtt_bridge/chatter";
        const char * cmd_vel = "mqtt_bridge/cmd_vel";
        const char * initial_pose = "mqtt_bridge/initial_pose";
    }
    namespace from_connection {
        const char * chatter = "connection_bridge/chatter";
        const char * robot_pose = "connection_bridge/robot_pose";
        const char * scan = "connection_bridge/scan";
        const char * tf = "connection_bridge/tf";
        const char * tf_static = "connection_bridge/tf_static";
        const char * odom = "connection_bridge/odom";
        const char * global_plan = "connection_bridge/global_plan";
        const char * local_plan = "conenction_bridge/local_plan";
        const char * cmd_vel = "/cmd_vel";
    }
}

/**
 * @brief namespace for declare mqtt topics
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
namespace mqtt_topics {
    namespace publisher {
        const char * chatter = "/chatter";
        const char * robot_pose = "/robot_pose";
        const char * scan = "/scan";
        const char * tf = "/tf";
        const char * tf_static = "/tf_static";
        const char * odom = "/odom";
        const char * global_plan = "/global_plan";
        const char * local_plan = "/local_plan";
        const char * cmd_vel = "/callback/cmd_vel";
    }
    namespace subscription {
        const char * chatter = "/chatter";
        const char * cmd_vel = "/cmd_vel";
        const char * initial_pose = "/initalpose";
    }
}

#endif