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
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "mqtt/async_client.h"

/**
 * include rclcpp header files
 * @see rclcpp/rclcpp.hpp
 * @see rclcpp_action/rclcpp_action.hpp
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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
#include "ros_mqtt_bridge/connections/ros_mqtt_message_converter.hpp"

/**
 * include tf2_msgs' header files
 * @see tf2_msgs::msg::TFMessage
*/
#include "tf2_msgs/msg/tf_message.hpp"

#define LOG_ROS_MQTT_BRIDGE "[ROS-MQTT-BRIDGE]"
#define LOG_ROS_MQTT_CONNECTION_TO_ROS "[MQTT to ROS]"
#define LOG_ROS_MQTT_CONNECTION_TO_MQTT "[ROS to MQTT]"
#define ROS_DEFAULT_QOS 10
#define MQTT_ADDRESS    "tcp://localhost:1883"
#define MQTT_CLIENT_ID    "ros_mqtt_bridge"
#define MQTT_QOS         0
#define MQTT_N_RETRY_ATTEMPTS 5

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief namespace for declare ros - mqtt connections
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
 * @see rclcpp::Publisher
 * @see rclcpp::Subscription
*/
namespace ros_mqtt_connections {
    namespace manager {
        class Bridge : public virtual mqtt::callback {
            private :
                const std::string& log_ros_mqtt_bridge_;
                const std::string& log_ros_mqtt_connections_to_mqtt_;
                const std::string& log_ros_mqtt_connections_to_ros_;
                std::shared_ptr<rclcpp::Node> ros_node_ptr_;
                const int ros_default_qos_;
                mqtt::async_client mqtt_async_client_;
                ros_message_converter::ros_std_msgs::StdMessageConverter * std_msgs_converter_ptr_;
                ros_message_converter::ros_geometry_msgs::GeometryMessageConverter * geometry_msgs_converter_ptr_;
                ros_message_converter::ros_sensor_msgs::SensorMessageConverter * sensor_msgs_converter_ptr_;
                ros_message_converter::ros_nav_msgs::NavMessageConverter * nav_msgs_converter_ptr_;
                ros_message_converter::ros_tf2_msgs::Tf2MessageConverter * tf2_msgs_converter_ptr_;
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_chatter_publisher_ptr_;
                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_publisher_ptr_;
                rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ros_initial_pose_publisher_ptr_;
                rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_chatter_subscription_ptr_;
                rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ros_robot_pose_subscription_ptr_;
                rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ros_scan_subscription_ptr_;
                rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ros_tf_subscription_ptr_;
                rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ros_tf_static_subscription_ptr_;
                rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_ptr_;
                rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ros_global_plan_subscription_ptr_;
                rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ros_local_plan_subscription_ptr_;
                rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_subscription_ptr_;
                const int mqtt_qos_;
                const int mqtt_is_success_;
                void mqtt_connect();
                void grant_mqtt_subscriptions();
                void connection_lost(const std::string& mqtt_connection_lost_cause) override;
                void message_arrived(mqtt::const_message_ptr mqtt_message) override;
                void delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) override;
                void mqtt_publish(const char * mqtt_topic, std::string mqtt_payload);
                void mqtt_subscribe(const char * topic);
                void bridge_ros_to_mqtt();
                void bridge_mqtt_to_ros();
                void bridge_mqtt_to_ros(std::string& mqtt_topic, std::string& mqtt_payload);
            public :
                Bridge(std::shared_ptr<rclcpp::Node> ros_node_ptr);
                virtual ~Bridge();
        };
    }
}

/**
 * @brief namespace for declare ros topics
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
namespace ros_topics {
    namespace to_ros {
        const char * chatter = "mqtt_bridge/chatter";
        const char * cmd_vel = "mqtt_bridge/cmd_vel";
        const char * initial_pose = "mqtt_bridge/initial_pose";
        const char * navigate_to_pose = "mqtt_bridge/navigate_to_pose";
        const char * map_server_map = "mqtt_bridge/map_server/map";
    }
    namespace from_ros {
        const char * chatter = "/chatter";
        const char * cmd_vel = "/cmd_vel";
        const char * robot_pose = "connection_bridge/robot_pose";
        const char * scan = "connection_bridge/scan";
        const char * tf = "connection_bridge/tf";
        const char * tf_static = "connection_bridge/tf_static";
        const char * odom = "connection_bridge/odom";
        const char * global_plan = "connection_bridge/global_plan";
        const char * local_plan = "conenction_bridge/local_plan";
    }
}

/**
 * @brief namespace for declare mqtt topics
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.11
*/
namespace mqtt_topics {
    namespace to_rcs {
        const char * chatter = "/callback/chatter";
        const char * robot_pose = "/robot_pose";
        const char * scan = "/scan";
        const char * tf = "/tf";
        const char * tf_static = "/tf_static";
        const char * odom = "/odom";
        const char * global_plan = "/global_plan";
        const char * local_plan = "/local_plan";
        const char * cmd_vel = "/callback/cmd_vel";
        const char * navigate_to_pose = "/navigate_to_pose/response";
        const char * map_server_map = "/map_server/map/response";
    }
    namespace from_rcs {
        const char * chatter = "/chatter";
        const char * cmd_vel = "/cmd_vel";
        const char * initial_pose = "/initialpose";
        const char * navigate_to_pose = "/navigate_to_pose/request";
        const char * map_server_map = "/map_server/map/request";
    }
}

#endif