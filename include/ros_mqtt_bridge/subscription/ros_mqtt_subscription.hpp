#ifndef ROS_MQTT_SUBSCRIPTION
#define ROS_MQTT_SUBSCRIPTION

#include <iostream>
#include <math.h>
#include <typeinfo>
#include <unistd.h>
#include <signal.h>
#include <cstring>
#include <functional>

#include "mqtt/mqtt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define LOG_ROS_SUBSCRITPION "[RosSubscription]"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosMqttSubscription {
    private :
        const std::string log_ros_subscription_;
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_std_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_;
    public :
        RosMqttSubscription(MqttMgr * mqtt_ptr, rclcpp::Node::SharedPtr ros_node_ptr_);
        virtual ~RosMqttSubscription();
        void create_ros_mqtt_bridge();
        void log_matches_ros_topics_and_types(char * ros_topic, const char * ros_topic_type) const;
};

#endif