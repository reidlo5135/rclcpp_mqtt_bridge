#ifndef ROS_MQTT_BRIDGE
#define ROS_MQTT_BRIDGE

#include <iostream>
#include <math.h>
#include <typeinfo>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "mqtt/mqtt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "subscription/ros_mqtt_subscription.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RclMqttBridge : public rclcpp::Node {
    private :
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> rcl_node_ptr_;
        RclMqttSubscription * rcl_mqtt_subscription_ptr_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rcl_std_subscription_;
    public :
        RclMqttBridge(MqttMgr * mqtt_ptr);
        virtual ~RclMqttBridge();
};

#endif