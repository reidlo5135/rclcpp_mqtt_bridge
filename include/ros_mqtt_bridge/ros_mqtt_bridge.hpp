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
#include "subscription/ros_subscription.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosMqttBridge : public rclcpp::Node {
    private :
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosSubscription * ros_subscription_ptr_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_std_subscription_;
    public :
        RosMqttBridge(MqttMgr * mqtt_ptr);
        virtual ~RosMqttBridge();
};

#endif