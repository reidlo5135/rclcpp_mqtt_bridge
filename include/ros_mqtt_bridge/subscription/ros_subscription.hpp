#ifndef ROS_SUBSCRIPTION
#define ROS_SUBSCRIPTION

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

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosSubscription {
    private :
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_std_subscription_;
    public :
        RosSubscription(MqttMgr * mqtt_ptr, rclcpp::Node::SharedPtr ros_node_ptr_);
        virtual ~RosSubscription();
        void sort_create_subscription();
};

#endif