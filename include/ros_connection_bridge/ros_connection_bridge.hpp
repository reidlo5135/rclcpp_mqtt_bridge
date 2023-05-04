#ifndef ROS_PUBLIHSER
#define ROS_PUBLIHSER

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

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosConnectionBridge : public rclcpp::Node {
    private :
        MqttMgr * mqtt_ptr_;
        rclcpp::TimerBase::SharedPtr ros_timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_std_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_std_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ros_odom_subscription_;
        size_t count_;
        void ros_timer_callback();
    public :
        RosConnectionBridge(MqttMgr * mqtt_ptr);
        virtual ~RosConnectionBridge();
};

#endif