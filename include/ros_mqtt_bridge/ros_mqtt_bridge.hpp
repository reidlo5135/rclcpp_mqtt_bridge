#ifndef ROS_MQTT_BRIDGE
#define ROS_MQTT_BRIDGE

#include <iostream>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "mqtt/mqtt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define LOG_RCL "[RCL]"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {    
    private :
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
        size_t m_count;
        Mqtt * mqtt_ptr;
    public :
        MinimalPublisher();
        virtual ~MinimalPublisher();
        void timer_callback();
};

#endif