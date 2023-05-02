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
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        Mqtt * mqtt_ptr_;
    public :
        MinimalPublisher(Mqtt * mqtt_ptr);
        virtual ~MinimalPublisher();
        void timer_callback();
};

#endif