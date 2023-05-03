#ifndef ROS_MQTT_PUBLIHSER
#define ROS_MQTT_PUBLIHSER

#include <iostream>
#include <math.h>
#include <typeinfo>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "mqtt/mqtt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosMqttPublisher : public rclcpp::Node {
    private :
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
    public :
        RosMqttPublisher();
        virtual ~RosMqttPublisher();
};

#endif