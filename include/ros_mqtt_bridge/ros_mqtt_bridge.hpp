#ifndef RCL
#define RCL

#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mqtt/mqtt.hpp"

#define LOG_RCL "[RCL]"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
    public :
        MinimalPublisher() : Node("minimal_publisher"), count(0) {
            publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        };
        void timer_callback();
    
    private :
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        size_t count;
};

#endif