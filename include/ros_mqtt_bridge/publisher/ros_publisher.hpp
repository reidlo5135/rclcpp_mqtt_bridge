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

class RosPublisher : public rclcpp::Node {
    private :
        MqttMgr * mqtt_ptr_;
        rclcpp::TimerBase::SharedPtr ros_timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_std_publisher_;
        size_t count_;
        void rcl_timer_callback();
    public :
        RosPublisher(MqttMgr * mqtt_ptr);
        virtual ~RosPublisher();
};

#endif