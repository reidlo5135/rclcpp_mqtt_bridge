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
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosMqttBridge : public rclcpp::Node {
    private :
        const std::string mqtt_log_;
        Mqtt * mqtt_ptr_;
    public :
        RosMqttBridge(Mqtt * mqtt_ptr);
        virtual ~RosMqttBridge();
};

template<typename message_type>
class RosSubscription {
    private :
        const std::string mqtt_log_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription::SharedPtr subscription_;
    public :
        RosSubscription(rclcpp::Node::SharedPtr node, const std::string& topic_name, size_t queue_size);
        virtual ~RosSubscription();
        void subscribe();
        void on_message(const typename message_type::SharedPtr message);
};


#endif