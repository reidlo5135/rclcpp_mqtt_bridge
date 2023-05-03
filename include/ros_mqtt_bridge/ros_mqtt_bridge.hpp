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
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_base.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosSubscription {
    private :
        Mqtt * mqtt_ptr_;
    public :
        RosSubscription(Mqtt * mqtt_ptr);
        virtual ~RosSubscription();
        void deliver_to_mqtt(char * mqtt_topic, const char * data);
};

class RosMqttBridge : public rclcpp::Node {
    private :
        Mqtt * mqtt_ptr_;
        RosSubscription * ros_subscription_ptr_;
        std::shared_ptr<rclcpp::Node> node_ptr_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr std_subscription_;
        std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
    public :
        RosMqttBridge(Mqtt * mqtt_ptr);
        virtual ~RosMqttBridge();
        void deliver_to_mqtt(char * mqtt_topic, const char * data);
        void sort_create_subscription();
};

#endif