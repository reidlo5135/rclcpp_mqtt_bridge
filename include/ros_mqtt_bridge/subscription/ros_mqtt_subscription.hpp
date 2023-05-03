#ifndef ROS_MQTT_SUBSCRIPTION
#define ROS_MQTT_SUBSCRIPTION

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

class RosMqttConnectionManager {
    private :
        Mqtt * mqtt_ptr_;
    public :
        RosMqttConnectionManager(Mqtt * mqtt_ptr);
        virtual ~RosMqttConnectionManager();
        void deliver_to_mqtt(char * mqtt_topic, const char * ros_callback_data);
};

class RosMqttSubscription : public rclcpp::Node {
    private :
        Mqtt * mqtt_ptr_;
        RosMqttConnectionManager * ros_subscription_ptr_;
        std::shared_ptr<rclcpp::Node> node_ptr_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr std_subscription_;
    public :
        RosMqttSubscription(Mqtt * mqtt_ptr);
        virtual ~RosMqttSubscription();
        void sort_create_subscription();
};

#endif