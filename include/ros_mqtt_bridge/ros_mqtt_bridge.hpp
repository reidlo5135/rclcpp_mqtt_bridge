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
#include "ros_mqtt_bridge/connections/ros_mqtt_connections.hpp"

#define LOG_ROS_SUBSCRITPION "[RosSubscription]"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RosMqttSubscription {
    private :
        const std::string log_ros_subscription_;
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
    public :
        RosMqttSubscription(MqttMgr * mqtt_ptr, rclcpp::Node::SharedPtr ros_node_ptr_);
        virtual ~RosMqttSubscription();
        void create_ros_mqtt_bridge();
};

class RosMqttBridge : public rclcpp::Node {
    private :
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosMqttSubscription * ros_subscription_ptr_;
    public :
        RosMqttBridge(MqttMgr * mqtt_ptr);
        virtual ~RosMqttBridge();
};

void check_rclcpp();

#endif