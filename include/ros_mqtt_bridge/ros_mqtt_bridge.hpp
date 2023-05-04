#ifndef ROS_MQTT_BRIDGE
#define ROS_MQTT_BRIDGE

#include <iostream>
#include "ros_mqtt_bridge/subscription/ros_mqtt_subscription.hpp"

class RosMqttBridge : public rclcpp::Node {
    private :
        MqttMgr * mqtt_mgr_ptr_;
        std::shared_ptr<rclcpp::Node> ros_node_ptr_;
        RosMqttSubscription * ros_subscription_ptr_;
    public :
        RosMqttBridge(MqttMgr * mqtt_ptr);
        virtual ~RosMqttBridge();
};

#endif