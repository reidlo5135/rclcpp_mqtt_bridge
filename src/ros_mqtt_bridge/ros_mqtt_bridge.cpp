#include "ros_mqtt_bridge.hpp"

RosMqttBridge::RosMqttBridge(MqttMgr * mqtt_mgr_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_mgr_ptr_(mqtt_mgr_ptr) {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_subscription_ptr_ = new RosSubscription(mqtt_mgr_ptr_, ros_node_ptr_);
    ros_subscription_ptr_->sort_create_subscription();
}

RosMqttBridge::~RosMqttBridge() {
    delete ros_subscription_ptr_;
}