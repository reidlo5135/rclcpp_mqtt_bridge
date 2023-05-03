#include "ros_mqtt_bridge.hpp"

RclMqttBridge::RclMqttBridge(MqttMgr * mqtt_mgr_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_mgr_ptr_(mqtt_mgr_ptr) {
    rcl_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    rcl_mqtt_subscription_ptr_ = new RclMqttSubscription(mqtt_mgr_ptr_, rcl_node_ptr_);
    rcl_mqtt_subscription_ptr_->sort_create_subscription();
}

RclMqttBridge::~RclMqttBridge() {
    delete rcl_mqtt_subscription_ptr_;
}