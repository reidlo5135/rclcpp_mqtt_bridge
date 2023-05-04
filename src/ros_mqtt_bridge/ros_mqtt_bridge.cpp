#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

RosMqttBridge::RosMqttBridge(MqttMgr * mqtt_mgr_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_mgr_ptr_(mqtt_mgr_ptr) {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_subscription_ptr_ = new RosMqttSubscription(mqtt_mgr_ptr_, ros_node_ptr_);
    ros_subscription_ptr_->create_ros_mqtt_bridge();
}

RosMqttBridge::~RosMqttBridge() {
    delete ros_subscription_ptr_;
}

int main(int argc, char** argv) {
    MqttMgr * mqtt_ptr = new MqttMgr(MQTT_ADDRESS, MQTT_CLIENT_ID);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosMqttBridge>(mqtt_ptr);
    while(rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    
    delete mqtt_ptr;
    return 0;
}