#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

RosMqttBridge::RosMqttBridge(Mqtt * mqtt_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_log_(LOG_MQTT),
mqtt_ptr_(mqtt_ptr) {
    std::cout << "ros mqtt bridge constructor invoked" << "\n";
    node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    const std::string topic = "/chatter";
    const int queue_size = 10;

    auto topic_names_and_types = node_ptr_->get_topic_names_and_types();

    for(const auto& topic : topic_names_and_types) {
        std::cout << "ros mqtt bridge topic : " << topic.first << ", type : " << topic.second << "\n";
    }
}

RosMqttBridge::~RosMqttBridge() {

}