#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

RosMqttBridge::RosMqttBridge(Mqtt * mqtt_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_log_(LOG_MQTT),
mqtt_ptr_(mqtt_ptr) {
    std::cout << "ros mqtt bridge constructor" << " \n";
    auto node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    const std::string topic = "/chatter";
    const int queue_size = 10;
    subscription<std_msgs::msg::String>(node, topic, queue_size);
}

RosMqttBridge::~RosMqttBridge() {

}

template<typename message_type>
void RosMqttBridge::subscription(const std::shared_ptr<rclcpp::Node> node, const std::string& topic_name, size_t queue_size) {
    auto callback = std::bind(&RosMqttBridge::on_message<message_type>, this, _1);
    const char * message_type_name = typeid(message_type).name();
    RCLCPP_INFO(this->get_logger(), "subscription message type '%s'", message_type_name);
    std_subscription_ = node->create_subscription<message_type>(topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), callback);
}

template<typename message_type>
void RosMqttBridge::on_message(const typename message_type::SharedPtr message) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", message->data.c_str());
}