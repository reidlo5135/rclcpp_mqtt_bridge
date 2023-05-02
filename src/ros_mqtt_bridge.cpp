#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

RosMqttBridge::RosMqttBridge(Mqtt * mqtt_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_log_(LOG_MQTT),
mqtt_ptr_(mqtt_ptr) {
    auto node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    RosSubscription<std_msgs::msg::String> test_subscription(node, "/chatter", 10);
    test_subscription.subscribe();
}

RosMqttBridge::~RosMqttBridge() {

}

template<typename message_type>
RosSubscription<message_type>::RosSubscription(rclcpp::Node::SharedPtr node, const std::string& topic_name, size_t queue_size)
: mqtt_log_(LOG_MQTT),
node_(node) {

}

template<typename message_type>
RosSubscription<message_type>::~RosSubscription() {

}

template<typename message_type>
void RosSubscription<message_type>::subscribe() {
    auto callback = std::bind(&RosSubscription::on_message, this, _1);
    this->subscription_ = node_->create_subscription<message_type>(topic_name_, rclcpp::QoS(rclcpp::KeepLast(queue_size_)), callback);
}

template<typename message_type>
void RosSubscription<message_type>::on_message(const typename message_type::SharedPtr message) {
    std::cout << "on message " << message << "\n";
}