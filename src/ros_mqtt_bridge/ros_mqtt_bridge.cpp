#include "ros_mqtt_bridge.hpp"

RosMqttConnectionManager::RosMqttConnectionManager(Mqtt * mqtt_ptr)
: mqtt_ptr_(mqtt_ptr) {
    
}

RosMqttConnectionManager::~RosMqttConnectionManager() {
    delete mqtt_ptr_;
}

void RosMqttConnectionManager::deliver_to_mqtt(char * mqtt_topic, const char * ros_callback_data) {
    std::cout << "[RosMqttConnectionManager] deliver_to_mqtt topic : " << mqtt_topic << '\n';
    mqtt_ptr_->mqtt_publish(mqtt_topic, ros_callback_data);
}

RosMqttBridge::RosMqttBridge(Mqtt * mqtt_ptr) 
: Node("ros_mqtt_subscription"),
mqtt_ptr_(mqtt_ptr) {
    node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    
    this->sort_create_subscription();
    ros_subscription_ptr_ = new RosMqttConnectionManager(mqtt_ptr_);
}

RosMqttBridge::~RosMqttBridge() {
    delete ros_subscription_ptr_;
}

void RosMqttBridge::sort_create_subscription() {
    auto topic_names_and_types = node_ptr_->get_topic_names_and_types();

    for(const auto& topic : topic_names_and_types) {
        std::cout << "[RosMqttBridge] sort_create topic : " << topic.first << ", type : ";
        std::copy(topic.second.begin(), topic.second.end(), std::ostream_iterator<std::string>(std::cout, ", "));
        std::cout << '\n';

        const std::vector<std::string>& topic_types = topic.second;
        for (const auto& topic_type : topic_types) {
            if (topic_type == "std_msgs/msg/String") {
                std::cout << "[RosMqttBridge] matches with " << topic_type << '\n';

                char * ros_topic = "/chatter";
                std_subscription_ = node_ptr_->create_subscription<std_msgs::msg::String>(
                    ros_topic,
                    rclcpp::QoS(rclcpp::KeepLast(10)),
                    [this](const std_msgs::msg::String::SharedPtr msg) {
                            auto callback_data = msg->data.c_str();
                            RCLCPP_INFO(node_ptr_->get_logger(), "I heard: '%s'", callback_data);
                            this->ros_subscription_ptr_->deliver_to_mqtt("/chatter", callback_data);
                    }
                );
                std::cout << "[RosMqttBridge] create subscription with topic '" << ros_topic << "'" << '\n';
            } else {
                mqtt_ptr_->mqtt_publish("/no", "no matches");
            }
        }
    }
}