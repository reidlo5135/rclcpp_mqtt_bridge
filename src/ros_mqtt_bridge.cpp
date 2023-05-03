#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

RosMqttBridge::RosMqttBridge(Mqtt * mqtt_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_ptr_(mqtt_ptr) {
    std::cout << "ros mqtt bridge constructor invoked" << "\n";
    node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    
    ros_subscription_ptr_ = new RosSubscription(mqtt_ptr_);
    this->sort_create_subscription();
}

RosMqttBridge::~RosMqttBridge() {
    delete ros_subscription_ptr_;
}

void RosMqttBridge::deliver_to_mqtt(char * mqtt_topic, const char * data) {
    mqtt_ptr_->mqtt_publish(mqtt_topic, data);
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
                            const char * data = msg->data.c_str();
                            RCLCPP_INFO(node_ptr_->get_logger(), "I heard: '%s'", data);
                            this->deliver_to_mqtt("/chatter", data);
                            // this->ros_subscription_ptr_->deliver_to_mqtt("/chatter", data);
                    }
                );
                subscriptions_.push_back(std_subscription_);
            } else {
                mqtt_ptr_->mqtt_publish("/no", "no matches");
            }
        }
    }
}

RosSubscription::RosSubscription(Mqtt * mqtt_ptr)
: mqtt_ptr_(mqtt_ptr) {
    
}

RosSubscription::~RosSubscription() {
    delete mqtt_ptr_;
}

void RosSubscription::deliver_to_mqtt(char * mqtt_topic, const char * data) {
    std::cout << "[RosSubscription] topic : " << mqtt_topic << '\n';
    mqtt_ptr_->mqtt_publish(mqtt_topic, data);
}