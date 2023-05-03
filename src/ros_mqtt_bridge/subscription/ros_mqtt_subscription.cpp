#include "subscription/ros_mqtt_subscription.hpp"

RclMqttSubscription::RclMqttSubscription(MqttMgr * mqtt_mgr_ptr, rclcpp::Node::SharedPtr rcl_node_ptr) 
: mqtt_mgr_ptr_(mqtt_mgr_ptr),
rcl_node_ptr_(rcl_node_ptr) {   
    this->sort_create_subscription();
}

RclMqttSubscription::~RclMqttSubscription() {

}

void RclMqttSubscription::sort_create_subscription() {
    auto topic_names_and_types = rcl_node_ptr_->get_topic_names_and_types();

    for(const auto& topic : topic_names_and_types) {
        std::cout << "[RosMqttSubscription] sort_create topic : " << topic.first << ", type : ";
        std::copy(topic.second.begin(), topic.second.end(), std::ostream_iterator<std::string>(std::cout, ", "));
        std::cout << '\n';

        const std::vector<std::string>& topic_types = topic.second;
        for (const auto& topic_type : topic_types) {
            if (topic_type == "std_msgs/msg/String") {
                std::cout << "[RosMqttSubscription] matches with " << topic_type << '\n';

                char * ros_topic = "/chatter";
                rcl_std_subscription_ = rcl_node_ptr_->create_subscription<std_msgs::msg::String>(
                    ros_topic,
                    rclcpp::QoS(rclcpp::KeepLast(10)),
                    [this](const std_msgs::msg::String::SharedPtr msg) {
                            const char* callback_data = const_cast<char*>(msg->data.c_str());
                            RCLCPP_INFO(rcl_node_ptr_->get_logger(), "I heard: '%s'", callback_data);
                            this->mqtt_mgr_ptr_->mqtt_publish("/chatter", callback_data);
                    }
                );
                std::cout << "[RosMqttSubscription] create subscription with topic '" << ros_topic << "'" << '\n';
            } else {
                mqtt_mgr_ptr_->mqtt_publish("/no", "no matches");
            }
        }
    }
}