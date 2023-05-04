#include "subscription/ros_subscription.hpp"

RosSubscription::RosSubscription(MqttMgr * mqtt_mgr_ptr, rclcpp::Node::SharedPtr rcl_node_ptr)
: log_ros_subscription_(LOG_ROS_SUBSCRITPION),
mqtt_mgr_ptr_(mqtt_mgr_ptr),
ros_node_ptr_(rcl_node_ptr) {   
    this->sort_and_create_subscriptions();
}

RosSubscription::~RosSubscription() {

}

void RosSubscription::sort_and_create_subscriptions() {
    auto topic_names_and_types = ros_node_ptr_->get_topic_names_and_types();

    for(const auto& topic : topic_names_and_types) {
        std::cout << log_ros_subscription_ << " create_subscriptions topic : " << topic.first << ", type : ";
        std::copy(topic.second.begin(), topic.second.end(), std::ostream_iterator<std::string>(std::cout, ", "));
        std::cout << '\n';

        const std::vector<std::string>& ros_topic_types = topic.second;
        for (const auto& ros_topic_type : ros_topic_types) {
            if (ros_topic_type == "std_msgs/msg/String") {                
                char * ros_topic = "/mqtt/chatter";
                // log_matches_ros_topics_and_types(ros_topic, ros_topic_type);
                ros_std_subscription_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
                    ros_topic,
                    rclcpp::QoS(rclcpp::KeepLast(10)),
                    [this](const std_msgs::msg::String::SharedPtr msg) {
                            const char* callback_data = const_cast<char*>(msg->data.c_str());
                            RCLCPP_INFO(ros_node_ptr_->get_logger(), "I heard: '%s'", callback_data);
                            this->mqtt_mgr_ptr_->mqtt_publish("/chatter", callback_data);
                    }
                );
                std::cout << log_ros_subscription_ << " create subscription with topic '" << ros_topic << "'" << '\n';
            } else if(ros_topic_type == "nav_msgs/msg/Odometry") {
                char * ros_topic = "/mqtt/odom";
                // log_matches_ros_topics_and_types(ros_topic, ros_topic_type);
                ros_odom_subscription_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
                    ros_topic,
                    rclcpp::QoS(rclcpp::KeepLast(20)),
                    [this](const nav_msgs::msg::Odometry::SharedPtr odom_message) {
                        auto callback_data = odom_message->pose.pose.position.x;
                        RCLCPP_INFO(ros_node_ptr_->get_logger(), "odom callback : '%s'", callback_data);
                        this->mqtt_mgr_ptr_->mqtt_publish("/odom", std::to_string(callback_data));
                    }
                );
                std::cout << log_ros_subscription_ << " create subscription with topic '" << ros_topic << "'" << '\n';
            } else {
                mqtt_mgr_ptr_->mqtt_publish("/no", "no matches");
            }
        }
    }
}

void RosSubscription::log_matches_ros_topics_and_types(char * ros_topic, const char * ros_topic_type) const {
    std::cout << log_ros_subscription_ << " matches with topic [" << ros_topic <<"] types [" << ros_topic_type << "]" << '\n';
}