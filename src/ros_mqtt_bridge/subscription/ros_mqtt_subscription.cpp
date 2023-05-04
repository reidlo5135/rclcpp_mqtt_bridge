#include "ros_mqtt_bridge/subscription/ros_mqtt_subscription.hpp"

RosMqttSubscription::RosMqttSubscription(MqttMgr * mqtt_mgr_ptr, rclcpp::Node::SharedPtr rcl_node_ptr)
: log_ros_subscription_(LOG_ROS_SUBSCRITPION),
mqtt_mgr_ptr_(mqtt_mgr_ptr),
ros_node_ptr_(rcl_node_ptr) {   
    // this->create_ros_mqtt_bridge();
}

RosMqttSubscription::~RosMqttSubscription() {

}

void RosMqttSubscription::create_ros_mqtt_bridge() {
    char * ros_chatter_topic = "ros_connection_bridge/chatter";
    ros_std_subscription_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
            ros_chatter_topic,
            rclcpp::QoS(rclcpp::KeepLast(10)),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                const char* callback_data = const_cast<char*>(msg->data.c_str());
                RCLCPP_INFO(ros_node_ptr_->get_logger(), "I heard: '%s'", callback_data);
                this->mqtt_mgr_ptr_->mqtt_publish("/chatter", callback_data);
            }
        );

    char * ros_odom_topic = "ros_connetion_bridge/odom";
    ros_odom_subscription_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_odom_topic,
        rclcpp::QoS(rclcpp::KeepLast(20)),
        [this](const nav_msgs::msg::Odometry::SharedPtr odom_message) {
            auto callback_data = odom_message->pose.pose.position.x;
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "odom callback : '%s'", callback_data);
            this->mqtt_mgr_ptr_->mqtt_publish("/odom", std::to_string(callback_data));
        }
    );
}

void RosMqttSubscription::log_matches_ros_topics_and_types(char * ros_topic, const char * ros_topic_type) const {
    std::cout << log_ros_subscription_ << " matches with topic [" << ros_topic <<"] types [" << ros_topic_type << "]" << '\n';
}