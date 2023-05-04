#include "ros_connection_bridge/ros_connection_bridge.hpp"

RosConnectionPublisher::RosConnectionPublisher(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: ros_node_ptr_(ros_node_ptr) {
    this->create_publishers();
}

RosConnectionPublisher::~RosConnectionPublisher() {

}

void RosConnectionPublisher::create_publishers() {
    ros_connections::publisher::ros_std_publisher_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(ros_topics::publisher::chatter, 10);
    ros_connections::publisher::ros_odom_publisher_ = ros_node_ptr_->create_publisher<nav_msgs::msg::Odometry>(ros_topics::publisher::odometry, 10);
}

RosConnectionSubscription::RosConnectionSubscription(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: ros_node_ptr_(ros_node_ptr) {
    this->create_connection_bridge();
}

RosConnectionSubscription::~RosConnectionSubscription() {

}

void RosConnectionSubscription::create_connection_bridge() {
    ros_connections::subscription::ros_std_subscription_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        ros_topics::subscription::chatter,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr std_message) {
            auto callback_data = std_message->data.c_str();
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "chatter callback : '%s'", callback_data);
            auto message = std_msgs::msg::String();
            message.data = callback_data;
            ros_connections::publisher::ros_std_publisher_->publish(message);
        }
    );
}

RosConnectionBridge::RosConnectionBridge()
: Node("ros_connection_bridge") {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_connection_publisher_ptr_ = new RosConnectionPublisher(ros_node_ptr_);
    ros_connection_subscription_ptr_ = new RosConnectionSubscription(ros_node_ptr_);
}

RosConnectionBridge::~RosConnectionBridge() {
    delete ros_connection_publisher_ptr_;
    delete ros_connection_subscription_ptr_;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosConnectionBridge>());
    rclcpp::shutdown();

    return 0;
}