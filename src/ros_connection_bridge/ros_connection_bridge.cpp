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
        [this](const std_msgs::msg::String::SharedPtr callback_std_msgs) {
            auto callback_data = callback_std_msgs->data.c_str();
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "std_msgs callback : '%s'", callback_data);

            auto std_message = std_msgs::msg::String();
            std_message.data = callback_data;
            ros_connections::publisher::ros_std_publisher_->publish(std_message);
        }
    );
    ros_connections::subscription::ros_odom_subscription_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_topics::subscription::odometry,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_msgs) {
            double position_x = callback_odom_msgs->pose.pose.position.x;
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "odom_msgs callback : '%e'", position_x);

            auto odom_message = nav_msgs::msg::Odometry();
            odom_message.pose.pose.position.x = position_x;
            ros_connections::publisher::ros_odom_publisher_->publish(odom_message);
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