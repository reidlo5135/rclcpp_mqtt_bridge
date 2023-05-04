#include "ros_connection_bridge.hpp"

RosConnectionBridge::RosConnectionBridge(MqttMgr * mqtt_ptr)
: Node("ros_mqtt_bridge_publisher"),
mqtt_ptr_(mqtt_ptr),
count_(0) {
    ros_std_publisher_ = this->create_publisher<std_msgs::msg::String>("/mqtt/chatter", 10);
    ros_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/mqtt/odom", 10);
    ros_timer_ = this->create_wall_timer(500ms, std::bind(&RosConnectionBridge::ros_timer_callback, this));

    ros_std_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter",
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr std_message) {
            auto callback_data = std_message->data;
            RCLCPP_INFO(this->get_logger(), "chatter callback : '%s'", callback_data);
            auto message = std_msgs::msg::String();
            message.data = callback_data;
            this->ros_std_publisher_->publish(message);
        }
    );
    // ros_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/odom",
    //     rclcpp::QoS(rclcpp::KeepLast(20)),
    //     [this](const nav_msgs::msg::Odometry::SharedPtr odom_message) {
    //         auto callback_data = odom_message->pose.pose.position.x;
    //         RCLCPP_INFO(this->get_logger(), "odom callback : '%s'", callback_data);
    //         auto message = nav_msgs::msg::Odometry();
    //         message.pose.pose.position.x = callback_data;
    //         this->ros_odom_publisher_->publish(message);
    //     }
    // );
}

RosConnectionBridge::~RosConnectionBridge() {

}

void RosConnectionBridge::ros_timer_callback() {
    // mqtt_ptr_->mqtt_subscribe("/publish/chatter");
    auto message = std_msgs::msg::String();
    message.data = "rclcpp spin " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "publishing with '%s'", message.data.c_str());
    ros_std_publisher_->publish(message);
}

int main(int argc, char** argv) {
    MqttMgr * mqtt_ptr = new MqttMgr(MQTT_ADDRESS, MQTT_CLIENT_ID);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosConnectionBridge>(mqtt_ptr));
    rclcpp::shutdown();

    delete mqtt_ptr;
    return 0;
}