#include "publisher/ros_mqtt_publisher.hpp"

RosMqttPublisher::RosMqttPublisher()
: Node("publisher_test"),
count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&RosMqttPublisher::timer_callback, this));
}

RosMqttPublisher::~RosMqttPublisher() {

}

void RosMqttPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "rclcpp spin " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "publishing with '%s'", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosMqttPublisher>());
    rclcpp::shutdown();
    return 0;
}