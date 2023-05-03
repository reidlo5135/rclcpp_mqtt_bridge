#include "publisher/ros_mqtt_publisher.hpp"

RosMqttPublisher::RosMqttPublisher(Mqtt * mqtt_ptr)
: Node("publisher_test"),
mqtt_ptr_(mqtt_ptr),
count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&RosMqttPublisher::timer_callback, this));
}

RosMqttPublisher::~RosMqttPublisher() {

}

void RosMqttPublisher::timer_callback() {
    mqtt_ptr_->mqtt_subscribe("/chatter");
    auto message = std_msgs::msg::String();
    message.data = "rclcpp spin " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "publishing with '%s'", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char** argv) {
    Mqtt * mqtt_ptr = new Mqtt(MQTT_ADDRESS, MQTT_CLIENT_ID);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosMqttPublisher>(mqtt_ptr));
    rclcpp::shutdown();

    delete mqtt_ptr;
    return 0;
}