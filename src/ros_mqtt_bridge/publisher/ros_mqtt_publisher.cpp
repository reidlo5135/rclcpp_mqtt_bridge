#include "publisher/ros_mqtt_publisher.hpp"

RclMqttPublisher::RclMqttPublisher(MqttMgr * mqtt_ptr)
: Node("ros_mqtt_bridge/publisher"),
mqtt_ptr_(mqtt_ptr),
count_(0) {
    rcl_std_publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
    rcl_timer_ = this->create_wall_timer(500ms, std::bind(&RclMqttPublisher::rcl_timer_callback, this));
}

RclMqttPublisher::~RclMqttPublisher() {

}

void RclMqttPublisher::rcl_timer_callback() {
    mqtt_ptr_->mqtt_subscribe("/chatter");
    auto message = std_msgs::msg::String();
    message.data = "rclcpp spin " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "publishing with '%s'", message.data.c_str());
    rcl_std_publisher_->publish(message);
}

int main(int argc, char** argv) {
    MqttMgr * mqtt_ptr = new MqttMgr(MQTT_ADDRESS, MQTT_CLIENT_ID);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RclMqttPublisher>(mqtt_ptr));
    rclcpp::shutdown();

    delete mqtt_ptr;
    return 0;
}