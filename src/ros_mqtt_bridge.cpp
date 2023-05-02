#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

MinimalPublisher::MinimalPublisher(Mqtt * mqtt_ptr)
: Node("minimal_publisher"),
count_(0),
mqtt_ptr_(mqtt_ptr) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        mqtt_ptr_->mqtt_subscribe(MQTT_INIT_TOPIC);
}

MinimalPublisher::~MinimalPublisher() {

}

void MinimalPublisher::timer_callback() {   
    auto message = std_msgs::msg::String();
    message.data = "Hello World! [" + std::to_string(count_++) + "]";
    RCLCPP_INFO(this->get_logger(), " Publishing : %s", message.data.c_str());
    publisher_->publish(message);

    if (mqtt_ptr_) {
        mqtt_ptr_->mqtt_publish("/chatter", message.data);
    } else {
        std::cerr << LOG_MQTT << " MQTT is not initialized!" << "\n";
    }
}