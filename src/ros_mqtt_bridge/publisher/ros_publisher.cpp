#include "publisher/ros_publisher.hpp"

RosPublisher::RosPublisher(MqttMgr * mqtt_ptr)
: Node("ros_mqtt_bridge_publisher"),
mqtt_ptr_(mqtt_ptr),
count_(0) {
    ros_std_publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
    ros_timer_ = this->create_wall_timer(500ms, std::bind(&RosPublisher::rcl_timer_callback, this));
}

RosPublisher::~RosPublisher() {

}

void RosPublisher::rcl_timer_callback() {
    mqtt_ptr_->mqtt_subscribe("/publish/chatter");
    auto message = std_msgs::msg::String();
    message.data = "rclcpp spin " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "publishing with '%s'", message.data.c_str());
    ros_std_publisher_->publish(message);
}

int main(int argc, char** argv) {
    MqttMgr * mqtt_ptr = new MqttMgr(MQTT_ADDRESS, MQTT_CLIENT_ID);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosPublisher>(mqtt_ptr));
    rclcpp::shutdown();

    delete mqtt_ptr;
    return 0;
}