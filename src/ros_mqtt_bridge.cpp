#include "mqtt/mqtt.hpp"
#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

MinimalPublisher::MinimalPublisher()
: Node("minimal_publisher"),
m_count(0) {
        m_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
        m_timer = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        std::thread mqtt_thread_run(&Mqtt::mqtt_subscribe, m_mqtt_cli);
        mqtt_thread_run.detach();
}

MinimalPublisher::~MinimalPublisher() {

}

void MinimalPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello World! [" + std::to_string(m_count++) + "]";
    RCLCPP_INFO(this->get_logger(), "%s Publishing : %s", LOG_RCL, message.data.c_str());
    m_publisher->publish(message);
    // m_mqtt_cli->mqtt_publish("/chatter", std::to_string(message.data));
}