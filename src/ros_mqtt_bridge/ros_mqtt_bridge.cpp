#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

/**
 * @brief Constructor for initialize this class instance & MqttMgr class' pointer & ros_mqtt_bridge rclcpp::Node shared pointer
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param mqtt_mgr_ptr MqttMgr *
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see MqttMgr
 * @see rclcpp::Node
*/
RosMqttSubscription::RosMqttSubscription(MqttMgr * mqtt_mgr_ptr, std::shared_ptr<rclcpp::Node> ros_node_ptr)
: log_ros_subscription_(LOG_ROS_SUBSCRITPION),
mqtt_mgr_ptr_(mqtt_mgr_ptr),
ros_node_ptr_(ros_node_ptr) {
    
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
RosMqttSubscription::~RosMqttSubscription() {

}

/**
 * @brief Function for create & register ros2 subscriptions to ros_mqtt_bridge rclcpp::Node & mqtt publish with ros2 callback data
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @return void
 * @see ros_mqtt_connections
 * @see mqtt_topics
*/
void RosMqttSubscription::create_ros_mqtt_bridge() {
    ros_mqtt_connections::subscription::ros_std_subscription_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        mqtt_topics::publisher::chatter_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            const char* callback_data = const_cast<char*>(msg->data.c_str());
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "I heard: '%s'", callback_data);
            this->mqtt_mgr_ptr_->mqtt_publish("/chatter", callback_data);
        }
    );

    ros_mqtt_connections::subscription::ros_odom_subscription_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        mqtt_topics::publisher::odom_topic,
        rclcpp::QoS(rclcpp::KeepLast(20)),
        [this](const nav_msgs::msg::Odometry::SharedPtr odom_message) {
            auto callback_data = odom_message->pose.pose.position.x;
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "odom callback : '%s'", callback_data);
            this->mqtt_mgr_ptr_->mqtt_publish("/odom", std::to_string(callback_data));
        }
    );
}

/**
 * @brief Constructor for initialize this class instance & ros_mqtt_bridge rclcpp::Node shared pointer & invoke this create_ros_mqtt_bridge()
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param mqtt_mgr_ptr MqttMgr *
 * @see std::shared_ptr
 * @see rclcpp::Node
 * @see RosMqttSubscription
 * @see RosMqttSubscription#create_ros_mqtt_bridge()
*/
RosMqttBridge::RosMqttBridge(MqttMgr * mqtt_mgr_ptr) 
: Node("ros_mqtt_bridge"),
mqtt_mgr_ptr_(mqtt_mgr_ptr) {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_subscription_ptr_ = new RosMqttSubscription(mqtt_mgr_ptr_, ros_node_ptr_);
    ros_subscription_ptr_->create_ros_mqtt_bridge();
}

/**
 * @brief Virtual Destructor for this class & delete RosMqttSubscription class' pointer
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @see ros_mqtt_subscription_ptr_
*/
RosMqttBridge::~RosMqttBridge() {
    delete ros_subscription_ptr_;
}

/**
 * @brief Function for check rclcpp status & init logs
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @return void
 * @see rclcpp::ok()
*/
void check_rclcpp_status() {
    if(rclcpp::ok()) {
        std::cout << R"(
     _____   ____   _____ ___    __  __  ____ _______ _______   ____  _____  _____ _____   _____ ______ 
    |  __ \ / __ \ / ____|__ \  |  \/  |/ __ \__   __|__   __| |  _ \|  __ \|_   _|  __ \ / ____|  ____|
    | |__) | |  | | (___    ) | | \  / | |  | | | |     | |    | |_) | |__) | | | | |  | | |  __| |__   
    |  _  /| |  | |\___ \  / /  | |\/| | |  | | | |     | |    |  _ <|  _  /  | | | |  | | | |_ |  __|  
    | | \ \| |__| |____) |/ /_  | |  | | |__| | | |     | |    | |_) | | \ \ _| |_| |__| | |__| | |____ 
    |_|  \_\\____/|_____/|____| |_|  |_|\___\_\ |_|     |_|    |____/|_|  \_\_____|_____/ \_____|______|                                                                                                     
                                                                                                     
        )" << '\n';
    } else {
        std::cerr << "[ros_mqtt_bridge] rclcpp is not ok" << '\n';
    }
}

/**
 * @brief Function for initialize this class instance & MqttMgr class & initialize rclcpp & spin ros_mqtt_bridge rclcpp::Node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param argc int
 * @param argv char**
 * @return int
 * @see MqttMgr
 * @see rclcpp
 * @see RosMqttBridge
 * @see check_rclcpp_status()
*/
int main(int argc, char** argv) {
    MqttMgr * mqtt_ptr = new MqttMgr(MQTT_ADDRESS, MQTT_CLIENT_ID);
    rclcpp::init(argc, argv);
    check_rclcpp_status();
    auto node = std::make_shared<RosMqttBridge>(mqtt_ptr);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    delete mqtt_ptr;
    return 0;
}