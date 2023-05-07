// Copyright [2023] [wavem-reidlo]
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros_mqtt_bridge/ros_mqtt_bridge.hpp"

/**
 * @brief Constructor for initialize this class instance & MqttMgr class' pointer & ros_mqtt_bridge rclcpp::Node shared pointer
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.07
 * @param mqtt_mgr_ptr MqttMgr *
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see MqttMgr
 * @see rclcpp::Node
*/
RosMqttPublisher::RosMqttPublisher(MqttMgr * mqtt_mgr_ptr, std::shared_ptr<rclcpp::Node> ros_node_ptr) 
: mqtt_mgr_ptr_(mqtt_mgr_ptr),
ros_node_ptr_(ros_node_ptr) {

}

RosMqttPublisher::~RosMqttPublisher() {

}

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
: mqtt_mgr_ptr_(mqtt_mgr_ptr),
ros_node_ptr_(ros_node_ptr) {
    std_msgs_converter_ptr_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
    nav_msgs_converter_ptr_ = new ros_message_converter::ros_nav_msgs::NavMessageConverter();
    this->create_ros_mqtt_bridge();
}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
*/
RosMqttSubscription::~RosMqttSubscription() {
    delete std_msgs_converter_ptr_;
    delete nav_msgs_converter_ptr_;
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
    ros_mqtt_connections::subscription::ros_std_subscription_ptr_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        ros_mqtt_connections::topic::chatter_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr callback_chatter_data) {
            std::string chatter_json_str = std_msgs_converter_ptr_->convert_chatter_to_json(callback_chatter_data);
            this->mqtt_mgr_ptr_->mqtt_publish(ros_mqtt_topics::publisher::chatter_topic, chatter_json_str);
        }
    );

    ros_mqtt_connections::subscription::ros_odom_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_mqtt_connections::topic::odom_topic,
        rclcpp::QoS(rclcpp::KeepLast(20)),
        [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_msgs) {
            std::string odom_json_str = nav_msgs_converter_ptr_->convert_odom_to_json(callback_odom_msgs);
            this->mqtt_mgr_ptr_->mqtt_publish(ros_mqtt_topics::publisher::odom_topic, odom_json_str);
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
log_ros_mqtt_bridge_(LOG_ROS_MQTT_BRIDGE),
mqtt_mgr_ptr_(mqtt_mgr_ptr) {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_subscription_ptr_ = new RosMqttSubscription(mqtt_mgr_ptr_, ros_node_ptr_);
    this->check_current_topics_and_types();
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

void RosMqttBridge::check_current_topics_and_types() {
    auto topic_names_and_types = ros_node_ptr_->get_topic_names_and_types();

    for (const auto& topic_name_and_type : topic_names_and_types) {
        const std::string& topic_name = topic_name_and_type.first;
        const std::vector<std::string>& message_types = topic_name_and_type.second;

        std::cout << log_ros_mqtt_bridge_ << " topic registered '" << topic_name << "' with type '";
        for (const auto& message_type : message_types) {
            std::cout << message_type << "'" << '\n';
        }
    }
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
    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    while(rclcpp::ok()) {
        ros_executor.spin();
    }
    
    delete mqtt_ptr;
    return 0;
}