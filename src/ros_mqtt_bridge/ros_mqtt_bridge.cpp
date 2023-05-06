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
        ros_mqtt_connections::topic::chatter_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr callback_chatter_data) {
            const char* callback_data = const_cast<char*>(callback_chatter_data->data.c_str());
            RCLCPP_INFO(ros_node_ptr_->get_logger(), "chatter I heard: '%s'", callback_data);
            this->mqtt_mgr_ptr_->mqtt_publish(ros_mqtt_topics::publisher::chatter_topic, callback_data);
        }
    );

    ros_mqtt_connections::subscription::ros_odom_subscription_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_mqtt_connections::topic::odom_topic,
        rclcpp::QoS(rclcpp::KeepLast(20)),
        [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_msgs) {
            Json::Value json_odom;
            json_odom["header"]["frame_id"] = callback_odom_msgs->header.frame_id;
            json_odom["header"]["seq"] = callback_odom_msgs->header.stamp.sec;
            json_odom["header"]["stamp"] = callback_odom_msgs->header.stamp.sec + callback_odom_msgs->header.stamp.nanosec * 1e-9;
            json_odom["child_frame_id"] = callback_odom_msgs->child_frame_id;
            json_odom["pose"]["pose"]["position"]["x"] = callback_odom_msgs->pose.pose.position.x;
            json_odom["pose"]["pose"]["position"]["y"] = callback_odom_msgs->pose.pose.position.y;
            json_odom["pose"]["pose"]["position"]["z"] = callback_odom_msgs->pose.pose.position.z;
            json_odom["pose"]["pose"]["orientation"]["x"] = callback_odom_msgs->pose.pose.orientation.x;
            json_odom["pose"]["pose"]["orientation"]["y"] = callback_odom_msgs->pose.pose.orientation.y;
            json_odom["pose"]["pose"]["orientation"]["z"] = callback_odom_msgs->pose.pose.orientation.z;
            json_odom["pose"]["pose"]["orientation"]["w"] = callback_odom_msgs->pose.pose.orientation.w;
            json_odom["pose"]["covariance"] = Json::arrayValue;
            for (const auto& cov : callback_odom_msgs->pose.covariance) {
                json_odom["pose"]["covariance"].append(cov);
            }

            json_odom["twist"]["twist"]["linear"]["x"] = callback_odom_msgs->twist.twist.linear.x;
            json_odom["twist"]["twist"]["linear"]["y"] = callback_odom_msgs->twist.twist.linear.y;
            json_odom["twist"]["twist"]["linear"]["z"] = callback_odom_msgs->twist.twist.linear.z;
            json_odom["twist"]["twist"]["angular"]["x"] = callback_odom_msgs->twist.twist.angular.x;
            json_odom["twist"]["twist"]["angular"]["y"] = callback_odom_msgs->twist.twist.angular.y;
            json_odom["twist"]["twist"]["angular"]["z"] = callback_odom_msgs->twist.twist.angular.z;
            json_odom["twist"]["covariance"] = Json::arrayValue;
            for (const auto& cov : callback_odom_msgs->twist.covariance) {
                json_odom["twist"]["covariance"].append(cov);
            }

            std::string json_str = Json::StyledWriter().write(json_odom);
            this->mqtt_mgr_ptr_->mqtt_publish(ros_mqtt_topics::publisher::odom_topic, json_str);
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