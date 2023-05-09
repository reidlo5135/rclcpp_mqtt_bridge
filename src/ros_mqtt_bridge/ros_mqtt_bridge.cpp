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
 * @brief Constructor for initialize thie class' istance & establish connection between ros - mqtt
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param ros_node_ptr std::shared_ptr<rclcpp::Node>
 * @see rclcpp
 * @see mqtt::callback
*/
RosMqttConnectionManager::RosMqttConnectionManager(std::shared_ptr<rclcpp::Node> ros_node_ptr)
: log_ros_mqtt_bridge_(LOG_ROS_MQTT_BRIDGE),
ros_node_ptr_(ros_node_ptr),
mqtt_async_client_(MQTT_ADDRESS, MQTT_CLIENT_ID),
mqtt_qos_(MQTT_QOS),
mqtt_is_success_(mqtt::SUCCESS) {
    this->mqtt_connect();
    this->grant_mqtt_subscriptions();
    this->create_ros_bridge();

    std_msgs_converter_ptr_ = new ros_message_converter::ros_std_msgs::StdMessageConverter();
    nav_msgs_converter_ptr_ = new ros_message_converter::ros_nav_msgs::NavMessageConverter();
    tf2_msgs_converter_ptr_ = new ros_message_converter::ros_tf2_msgs::Tf2MessageConverter();
};

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
RosMqttConnectionManager::~RosMqttConnectionManager() {
    delete std_msgs_converter_ptr_;
    delete nav_msgs_converter_ptr_;
}

/**
 * @brief Function for connect to mqtt by mqtt::async_client
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @return void
 * @see mqtt::async_client
 * @see mqtt::connect_options
 * @see mqtt::exception
*/
void RosMqttConnectionManager::mqtt_connect() {
    try {
        mqtt::connect_options mqtt_connect_opts;
        mqtt_connect_opts.set_clean_session(true);
        mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(60));
        if(mqtt_async_client_.is_connected()) {
            std::cout << log_ros_mqtt_bridge_ << " MQTT connection success" << '\n';
            mqtt_async_client_.set_callback(*this);
        } else {
            std::cout << log_ros_mqtt_bridge_ << " MQTT connection failed... trying to reconnect" << '\n';
            mqtt_async_client_.connect(mqtt_connect_opts)->wait_for(std::chrono::seconds(30));
        }
    } catch (const mqtt::exception& mqtt_expn) {
        std::cerr << log_ros_mqtt_bridge_ << " connection error : " << mqtt_expn.what() << '\n';
    }
}

/**
 * @brief Function for synthesize mqtt subscription
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @return void
 * @see mqtt_subscribe
*/
void RosMqttConnectionManager::grant_mqtt_subscriptions() {
    this->mqtt_subscribe(mqtt_topics::subscription::chatter_topic);
}

/**
 * @brief Function for handle mqtt callback message for ros publish
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param mqtt_topic std::string&
 * @param mqtt_payload std::string&
 * @return void
 * @see message_arrived
 * @see ros_mqtt_connections
*/
void RosMqttConnectionManager::publish_to_ros(std::string& mqtt_topic, std::string& mqtt_payload) {
    std::cout << "[RosMqttConnectionManager] message arrived" << '\n';
    std::cout << "\ttopic: '" << mqtt_topic << "'" << '\n';
    std::cout << "\tpayload: '" << mqtt_payload << "'" << '\n';

    if(mqtt_topic == "/chatter") {
        std::cout << "[RosMqttConnectionManager] publish to " << mqtt_topic << '\n';

        auto std_message = std_msgs::msg::String();
        std_message.data = mqtt_payload;

        ros_mqtt_connections::publisher::ros_chatter_publisher_ptr_->publish(std_message);
    }
}

/**
 * @brief Overrided function for handle cause when mqtt connection lost
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param mqtt_connection_lost_cause const std::string&
 * @return void
 * @see mqtt::callback
*/
void RosMqttConnectionManager::connection_lost(const std::string& mqtt_connection_lost_cause) {
    std::cerr << log_ros_mqtt_bridge_ << " connection lost : " << mqtt_connection_lost_cause << '\n';
}

/**
 * @brief Overrided function for handle message when mqtt subscription get callback mqtt message
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param mqtt_message mqtt::const_message_ptr
 * @return void
 * @see mqtt::callback
 * @see mqtt::const_message_ptr
 * @see ros_mqtt_connections::publisher::ros_chatter_publisher_ptr
*/
void RosMqttConnectionManager::message_arrived(mqtt::const_message_ptr mqtt_message) {
    std::string mqtt_topic = mqtt_message->get_topic();
    std::string mqtt_payload = mqtt_message->to_string();
    this->publish_to_ros(mqtt_topic, mqtt_payload);
}

/**
 * @brief Overrided function for handle delivered token
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @param mqtt_delivered_token mqtt::delivery_token_ptr
 * @return void
 * @see mqtt::callback
*/
void RosMqttConnectionManager::delivery_complete(mqtt::delivery_token_ptr mqtt_delivered_token) {
	std::cout << log_ros_mqtt_bridge_ << " delivery complete with [" << mqtt_delivered_token <<  "] \n";
}

/**
 * @brief Function for mqtt publish into mqtt Broker
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param topic char *
 * @param payload std::string
 * @return void
 * @see mqtt::message_ptr
 * @see mqtt::exception
*/
void RosMqttConnectionManager::mqtt_publish(char * mqtt_topic, std::string mqtt_payload) {
	try {
		mqtt::message_ptr mqtt_publish_msg = mqtt::make_message(mqtt_topic, mqtt_payload);
		mqtt_publish_msg->set_qos(mqtt_qos_);
		auto delivery_token = mqtt_async_client_.publish(mqtt_publish_msg);
        delivery_token->wait();
        if (delivery_token->get_return_code() != mqtt_is_success_) {
            std::cerr << log_ros_mqtt_bridge_ << " publishing error : " << delivery_token->get_return_code() << '\n';
        }
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << log_ros_mqtt_bridge_ << " publishing error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Function for create mqtt subscription from mqtt broker
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param topic char *
 * @see mqtt::exception
*/
void RosMqttConnectionManager::mqtt_subscribe(char * mqtt_topic) {
	try {
		std::cout << log_ros_mqtt_bridge_ << " grant subscription with '" << mqtt_topic << "' " << '\n';
		mqtt_async_client_.subscribe(mqtt_topic, mqtt_qos_);
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << log_ros_mqtt_bridge_ << " grant subscription error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Function for create ros publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @return void
 * @see rclcpp
 * @see ros_mqtt_connections
 * @see ros_mqtt_topics
*/
void RosMqttConnectionManager::create_ros_publishers() {
    ros_mqtt_connections::publisher::ros_chatter_publisher_ptr_ = ros_node_ptr_->create_publisher<std_msgs::msg::String>(ros_topics::to_connection::chatter_topic, rclcpp::QoS(rclcpp::KeepLast(0)));
}

/**
 * @brief Function for create ros subscription
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @return void
 * @see rclcpp
 * @see ros_mqtt_connections
 * @see ros_mqtt_topics
*/
void RosMqttConnectionManager::create_ros_subscriptions() {
    ros_mqtt_connections::subscription::ros_chatter_subscription_ptr_ = ros_node_ptr_->create_subscription<std_msgs::msg::String>(
        ros_topics::from_connection::chatter_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr callback_chatter_data) {
            std::cout << "[RosMqttConnectionManager] chatter callback : " << callback_chatter_data->data.c_str() << '\n';
            std::string chatter_json_str = std_msgs_converter_ptr_->convert_chatter_to_json(callback_chatter_data);
            mqtt_publish(mqtt_topics::publisher::chatter_topic, chatter_json_str);
        }
    );
    ros_mqtt_connections::subscription::ros_odom_subscription_ptr_ = ros_node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
        ros_topics::from_connection::odom_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const nav_msgs::msg::Odometry::SharedPtr callback_odom_data) {
            std::string odom_json_str = nav_msgs_converter_ptr_->convert_odom_to_json(callback_odom_data);
            mqtt_publish(mqtt_topics::publisher::odom_topic, odom_json_str);
        }
    );
    ros_mqtt_connections::subscription::ros_tf_subscription_ptr_ = ros_node_ptr_->create_subscription<tf2_msgs::msg::TFMessage>(
        ros_topics::from_connection::tf_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const tf2_msgs::msg::TFMessage::SharedPtr callback_tf_data) {
            std::string tf_json_str = tf2_msgs_converter_ptr_->convert_tf_to_json(callback_tf_data);
            mqtt_publish(mqtt_topics::publisher::tf_topic, tf_json_str);
        }
    );
}

/**
 * @brief Function for synthesize ros connections
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @return void
 * @see create_ros_publisher
 * @see create_ros_subscriptions
*/
void RosMqttConnectionManager::create_ros_bridge() {
    this->create_ros_publishers();
    this->create_ros_subscriptions();
}

/**
 * @brief Constructor for initialize this class instance & create rclcpp::Node named with ros_mqtt_bridge & invoke ros_mqtt_connection_manager classes' constructors
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
 * @see rclcpp::Node
*/
RosMqttBridge::RosMqttBridge()
: Node("ros_mqtt_bridge"),
log_ros_mqtt_bridge_(LOG_ROS_MQTT_BRIDGE) {
    ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    ros_mqtt_connection_manager_ptr_ = new RosMqttConnectionManager(ros_node_ptr_);
}

/**
 * @brief Virtual Destructor for this class & delete ros_mqtt_connection_manager_ptr_
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.09
*/
RosMqttBridge::~RosMqttBridge() {
    delete ros_mqtt_connection_manager_ptr_;
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
 * @brief Function for initialize this class instance & initialize rclcpp & spin ros_mqtt_bridge rclcpp::Node
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.04
 * @param argc int
 * @param argv char**
 * @return int
 * @see rclcpp
 * @see RosMqttBridge
 * @see check_rclcpp_status()
*/
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    check_rclcpp_status();
    auto node = std::make_shared<RosMqttBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}