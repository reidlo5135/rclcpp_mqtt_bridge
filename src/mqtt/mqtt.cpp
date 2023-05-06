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

#include "mqtt/mqtt.hpp"

/**
 * @brief Constructor for initialize MqttCallback class' instance & mqtt_log_ with LOG_MQTT
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @see mqtt::callback
*/
MqttCallback::MqttCallback()
: mqtt_log_(LOG_MQTT) {

}

/**
 * @brief Virtual Destructor for this class
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
*/
MqttCallback::~MqttCallback() {
	
}

/**
 * @brief Overrided Function for log error when MQTT connection has been lost
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param cause const std::string&
 * @return void
 * @see mqtt::callback
*/
void MqttCallback::connection_lost(const std::string& cause) {
	std::cerr << mqtt_log_ << " connection lost: " << cause << '\n';
}

/**
 * @brief Overrided Function for log recieved message from MQTT publishers
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param msg mqtt::const_message_ptr
 * @return void
 * @see mqtt::callback
 * @see mqtt::const_messaget_ptr
*/
void MqttCallback::message_arrived(mqtt::const_message_ptr msg) {
	std::cout << mqtt_log_ << " message arrived" << '\n';
    std::cout << "\ttopic: '" << msg->get_topic() << "'" << '\n';
    std::cout << "\tpayload: '" << msg->to_string() << "'" << '\n';
}

/**
 * @brief Overrided Function for log when MQTT devlivery has been completed
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param token mqtt::delivery_token_ptr
 * @return void
 * @see mqtt::callback
 * @see mqtt::delivery_token_ptr
*/
void MqttCallback::delivery_complete(mqtt::delivery_token_ptr token) {
	std::cout << mqtt_log_ << " delivery complete with [" << token <<  "] \n";
}

/**
 * @brief Constructor for initialize this class instance with address, client_id & establish MQTT connections & set callback by MqttCallback
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param address const std::string
 * @param client_id const std::string
 * @see MqttCallback
 * @see mqtt::connect_options
 * @see mqtt::exception
*/
MqttMgr::MqttMgr(const std::string address, const std::string client_id)
: cli_(address, client_id),
mqtt_log_(LOG_MQTT),
mqtt_qos_(MQTT_QOS),
mqtt_is_success_(mqtt::SUCCESS) {
	try {
		mqtt_callback_ptr_ = new MqttCallback();
		mqtt::connect_options connect_opts;
		connect_opts.set_clean_session(false);
		cli_.connect(connect_opts)->wait_for(std::chrono::seconds(60));
		std::cout << mqtt_log_ << " connection success" << '\n' << '\n';
		cli_.set_callback(*mqtt_callback_ptr_);
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << mqtt_log_ << " connection error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Virtual Destructor for this class & delete mqtt_callback_ptr_
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @see mqtt_callback_ptr_
*/
MqttMgr::~MqttMgr() {
	delete mqtt_callback_ptr_;
}

/**
 * @brief Function for MQTT publish into MQTT Broker
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param topic char *
 * @param payload std::string
 * @return void
 * @see mqtt::message_ptr
 * @see mqtt::exception
*/
void MqttMgr::mqtt_publish(char * topic, std::string payload) {
	try {
		mqtt::message_ptr mqtt_publish_msg = mqtt::make_message(topic, payload);
		mqtt_publish_msg->set_qos(mqtt_qos_);
		auto delivery_token = cli_.publish(mqtt_publish_msg);
        delivery_token->wait();
        if (delivery_token->get_return_code() != mqtt_is_success_) {
            std::cerr << mqtt_log_ << " publishing error : " << delivery_token->get_return_code() << '\n';
        }
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << mqtt_log_ << " publishing error : " << mqtt_expn.what() << '\n';
	}
}

/**
 * @brief Function for create MQTT subscription from MQTT Broker
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @param topic char *
 * @see mqtt::exception
*/
void MqttMgr::mqtt_subscribe(char * topic) {
	try {
		std::cout << mqtt_log_ << " grant subscription with '" << topic << "' " << '\n';
		cli_.subscribe(topic, mqtt_qos_);
	} catch (const mqtt::exception& mqtt_expn) {
		std::cerr << mqtt_log_ << " grant subscriptions error : " << mqtt_expn.what() << '\n';
	}
}