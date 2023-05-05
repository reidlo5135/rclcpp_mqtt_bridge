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

#ifndef MQTT
#define MQTT

/**
 * include cpp header files
 * @see iostream
 * @see string
 * @see cstring
 * @see chrono
*/
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>

/**
 * include mqtt async_client.h in paho.mqtt.cpp
 * @see /usr/local/include/mqtt
*/
#include "mqtt/async_client.h"

// define mqtt log
#define LOG_MQTT "[MQTT]"
// define mqtt address
#define MQTT_ADDRESS    "tcp://localhost:1883"
// define mqtt client id
#define MQTT_CLIENT_ID    "ros_mqtt_bridge"
// define mqtt qos
#define MQTT_QOS         0
// define mqtt retry attempts
#define MQTT_N_RETRY_ATTEMPTS 5

/**
 * @brief Class for override mqtt::callback functions
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
*/
class MqttCallback : public virtual mqtt::callback {
	private :
		const std::string mqtt_log_;
	public :
		MqttCallback();
		virtual ~MqttCallback();
		void connection_lost(const std::string& cause) override;
		void message_arrived(mqtt::const_message_ptr msg) override;
		void delivery_complete(mqtt::delivery_token_ptr token) override;
};

/**
 * @brief Class for establish MQTT connections
 * @author reidlo(naru5135@wavem.net)
 * @date 23.05.01
 * @see mqtt::async_client
 * @see MqttCallback
*/
class MqttMgr {
	private :
		mqtt::async_client cli_;
		MqttCallback * mqtt_callback_ptr_;
		const std::string mqtt_log_;
		const int mqtt_qos_;
		const int mqtt_is_success_;
	public :
		MqttMgr(const std::string address, const std::string client_id);
		virtual ~MqttMgr();
		void mqtt_publish(char * topic, std::string payload);
		void mqtt_subscribe(char * topic);
};

#endif