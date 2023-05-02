#ifndef MQTT
#define MQTT

#include <iostream>
#include <string>
#include <cstring>
#include <chrono>

#include "mqtt/async_client.h"

#define LOG_MQTT "[MQTT]"
#define MQTT_ADDRESS    "tcp://localhost:1883"
#define MQTT_CLIENT_ID    "ros_mqtt_bridge"
#define MQTT_QOS         0
#define MQTT_N_RETRY_ATTEMPTS 5
#define MQTT_INIT_TOPIC "ros_message_init"

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

class Mqtt {
	private :
		mqtt::async_client cli_;
		MqttCallback callback_;
		const std::string mqtt_log_;
		const int mqtt_qos_;
		const int mqtt_is_success_;
	public :
		Mqtt(const std::string address, const std::string client_id);
		virtual ~Mqtt();
		void mqtt_publish(char * topic, std::string payload);
		void mqtt_subscribe(char * topic);
};

#endif