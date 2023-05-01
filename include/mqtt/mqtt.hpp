#ifndef MQTT
#define MQTT

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>

#include "mqtt/async_client.h"

#define LOG_MQTT "[MQTT]"
#define MQTT_ADDRESS    "tcp://localhost:1883"
#define MQTT_CLIENT_ID    "ros_mqtt_bridge"
#define MQTT_QOS         1
#define MQTT_N_RETRY_ATTEMPTS 5
#define MQTT_INIT_TOPIC "ros_message_init"

class MQTTActionListener : public virtual mqtt::iaction_listener {
    private :
        std::string name_;
        void on_success(const mqtt::token& tok) override;
        void on_failure(const mqtt::token& tok) override;
    public:
        MQTTActionListener(const std::string& name);
        virtual ~MQTTActionListener();
};

class MQTTCallback : public virtual mqtt::callback, public virtual mqtt::iaction_listener {
	private :
		int n_retry_;
		mqtt::async_client& cli_;
		mqtt::connect_options& connect_opts_;
		MQTTActionListener sub_listener_;
		void reconnect();
		void on_success(const mqtt::token& tok) override;
		void on_failure(const mqtt::token& tok) override;
		void connected(const std::string& cause) override;
		void connection_lost(const std::string& cause) override;
		void message_arrived(mqtt::const_message_ptr msg) override;
		void delivery_complete(mqtt::delivery_token_ptr token) override;
	public:
		MQTTCallback(mqtt::async_client& cli, mqtt::connect_options& connect_opts);
		virtual ~MQTTCallback();
};

class Mqtt {
	public :
		Mqtt();
		virtual ~Mqtt();
		void mqtt_publish(char * topic, char * payload);
		void mqtt_subscribe();
};

#endif