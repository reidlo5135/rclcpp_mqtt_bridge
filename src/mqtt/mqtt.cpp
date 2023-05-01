#include "mqtt/mqtt.hpp"

Mqtt::Mqtt(const std::string address, const std::string client_id)
: cli_(address, client_id),
callback_() {
	try {
		mqtt::connect_options connect_opts;
		connect_opts.set_clean_session(false);
		cli_.connect(connect_opts)->wait_for(std::chrono::seconds(60));
		std::cout << LOG_MQTT << " Connection success" << "\n\n";
		cli_.set_callback(callback_);
	} catch (const mqtt::exception& ex) {
		std::cerr << LOG_MQTT << " connection error : " << ex.what();
	}
}

Mqtt::~Mqtt() {

}

void Mqtt::mqtt_publish(char * topic, std::string payload) {
	try {
		std::cout << LOG_MQTT << " publish into '" << topic  << "' with {" << payload << "}" <<  "\n";
		mqtt::message_ptr pub_msg = mqtt::make_message(topic, payload);
		pub_msg->set_qos(MQTT_QOS);
		auto delivery_token = cli_.publish(pub_msg);
        delivery_token->wait();
        if (delivery_token->get_return_code() != mqtt::SUCCESS) {
            std::cerr << LOG_MQTT << " publishing error : " << delivery_token->get_return_code();
        }
	} catch (const mqtt::exception& ex) {
		std::cerr << LOG_MQTT << " publishing error : " << ex.what();
	}
}

void Mqtt::mqtt_subscribe(char * topic) {
	try {
		std::cout << LOG_MQTT << " subscribe in '" << topic << "' " << "\n";
		cli_.subscribe(topic, MQTT_QOS);
	} catch(const mqtt::exception& ex) {
		std::cerr << LOG_MQTT << " subscribing error : " << ex.what();
	}
}

MqttCallback::MqttCallback() {

}

MqttCallback::~MqttCallback() {
	
}

void MqttCallback::connection_lost(const std::string& cause) {
	std::cout << LOG_MQTT << " Connection lost: " << cause << "\n";
}

void MqttCallback::message_arrived(mqtt::const_message_ptr msg) {
	std::cout << LOG_MQTT << " Message arrived" << "\n";
    std::cout << LOG_MQTT << "\ttopic: '" << msg->get_topic() << "'" << "\n";
    std::cout << LOG_MQTT << "\tpayload: '" << msg->to_string() << "'" << "\n";
}

void MqttCallback::delivery_complete(mqtt::delivery_token_ptr token) {
	std::cout << LOG_MQTT << " Delivery complete with [" << token <<  "] \n";
}