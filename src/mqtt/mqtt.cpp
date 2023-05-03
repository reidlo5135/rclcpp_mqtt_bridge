#include "mqtt/mqtt.hpp"

Mqtt::Mqtt(const std::string address, const std::string client_id)
: cli_(address, client_id),
callback_(),
mqtt_log_(LOG_MQTT),
mqtt_qos_(MQTT_QOS),
mqtt_is_success_(mqtt::SUCCESS) {
	try {
		mqtt::connect_options connect_opts;
		connect_opts.set_clean_session(false);
		cli_.connect(connect_opts)->wait_for(std::chrono::seconds(60));
		std::cout << mqtt_log_ << " Connection success" << '\n' << '\n';
		cli_.set_callback(callback_);
	} catch (const mqtt::exception& ex) {
		std::cerr << mqtt_log_ << " connection error : " << ex.what();
	}
}

Mqtt::~Mqtt() {

}

void Mqtt::mqtt_publish(char * topic, std::string payload) {
	try {
		std::cout << mqtt_log_ << " publish into '" << topic  << "' with {" << payload << "}" <<  '\n';
		mqtt::message_ptr pub_msg = mqtt::make_message(topic, payload);
		pub_msg->set_qos(mqtt_qos_);
		auto delivery_token = cli_.publish(pub_msg);
        delivery_token->wait();
        if (delivery_token->get_return_code() != mqtt_is_success_) {
            std::cerr << mqtt_log_ << " publishing error : " << delivery_token->get_return_code();
        }
	} catch (const mqtt::exception& ex) {
		std::cerr << mqtt_log_ << " publishing error : " << ex.what();
	}
}

void Mqtt::mqtt_subscribe(char * topic) {
	try {
		std::cout << mqtt_log_ << " subscribe in '" << topic << "' " << '\n';
		cli_.subscribe(topic, mqtt_qos_);
	} catch (const mqtt::exception& ex) {
		std::cerr << mqtt_log_ << " subscribing error : " << ex.what();
	}
}

MqttCallback::MqttCallback()
: mqtt_log_(LOG_MQTT) {

}

MqttCallback::~MqttCallback() {
	
}

void MqttCallback::connection_lost(const std::string& cause) {
	std::cout << mqtt_log_ << " Connection lost: " << cause << '\n';
}

void MqttCallback::message_arrived(mqtt::const_message_ptr msg) {
	std::cout << mqtt_log_ << " Message arrived" << '\n';
    std::cout << mqtt_log_ << "\ttopic: '" << msg->get_topic() << "'" << '\n';
    std::cout << mqtt_log_ << "\tpayload: '" << msg->to_string() << "'" << '\n';
}

void MqttCallback::delivery_complete(mqtt::delivery_token_ptr token) {
	std::cout << mqtt_log_ << " Delivery complete with [" << token <<  "] \n";
}