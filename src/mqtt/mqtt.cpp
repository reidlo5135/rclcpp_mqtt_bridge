#include "mqtt/mqtt.hpp"

MQTTActionListener::MQTTActionListener(const std::string& name)
: name_(name) {

}

MQTTActionListener::~MQTTActionListener() {

}

void MQTTActionListener::on_success(const mqtt::token& tok) {
    std::cout << LOG_MQTT << name_ << " success" << "\n";
    const int message_id = tok.get_message_id();
    if (message_id != 0) {
        std::cout << LOG_MQTT << " for token: [" << message_id << "]" << "\n";
    }
    auto top = tok.get_topics();
    if (top && !top->empty()) {
        std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << "\n";
    }
    std::cout << "\n";
}

void MQTTActionListener::on_failure(const mqtt::token& tok) {
    std::cout << LOG_MQTT << name_ << " failure";
    const int message_id = tok.get_message_id();
    if (message_id != 0) {
        std::cout << LOG_MQTT << " for token: [" << message_id << "]" << "\n";
    }
    std::cout << "\n";
}

MQTTCallback::MQTTCallback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
: n_retry_(0),
cli_(cli),
connect_opts_(connOpts),
sub_listener_("Subscription") {

}

MQTTCallback::~MQTTCallback() {

}

void MQTTCallback::reconnect() {
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
	try {
		cli_.connect(connect_opts_, nullptr, *this);
	}
	catch (const mqtt::exception& exc) {
		std::cerr << LOG_MQTT << " Error: " << exc.what() << "\n";
		exit(1);
	}
}

void MQTTCallback::on_success(const mqtt::token& tok) {

}

void MQTTCallback::on_failure(const mqtt::token& tok) {
	std::cout << LOG_MQTT << " Connection attempt failed" << "\n";
	if (++n_retry_ > MQTT_N_RETRY_ATTEMPTS) {
        exit(1);
    }
	reconnect();
}

void MQTTCallback::connected(const std::string& cause) {
    const char * topic = "/test";
	std::cout << LOG_MQTT << " Connection success" << "\n\n";
	std::cout << LOG_MQTT << " Subscribing to topic '" << topic << "' for client " << MQTT_CLIENT_ID << " using QoS " << MQTT_QOS << "\n" << "\nPress Q<Enter> to quit\n" << "\n";
    cli_.subscribe(topic, MQTT_QOS, nullptr, sub_listener_);
}

void MQTTCallback::connection_lost(const std::string& cause) {
    std::cout << "\nConnection lost" << "\n";
	if (!cause.empty()) {
        std::cout << "\tcause: " << cause << "\n";
    }
    std::cout << LOG_MQTT << "Reconnecting..." << "\n";
	n_retry_ = 0;
	reconnect();
}

void MQTTCallback::message_arrived(mqtt::const_message_ptr msg) {
	std::cout << LOG_MQTT << " Message arrived" << "\n";
	std::cout << "\ttopic: '" << msg->get_topic() << "'" << "\n";
	std::cout << "\tpayload: '" << msg->to_string() << "'\n" << "\n";
}

void MQTTCallback::delivery_complete(mqtt::delivery_token_ptr token) {

}

Mqtt::Mqtt() {

}

Mqtt::~Mqtt() {

}

void Mqtt::mqtt_publish(char * topic, char * payload) {
	std::cout << LOG_MQTT << " publish to " << topic << " with " << payload;
}

void Mqtt::mqtt_subscribe() {
	mqtt::async_client cli(MQTT_ADDRESS, MQTT_CLIENT_ID);

	mqtt::connect_options connect_opts;
	connect_opts.set_clean_session(false);

	MQTTCallback cb(cli, connect_opts);
	cli.set_callback(cb);

	try {
		std::cout << LOG_MQTT << " Connecting to the MQTT server..." << "\n" << std::flush;
		cli.connect(connect_opts, nullptr, cb)->wait();
	}
	catch (const mqtt::exception& exc) {
		std::cerr << LOG_MQTT << " ERROR: Unable to connect to MQTT server: '" << MQTT_ADDRESS << "'" << exc << "\n";
	}

	while(true) {}

	// try {
	// 	std::cout << "\nDisconnecting from the MQTT server..." << std::flush;
	// 	cli.disconnect()->wait();
	// 	std::cout << "OK" << std::endl;
	// }
	// catch (const mqtt::exception& exc) {
	// 	std::cerr << exc << std::endl;
	// }
}