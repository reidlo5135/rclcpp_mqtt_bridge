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
#define MQTT_TIMEOUT     10000L

class mqtt_callback : public virtual mqtt::callback {
    public:
        void connection_lost(const std::string& cause) override {
            std::cout << "\nConnection lost" << std::endl;
            if (!cause.empty()) {
                std::cout << "Cause: " << cause << std::endl;
            }
        }

        void message_arrived(mqtt::const_message_ptr msg) override {
            std::cout << "Message arrived:" << std::endl;
            std::cout << "  topic: " << msg->get_topic() << std::endl;
            std::cout << "  payload: " << msg->to_string() << std::endl;
        }

        void delivery_complete(mqtt::delivery_token_ptr token) override {}
};

class mqtt_client {
    public:
        mqtt_client() : client_(MQTT_ADDRESS, MQTT_CLIENT_ID) {
            mqtt::connect_options conn_opts;
            conn_opts.set_keep_alive_interval(20);
            conn_opts.set_clean_session(true);

            client_.set_callback(mqtt_cb_);

            try {
                std::cout << "Connecting to the MQTT server..." << std::flush;
                client_.connect(conn_opts)->wait();
                std::cout << "OK" << std::endl;
            }
            catch (const mqtt::exception& exc) {
                std::cerr << "\nERROR: Unable to connect to MQTT server: " << exc.what() << std::endl;
                exit(1);
            }
        }

        void subscribe() {
            mqtt::token_ptr tok;
            mqtt::subscribe_options sub_opts;
            std::cout << "Subscribing to topic '" << "/tester" << "'..." << std::flush;
            tok = client_.subscribe("/tester", MQTT_QOS, sub_opts);
            tok->wait();
            std::cout << "OK" << std::endl;
        }

        void run() {
            subscribe();
        }

    private:
        mqtt::async_client client_;
        mqtt_callback mqtt_cb_;
};

#endif