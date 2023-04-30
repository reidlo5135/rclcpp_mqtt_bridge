#include <iostream>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include "mqtt/async_client.h"

const std::string SERVER_ADDRESS { "tcp://localhost:1883" };
const std::string CLIENT_ID { "paho_cpp_async_subcribe" };
const std::string TOPIC { "test" };

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
        mqtt_client() : client_(SERVER_ADDRESS, CLIENT_ID) {
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
            std::cout << "Subscribing to topic '" << TOPIC << "'..." << std::flush;
            tok = client_.subscribe(TOPIC, 1, sub_opts);
            tok->wait();
            std::cout << "OK" << std::endl;
        }

        void run() {
            subscribe();
            while (true) {}
        }

    private:
        mqtt::async_client client_;
        mqtt_callback mqtt_cb_;
};