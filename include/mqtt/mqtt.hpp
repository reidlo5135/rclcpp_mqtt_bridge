#ifndef MQTT
#define MQTT

#include <iostream>
#include <cstring>

#include "MQTTClient.h"

#define LOG_MQTT "[MQTT]"
#define MQTT_ADDRESS    "tcp://localhost:1883"
#define MQTT_CLIENT_ID    "ros_mqtt_bridge"
#define MQTT_QOS         1
#define MQTT_TIMEOUT     10000L

class Mqtt {
    private:
        MQTTClient client;
        MQTTClient_connectOptions connection_opts;
        MQTTClient_message message;
        MQTTClient_deliveryToken token;
        volatile MQTTClient_deliveryToken delivered_token;
    public :
        Mqtt();
        virtual ~Mqtt();
        void mqtt_publish(char * topic, char * payload);
        void on_delivered(MQTTClient_deliveryToken dt);
        int on_message(char * topic, int topic_len, MQTTClient_message * message);
        void on_connection_lost(char * cause);
        int mqtt_subscribe(char * topic);
};

#endif