#ifndef MQTT
#define MQTT

#include <iostream>
#include <cstring>

#include "MQTTClient.h"

#define LOG_MQTT "[MQTT]"
#define MQTT_ADDRESS    "tcp://localhost:1883"
#define MQTT_CLIENTID    "ros_mqtt_bridge"
#define MQTT_QOS         1
#define MQTT_TIMEOUT     10000L

class Mqtt {
    public :
        void mqtt_publish(char * topic, char * payload);
        void mqtt_subscribe(char * topic);
        void on_delivered(MQTTClient_deliveryToken dt);
        void on_message(char * topic, int topic_len, MQTTClient_message * message);
        void on_connection_lost(char * cause);
};

#endif