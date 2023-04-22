#include "mqtt/mqtt.hpp"

void Mqtt::mqtt_publish(char * topic, char * payload) {
    MQTTClient client;
    MQTTClient_connectOptions connection_options = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc;

    if ((rc = MQTTClient_create(&client, MQTT_ADDRESS, MQTT_CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS) {
        std::cerr << LOG_MQTT << " failed to create client, return code "<< rc << std::endl;
        rc = EXIT_FAILURE;
    }

    connection_options.keepAliveInterval = 20;
    connection_options.cleansession = 1;

    if ((rc = MQTTClient_connect(client, &connection_options)) != MQTTCLIENT_SUCCESS) {
        std::cerr << LOG_MQTT << " failed to connect broker, return code "<< rc << std::endl;
        rc = EXIT_FAILURE;
    }

    pubmsg.payload = payload;
    pubmsg.payloadlen = (int)strlen(payload);
    pubmsg.qos = MQTT_QOS;
    pubmsg.retained = 0;
    if ((rc = MQTTClient_publishMessage(client, topic, &pubmsg, &token)) != MQTTCLIENT_SUCCESS) {
        std::cerr << LOG_MQTT << " failed to publish, return code "<< rc << std::endl;
        rc = EXIT_FAILURE;
    } else {
        std::cout << LOG_MQTT << " published to [" << topic << "] with [" << pubmsg.payload << "] --length [" << pubmsg.payloadlen << std::endl;
    }
 
    std::cout << LOG_MQTT << " waiting for up to " << (int)(MQTT_TIMEOUT/1000) << " seconds for publication of [" << payload << "] on topic [" << topic << "]" << " for client with id [" << MQTT_CLIENTID << "]" << std::endl;
    rc = MQTTClient_waitForCompletion(client, token, MQTT_TIMEOUT);
    std::cout << LOG_MQTT << " message with delivery token " << token << " delivered" << std::endl;
}