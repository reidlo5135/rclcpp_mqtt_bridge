#include "mqtt/mqtt.hpp"

Mqtt::Mqtt() : connection_opts(MQTTClient_connectOptions_initializer), message(MQTTClient_message_initializer)  {

}

Mqtt::~Mqtt() {

}

void Mqtt::mqtt_publish(char * topic, char * payload) {
    int rc;

    if ((rc = MQTTClient_create(&client, MQTT_ADDRESS, MQTT_CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS) {
        std::cerr << LOG_MQTT << " failed to create client, return code "<< rc << "\n";
        rc = EXIT_FAILURE;
    }

    connection_opts.keepAliveInterval = 20;
    connection_opts.cleansession = 1;

    if ((rc = MQTTClient_connect(client, &connection_opts)) != MQTTCLIENT_SUCCESS) {
        std::cerr << LOG_MQTT << " failed to connect broker, return code "<< rc << "\n";
        rc = EXIT_FAILURE;
    }

    message.payload = payload;
    message.payloadlen = (int)strlen(payload);
    message.qos = MQTT_QOS;
    message.retained = 0;
    if ((rc = MQTTClient_publishMessage(client, topic, &message, &token)) != MQTTCLIENT_SUCCESS) {
        std::cerr << LOG_MQTT << " failed to publish, return code "<< rc << "\n";
        rc = EXIT_FAILURE;
    } else {
        std::cout << LOG_MQTT << " published to [" << topic << "] with [" << message.payload << "] --length [" << message.payloadlen << "\n";
    }
 
    std::cout << LOG_MQTT << " waiting for up to " << (int)(MQTT_TIMEOUT/1000) << " seconds for publication of [" << payload << "] on topic [" << topic << "]" << " for client with id [" << MQTT_CLIENT_ID << "]" << "\n";
    rc = MQTTClient_waitForCompletion(client, token, MQTT_TIMEOUT);
    std::cout << LOG_MQTT << " message with delivery token " << token << " delivered" << "\n";
}

void Mqtt::on_delivered(MQTTClient_deliveryToken dt) {
    std::cout << LOG_MQTT << " subscription message with token value " << dt << " delivery confirmed" << "\n";
    delivered_token = dt;
}

int Mqtt::on_message(char * topic, int topic_len, MQTTClient_message * message) {
    std::cout << LOG_MQTT << " message arrived from {" << topic << "} with message [" << message -> payload << "\n";
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topic);
    return 1;
}

void Mqtt::on_connection_lost(char * cause) {
    std::cout << LOG_MQTT << " connection lost" << "\n";
    std::cout << LOG_MQTT << " cause" << cause << "\n";
}

int Mqtt::mqtt_subscribe(char * topic) {
    int rc;
    Mqtt * mqtt;

    if((rc = MQTTClient_create(&client, MQTT_ADDRESS, MQTT_CLIENT_ID, MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS) {
        printf("%s failed to create client, return code %d \n", LOG_MQTT, rc);
        rc = EXIT_FAILURE;
        goto exit;
    } else {
        printf("%s client created... %d \n", LOG_MQTT, rc);
    };

    connection_opts.keepAliveInterval = 20;
    connection_opts.cleansession = 1;
    if((rc = MQTTClient_connect(client, &connection_opts)) != MQTTCLIENT_SUCCESS) {
        printf("%s failed to connect, return code %d \n", LOG_MQTT, rc);
        rc = EXIT_FAILURE;
        goto destroy_exit;
    } else {
        printf("%s connection established... %d \n", LOG_MQTT, rc);
    };

    printf("%s subscribing to topic %s for client id %s using MQTT_QOS %d \n\n", LOG_MQTT, topic, MQTT_CLIENT_ID, MQTT_QOS);
    printf("%s Press Q<Enter> to quit \n\n", LOG_MQTT);
    if((rc = MQTTClient_subscribe(client, topic, MQTT_QOS)) != MQTTCLIENT_SUCCESS) {
        printf("%s failed to subscribe, return code %d \n", LOG_MQTT, rc);
        rc = EXIT_FAILURE;
    } else {
        int ch;
        do {
            ch = getchar();
        } while(ch != 'Q' && ch != 'q');
        if((rc = MQTTClient_unsubscribe(client, topic)) != MQTTCLIENT_SUCCESS) {
                printf("%s failed to unsubscribe, return code %d \n", LOG_MQTT, rc);
                rc = EXIT_FAILURE;
        };
    };

    if((rc = MQTTClient_disconnect(client, 10000)) != MQTTCLIENT_SUCCESS) {
        printf("%s faield to disconnect, return code %d \n", LOG_MQTT, rc);
        rc = EXIT_FAILURE;
    };

    destroy_exit:
        MQTTClient_destroy(&client);
    exit:
        return rc;
}