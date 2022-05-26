#include "mqtt.h"

#include "config.h"
#include "main.h"

#ifdef __MQTT_H

WiFiClient mqttWiFiClient;
PubSubClient mqttClient(mqttWiFiClient);

void MQTT_ConnectAndSend(char *topic, char *buf)
{
#ifdef DEBUG
    Serial.printf("\nConnecting to MQTT broker %s:%d using client id %s and sendig to %s\n", MQTT_BROKER, MQTT_PORT, clientId, topic);
#endif

    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setBufferSize(512);

    // Loop until we're reconnected
    int retries = 3;
    while (!mqttClient.connected() && retries > 0)
    {
        DEBUG_PRINTLN("Attempting new MQTT connection...");
        // Attempt to connect
        if (mqttClient.connect(clientId) == false)
        {
            DEBUG_PRINT("failed, rc=");
            DEBUG_PRINT(mqttClient.state());
            DEBUG_PRINTLN(".  Trying again in 1 second");
            // Wait 1 seconds before retrying
            delay(1000);
        }

        retries -= 1;
    }

    if (retries > 0 && mqttClient.connected())
    {
        DEBUG_PRINTLN("Publishing:");
        DEBUG_PRINTLN(buf);
        // Once connected, publish an announcement...
        if (mqttClient.publish(topic, buf) == false)
        {
            DEBUG_PRINTLN("Did not publish");
        }
    }
}

#endif
