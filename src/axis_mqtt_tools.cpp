#include "axis_mqtt_tools.h"
#include <WiFi.h> // Needed for WiFi.macAddress()
#include <ArduinoJson.h>

// --- Define the MQTT Broker Settings ---
const char* mqtt_server = "rollyServer.local"; //  <<--- YOUR MQTT BROKER IP
const int mqtt_port = 1883;
const char* mqtt_user = NAME;
const char* mqtt_password = "pillbugs";
const char* mqtt_topic = "rolly";         // General data topic

const char* mqtt_sub_topic = "rolly/cmd/#"; // Subscribe topic
const char* mqtt_ota_topic = "rolly/snap/ota/update";        // OTA topic
const char* mqtt_ota_status_topic = "rolly/snap/ota/status";      // OTA Status


// const char* mqtt_sub_topic = ("rolly/"+ std::string(NAME) +"/#").c_str(); // Subscribe topic
// const char* mqtt_ota_topic = ("rolly/" + std::string(NAME) + "/ota/update").c_str();        // OTA topic
// const char* mqtt_ota_status_topic = ("rolly/" + std::string(NAME) + "/ota/status").c_str();      // OTA Status

WiFiClient espClient;
PubSubClient client(espClient);  // Define the client object

// Define the atomic variables (allocate memory for them)
extern std::atomic<bool> enable_flag;
extern std::atomic<bool> disable_flag;
extern std::atomic<float> target0;
extern std::atomic<float> target1;


extern std::atomic<float> servo3_pos;
extern std::atomic<float> servo0_pos;
extern std::atomic<float> servo1_pos;
extern std::atomic<float> servo2_pos;

extern std::atomic<uint> last_commanded_mode;
extern std::atomic<float> command_vel_p_gain;
extern std::atomic<float> command_vel_i_gain;
extern std::atomic<float> command_vel_d_gain;
extern std::atomic<float> command_vel_lpf;

void setupMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqtt_callback);
    client.setBufferSize(512);
}

void reconnectMQTT() {
    while (!client.connected()) {                                                                                                        
        Serial.print("Attempting MQTT connection...");
        String clientId = "crackle-";//should maybe be snap/crackle/pop
        clientId += String(WiFi.macAddress());
        Serial.print("Client ID: ");
        Serial.println(clientId);

        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("MQTT connected");
            client.subscribe(mqtt_ota_topic);    // Subscribe to OTA
            client.subscribe(mqtt_sub_topic);   // Subscribe to commands
        } else {
            Serial.print("MQTT connection failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void mqttLoop() {
    client.loop();
}

void publishMQTT(const char* payload) {
    publishMQTT(payload, mqtt_topic); // Call the overloaded version
}

void publishMQTT(const char* payload, const char* topic) {
    if (client.publish(topic, payload)) {
        Serial.print("Published message to topic: ");
        Serial.println(topic);
    } else {
        Serial.println("Failed to publish message");
    }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.println(message);

    if (strcmp(topic, mqtt_ota_topic) == 0) {
        if (strcmp(message, "update") == 0) {
            Serial.println("OTA update requested");
            publishMQTT("Starting OTA update", mqtt_ota_status_topic);
        }
    } 
    // else if topic string contains "cmd/"
    else if (strstr(topic, "cmd/") != NULL) {
        StaticJsonDocument<512> doc;
        deserializeJson(doc, message);

        if (doc.containsKey("motor0")) {
            target0.store(doc["motor0"].as<float>());
            Serial.println(doc["motor0"].as<float>());
        }
        if (doc.containsKey("motor1")) {
            target1.store(doc["motor1"].as<float>());
        }
        if (doc.containsKey("mode")) {
            last_commanded_mode.store(doc["mode"].as<uint>());
        }
        if (doc.containsKey("vel_p")) {
            command_vel_p_gain.store(doc["vel_p"].as<float>());
        }
        if (doc.containsKey("vel_i")) {
            command_vel_i_gain.store(doc["vel_i"].as<float>());
        }
        if (doc.containsKey("vel_d")) {
            command_vel_d_gain.store(doc["vel_d"].as<float>());
        }
        if (doc.containsKey("vel_lpf")) {
            command_vel_lpf.store(doc["vel_lpf"].as<float>());
        }
        if (doc.containsKey("enable")) {
            enable_flag.store(doc["enable"].as<bool>());
        }
        if (doc.containsKey("disable")) {
            disable_flag.store(doc["disable"].as<bool>());
        }
        if (doc.containsKey("servo0")) {
            servo0_pos.store(doc["servo0"].as<float>());
        }
        if (doc.containsKey("servo1")) {
            servo1_pos.store(doc["servo1"].as<float>());
        }
        if (doc.containsKey("servo2")) {
            servo2_pos.store(doc["servo2"].as<float>());
        }
        if (doc.containsKey("servo3")) {
            servo3_pos.store(doc["servo3"].as<float>());
        }
    }
}

bool isMQTTConnected() {
    return client.connected();
}