#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT20.h"
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClient.h>

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;
TaskHandle_t MqttTaskHandle = NULL;
TaskHandle_t OTATaskHandle = NULL;

// DHT20 Sensor
DHT20 DHT;

// Define I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// WiFi Credentials (Thay bằng WiFi của bạn)
const char* ssid = "SonTung";      
const char* password = "123456789";

// MQTT Server CoreIoT
const char* mqtt_server = "app.coreiot.io";  // Sử dụng đúng server
const int mqtt_port = 1883;
const char* access_token = "5cbgkd8ggnlt8lq4eeet";  // Access Token từ CoreIoT
const char* mqtt_topic = "v1/devices/me/telemetry";  // Topic mặc định của CoreIoT

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Kết nối WiFi
void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());  // In địa chỉ IP của ESP32
}

// Kiểm tra DNS để phân giải "app.coreiot.io"
void testDNS() {
    IPAddress mqttServerIP;
    if (WiFi.hostByName(mqtt_server, mqttServerIP)) {
        Serial.print("Resolved IP for MQTT Broker: ");
        Serial.println(mqttServerIP);
    } else {
        Serial.println("Failed to resolve MQTT Broker IP");
    }
}

// Kết nối MQTT bằng Access Token
void connectMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client", access_token, NULL)) {  // Dùng Access Token thay vì username/password
            Serial.println("Connected to MQTT!");
        } else {
            Serial.print("Failed, rc=");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

void checkOTAUpdate() {
    const char* firmware_url = "http://10.0.232.103:8000/firmware.bin"; // ✅ Đảm bảo IP đúng
    Serial.println("[OTA] Checking for firmware update...");

    HTTPClient http;
    http.begin(firmware_url);

    int httpCode = http.GET();
    if (httpCode == 200) {
        int contentLength = http.getSize();
        WiFiClient* stream = http.getStreamPtr();

        if (Update.begin(contentLength)) {
            size_t written = Update.writeStream(*stream);

            if (written == contentLength) {
                Serial.println("[OTA] Write successful.");
            } else {
                Serial.printf("[OTA] Write failed! Only %d/%d bytes written.\n", written, contentLength);
            }

            if (Update.end()) {
                if (Update.isFinished()) {
                    Serial.println("[OTA] Update successful, restarting...");
                    ESP.restart();
                } else {
                    Serial.println("[OTA] Update incomplete.");
                }
            } else {
                Serial.printf("[OTA] Update error #%d\n", Update.getError());
            }
        } else {
            Serial.println("[OTA] Not enough space to begin update.");
        }
    } else {
        Serial.printf("[OTA] HTTP error code: %d\n", httpCode);
    }

    http.end();
}



// Task 1: In ra console
void Task1(void *pvParameters) {
    while (1) {
        Serial.println("Hello from Task1");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task 2: In ra console
void Task2(void *pvParameters) {
    while (1) {
        Serial.println("Hello from Task2");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// Task 3: Đọc dữ liệu từ DHT20
float temperature, humidity;

void Task3(void *pvParameters) {
    while (1) {
        if (millis() - DHT.lastRead() >= 5000) {  // Đo mỗi 5 giây
            int status = DHT.read();
            temperature = DHT.getTemperature();
            humidity = DHT.getHumidity();

            Serial.print("DHT20 Temperature: ");
            Serial.print(temperature, 1);
            Serial.println(" °C");

            Serial.print("DHT20 Humidity: ");
            Serial.print(humidity, 1);
            Serial.println(" %");

            Serial.println();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));  // Chờ 5 giây trước khi đo lại
    }
}

// Task 4: Gửi dữ liệu lên CoreIoT bằng MQTT
void MqttTask(void *pvParameters) {
    while (1) {
        if (!client.connected()) {
            connectMQTT();
        }
        client.loop();

        // Gửi dữ liệu lên CoreIoT
        char payload[100];
        snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);
        client.publish(mqtt_topic, payload);

        Serial.print("Sent to MQTT: ");
        Serial.println(payload);

        vTaskDelay(pdMS_TO_TICKS(5000));  // Gửi dữ liệu mỗi 5 giây
    }
}

void OTATask(void *pvParameters) {
    while (1) {
        checkOTAUpdate();
        vTaskDelay(pdMS_TO_TICKS(60000));  // Kiểm tra mỗi 60 giây
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);

    // Kết nối WiFi
    connectWiFi();

    // Kiểm tra DNS của server
    testDNS();

    // Khởi tạo DHT20
    if (!DHT.begin()) {
        Serial.println("Failed to initialize DHT20 sensor!");
        while (1);
    }
    Serial.println("DHT20 sensor initialized.");

    // Tạo Tasks với FreeRTOS
    xTaskCreate(Task1, "Task1", 1000, NULL, 1, &Task1Handle);
    xTaskCreate(Task2, "Task2", 1000, NULL, 1, &Task2Handle);
    xTaskCreate(Task3, "DHT20Task", 2000, NULL, 1, &Task3Handle);
    xTaskCreate(MqttTask, "MQTTTask", 4000, NULL, 1, &MqttTaskHandle);
    xTaskCreate(OTATask, "OTATask", 4000, NULL, 1, &OTATaskHandle);
}

void loop() {
    // FreeRTOS quản lý Tasks
}