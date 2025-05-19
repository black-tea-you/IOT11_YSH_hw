/*
 * ESP32 BLE Server for Distance Measurement
 * This device acts as a BLE server and advertises BLE signals with txPower
 * It also creates its own WiFi STA for direct connection and hosts a web page to display distance data
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WiFi.h>

// BLE device, service, and characteristic UUIDs
#define DEVICE_NAME           "11_server"
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// WiFi credentials (Station Mode)
const char* ssid     = "GC_free_WiFi";

// Access Point credentials (if you prefer AP mode)
const char* ap_ssid     = "ESP32-Distance_Seo";
const char* ap_password = "12345678";

// Distance measurement parameters
#define TX_POWER_AT_1M     -59      // txPower value at 1 meter distance (for calibration)
#define PATH_LOSS_EXPONENT   2.7    // Path loss exponent (environment dependent)

// Built-in LED pin for proximity alert
const int LED_PIN = 2;

// Global BLE objects
BLEServer*      pServer        = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
BLEAdvertising*   pAdvertising   = nullptr;

// Connection flags
bool deviceConnected    = false;
bool oldDeviceConnected = false;

// Last received distance (as string)
String distance_data = "Loading...";

// Web server (HTTP) on port 80
WiFiServer server(80);
String header;

// BLE Server connection callbacks
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("[BLE] Client connected");
    }
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("[BLE] Client disconnected");
    }
};

// BLE Characteristic write callbacks
class MyCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        String val = pChar->getValue();
        if (val.startsWith("DIST:")) {
            String numStr = val.substring(5);
            float newDist = numStr.toFloat();
            if (newDist >= 0.0) {
                distance_data = numStr;
                Serial.print("[BLE] Distance updated: ");
                Serial.println(distance_data + " m");
            } else {
                Serial.print("[BLE] Ignored negative distance: ");
                Serial.println(numStr + " m");
            }
        }
    }
};

// Generate HTML page with current distance
String SendHTML() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 BLE Distance Monitor</title>
  <meta http-equiv='refresh' content='5'>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #f0f0f0; }
    .container { margin: 50px auto; width: 80%; padding: 20px;
                 background-color: #fff; border-radius: 10px;
                 box-shadow: 0 0 10px rgba(0,0,0,0.1); }
    .distance { font-size: 60px; color: #333; margin: 30px 0; }
    .alert { color: #ff0000; font-weight: bold; }
  </style>
</head>
<body>
  <div class='container'>
    <h2>ESP32 BLE Distance Monitor</h2>
    <div class='distance'>)rawliteral" + distance_data + " m" + R"rawliteral(</div>
    <p>Last updated: )rawliteral" + String(millis()/1000) + R"rawliteral( seconds since boot</p>
    )rawliteral";

    // Proximity alert message
    float dist = distance_data.toFloat();
    if (dist > 0 && dist <= 1.0) {
        html += "<p class='alert'>Proximity Alert: Device within 1 meter!</p>";
    }

    html += R"rawliteral(
  </div>
</body>
</html>
)rawliteral";
    return html;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[INFO] Starting BLE Distance Monitor...");

    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize WiFi (Station Mode)
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid);
    Serial.print("[WIFI] Connecting to "); Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("[WIFI] Connected! IP address: "); Serial.println(WiFi.localIP());

    // Start web server
    server.begin();
    Serial.println("[HTTP] Web server started on port 80");

    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE service
    BLEService* pService = pServer->createService(SERVICE_UUID);

    // Create BLE characteristic for distance updates
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ    |
                        BLECharacteristic::PROPERTY_NOTIFY  |
                        BLECharacteristic::PROPERTY_WRITE   |  // Write with response
                        BLECharacteristic::PROPERTY_WRITE_NR   // Write without response
                      );
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharCallbacks());
    // Set initial txPower at 1m
    pCharacteristic->setValue(String(TX_POWER_AT_1M).c_str());

    // Start the BLE service
    pService->start();

    // Configure advertising data
    pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEAdvertisementData advData;
    advData.setFlags(0x06);
    advData.setCompleteServices(BLEUUID(SERVICE_UUID));
    advData.setName(DEVICE_NAME);
    pAdvertising->setAdvertisementData(advData);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    pAdvertising->start();
    Serial.println("[BLE] Advertising started");
}

void loop() {
    // Handle BLE connection state changes
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("[BLE] Client connected");
    }
    if (!deviceConnected && oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("[BLE] Client disconnected, restarting advertising");
        pServer->startAdvertising();
    }

    // Blink LED on proximity (<1m)
    float distVal = distance_data.toFloat();
    if (distVal > 0 && distVal <= 1.0) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
    }

    // Handle incoming HTTP clients
    WiFiClient client = server.available();
    if (client) {
        unsigned long startTime = millis();
        while (client.connected() && millis() - startTime < 2000) {
            if (client.available()) {
                char c = client.read();
                header += c;
                if (c == '\n' && header.endsWith("\r\n\r\n")) {
                    // Send HTTP response
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:text/html");
                    client.println("Connection: close");
                    client.println();
                    client.println(SendHTML());
                    break;
                }
            }
        }
        // Give the client time to receive the data
        delay(1);
        client.stop();
        header = "";
    }

    delay(10);
}
