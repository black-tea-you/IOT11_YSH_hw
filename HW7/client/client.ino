/*
 * ESP32 BLE Client for Distance Measurement - With Persistent Connection
 * This device scans for BLE signals, calculates distance based on RSSI,
 * and maintains a persistent connection to the server
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// Define target device
#define TARGET_DEVICE_NAME "11_server"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Define measurement parameters
#define SCAN_TIME  1         // Scan time in seconds
#define N_SAMPLES  3         // Number of RSSI measurements to average
#define SCAN_INTERVAL 300    // Time between scans in milliseconds
#define PATH_LOSS_EXPONENT 2.0  // Path loss exponent (free space)
#define CALIBRATION_FACTOR 0.4  // Calibration factor for distance

// LED pin for proximity alert
#define LED_PIN 2            // Built-in LED on ESP32 is typically pin 2

// Thresholds for proximity alerts
#define CLOSE_DISTANCE 1.0   // Distance threshold for "close" (triggers LED)
#define MAX_VALID_DISTANCE 10.0 // Maximum realistic distance for BLE

// Variables for RSSI measurements
int rssiValues[N_SAMPLES];
int sampleIndex = 0;
int txPowerAt1m = -59;       // Transmission power at 1m

// Scanning and client objects
BLEScan* pBLEScan;
BLEClient* pClient = NULL;
BLERemoteCharacteristic* pRemoteCharacteristic = NULL;

// Server info storage variables
BLEAddress* serverAddress = NULL;
bool connected = false;

// Variable to store last distance measurement
float lastDistance = -1;
float lastRawDistance = -1;
float lastRssi = 0;

// Error handling
bool measurementValid = false;
int consecutiveErrorCount = 0;
#define MAX_CONSECUTIVE_ERRORS 2

// Timing variables
unsigned long lastMeasurementTime = 0;
unsigned long lastConnectionAttempt = 0;
#define MEASUREMENT_INTERVAL 2000  // Time between measurements (2s)
#define RECONNECT_INTERVAL 5000    // Time between reconnection attempts (5s)
#define DATA_SEND_INTERVAL 1000    // Time between data transmissions (1s)

// Connection state machine
enum ConnectionState {
  SCANNING,
  CONNECTING,
  CONNECTED,
  RECONNECTING
};

ConnectionState currentState = SCANNING;

// Status callback for client connections
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    connected = true;
    Serial.println("Connected to server");
  }
  
  void onDisconnect(BLEClient* pClient) {
    connected = false;
    Serial.println("Disconnected from server");
    currentState = RECONNECTING;
    lastConnectionAttempt = millis();
  }
};

// Function to connect to the BLE server
bool connectToServer() {
  if (serverAddress == NULL) {
    Serial.println("Server address not set, cannot connect");
    return false;
  }
  
  if (connected) {
    Serial.println("Already connected");
    return true;
  }
  
  Serial.print("Connecting to: ");
  Serial.println(serverAddress->toString().c_str());
  
  // Create and connect client
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  
  // Connect to the remote BLE Server
  if (!pClient->connect(*serverAddress)) {
    Serial.println("Failed to connect to server");
    return false;
  }
  
  // Find service
  BLERemoteService* pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service: ");
    Serial.println(SERVICE_UUID);
    pClient->disconnect();
    return false;
  }

  // Find characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find characteristic: ");
    Serial.println(CHARACTERISTIC_UUID);
    pClient->disconnect();
    return false;
  }

  // Read txPower value from server if available
  if(pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    if(value.length() > 0) {
      // Only update if the value seems reasonable (between -100 and -30 dBm)
      int readTxPower = atoi(value.c_str());
      if (readTxPower >= -100 && readTxPower <= -30) {
        txPowerAt1m = readTxPower;
        Serial.print("Server txPower: ");
        Serial.println(txPowerAt1m);
      }
    }
  }
  
  connected = true;
  currentState = CONNECTED;
  return true;
}

// Function to send data to server (distance or error)
void sendDataToServer() {
  if (!connected || pRemoteCharacteristic == NULL) {
    Serial.println("Not connected, cannot send data");
    return;
  }
  
  char buf[40]; // Buffer for data or error messages
  
  // If measurement is valid, send distance
  if (measurementValid && lastDistance > 0) {
    sprintf(buf, "DIST:%.2f", lastDistance);
    consecutiveErrorCount = 0; // Reset error count on valid measurement
    Serial.printf("Sending valid distance: %.2f m\n", lastDistance);
  } 
  // If measurement is invalid, send error message
  else {
    consecutiveErrorCount++; // Increment error counter
    
    // Send error message after several consecutive errors
    if (consecutiveErrorCount >= MAX_CONSECUTIVE_ERRORS) {
      sprintf(buf, "ERROR:RSSI:%.1f", lastRssi);
      Serial.printf("Sending measurement error, RSSI: %.1f\n", lastRssi);
      consecutiveErrorCount = 0; // Reset after sending
    } else {
      // Skip sending for occasional errors
      Serial.printf("Measurement error (%d/%d) - skipping transmission\n", 
                   consecutiveErrorCount, MAX_CONSECUTIVE_ERRORS);
      return;
    }
  }
  
  pRemoteCharacteristic->writeValue(buf);
  Serial.printf("Sent to server: %s\n", buf);
}

// Function to calculate distance from RSSI
float calculateDistance(float rssi, float txPower) {
  // Validity checks
  if (rssi == 0 || rssi >= txPower) {
    return -1.0; // Error cases
  }
  
  // Distance calculation using the path loss model
  // distance = 10 ^ ((txPower - RSSI) / (10 * n))
  // where n is the path loss exponent
  float ratio = (txPower - rssi) / (10 * PATH_LOSS_EXPONENT);
  return pow(10, ratio);
}

// Callback for BLE scanning
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Skip if we're already connected or not the target device
    if (connected || advertisedDevice.getName() != TARGET_DEVICE_NAME)
      return;

    // Store the address and move to connecting state
    Serial.print("Found target device: ");
    Serial.println(advertisedDevice.toString().c_str());
    
    // If we already have an address, delete it to avoid memory leaks
    if (serverAddress != NULL) {
      delete serverAddress;
    }
    
    // Store the new address
    serverAddress = new BLEAddress(advertisedDevice.getAddress());
    currentState = CONNECTING;
    
    // Stop scanning to free up resources for the connection
    pBLEScan->stop();
  }
};

// Function to measure distance using RSSI
void performDistanceMeasurement() {
  // We need to be connected to the server to measure distance
  if (!connected) {
    return;
  }
  
  // Check if enough time has passed since last measurement
  unsigned long currentTime = millis();
  if (currentTime - lastMeasurementTime < MEASUREMENT_INTERVAL) {
    return;
  }
  
  // Get RSSI from the connection
  int rssi = pClient->getRssi();
  lastRssi = rssi;
  
  // Check if RSSI is within a reasonable range for valid measurements
  if (rssi < txPowerAt1m && rssi > -100) {
    // Calculate raw distance
    float rawDistance = calculateDistance(rssi, txPowerAt1m);
    lastRawDistance = rawDistance;
    
    // Check if distance calculation is valid
    if (rawDistance > 0) {
      // Apply calibration factor and limit maximum distance
      float calibratedDistance = rawDistance * CALIBRATION_FACTOR;
      
      // Limit to realistic values
      if (calibratedDistance > MAX_VALID_DISTANCE) {
        calibratedDistance = MAX_VALID_DISTANCE;
      }
      
      lastDistance = calibratedDistance;
      measurementValid = true;
      
      Serial.printf("RSSI: %d dBm, Raw dist: %.2f m, Calibrated: %.2f m\n", 
                  rssi, rawDistance, calibratedDistance);
    } 
    else {
      // Invalid distance calculation
      measurementValid = false;
      lastDistance = -1;
      
      Serial.printf("RSSI: %d dBm, Invalid distance calculation\n", rssi);
    }
  }
  else {
    // Invalid RSSI value
    measurementValid = false;
    lastDistance = -1;
    
    Serial.printf("RSSI: %d dBm, Invalid RSSI (should be < %d and > -100)\n", 
                rssi, txPowerAt1m);
  }
  
  // Update last measurement time
  lastMeasurementTime = currentTime;
  
  // Send data to server (if it's time to send)
  static unsigned long lastSendTime = 0;
  if (currentTime - lastSendTime >= DATA_SEND_INTERVAL) {
    sendDataToServer();
    lastSendTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Client with Persistent Connection...");
  
  // Setup LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize BLE
  BLEDevice::init("");
  
  // Create scanner
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(45);
  
  Serial.println("BLE Client ready. Scanning for target device...");
  Serial.printf("Target device: %s\n", TARGET_DEVICE_NAME);
  Serial.printf("txPower at 1m: %d dBm\n", txPowerAt1m);
  Serial.printf("Path loss exponent: %.1f\n", PATH_LOSS_EXPONENT);
  Serial.printf("Calibration factor: %.1f\n", CALIBRATION_FACTOR);
  
  // Set initial state
  currentState = SCANNING;
}

void loop() {
  unsigned long currentTime = millis();
  
  // State machine for connection management
  switch (currentState) {
    case SCANNING:
      // Start scanning for devices
      Serial.println("Scanning for devices...");
      pBLEScan->start(SCAN_TIME, false);
      delay(SCAN_INTERVAL);
      break;
      
    case CONNECTING:
      // Try to connect to the server
      if (connectToServer()) {
        Serial.println("Successfully connected to server");
        currentState = CONNECTED;
      } else {
        Serial.println("Failed to connect, will retry scanning");
        currentState = SCANNING;
      }
      break;
      
    case CONNECTED:
      // Perform regular distance measurements while connected
      performDistanceMeasurement();
      
      // Control LED based on proximity - only light up on valid measurements
      if (measurementValid && lastDistance > 0 && lastDistance < CLOSE_DISTANCE) {
        digitalWrite(LED_PIN, HIGH);  // Turn on LED when close
      } else {
        digitalWrite(LED_PIN, LOW);   // Turn off LED when far or no measurement
      }
      
      // If connection was lost, go to reconnecting state
      if (!connected) {
        currentState = RECONNECTING;
        lastConnectionAttempt = currentTime;
      }
      
      delay(100); // Small delay to prevent excessive CPU usage
      break;
      
    case RECONNECTING:
      // Flash LED to indicate reconnecting state
      if ((currentTime % 500) < 250) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      
      // Wait for reconnect interval before attempting
      if (currentTime - lastConnectionAttempt >= RECONNECT_INTERVAL) {
        Serial.println("Attempting to reconnect...");
        if (connectToServer()) {
          currentState = CONNECTED;
        } else {
          // If reconnection fails, go back to scanning
          currentState = SCANNING;
        }
      }
      
      delay(100);
      break;
  }
}