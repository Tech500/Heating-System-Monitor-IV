/* Heating System Monitor III 
   ESP_NOW_BME280.ino   
   June 12, 2026 
   ESP Now, Verified latest Arduino Core 3.3.10 
*/


#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <WiFi.h>
#include <ESP32_NOW.h>    // Production Core 3.3.10 Architecture Header
#include <WiFiUdp.h>

// WiFi credentials
const char *ssid = "ssid";
const char *password = "password";

volatile bool alertFlag = false;

BME280I2C bme;

// Master MAC address
uint8_t masterAddress[] = { 0x3C, 0xE9, 0x0E, 0x84, 0xEE, 0x80 };

#define CHANNEL 11

String macAddr = WiFi.macAddress();
#define SEALEVELPRESSURE_HPA (1013.25)

enum MessageType : uint8_t {
  MSG_BME280 = 0,
  MSG_ALERT_FLAG,
};

struct __attribute__((packed)) BME280Data {
  MessageType type = MSG_BME280;
  float temp = 0.0;
  float hum = 0.0;
  float pres = 0.0;
};

BME280Data localReadings;
BME280Data receivedReadings;
BME280Data sensorData;

struct __attribute__((packed)) AlertFlag {
  MessageType type;
  bool alert;
};

struct RtcData {
  char timestamp[20];
  int cycleCount;
};
RtcData rtcData;

volatile bool interruptFlag = false;
const int interruptPin = 2;  

// ─────────────────────────────────────────────
// CORE 3.3.10 CUSTOM PEER CLASS DEFINITION
// ─────────────────────────────────────────────
class Custom_Master_Peer : public ESP_NOW_Peer {
public:
  Custom_Master_Peer(const uint8_t *mac_addr, wifi_interface_t iface = WIFI_IF_STA)
    : ESP_NOW_Peer(mac_addr, CHANNEL, iface, NULL) {}

  ~Custom_Master_Peer() {}

  bool begin() {
    if (!add()) {
      Serial.println("Failed to register peer within hardware configuration table.");
      return false;
    }
    return true;
  }

  // Expose transmission functionality safely to the outer sketch context
  bool sendMessage(const uint8_t *data, size_t len) {
    return send(data, len); 
  }

  // 🌟 NATIVE VIRTUAL RECEPTION METHOD - EXTACT MATCH TO GENERATED CANDIDATE SIGNATURE
  void onReceive(const uint8_t *incomingData, size_t len, bool broadcast) {
    if (len <= 0 || incomingData == NULL) return;

    MessageType msgType = *(MessageType *)incomingData;
    switch (msgType) {
      case MSG_ALERT_FLAG:
        if (len == sizeof(AlertFlag)) {
          AlertFlag receivedAlert;
          memcpy(&receivedAlert, incomingData, sizeof(AlertFlag));
          alertFlag = receivedAlert.alert;
          Serial.print("\nReceived Alert Flag via Peer: ");
          Serial.println(alertFlag);
        }
        break;

      case MSG_BME280:
        if (len == sizeof(BME280Data)) {
          BME280Data received;
          memcpy(&received, incomingData, sizeof(BME280Data));
          receivedReadings = received;
        }
        break;
    }
  }
};

// Instantiation of the global tracking peer object
Custom_Master_Peer master_peer(masterAddress, WIFI_IF_STA);

// ─────────────────────────────────────────────
// BME280 Sensor Data Transmission Engine
// ─────────────────────────────────────────────
void BME280();
void updateDisplay();
void setup();
void loop();
void BME280() {

float temp = NAN, hum = NAN, pres = NAN;

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  delay(500);

  if (isnan(temp) || isnan(hum) || isnan(pres)) {
    Serial.println("Error reading sensor telemetry from BME280 hardware.");
    return;
  }

  sensorData.type = MSG_BME280;
  sensorData.temp = temp;
  sensorData.hum = hum;
  sensorData.pres = pres;

  Serial.println("\n--- Local BME280 Sensor Telemetry ---");
  Serial.printf("Temp: %.2f °F | Hum: %.2f %% | Pres: %.2f hPa\n", temp, hum, pres);

  // Transfer packet to network pipeline via custom peer interface
  bool result = master_peer.sendMessage((uint8_t *)&sensorData, sizeof(BME280Data));
  if (result) {
    Serial.println("BME280 data successfully staged for transmission.");
  } else {
    Serial.println("Error staging network package.");
  }
}

void updateDisplay() {
  Serial.println("[Telemetry Display Update Complete]");
}

// ─────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial) {};

  Serial.println("\nHeating System Monitor - Sender Core 3.3.10 Production\n");

  WiFi.mode(WIFI_MODE_APSTA); // Required for simultaneous operation
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi infrastructure link established.");

  // Initialize the low-level systemic ESP-NOW layer
  if (!ESP_NOW.begin()) {
    Serial.println("Error starting system-level ESP-NOW engine.");
    return;
  }

  // Execute peer binding setup
  if (!master_peer.begin()) {
    Serial.println("Failed to bind target master device connection parameters.");
    return;
  }
  Serial.println("Master Target configuration active.");

  Wire.begin(21, 22);
  while (!bme.begin()) {
    Serial.println("Could not establish communication with local BME280 sensor.");
    delay(1000);
  }
}

// ─────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────
void loop() {
  if (alertFlag == true) {
    BME280();
    updateDisplay();
    delay(1000);    
    alertFlag = false; 
  }
}
