#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32_NOW.h>
#include <LittleFS.h>
#include <FTPServer.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

const char* ssid     = "R2D2";
const char* password = "Sky7388500";

// ─── GOOGLE DEPLOYMENT ID ────────────────────────────────────────────────────
const String googleDeploymentID = "AKfycbwXzMgSQXMz96mz-a7ZeN6b3Yk_IfjqBgnnPcxp3zILSZLA0XPDDVF1FjewD6i-t4rd";
const String googleURL          = "https://script.google.com/macros/s/" + googleDeploymentID + "/exec";
// ─────────────────────────────────────────────────────────────────────────────

// ─── Message / Packet Structures (identical on all boards) ───────────────────
enum MessageType : uint8_t {
  MSG_BME280       = 0,
  MSG_ALERT_FLAG   = 1,
  MSG_BLOWER_STATE = 2
};

struct __attribute__((packed)) BlowerData {
  MessageType type;
  bool on;
};

struct __attribute__((packed)) BmeData {
  MessageType type;
  float temperature;
  float humidity;
  float pressure;
};

struct SensorRegisters {
  float insideTemp;
  float lastEventMinutes;
  float dailyTotalMinutes;
};
SensorRegisters sensordata;

// ─── Peripherals ─────────────────────────────────────────────────────────────
FTPServer ftpSrv(LittleFS);
WebServer server(80);

// ─── ESP-NOW Peer Class (Core 3.x compliant) ─────────────────────────────────
class HeatingSenderPeer : public ESP_NOW_Peer {
  public:
    HeatingSenderPeer(const uint8_t *mac_addr, uint8_t channel,
                      wifi_interface_t iface = WIFI_IF_STA,
                      const uint8_t *lmk = NULL)
      : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

    bool add_to_system() { return ESP_NOW_Peer::add(); }

  protected:
    void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
      extern void processIncomingPacket(const uint8_t *data, int len);
      processIncomingPacket(data, (int)len);
    }
};

// ─── Hardware MAC Map ─────────────────────────────────────────────────────────
uint8_t senderBlowerMAC[] = { 0xE4, 0x65, 0xB8, 0x20, 0x20, 0xA0 };
uint8_t senderBmeMAC[]    = { 0xE4, 0x65, 0xB8, 0x25, 0x42, 0xF8 };
#define CHANNEL 11

HeatingSenderPeer blowerNode(senderBlowerMAC, CHANNEL, WIFI_IF_STA);
HeatingSenderPeer bmeNode(senderBmeMAC,       CHANNEL, WIFI_IF_STA);

// ─── Global Tracking Registers ───────────────────────────────────────────────
bool blowerOn          = false;
bool blowerIsOn        = false;
bool lastBlowerState   = false;
bool alertFlag         = false;
bool oneSecondElapsed  = false;

double secondsCounter    = 0.0;
double elapsedMinutes    = 0.0;
double dailyTotalMinutes = 0.0;
double lastEventMinutes  = 0.0;

int HOUR   = 0;
int MINUTE = 0;
int SECOND = 0;

float globalTemp         = 0.0;
float globalHumidity     = 0.0;
float globalPressure     = 0.0;
float thermostatSetpoint = 72.0;
String dtStamp           = "00/00/0000 00:00:00";

// ─── Forward Declarations ─────────────────────────────────────────────────────
void processIncomingPacket(const uint8_t *data, int len);
void sendGoogleSheetsData();
void sendDataToServer(String updateTime, float outT, float inT, float regT,
                      float therm, double eventM, double dailyM);
void updateHeatingData();
void displayData();
void logData();
void disabledailyCooling();
void resetDailyStats();
void temperatureInterrupt();
void getDateTime();
String urlEncode(String str);

// ─── URL Encoder ─────────────────────────────────────────────────────────────
String urlEncode(String str) {
  String encoded = "";
  for (unsigned int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (isalnum(c)) {
      encoded += c;
    } else if (c == ' ') {
      encoded += '+';
    } else {
      char code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) code1 = (c & 0xf) - 10 + 'A';
      char c2 = (c >> 4) & 0xf;
      char code0 = c2 + '0';
      if (c2 > 9) code0 = c2 - 10 + 'A';
      encoded += '%';
      encoded += code0;
      encoded += code1;
    }
  }
  return encoded;
}

// ─── Packet Processing Pipeline ──────────────────────────────────────────────
void processIncomingPacket(const uint8_t *data, int len) {
  if (len < 1) return;

  MessageType incomingType = (MessageType)data[0];

  switch (incomingType) {

    case MSG_BLOWER_STATE:
      if (len == sizeof(BlowerData)) {
        BlowerData blowerPacket;
        memcpy(&blowerPacket, data, sizeof(BlowerData));
        blowerOn   = blowerPacket.on;
        blowerIsOn = blowerOn;
        Serial.printf("\n[Radio Link] Blower Update Caught -> State: %s\n",
                      blowerOn ? "ON" : "OFF");
        // Flag only — loop() handles HTTP safely from main task
        alertFlag = true;
      } else {
        Serial.printf("\n⚠️ Size Mismatch for Blower Packet: Expected %d, got %d bytes\n",
                      sizeof(BlowerData), len);
      }
      break;

    case MSG_BME280:
      if (len == sizeof(BmeData)) {
        BmeData bmePacket;
        memcpy(&bmePacket, data, sizeof(BmeData));
        globalTemp     = bmePacket.temperature;
        globalHumidity = bmePacket.humidity;
        globalPressure = bmePacket.pressure;
        Serial.printf("\n[Radio Link] BME280 Update Caught -> Temp: %.2f F, Hum: %.1f%%, Pres: %.2f inHg\n",
                      globalTemp, globalHumidity, globalPressure);
      } else {
        Serial.printf("\n⚠️ Size Mismatch for BME280 Packet: Expected %d, got %d bytes\n",
                      sizeof(BmeData), len);
      }
      break;

    default:
      Serial.printf("\n⚠️ Unknown Data Header Byte: %d\n", data[0]);
      break;
  }
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Heating Monitor - Central Dual Receiver");

  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Infrastructure Connected.");

  if (!ESP_NOW.begin()) {
    Serial.println("ESP-NOW Engine Failed to Start");
    return;
  }

  if (!blowerNode.add_to_system()) Serial.println("Failed to bind Furnace Blower Node");
  if (!bmeNode.add_to_system())    Serial.println("Failed to bind BME280 Environmental Node");

  Serial.println("Receiver listening for Blower and BME280... Setup complete.");
}

// ─── Loop State Machine ───────────────────────────────────────────────────────
void loop() {
  ftpSrv.handleFTP();

  // ── Gatekeeper: alertFlag handler ────────────────────────────────────────
  if (alertFlag) {
    alertFlag = false;          // Clear first to prevent re-entry
    getDateTime();
    sendGoogleSheetsData();
    logData();
    displayData();
  }

  // ── One-second blower run-time accumulator ────────────────────────────────
  if (oneSecondElapsed) {
    if (blowerIsOn) {
      secondsCounter++;
      elapsedMinutes = secondsCounter / 60.0;
    }
    oneSecondElapsed = false;
  }

  // ── Blower OFF edge detection ─────────────────────────────────────────────
  if (lastBlowerState && !blowerIsOn) {
    lastEventMinutes  = elapsedMinutes;
    dailyTotalMinutes += lastEventMinutes;
    Serial.printf("Blower turned OFF. Added %.2f minutes. Daily total: %.2f\n",
                  lastEventMinutes, dailyTotalMinutes);
    updateHeatingData();
    secondsCounter = 0;
  }
  lastBlowerState = blowerIsOn;

  // ── Midnight daily reset ──────────────────────────────────────────────────
  if (HOUR == 0 && MINUTE == 0 && SECOND == 0) {
    dailyTotalMinutes = 0;
  }
}

// ─── Google Sheets Bridge ─────────────────────────────────────────────────────
void sendGoogleSheetsData() {
  sendDataToServer(dtStamp, globalTemp, sensordata.insideTemp, 0.0,
                   thermostatSetpoint, elapsedMinutes, dailyTotalMinutes);
}

void sendDataToServer(String updateTime, float outT, float inT, float regT,
                      float therm, double eventM, double dailyM) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Sheet update blocked: Wi-Fi offline.");
    return;
  }

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;

  String data = "?lastUpdate="        + urlEncode(updateTime)
              + "&outsideTemp="       + String(outT, 2)
              + "&insideTemp="        + String(inT, 2)
              + "&registerTemp="      + String(regT, 2)
              + "&thermostat="        + String(therm, 2)
              + "&elapsedMinutes="    + String(eventM, 2)
              + "&dailyTotalMinutes=" + String(dailyM, 2);

  String urlFinal = googleURL + data;

  Serial.println("\n[HTTP] Connection pipeline opened...");
  Serial.println("[HTTP] Target: " + urlFinal);

  if (http.begin(secureClient, urlFinal)) {
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.setTimeout(10000);

    int httpCode = http.GET();
    Serial.printf("[HTTP] Status Code: %d\n", httpCode);

    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println("[HTTP] Response: " + payload);
    } else {
      Serial.printf("[HTTP] Error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.println("⚠️ Secure connection failed to initialize.");
  }
}

// ─── Local File Logging ───────────────────────────────────────────────────────
void logData() {
  if (!LittleFS.exists("/log.txt")) {
    File setupFile = LittleFS.open("/log.txt", FILE_WRITE);
    if (!setupFile) return;
    setupFile.println("Timestamp, OutsideTemp, InsideTemp, Setpoint, EventDuration, TotalRuntime");
    setupFile.close();
  }
  File appendLog = LittleFS.open("/log.txt", FILE_APPEND);
  if (!appendLog) return;
  appendLog.print(dtStamp);
  appendLog.print(" , Outside: ");   appendLog.print(globalTemp, 2);
  appendLog.print(" F, Inside: ");   appendLog.print(sensordata.insideTemp, 2);
  appendLog.print(" F, Thermostat: "); appendLog.print(thermostatSetpoint, 2);
  appendLog.print(" F, Event: ");    appendLog.print(sensordata.lastEventMinutes, 2);
  appendLog.print(", Total 24HR: "); appendLog.print(sensordata.dailyTotalMinutes, 2);
  appendLog.print("\n");
  appendLog.close();
}

// ─── Stubs ────────────────────────────────────────────────────────────────────
void updateHeatingData()    {}
void displayData()          {}
void disabledailyCooling()  {}
void resetDailyStats()      {}
void temperatureInterrupt() {}

void getDateTime() {
  dtStamp = "06/07/2026 01:45:00";   // Replace with NTP call when ready
}