/* Heating System Monitor III
   ESP_NOW_Receeeiver.ino with temperature Offset
   June 12,2026
   ESP32_-NOW,  ESP32 Core 3.3.10 
   Receives MSG_BLOWER_STATE from ESP_NOW_Blower node
   Receives MSG_BME280 from ESP_NOW_BME280 node
   Sends MSG_ALERT_FLAG to BME280 node to request outside temp
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <ESP32_NOW.h>
#include <LittleFS.h>
#include <FTPServer.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Ticker.h>

const char* ssid     = "ssid";
const char* password = "password";

// ─── GOOGLE DEPLOYMENT ID ────────────────────────────────────────────────────
const String googleDeploymentID = "removed for security";
const String googleURL          = "https://script.google.com/macros/s/" + googleDeploymentID + "/exec";

// ─── MCP9808 Direct Wire Register Definitions ────────────────────────────────
#define MCP9808_ADDR      0x18
#define MCP9808_REG_CFG   0x01
#define MCP9808_REG_UPPER 0x02
#define MCP9808_REG_LOWER 0x03
#define MCP9808_REG_CRIT  0x04
#define MCP9808_REG_AMB   0x05

// ─────────────────────────────────────────────
// MCP9808 Temperature Calibration
// Reference: Pak HOLD DMM HP-770HD reads 76.00 °F
// MCP9808 raw reading: 82.29 °F  (same location)
// Offset applied to MCP9808 temperature output
// ─────────────────────────────────────────────
const float MCP9808_TEMP_CAL_OFFSET_F = -6.29;

const float LOW_TEMP_F  = 65.0;
const float HIGH_TEMP_F = 74.0;
const float LOW_TEMP_C  = (LOW_TEMP_F  - 32.0) / 1.8;
const float HIGH_TEMP_C = (HIGH_TEMP_F - 32.0) / 1.8;

// ─── NTP / Time ──────────────────────────────────────────────────────────────
const char* udpAddress1 = "pool.ntp.org";
const char* udpAddress2 = "time.nist.gov";

int DOW, MONTH, DATE, YEAR, HOUR, MINUTE, SECOND;
String weekDay;
char strftime_buf[64];
String dtStamp    = "";
String lastUpdate = "";
time_t tnow;

// ─── Ticker / One-Second ISR ─────────────────────────────────────────────────
Ticker secondTicker;
volatile bool oneSecondElapsed = false;

void IRAM_ATTR countSecondsISR() {
  oneSecondElapsed = true;
}

// ─── Message / Packet Structures ─────────────────────────────────────────────
enum MessageType : uint8_t {
  MSG_BME280       = 0,
  MSG_ALERT_FLAG   = 1,
  MSG_BLOWER_STATE = 2
};

struct __attribute__((packed)) BlowerData {
  MessageType type;
  bool        on;
  float       elapsedMinutes;
  float       dailyTotalMinutes;
};

struct __attribute__((packed)) BME280Data {
  MessageType type;
  float temperature;
  float humidity;
  float pressure;
};

struct __attribute__((packed)) AlertFlag {
  MessageType type;
  bool alert;
};

struct SensorRegisters {
  float outsideTemp;
  float insideTemp;
  float registerTemp;
  float thermostat;
  float lastEventMinutes;
  float dailyTotalMinutes;
  float lastRecordedMinute;
};
SensorRegisters sensordata;

// ─── Peripherals ─────────────────────────────────────────────────────────────
FTPServer ftpSrv(LittleFS);
WebServer server(80);

// ─── ESP-NOW Peer Classes ─────────────────────────────────────────────────────

// Forward declaration required by peer classes
void processIncomingPacket(const uint8_t *data, int len);

// Blower node — receive only
class HeatingSenderPeer : public ESP_NOW_Peer {
  public:
    HeatingSenderPeer(const uint8_t *mac_addr, uint8_t channel,
                      wifi_interface_t iface = WIFI_IF_STA,
                      const uint8_t *lmk = NULL)
      : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

    bool add_to_system() { return ESP_NOW_Peer::add(); }

  protected:
    void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
      processIncomingPacket(data, (int)len);
    }
};

// BME280 node — bidirectional
class BME280Peer : public ESP_NOW_Peer {
  public:
    BME280Peer(const uint8_t *mac_addr, uint8_t channel,
               wifi_interface_t iface = WIFI_IF_STA,
               const uint8_t *lmk = NULL)
      : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

    bool add_to_system() { return ESP_NOW_Peer::add(); }

    bool sendAlert(bool alertState) {
      AlertFlag pkt;
      pkt.type  = MSG_ALERT_FLAG;
      pkt.alert = alertState;
      return send((uint8_t *)&pkt, sizeof(AlertFlag));
    }

  protected:
    void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
      processIncomingPacket(data, (int)len);
    }
};

// ─── Hardware MAC Map ─────────────────────────────────────────────────────────
uint8_t senderBlowerMAC[] = { 0xE4, 0x65, 0xB8, 0x24, 0xF6, 0x4C };
uint8_t senderBmeMAC[]    = { 0xE4, 0x65, 0xB8, 0x25, 0x42, 0xF8 };
#define CHANNEL 0

HeatingSenderPeer blowerNode(senderBlowerMAC, CHANNEL, WIFI_IF_STA);
BME280Peer        bmeNode(senderBmeMAC,       CHANNEL, WIFI_IF_STA);

// ─── Global Tracking Registers ───────────────────────────────────────────────
bool blowerIsOn        = false;
bool alertFlag         = false;
bool googleSheetsSent  = false;

double elapsedMinutes    = 0.0;
double dailyTotalMinutes = 0.0;

float globalTemp         = 0.0;
float globalHumidity     = 0.0;
float globalPressure     = 0.0;
float thermostatSetpoint = 72.0;

// ─── Forward Declarations ─────────────────────────────────────────────────────
void sendGoogleSheetsData();
void sendDataToServer(String updateTime, float outT, float inT, float regT,
                      float therm, double eventM, double dailyM);
void updateHeatingData();
void displayData();
void logData();
void disabledailyCooling();
void resetDailyStats();
void temperatureInterrupt();
void initNTP();
String getDateTime();
void initMCP9808();
float readMCP9808();
String urlEncode(String str);

// ─── NTP Init ────────────────────────────────────────────────────────────────
void initNTP() {
  configTime(0, 0, udpAddress1, udpAddress2);
  setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 2);
  tzset();
  Serial.print("Waiting for first valid NTP timestamp");
  while (time(nullptr) < 100000ul) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("\nNTP Synchronized.");
}

// ─── Date/Time Stamp ─────────────────────────────────────────────────────────
String getDateTime() {
  tnow = time(nullptr) + 1;
  struct tm *ti = localtime(&tnow);
  DOW    = ti->tm_wday;
  YEAR   = ti->tm_year + 1900;
  MONTH  = ti->tm_mon + 1;
  DATE   = ti->tm_mday;
  HOUR   = ti->tm_hour;
  MINUTE = ti->tm_min;
  SECOND = ti->tm_sec;
  strftime(strftime_buf, sizeof(strftime_buf), "%m/%d/%Y %H:%M:%S", localtime(&tnow));
  dtStamp = strftime_buf;
  return dtStamp;
}

// ─── MCP9808 ─────────────────────────────────────────────────────────────────
void initMCP9808() {
  Wire.beginTransmission(MCP9808_ADDR);
  Wire.write(MCP9808_REG_CFG);
  Wire.write(0x00);
  Wire.write(0x08);
  Wire.endTransmission();

  int16_t upper = (int16_t)(HIGH_TEMP_C * 4) << 2;
  Wire.beginTransmission(MCP9808_ADDR);
  Wire.write(MCP9808_REG_UPPER);
  Wire.write((uint8_t)(upper >> 8));
  Wire.write((uint8_t)(upper & 0xFF));
  Wire.endTransmission();

  int16_t lower = (int16_t)(LOW_TEMP_C * 4) << 2;
  Wire.beginTransmission(MCP9808_ADDR);
  Wire.write(MCP9808_REG_LOWER);
  Wire.write((uint8_t)(lower >> 8));
  Wire.write((uint8_t)(lower & 0xFF));
  Wire.endTransmission();

  int16_t crit = (int16_t)(23.0 * 4) << 2;
  Wire.beginTransmission(MCP9808_ADDR);
  Wire.write(MCP9808_REG_CRIT);
  Wire.write((uint8_t)(crit >> 8));
  Wire.write((uint8_t)(crit & 0xFF));
  Wire.endTransmission();

  Serial.printf("MCP9808 Init: LOW=%.2f°C  HIGH=%.2f°C\n", LOW_TEMP_C, HIGH_TEMP_C);
}

float readMCP9808() {
  Wire.beginTransmission(MCP9808_ADDR);
  Wire.write(MCP9808_REG_AMB);
  Wire.endTransmission();
  Wire.requestFrom(MCP9808_ADDR, 2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  bool negative = (msb & 0x10);
  msb &= 0x0F;
  int16_t raw = ((int16_t)msb << 8) | lsb;
  if (negative) raw -= 4096;
  float tempC = raw / 16.0;
  float tempF = (tempC * 9.0 / 5.0) + 32.0;

  // Apply calibration offset — reference: HP-770HD DMM thermometer
  tempF += MCP9808_TEMP_CAL_OFFSET_F;

  return tempF;
}

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
  Serial.printf(">>> processIncomingPacket: len=%d  type=%d  sizeof(BlowerData)=%d\n",
                len, data[0], sizeof(BlowerData));
  if (len < 1) return;

  MessageType incomingType = (MessageType)data[0];

  switch (incomingType) {

    case MSG_BLOWER_STATE:
      if (len == sizeof(BlowerData)) {
        BlowerData blowerPacket;
        memcpy(&blowerPacket, data, sizeof(BlowerData));
        blowerIsOn = blowerPacket.on;
        Serial.printf("\n[Radio Link] Blower Update Caught -> State: %s  Elapsed: %.2f min  Daily: %.2f min\n",
                      blowerIsOn ? "ON" : "OFF",
                      blowerPacket.elapsedMinutes,
                      blowerPacket.dailyTotalMinutes);
        elapsedMinutes               = blowerPacket.elapsedMinutes;
        dailyTotalMinutes            = blowerPacket.dailyTotalMinutes;
        sensordata.lastEventMinutes  = blowerPacket.elapsedMinutes;
        sensordata.dailyTotalMinutes = blowerPacket.dailyTotalMinutes;
        alertFlag = true;
      } else {
        Serial.printf("\n⚠️ Size Mismatch — BlowerData: Expected %d got %d bytes\n",
                      sizeof(BlowerData), len);
      }
      break;

    case MSG_ALERT_FLAG:
      if (len == sizeof(AlertFlag)) {
        AlertFlag alertPacket;
        memcpy(&alertPacket, data, sizeof(AlertFlag));
        Serial.printf("\n[Radio Link] AlertFlag received: %s\n",
                      alertPacket.alert ? "true" : "false");
        // alertFlag already set by MSG_BLOWER_STATE — no action needed here
      }
      break;

    case MSG_BME280:
      if (len == sizeof(BME280Data)) {
        BME280Data bmePacket;
        memcpy(&bmePacket, data, sizeof(BME280Data));
        globalTemp     = bmePacket.temperature;
        globalHumidity = bmePacket.humidity;
        globalPressure = bmePacket.pressure;
        Serial.printf("\n[Radio Link] BME280 Update -> Temp: %.2f F  Hum: %.1f%%  Pres: %.2f hPa\n",
                      globalTemp, globalHumidity, globalPressure);
      } else {
        Serial.printf("\n⚠️ Size Mismatch — BmeData: Expected %d got %d bytes\n",
                      sizeof(BME280Data), len);
      }
      break;

    default:
      Serial.printf("\n⚠️ Unknown packet type: %d\n", data[0]);
      break;
  }
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\nHeating System Monitor III - ESP_NOW_Receiver.ino\n");

  Wire.begin();

  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Infrastructure Connected.");
  Serial.println("Receiver MAC: " + WiFi.macAddress());
  Serial.printf("WiFi Channel: %d\n", WiFi.channel());

  initNTP();
  initMCP9808();

  bool fsok = LittleFS.begin(true);
  Serial.printf("FS init: %s\n", fsok ? "ok" : "fail!");

  ftpSrv.begin(F("admin"), F("admin"));

  secondTicker.attach(1, countSecondsISR);

  // ── ESP-NOW init — once only ──────────────────────────────────────────────
  if (!ESP_NOW.begin()) {
    Serial.println("ESP-NOW Engine Failed to Start");
    return;
  }
  Serial.println("ESP-NOW init OK");
  Serial.printf("sizeof(BlowerData)=%d  sizeof(AlertFlag)=%d  sizeof(BME280Data)=%d\n",
                sizeof(BlowerData), sizeof(AlertFlag), sizeof(BME280Data));

  if (!blowerNode.add_to_system()) {
    Serial.println("Failed to bind Blower Node");
  } else {
    Serial.println("Blower node bound OK");
  }

  if (!bmeNode.add_to_system()) {
    Serial.println("Failed to bind BME280 Node");
  } else {
    Serial.println("BME280 node bound OK");
  }

  Serial.println("Receiver listening... Setup complete.");
}

// ─── Loop State Machine ───────────────────────────────────────────────────────
void loop() {
  ftpSrv.handleFTP();

  // ── Gatekeeper: alertFlag handler ────────────────────────────────────────
  if (alertFlag) {
    alertFlag = false;

    Serial.println("[ESP-NOW] Sending alert to BME280 node...");
    bool sent = bmeNode.sendAlert(true);
    Serial.printf("[ESP-NOW] Alert send: %s\n", sent ? "OK" : "FAILED");
    delay(1000);

    float mcpTempF = readMCP9808();
    sensordata.insideTemp   = mcpTempF;
    sensordata.registerTemp = mcpTempF;
    sensordata.outsideTemp  = globalTemp;
    sensordata.thermostat   = thermostatSetpoint;

    Serial.printf("[MCP9808] Inside/Register Temp: %.2f °F\n", mcpTempF);

    getDateTime();
    displayData();
    logData();
    sendGoogleSheetsData();
    googleSheetsSent = true;
  }

  // ── Reset after pipeline ──────────────────────────────────────────────────
  if (googleSheetsSent) {
    elapsedMinutes   = 0;
    googleSheetsSent = false;
  }

  // ── One-second — midnight reset ───────────────────────────────────────────
  if (oneSecondElapsed) {
    oneSecondElapsed = false;

    static bool didMidnightReset = false;
    if (HOUR == 0 && MINUTE == 0 && SECOND == 0) {
      if (!didMidnightReset) {
        dailyTotalMinutes = 0;
        Serial.println("Midnight reset: dailyTotalMinutes = 0");
        didMidnightReset = true;
      }
    } else {
      didMidnightReset = false;
    }
  }
}

// ─── Google Sheets Bridge ─────────────────────────────────────────────────────
void sendGoogleSheetsData() {
  sendDataToServer(dtStamp,
                   sensordata.outsideTemp,
                   sensordata.insideTemp,
                   sensordata.registerTemp,
                   thermostatSetpoint,
                   elapsedMinutes,
                   dailyTotalMinutes);
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
  Serial.println("\n[HTTP] Target: " + urlFinal);

  if (http.begin(secureClient, urlFinal)) {
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.setTimeout(10000);
    int httpCode = http.GET();
    Serial.printf("[HTTP] Status Code: %d\n", httpCode);
    if (httpCode > 0) {
      Serial.println("[HTTP] Response: " + http.getString());
    } else {
      Serial.printf("[HTTP] Error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  } else {
    Serial.println("⚠️ Secure connection failed.");
  }
}

// ─── Local File Logging ───────────────────────────────────────────────────────
void logData() {
  if (!LittleFS.exists("/log.txt")) {
    File setupFile = LittleFS.open("/log.txt", FILE_WRITE);
    if (!setupFile) return;
    setupFile.println("Timestamp, OutsideTemp, InsideTemp, RegisterTemp, Setpoint, EventDuration, TotalRuntime");
    setupFile.close();
  }
  File appendLog = LittleFS.open("/log.txt", FILE_APPEND);
  if (!appendLog) return;
  appendLog.print(dtStamp);
  appendLog.print(" , Outside: ");     appendLog.print(sensordata.outsideTemp, 2);
  appendLog.print(" F, Inside: ");     appendLog.print(sensordata.insideTemp, 2);
  appendLog.print(" F, Register: ");   appendLog.print(sensordata.registerTemp, 2);
  appendLog.print(" F, Thermostat: "); appendLog.print(thermostatSetpoint, 2);
  appendLog.print(" F, Event: ");      appendLog.print(sensordata.lastEventMinutes, 2);
  appendLog.print(", Total 24HR: ");   appendLog.print(sensordata.dailyTotalMinutes, 2);
  appendLog.print("\n");
  appendLog.close();
}

// ─── Stubs ────────────────────────────────────────────────────────────────────
void updateHeatingData()    {}
void displayData()          {}
void disabledailyCooling()  {}
void resetDailyStats()      {}
void temperatureInterrupt() {}
