/* Heating System Monitor III
   ESP_NOW_Blower.ino
   June 12, 2026
   ESP-NOW, Verified ESP32 Core 3.3.10 
   KY-038 Raw ADC Variance Detection
   Calibrated: OFF variance ~3.7-7.3 | ON variance ~300-500+ | Threshold: 20.0
   Elapsed time computed via NTP difftime() — no secondsCounter drift
   On OFF confirmation: sends MSG_BLOWER_STATE then MSG_ALERT_FLAG to receiver
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32_NOW.h>
#include <LittleFS.h>
#include <FTPServer.h>
#include <WebServer.h>
#include <time.h>

// ─────────────────────────────────────────────
// WiFi & FTP Credentials
// ─────────────────────────────────────────────
const char* ssid     = "ssid";
const char* password = "password";

FTPServer ftpSrv(LittleFS);
WebServer server(80);

// ─────────────────────────────────────────────
// NTP Configuration
// ─────────────────────────────────────────────
const char* udpAddress1 = "pool.ntp.org";
const char* udpAddress2 = "time.nist.gov";

int HOUR = 0, MINUTE = 0, SECOND = 0;
int DOW  = 0, MONTH  = 0, DATE   = 0, YEAR = 0;
char strftime_buf[64];
String dtStamp = "";
time_t tnow;

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

void initNTP() {
  configTime(0, 0, udpAddress1, udpAddress2);
  setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 2);
  tzset();
  Serial.print("Waiting for NTP sync");
  while (time(nullptr) < 100000ul) {
    Serial.print(".");
    delay(5000);
  }
  getDateTime();
  Serial.println("\nNTP Synchronized: " + dtStamp);
}

// ─────────────────────────────────────────────
// ESP-NOW Configuration
// CHANNEL 0 = match home channel dynamically
// ─────────────────────────────────────────────
uint8_t masterAddress[] = { 0x3C, 0xE9, 0x0E, 0x84, 0xEE, 0x80 };
#define CHANNEL 0

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

struct __attribute__((packed)) AlertFlag {
  MessageType type;
  bool        alert;
};

// ─────────────────────────────────────────────
// KY-038 Variance Detection Configuration
// ─────────────────────────────────────────────
const int   ANALOG_PIN      = 34;
const int   SAMPLE_COUNT    = 64;
const int   SAMPLE_DELAY_MS = 5;
const float VAR_THRESHOLD   = 20.0f;
const int   ON_CONFIRM      = 3;
const int   OFF_CONFIRM     = 90;

// ─────────────────────────────────────────────
// Global State Registers
// ─────────────────────────────────────────────
bool  blowerOn            = false;
int   consecutiveOnCount  = 0;
int   consecutiveOffCount = 0;
float currentVariance     = 0.0f;

// ─────────────────────────────────────────────
// NTP-based Elapsed Time Tracking
// ─────────────────────────────────────────────
time_t blowerStartTime   = 0;
double elapsedMinutes    = 0.0;
double dailyTotalMinutes = 0.0;

// ─────────────────────────────────────────────
// LittleFS Log Handling
// ─────────────────────────────────────────────
const char* LOG_FILE    = "/blower_log.csv";
const int   MAX_RECORDS = 2000;
int         recordCount  = 0;
bool        loggingActive = true;

// ─────────────────────────────────────────────
// Forward Declarations
// ─────────────────────────────────────────────
void   handleClear();
void   handleDownload();
void   handleStatus();
void   initLogging();
void   initWebServer();
void   logToFile(bool blowerState);
void   sendData(bool state);
void   sendAlert();
void   checkSerial();
float  computeVariance();
bool   detectBlower();

// ─────────────────────────────────────────────
// Variance Computation
// ─────────────────────────────────────────────
float computeVariance() {
  int   samples[SAMPLE_COUNT];
  float sum = 0.0f;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    samples[i] = analogRead(ANALOG_PIN);
    delay(SAMPLE_DELAY_MS);
    sum += samples[i];
  }
  float mean     = sum / SAMPLE_COUNT;
  float variance = 0.0f;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float diff = samples[i] - mean;
    variance  += diff * diff;
  }
  return variance / SAMPLE_COUNT;
}

// ─────────────────────────────────────────────
// ESP-NOW Peer Class
// onSent overridden inside class — Core 3.x correct approach
// ─────────────────────────────────────────────
class HeatingMasterPeer : public ESP_NOW_Peer {
public:
  HeatingMasterPeer(const uint8_t* mac_addr, uint8_t channel,
                    wifi_interface_t iface = WIFI_IF_STA,
                    const uint8_t* lmk = NULL)
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  bool   add_to_system() { return ESP_NOW_Peer::add(); }
  size_t send_packet(const uint8_t* data, int len) { return ESP_NOW_Peer::send(data, len); }

  void onSent(bool success) override {
    Serial.printf("Send Status: %s\n", success ? "Success" : "Fail");
  }
};

HeatingMasterPeer masterNode(masterAddress, CHANNEL);

// ─────────────────────────────────────────────
// Send blower state + timing to receiver
// ─────────────────────────────────────────────
void sendData(bool state) {
  BlowerData blower;
  blower.type              = MSG_BLOWER_STATE;
  blower.on                = state;
  blower.elapsedMinutes    = (float)elapsedMinutes;
  blower.dailyTotalMinutes = (float)dailyTotalMinutes;

  Serial.printf(">>> send_packet: state=%d  elapsed=%.2f  daily=%.2f  sizeof=%d\n",
                state, elapsedMinutes, dailyTotalMinutes, sizeof(BlowerData));

  size_t bytesSent = masterNode.send_packet((uint8_t*)&blower, sizeof(BlowerData));
  Serial.printf(">>> send_packet returned: %d\n", bytesSent);

  if (bytesSent > 0) {
    Serial.printf("MSG_BLOWER_STATE sent: %s  Elapsed: %.2f min  Daily: %.2f min\n",
                  state ? "ON" : "OFF", elapsedMinutes, dailyTotalMinutes);
  } else {
    Serial.println("MSG_BLOWER_STATE send failed");
  }
}

// ─────────────────────────────────────────────
// Send alert flag to trigger receiver gatekeeper
// ─────────────────────────────────────────────
void sendAlert() {
  AlertFlag alert;
  alert.type  = MSG_ALERT_FLAG;
  alert.alert = true;

  Serial.printf(">>> sendAlert: sizeof=%d\n", sizeof(AlertFlag));
  size_t bytesSent = masterNode.send_packet((uint8_t*)&alert, sizeof(AlertFlag));
  Serial.printf(">>> sendAlert returned: %d\n", bytesSent);

  if (bytesSent > 0) {
    Serial.println("MSG_ALERT_FLAG sent: true");
  } else {
    Serial.println("MSG_ALERT_FLAG send failed");
  }
}

// ─────────────────────────────────────────────
// Blower State Machine
// ─────────────────────────────────────────────
bool detectBlower() {
  currentVariance = computeVariance();

  if (!blowerOn) {
    if (currentVariance >= VAR_THRESHOLD) {
      consecutiveOnCount++;
      consecutiveOffCount = 0;
    } else {
      consecutiveOnCount = 0;
    }
    if (consecutiveOnCount >= ON_CONFIRM) {
      blowerOn           = true;
      consecutiveOnCount = 0;
      blowerStartTime    = time(nullptr);
      getDateTime();
      Serial.println(">>> Blower Detected: ON  @ " + dtStamp);
      sendData(true);
      delay(500);
    }
  } else {
    if (currentVariance < VAR_THRESHOLD) {
      consecutiveOffCount++;
      consecutiveOnCount = 0;
    } else {
      consecutiveOffCount = 0;
    }
    if (consecutiveOffCount >= OFF_CONFIRM) {
      blowerOn            = false;
      consecutiveOffCount = 0;

      time_t blowerStopTime = time(nullptr);
      elapsedMinutes        = difftime(blowerStopTime, blowerStartTime) / 60.0;
      dailyTotalMinutes    += elapsedMinutes;

      getDateTime();
      Serial.printf(">>> Blower Detected: OFF @ %s\n", dtStamp.c_str());
      Serial.printf("    Elapsed: %.2f min  Daily total: %.2f min\n",
                    elapsedMinutes, dailyTotalMinutes);

      sendData(false);
      delay(500);
      sendAlert();
      delay(500);
    }
  }
  return blowerOn;
}

// ─────────────────────────────────────────────
// LittleFS Logging
// ─────────────────────────────────────────────
void initLogging() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    loggingActive = false;
    return;
  }
  if (!LittleFS.exists(LOG_FILE)) {
    File f = LittleFS.open(LOG_FILE, FILE_WRITE);
    if (f) {
      f.println("Record,Timestamp,Variance,BlowerState,ElapsedMin,DailyTotalMin");
      f.close();
    } else {
      loggingActive = false;
    }
  } else {
    File f = LittleFS.open(LOG_FILE, "r");
    if (f) {
      while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.length() > 1) recordCount++;
      }
      f.close();
      recordCount--;
    }
  }
  Serial.printf("LittleFS OK — %d existing records\n", recordCount);
}

void logToFile(bool blowerState) {
  if (!loggingActive) return;
  if (recordCount >= MAX_RECORDS) {
    Serial.println("Log hit 2000 record limit — automatic rollover...");
    handleClear();
  }
  File f = LittleFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return;
  recordCount++;
  f.print(recordCount);                   f.print(",");
  f.print(dtStamp);                       f.print(",");
  f.print(currentVariance,           4);  f.print(",");
  f.print(blowerState ? "ON" : "OFF");    f.print(",");
  f.print((float)elapsedMinutes,     2);  f.print(",");
  f.println((float)dailyTotalMinutes, 2);
  f.close();
}

// ─────────────────────────────────────────────
// Serial Command Handler
// ─────────────────────────────────────────────
void checkSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  switch (cmd) {
    case 'r': {
      if (!LittleFS.exists(LOG_FILE)) { Serial.println("No log to rotate."); break; }
      int index = 1;
      String newName;
      while (true) {
        newName = "/blower_log_" + String(index) + ".csv";
        if (!LittleFS.exists(newName)) break;
        index++;
      }
      LittleFS.rename(LOG_FILE, newName);
      recordCount = 0; loggingActive = true;
      File f = LittleFS.open(LOG_FILE, FILE_WRITE);
      if (f) { f.println("Record,Timestamp,Variance,BlowerState,ElapsedMin,DailyTotalMin"); f.close(); }
      Serial.println("Log rotated to: " + newName);
      break;
    }
    case 'd': {
      int total = LittleFS.exists(LOG_FILE) ? 1 : 0;
      int index = 1;
      while (LittleFS.exists("/blower_log_" + String(index) + ".csv")) { total++; index++; }
      Serial.printf("Delete all %d log file(s)? (y/n): ", total);
      while (!Serial.available());
      char confirm = Serial.read();
      if (confirm == 'y' || confirm == 'Y') {
        if (LittleFS.exists(LOG_FILE)) { LittleFS.remove(LOG_FILE); Serial.println("Deleted: " + String(LOG_FILE)); }
        for (int i = 1; i < index; i++) {
          String name = "/blower_log_" + String(i) + ".csv";
          if (LittleFS.exists(name)) { LittleFS.remove(name); Serial.println("Deleted: " + name); }
        }
        recordCount = 0; loggingActive = true;
        File f = LittleFS.open(LOG_FILE, FILE_WRITE);
        if (f) { f.println("Record,Timestamp,Variance,BlowerState,ElapsedMin,DailyTotalMin"); f.close(); }
        Serial.println("All logs deleted. Fresh log ready.");
      } else {
        Serial.println("Delete cancelled.");
      }
      break;
    }
    case 'l': {
      Serial.println("--- Log Files ---");
      if (LittleFS.exists(LOG_FILE)) {
        File f = LittleFS.open(LOG_FILE, FILE_READ);
        Serial.printf("  blower_log.csv  (%d bytes)  %d records\n", f.size(), recordCount);
        f.close();
      } else {
        Serial.println("  blower_log.csv  (not found)");
      }
      int index = 1;
      while (true) {
        String name = "/blower_log_" + String(index) + ".csv";
        if (!LittleFS.exists(name)) break;
        File f = LittleFS.open(name, FILE_READ);
        Serial.printf("  blower_log_%d.csv  (%d bytes)\n", index, f.size());
        f.close();
        index++;
      }
      Serial.println("-----------------");
      break;
    }
    default:
      Serial.println("Commands: r=rotate | d=delete all | l=list");
      break;
  }
}

// ─────────────────────────────────────────────
// Web Server
// ─────────────────────────────────────────────
void handleDownload() {
  File f = LittleFS.open(LOG_FILE, FILE_READ);
  if (!f) { server.send(404, "text/plain", "Not found"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=blower_log.csv");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleClear() {
  LittleFS.remove(LOG_FILE);
  recordCount = 0; loggingActive = true;
  File f = LittleFS.open(LOG_FILE, FILE_WRITE);
  if (f) { f.println("Record,Timestamp,Variance,BlowerState,ElapsedMin,DailyTotalMin"); f.close(); }
  if (server.client()) server.send(200, "text/plain", "Log cleared.");
}

void handleStatus() {
  getDateTime();
  String msg  = "Time: "           + dtStamp;
  msg += "\nRecords: "             + String(recordCount) + " / 2000";
  msg += "\nLogging: "             + String(loggingActive ? "YES" : "NO");
  msg += "\nBlower: "              + String(blowerOn ? "ON" : "OFF");
  msg += "\nVariance: "            + String(currentVariance, 4);
  msg += "\nVAR_THRESHOLD: "       + String(VAR_THRESHOLD);
  msg += "\nON_CONFIRM: "          + String(ON_CONFIRM);
  msg += "\nOFF_CONFIRM: "         + String(OFF_CONFIRM);
  msg += "\nC_On: "                + String(consecutiveOnCount);
  msg += "\nC_Off: "               + String(consecutiveOffCount);
  msg += "\nElapsed min: "         + String(elapsedMinutes,    2);
  msg += "\nDaily total min: "     + String(dailyTotalMinutes, 2);
  server.send(200, "text/plain", msg);
}

void initWebServer() {
  server.on("/download", handleDownload);
  server.on("/clear",    handleClear);
  server.on("/status",   handleStatus);
  server.begin();
}

// ─────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n\n\nHeating System Monitor III, ESP_NOW_Blower.ino — ESP32 Core 3.3.10\n");
  Serial.printf("VAR_THRESHOLD=%.1f  ON_CONFIRM=%d  OFF_CONFIRM=%d\n",
                VAR_THRESHOLD, ON_CONFIRM, OFF_CONFIRM);
  Serial.println("Commands: r=rotate | d=delete all | l=list");

  initLogging();

  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  Serial.printf("Blower MAC: %s  Channel: %d\n",
                WiFi.macAddress().c_str(), WiFi.channel());

  initNTP();

  ftpSrv.begin(F("admin"), F("Sky7388"));
  initWebServer();

  if (!ESP_NOW.begin()) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  Serial.println("ESP-NOW init OK");

  if (!masterNode.add_to_system()) {
    Serial.println("Failed to add master peer");
    return;
  }
  Serial.println("Master peer added OK");
  Serial.println("Setup complete. Monitoring...\n");
}

// ─────────────────────────────────────────────
// Main Loop
// ─────────────────────────────────────────────
void loop() {

  static unsigned long lastNetworkCheck = 0;
  if (millis() - lastNetworkCheck >= 100) {
    lastNetworkCheck = millis();
    ftpSrv.handleFTP();
    server.handleClient();
  }

  checkSerial();

  bool blowerIsOn = detectBlower();

  static unsigned long lastOneSecondCheck = millis();
  if (millis() - lastOneSecondCheck >= 1000) {
    lastOneSecondCheck += 1000;

    getDateTime();
    logToFile(blowerIsOn);

    Serial.printf("[%s] Var: %.2f  State: %s  C_On: %d  C_Off: %d  Elapsed: %.2f min  Daily: %.2f min\n",
                  dtStamp.c_str(),
                  currentVariance,
                  blowerIsOn ? "ON" : "OFF",
                  consecutiveOnCount,
                  consecutiveOffCount,
                  elapsedMinutes,
                  dailyTotalMinutes);

    static bool didMidnightReset = false;
    if (HOUR == 0 && MINUTE == 0 && SECOND == 0) {
      if (!didMidnightReset) {
        dailyTotalMinutes = 0.0;
        Serial.println("Midnight reset: dailyTotalMinutes = 0");
        didMidnightReset = true;
      }
    } else {
      didMidnightReset = false;
    }
  }
}
