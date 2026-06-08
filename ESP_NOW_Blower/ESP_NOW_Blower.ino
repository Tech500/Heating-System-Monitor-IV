/* Heating System Monitor  
   ESP_NOW_Blower.ino
   June 6, 2026 
   ESP Now, Verified Core 3.3.10 Native Virtual Function Mapping
*/



#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32_NOW.h>
#include <LittleFS.h>
#include <FTPServer.h>
#include <WebServer.h>
#include "Filters.h"    // Jonathan Driscoll "DSPFilters" library

// ─────────────────────────────────────────────
// WiFi & FTP Credentials
// ─────────────────────────────────────────────
const char* ssid = "ssid";
const char* password = "password";

FTPServer ftpSrv(LittleFS);
WebServer server(80);

// Master MAC address
uint8_t masterAddress[] = { 0x3C, 0xE9, 0x0E, 0x84, 0xEE, 0x80 };
#define CHANNEL 11

enum MessageType : uint8_t {
  MSG_BME280       = 0,
  MSG_ALERT_FLAG   = 1,
  MSG_BLOWER_STATE = 2
};

struct __attribute__((packed)) BlowerData {
  MessageType type;
  bool on;
  float elapsedMinutes;
  float dailyTotalMinutes;
};

// ─────────────────────────────────────────────
// Microphone and Thresholds
// ─────────────────────────────────────────────
const int micPin = 34;
const int sampleWindowMs = 50;

// Calibrated Thresholds based on room ambient noise floor
const float varThresholdOn  = 0.350; 
const float varThresholdOff = 0.220;
const int requiredWindows   = 5;

const float attenuation = 1.0f;

// Fixed sampling configuration for the filter (10 kHz sampling rate)
const unsigned long SAMPLE_PERIOD_MICROS = 100; 

// Filter parameters
const float lowerCutoff = 500.0;
const float upperCutoff = 8000.0;

FilterOnePole highPassFilter;
FilterTwoPole lowPassFilter;

struct FilteredWindowResult {
  float signalMin;
  float signalMax;
  float peakToPeak;
  float lastFiltered;
  float variance;       
};

// ─────────────────────────────────────────────
// Global Tracking Registers (Accessible Everywhere)
// ─────────────────────────────────────────────
bool blowerOn              = false;
int  consecutiveOnWindows  = 0;
int  consecutiveOffWindows = 0;
float currentVariance       = 0.0;
float p2pTrack              = 0.0;
float minTrack              = 0.0;
float maxTrack              = 0.0;

// ─────────────────────────────────────────────
// Running Metrics & Analytics Registers
// ─────────────────────────────────────────────
double secondsCounter     = 0.0;
double elapsedMinutes     = 0.0;
double dailyTotalMinutes   = 0.0;
double lastEventMinutes    = 0.0;

bool alertFlag             = false;
bool googleSheetsSent      = false;



int HOUR   = 0;
int MINUTE = 0;
int SECOND = 0;

unsigned long lastOneSecondCheck = 0;

// ─────────────────────────────────────────────
// LittleFS Log Handling (Circular Rotation Limit)
// ─────────────────────────────────────────────
const char* LOG_FILE    = "/blower_log.csv";
const int   MAX_RECORDS = 2000; 
int         recordCount = 0;
bool        loggingActive = true;

// Forward Declarations
void handleClear();
void initLogging();
void logToFile(bool blowerState);
void handleDownload();
void handleStatus();
void initWebServer();
void sendData(bool state);
void displayData();
void logData();
void sendGoogleSheetsData();
void updateHeatingData();
FilteredWindowResult sampleFilteredWindow(int analogPin, int durationMs);
bool detectSound();
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);

// ─────────────────────────────────────────────
// LittleFS Logging Implementation
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
      f.println("Record,Millis,P2P,Min,Max,currentVariance,BlowerState");
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
}

void logToFile(bool blowerState) {
  if (!loggingActive) return;

  if (recordCount >= MAX_RECORDS) {
    Serial.println("Log size hit 2000 limit. Performing automatic rollover...");
    handleClear();
  }

  File f = LittleFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return;
  
  recordCount++;
  f.print(recordCount);        f.print(",");
  f.print(millis());           f.print(",");
  f.print(p2pTrack,          3); f.print(","); 
  f.print(minTrack,          3); f.print(","); 
  f.print(maxTrack,          3); f.print(","); 
  f.print(currentVariance,   4); f.print(","); 
  f.println(blowerState ? "ON" : "OFF");       
  f.close();
}

// ─────────────────────────────────────────────
// Web Server Routines
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
  recordCount   = 0;
  loggingActive = true;
  File f = LittleFS.open(LOG_FILE, FILE_WRITE);
  if (f) {
    f.println("Record,Millis,P2P,Min,Max,currentVariance,BlowerState");
    f.close();
  }
  if (server.client()) {
    server.send(200, "text/plain", "Log cleared.");
  }
}

void handleStatus() {
  String msg  = "Records: " + String(recordCount) + " / Circular Buffer Loop (2000)";
  msg += "\nLogging: " + String(loggingActive ? "YES" : "NO");
  msg += "\nBlower: " + String(blowerOn ? "ON" : "OFF");
  msg += "\nVarOn:  " + String(varThresholdOn);
  msg += "\nVarOff: " + String(varThresholdOff);
  server.send(200, "text/plain", msg);
}

void initWebServer() {
  server.on("/download", handleDownload);
  server.on("/clear",    handleClear);
  server.on("/status",   handleStatus);
  server.begin();
}

class HeatingMasterPeer : public ESP_NOW_Peer {
  public:
    HeatingMasterPeer(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface = WIFI_IF_STA, const uint8_t *lmk = NULL)
      : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

    bool add_to_system() { return ESP_NOW_Peer::add(); }
    size_t send_packet(const uint8_t *data, int len) { return ESP_NOW_Peer::send(data, len); }
};

HeatingMasterPeer masterNode(masterAddress, CHANNEL);

void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void sendData(bool state) {
  BlowerData blower;
  blower.type             = MSG_BLOWER_STATE;
  blower.on               = state;
  blower.elapsedMinutes   = (float)elapsedMinutes;
  blower.dailyTotalMinutes = (float)dailyTotalMinutes;

  size_t bytesSent = masterNode.send_packet((uint8_t*)&blower, sizeof(BlowerData));

  if (bytesSent > 0) {
    Serial.printf("Sent blower state over air: %s  Elapsed: %.2f min  Daily: %.2f min\n",
                  state ? "ON" : "OFF", elapsedMinutes, dailyTotalMinutes);
  } else {
    Serial.println("Error invoking ESP-NOW send method");
  }
}

void displayData() { Serial.println("[Display] Local UI updated."); }
void logData() { Serial.println("[Local Log] Cache metrics indexed."); }
void sendGoogleSheetsData() { Serial.println("[Cloud] Synchronizing log with Google Sheets..."); }
void updateHeatingData() { Serial.println("[System] Runtime registers compiled."); }

// ─────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Blower Bandpass Detector Setup Executing...");

  initLogging();

  highPassFilter.setFilter(HIGHPASS, lowerCutoff, 0);
  lowPassFilter.setAsFilter(LOWPASS_BUTTERWORTH, upperCutoff, 0);

  WiFi.mode(WIFI_MODE_APSTA); 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Infrastructure connected");
  
  ftpSrv.begin(F("admin"), F("Sky7388"));
  initWebServer();

  if (!ESP_NOW.begin()) {
    Serial.println("ESP-NOW API initialization failed");
    return;
  }
  
  if (!masterNode.add_to_system()) {
    Serial.println("Failed to bind peer node address");
    return;
  }

  masterNode.onSent(onDataSent);
  lastOneSecondCheck = millis();
}

// ─────────────────────────────────────────────
// Main Loop State Machine
// ─────────────────────────────────────────────
void loop() {
  static unsigned long lastNetworkCheck = 0;
  if (millis() - lastNetworkCheck >= 100) {
    lastNetworkCheck = millis();
    ftpSrv.handleFTP();
    server.handleClient();
  }

  // Step 1: Capture sensor state
  bool blowerIsOn = detectSound();
  static bool lastBlowerState = false;

  // Step 2: Edge detection triggers strictly when the cycle finishes
  if (lastBlowerState && !blowerIsOn) {
    lastEventMinutes = elapsedMinutes;  
    dailyTotalMinutes += lastEventMinutes;

    Serial.printf("Blower OFF. Added %.2f min to daily total. New total: %.2f min\n",
                  lastEventMinutes, dailyTotalMinutes);
    updateHeatingData();
    
    alertFlag = true; // Sets the pipeline flag active safely
  }
  lastBlowerState = blowerIsOn;

  // Step 3: Precise 1-Second Tracking Intervals
  // Step 3: Precise 1-Second Tracking Intervals
  if (millis() - lastOneSecondCheck >= 1000) {
    lastOneSecondCheck += 1000;
    
    SECOND++;
    if (SECOND >= 60) { SECOND = 0; MINUTE++; }
    if (MINUTE >= 60) { MINUTE = 0; HOUR++; }
    if (HOUR >= 24)   { HOUR = 0; }

    logToFile(blowerIsOn);

    // Keep the time accumulators working strictly when running
    if (blowerIsOn) {
      secondsCounter++;
      elapsedMinutes = secondsCounter / 60.0;
    }

    // --- THE FIX: THIS LINE IS NOW OUTSIDE THE IF-BLOCK ---
    // It will now print EVERY second, even when the blower is completely OFF!
    Serial.printf("Seconds: %.2f  Elapsed min: %.2f  Daily total: %.2f | Current Var: %.4f | State: %s | C_On: %d | C_Off: %d\n",
                  secondsCounter, 
                  elapsedMinutes, 
                  dailyTotalMinutes, 
                  currentVariance, 
                  blowerIsOn ? "ON" : "OFF",
                  consecutiveOnWindows,
                  consecutiveOffWindows);

    static bool didMidnightReset = false;
    if (HOUR == 0 && MINUTE == 0 && SECOND == 0) {
      if (!didMidnightReset) {
        dailyTotalMinutes = 0;
        Serial.println("Midnight reset completed.");
        didMidnightReset = true;
      }
    } else {
      didMidnightReset = false;
    }
  }

  // Step 4: Streamlined pipeline execution
  if (alertFlag) {
    sendData(blowerIsOn); // FIXED: Sends actual blowerIsOn state (false), not the alert flag!
    delay(500); 
    displayData();
    logData();
    sendGoogleSheetsData(); 
    
    googleSheetsSent = true;
    alertFlag = false; // Turn flag off cleanly
  }

  // Step 5: Clean registers only AFTER reporting pipeline completes
  if (googleSheetsSent) {
    secondsCounter = 0;
    elapsedMinutes = 0;
    lastEventMinutes = 0;
    googleSheetsSent = false;
  }
}

// ─────────────────────────────────────────────
// Audio Window Capture (Fixed Microsecond Intervals)
// ─────────────────────────────────────────────
FilteredWindowResult sampleFilteredWindow(int analogPin, int durationMs) {
  FilteredWindowResult result;
  result.signalMin    =  3.3f;
  result.signalMax    =  0.0f;
  result.lastFiltered =  0.0f;
  result.variance     =  0.0f;

  float sum   = 0.0f;
  float sumSq = 0.0f;
  int   count = 0;

  unsigned long startMillis = millis();
  unsigned long nextSampleMicros = micros();
  
  while (millis() - startMillis < durationMs) {
    while (micros() < nextSampleMicros) {
        delayMicroseconds(1);
    }
    nextSampleMicros += SAMPLE_PERIOD_MICROS;

    int   raw      = analogRead(analogPin);
    float voltage  = raw * (3.3f / 4095.0f) * attenuation;
    float highPass = highPassFilter.input(voltage);
    float filtered = lowPassFilter.input(highPass);
    
    result.lastFiltered = filtered;
    if (filtered < result.signalMin) result.signalMin = filtered;
    if (filtered > result.signalMax) result.signalMax = filtered;
    sum   += filtered;
    sumSq += filtered * filtered;
    count++;
  }

  result.peakToPeak = result.signalMax - result.signalMin;
  if (count > 0) {
    float mean      = sum / count;
    result.variance = (sumSq / count) - (mean * mean);
    if (result.variance < 0) result.variance = 0;  
  }

  return result;
}

// ─────────────────────────────────────────────
// Microphone Sound Evaluation Machine
// ─────────────────────────────────────────────
bool detectSound() {
  FilteredWindowResult res = sampleFilteredWindow(micPin, sampleWindowMs);
  currentVariance   = res.variance;
  p2pTrack          = res.peakToPeak;
  minTrack          = res.signalMin;
  maxTrack          = res.signalMax;

  if (!blowerOn) {
    if (currentVariance >= varThresholdOn) {
      consecutiveOnWindows++;
    } else {
      consecutiveOnWindows = 0; 
    }
    consecutiveOffWindows = 0;
    if (consecutiveOnWindows >= requiredWindows) {
      blowerOn = true;
      consecutiveOnWindows = 0;
      Serial.println(">>> Blower Detected: ON");
    }
  } 
  else { 
    if (currentVariance <= varThresholdOff) {
      consecutiveOffWindows++;
    } else {
      consecutiveOffWindows = 0; 
    }
    consecutiveOnWindows = 0;
    if (consecutiveOffWindows >= requiredWindows) {
      blowerOn = false;
      consecutiveOffWindows = 0;
      Serial.println(">>> Blower Detected: OFF");
    }
  }
  return blowerOn;
}
