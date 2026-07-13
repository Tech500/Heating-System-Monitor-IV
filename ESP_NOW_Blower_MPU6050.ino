/* Heating System Monitor IV
   ESP_NOW_Blower_MPU6050.ino
   June 2026
   ESP-NOW, Verified ESP32 Core 3.3.10
   MPU-6050 Accelerometer Vector Magnitude Variance Detection
   I2C: SDA=GPIO5  SCL=GPIO4  Address: 0x68
   Calibrated: OFF variance ~TBD | ON variance ~TBD | Threshold: 500.0 (tune on bench)
   OFF_CONFIRM reduced to 5 — mechanical stop is clean, no acoustic bleed
   Elapsed time computed via NTP difftime() — no secondsCounter drift
   On OFF confirmation: sends MSG_BLOWER_STATE then MSG_ALERT_FLAG to receiver
   FTP default user:  admin  password:  admin

   --- CHANGE LOG (this revision) ---
   FTP RETR timeout fix: computeVariance() was blocking for
   SAMPLE_COUNT * SAMPLE_DELAY_MS (64*5ms = 320ms) every single loop()
   pass, unconditionally. That 320ms blackout meant ftpSrv.handleFTP()
   never got serviced during an active data transfer, so FTP clients
   timed out on RETR (LIST worked fine — it's fast, doesn't span the gap).
   Fix: call ftpSrv.handleFTP() + server.handleClient() once per sample
   INSIDE computeVariance()'s sampling loop, so FTP is serviced every
   ~5ms instead of going dark for 320ms. Same interleave applied to the
   two delay(500) blocks in detectBlower() after sendData()/sendAlert(),
   since those are the same class of blocking gap, just rarer (only on
   ON/OFF transitions).

   --- CHANGE LOG (July 12, 2026) ---
   NVS persistence added for dailyTotalMinutes. This node is battery
   powered, so a battery change / brownout previously wiped the plain
   RAM `dailyTotalMinutes` variable back to 0, and the receiver would
   faithfully mirror that zero to Sheets (the receiver has no
   independent copy — it just relays whatever this node sends).
   Now: dailyTotalMinutes + lastResetEpochDay are written to NVS
   (Preferences, namespace "hsm4blower") on every OFF transition and
   at the daily rollover. On boot, the value is restored from NVS
   instead of starting at 0, and esp_reset_reason() is logged to
   Serial so you can distinguish POWERON/BROWNOUT (battery change)
   from DEEPSLEEP/SW resets in the log.
   Midnight reset logic also changed from an exact-second match
   (HOUR==0 && MINUTE==0 && SECOND==0, which a missed loop() pass
   could skip entirely) to a stored-epoch-day comparison, which
   fires exactly once regardless of what the clock reads at the
   moment of the check, and also self-corrects if the node was
   powered off across a midnight boundary.
   Write frequency: OFF transitions are infrequent (blower cycles),
   so this stays well within NVS wear limits on a battery node.
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <ESP32_NOW.h>
#include <LittleFS.h>
#include <FTPServer.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <time.h>

// ─────────────────────────────────────────────
// I2C / MPU-6050
// ─────────────────────────────────────────────
// SDA = GPIO5   SCL = GPIO4
#define SDA 5
#define SCL 4

MPU6050 mpu;

// ─────────────────────────────────────────────
// WiFi (via WiFiManager captive portal) & FTP Credentials
// ─────────────────────────────────────────────
WiFiManager wm;

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
uint8_t masterAddress[] = { 0xE4, 0x65, 0xB8, 0x20, 0xEC, 0xD8 };
                            
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
// MPU-6050 Variance Detection Configuration
//
// Hysteresis pair, set from real mounted-furnace data
// (07/01/2026 bench + closet sessions), replacing the original
// single-threshold placeholder. Confirmed separation across
// every condition tested:
//   OFF / quiet ambient:        ~1,500 - 4,000
//   Disposal running (10-15ft): ~2,000 - 4,600  (barely registers)
//   Footstep/stomp shocks:      sustained 130,000+ (see ceiling note below)
//   Blower ON, heating fan-only: ~4,500 - 9,500
//   Blower ON, cooling mode:     ~44,000 - 114,000 (higher fan speed)
//
// A single threshold at 6000 caused chatter when readings hovered
// right around that value at a state transition (observed 07/01/2026
// evening — ON/OFF/ON flipping within 5 seconds, variance bouncing
// 4800-7800). Hysteresis fixes this: once ON, must drop below
// OFF_THRESHOLD to start counting toward OFF; once OFF, must rise
// above ON_THRESHOLD to start counting toward ON. The gap between
// the two absorbs noise that would otherwise cross a single cutoff
// back and forth.
//
// NOTE: sustained shock events (stomping, drops) can read
// 100,000+ and would currently misclassify as ON if they
// last >= ON_CONFIRM cycles. No ceiling filter implemented
// yet — revisit if false ON triggers are observed in practice.
//
// OFF_CONFIRM: 5 cycles — mechanical stop is clean.
//   Each cycle = SAMPLE_COUNT * SAMPLE_DELAY_MS = 64 * 5ms = ~320ms
//   5 cycles = ~1.6 seconds confirmation before declaring OFF.
//   Increase if spurious OFF triggers observed.
// ─────────────────────────────────────────────
const int   SAMPLE_COUNT    = 64;
const int   SAMPLE_DELAY_MS = 5;
const float ON_THRESHOLD    = 7000.0f;  // must rise above this to start counting toward ON
const float OFF_THRESHOLD   = 5000.0f;  // must drop below this to start counting toward OFF
const int   ON_CONFIRM      = 3;
const int   OFF_CONFIRM     = 5;        // reduced from 90 — clean mechanical stop

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
double dailyTotalMinutes = 0.0;   // restored from NVS in setup() -- see loadDailyTotal()

// ─────────────────────────────────────────────
// NVS Persistence (survives power loss / battery change)
// ─────────────────────────────────────────────
Preferences prefs;
uint32_t    lastResetEpochDay = 0;

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
void   serviceNetwork();
uint32_t getCurrentEpochDay();
void   loadDailyTotal();
void   saveDailyTotal();
void   logResetReason();

// ─────────────────────────────────────────────
// NVS Persistence for dailyTotalMinutes
// Written only on OFF transitions and at the daily rollover --
// infrequent enough to be safe for NVS wear on a battery node.
// ─────────────────────────────────────────────
uint32_t getCurrentEpochDay() {
  time_t now = time(nullptr);
  return (uint32_t)(now / 86400UL);
}

void loadDailyTotal() {
  prefs.begin("hsm4blower", false);
  dailyTotalMinutes = prefs.getDouble("dailyTotal", 0.0);
  lastResetEpochDay = prefs.getUInt("resetDay", 0);
  prefs.end();
  Serial.printf("NVS restore: dailyTotalMinutes=%.2f  lastResetEpochDay=%u\n",
                dailyTotalMinutes, lastResetEpochDay);
}

void saveDailyTotal() {
  prefs.begin("hsm4blower", false);
  prefs.putDouble("dailyTotal", dailyTotalMinutes);
  prefs.putUInt("resetDay", lastResetEpochDay);
  prefs.end();
}

// Logs *why* the board booted -- lets you tell a battery change /
// brownout apart from a deep-sleep wake or a code-push reset in Serial.
void logResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  const char* reasonStr;
  switch (reason) {
    case ESP_RST_POWERON:   reasonStr = "POWERON (power loss/battery change)"; break;
    case ESP_RST_BROWNOUT:  reasonStr = "BROWNOUT (power loss)";               break;
    case ESP_RST_DEEPSLEEP: reasonStr = "DEEPSLEEP wake";                      break;
    case ESP_RST_SW:        reasonStr = "software reset";                     break;
    case ESP_RST_PANIC:     reasonStr = "PANIC/crash";                        break;
    case ESP_RST_WDT:       reasonStr = "watchdog";                           break;
    default:                reasonStr = "other";                              break;
  }
  Serial.print("Boot reason: ");
  Serial.println(reasonStr);
}

// ─────────────────────────────────────────────
// Service FTP + WebServer.
// Called from loop() AND interleaved inside any blocking
// section (sampling loop, post-transition delays) so neither
// ever goes dark for more than a few ms at a time.
// ─────────────────────────────────────────────
// FTP burst — confirmed working at 500 on the bench.
// Tune down if a smaller value still clears RETR cleanly —
// lower cost per loop() pass. Suggested test sequence:
// 500 (known good) -> 250 -> 100 -> 50, retrying FTP RETR at each.
// If a value fails, back up to the last one that worked.
const int FTP_BURST_COUNT = 500;

void serviceNetwork() {
  ftpSrv.handleFTP();
  server.handleClient();
}

// ─────────────────────────────────────────────
// FTP burst service — confirmed on the bench to clear
// RETR transfers that the single-call-per-sample interleave
// alone didn't fix. Call once per loop() pass (NOT nested
// inside computeVariance()'s sampling loop — that would
// multiply out to tens of thousands of calls per detection
// cycle and start competing with sensor timing).
// yield() every 50 iterations keeps the WiFi/lwIP task fed
// so this doesn't starve the radio mid-burst.
// ─────────────────────────────────────────────
void ftpBurstService() {
  for (int x = 0; x < FTP_BURST_COUNT; x++) {
    ftpSrv.handleFTP();
    if (x % 50 == 0) yield();
  }
}

// ─────────────────────────────────────────────
// MPU-6050 Vector Magnitude Variance
// Reads raw accel X, Y, Z; computes magnitude per sample;
// returns variance of magnitude over SAMPLE_COUNT samples.
// Orientation-independent — no need to pick an axis.
//
// serviceNetwork() is called once per sample so FTP/web stay
// responsive across the full ~320ms sampling window instead of
// blacking out for the whole thing.
// ─────────────────────────────────────────────
float computeVariance() {
  float magnitudes[SAMPLE_COUNT];
  float sum = 0.0f;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // magnitude of acceleration vector (raw int16 units)
    float mag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az);
    magnitudes[i] = mag;
    sum += mag;

    serviceNetwork();   // keep FTP/web alive during this sample window

    delay(SAMPLE_DELAY_MS);
  }

  float mean     = sum / SAMPLE_COUNT;
  float variance = 0.0f;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float diff = magnitudes[i] - mean;
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
// Non-blocking replacement for delay(500) after state-transition
// sends. Services FTP/web every 5ms instead of going dark for
// the full 500ms in one block.
// ─────────────────────────────────────────────
void settleDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    serviceNetwork();
    delay(5);
  }
}

// ─────────────────────────────────────────────
// Blower State Machine — hysteresis added 07/01/2026 evening
// to eliminate chatter observed when a single threshold sat
// right at the boundary during a real transition.
// ─────────────────────────────────────────────
bool detectBlower() {
  currentVariance = computeVariance();

  if (!blowerOn) {
    if (currentVariance >= ON_THRESHOLD) {
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
      settleDelay(500);
    }
  } else {
    if (currentVariance < OFF_THRESHOLD) {
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
      saveDailyTotal();   // persist to NVS -- survives battery change

      getDateTime();
      Serial.printf(">>> Blower Detected: OFF @ %s\n", dtStamp.c_str());
      Serial.printf("    Elapsed: %.2f min  Daily total: %.2f min\n",
                    elapsedMinutes, dailyTotalMinutes);

      logToFile(false);   // one-time OFF summary row — no more per-second OFF spam

      sendData(false);
      settleDelay(500);
      sendAlert();
      settleDelay(500);
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
  msg += "\nON_THRESHOLD: "        + String(ON_THRESHOLD);
  msg += "\nOFF_THRESHOLD: "       + String(OFF_THRESHOLD);
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
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\n\n\nHeating System Monitor IV, ESP_NOW_Blower_MPU6050.ino — ESP32 Core 3.3.10\n");
  Serial.printf("ON_THRESHOLD=%.1f  OFF_THRESHOLD=%.1f  ON_CONFIRM=%d  OFF_CONFIRM=%d\n",
                ON_THRESHOLD, OFF_THRESHOLD, ON_CONFIRM, OFF_CONFIRM);
  Serial.println("Commands: r=rotate | d=delete all | l=list");

  logResetReason();   // tells you POWERON/BROWNOUT (battery change) vs. DEEPSLEEP/SW in Serial

  // I2C: SDA=GPIO5  SCL=GPIO4
  Wire.begin(SDA, SCL);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU-6050 connection FAILED — check wiring");
    while (true) delay(1000);
  }
  Serial.println("MPU-6050 connected OK at 0x68");

  // Accelerometer range: ±2g (default) — most sensitive, best for low vibration detection
  // If clipping observed, increase to ±4g: mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  Serial.println("Accel range: ±2g");

  initLogging();

  WiFi.setSleep(WIFI_PS_NONE);
  // NOTE: deliberately NOT setting WiFi.mode(WIFI_MODE_APSTA) here —
  // testing whether a pre-set mode is what's blocking WiFiManager's
  // own AP ("HSM-IV-Setup") from actually broadcasting. Letting
  // WiFiManager fully own mode-switching during autoConnect()/portal.
  // APSTA gets re-asserted below, after WiFiManager is done, since
  // ESP-NOW still needs STA+AP together once we're past this point.

  // WiFiManager: tries saved credentials first. If it can't connect
  // (e.g. the PMF/WPA3 association issue we were chasing manually),
  // it spins up its own AP named "HSM-IV-Setup" — connect a phone/laptop
  // to that, a captive portal pops up, pick R2D2, enter password there.
  // No timeout set — this blocks indefinitely until configured, rather
  // than the portal vanishing after N seconds before there's time to
  // actually connect and finish the captive portal flow.
  bool wifiOK = wm.autoConnect("HSM");

  WiFi.mode(WIFI_MODE_APSTA);   // re-assert now that WiFiManager is done — ESP-NOW needs this

  if (!wifiOK) {
    Serial.println("\nWiFiManager: failed to connect / portal timed out — continuing without WiFi.");
    Serial.println("  FTP, web server, NTP, and ESP-NOW sends will be unavailable until reconnected.");
    Serial.println("  Reset the board and it will retry the portal on next boot.");
  } else {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    Serial.printf("Blower MAC: %s  Channel: %d\n",
                  WiFi.macAddress().c_str(), WiFi.channel());
  }

  if (wifiOK) {
    initNTP();

    // Restore dailyTotalMinutes from NVS -- survives the battery change
    // that would otherwise silently reset it to 0.
    loadDailyTotal();

    // If a calendar day boundary passed while this node was powered
    // off (or on a previous boot), catch up now rather than relying
    // on the loop() SECOND==0 check, which could be skipped entirely
    // by a missed loop() pass or a boot that lands mid-day.
    uint32_t today = getCurrentEpochDay();
    if (lastResetEpochDay == 0 || today > lastResetEpochDay) {
      Serial.println("Day boundary crossed since last save -- resetting dailyTotalMinutes.");
      dailyTotalMinutes = 0.0;
      lastResetEpochDay = today;
      saveDailyTotal();
    }
  } else {
    Serial.println("Skipping NTP sync — no WiFi.");
    // Still restore the last known value even without time sync --
    // better a stale-but-correct total than starting over at 0.
    loadDailyTotal();
  }

  ftpSrv.begin(F("admin"), F("admin"));
  ftpSrv.setTimeout(30000);   // 30s instead of library default 5 min —
                               // a dangling/half-closed FileZilla session
                               // now clears fast instead of blocking new
                               // connections for up to 5 minutes
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

  // Kept as a redundant safety net — computeVariance() now services
  // FTP/web internally every ~5ms, so this 100ms gate rarely matters,
  // but it's harmless to leave in for the moments loop() is between
  // detectBlower() calls.
  // FTP burst — confirmed on the bench to clear RETR transfers.
  // Runs once per loop() pass, bounded and yield()-protected.
  ftpBurstService();
  server.handleClient();

  checkSerial();

  bool blowerIsOn = detectBlower();

  static unsigned long lastOneSecondCheck = millis();
  if (millis() - lastOneSecondCheck >= 1000) {
    lastOneSecondCheck += 1000;

    getDateTime();
    if (blowerIsOn) {
      logToFile(true);   // continuous per-second logging while running
    }
    // while OFF: no per-second file write — the single OFF summary row
    // was already written at the transition in detectBlower()

    Serial.printf("[%s] Var: %.2f  State: %s  C_On: %d  C_Off: %d  Elapsed: %.2f min  Daily: %.2f min\n",
                  dtStamp.c_str(),
                  currentVariance,
                  blowerIsOn ? "ON" : "OFF",
                  consecutiveOnCount,
                  consecutiveOffCount,
                  elapsedMinutes,
                  dailyTotalMinutes);

    // Daily rollover: stored-epoch-day comparison instead of an exact
    // HOUR==0 && MINUTE==0 && SECOND==0 match. The old check could be
    // skipped entirely by a missed loop() pass; this fires exactly
    // once per real calendar day regardless of the clock reading at
    // the moment loop() happens to run, and also catches a day that
    // rolled over while the node was powered off.
    uint32_t today = getCurrentEpochDay();
    if (today > lastResetEpochDay) {
      dailyTotalMinutes = 0.0;
      lastResetEpochDay = today;
      saveDailyTotal();
      Serial.println("Midnight reset (day boundary): dailyTotalMinutes = 0");
    }
  }
}
