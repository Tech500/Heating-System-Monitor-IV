/* Heating System Monitor IV
   ESP_NOW_Receiver.ino with temperature Offset
   July 19, 2026
   ESP32_-NOW,  ESP32 Core 3.3.10
   Receives MSG_BLOWER_STATE from ESP_NOW_Blower node
   Receives MSG_BME280 from ESP_NOW_BME280 node
   Sends MSG_ALERT_FLAG to BME280 node to request outside temp

   --- Updated June 20, 2026 ---
   Inside sensing changed from MCP9808 (direct register I2C) to BME280I2C library.
   Adds insideHumidity to Google Sheets logging; removes registerTemp column.

   --- Updated June 20, 2026 (cont.) ---
   Adds outside/inside pressure capture and diff (outsidePressure - insidePressure).
   Fixes read-gate: was isnan(temp)||isnan(hum), which permanently rejects readings
   on the interim HW-611/BMP280 since it has no humidity element (hum always NaN).
   Now gates on temp/pressure only; humidity passes through as NaN.

   --- Updated June 25, 2026 ---
   Original HW-394 receiver board replaced -- defective 3.3V rail (2.04V measured),
   causing corrupted I2C reads and frozen BME280 output.
   Replacement HW-394: I2C reassigned to D4 (SDA/GPIO4) and D5 (SCL/GPIO5).
   BME280 inside temp calibration offset updated to -2.51F (ref: HP-770HD DMM).
   Note: re-verify offset once sensor is in permanent installed location.
   ESP32 Core downgraded to 2.0.17 to resolve BME280 library I2C compatibility issues.
   Pressure converted hPa->inHg (x 0.02953) for Serial, LittleFS, and Sheets POST.
   Absolute pressure converted to sea-level relative using temperature-compensated
   hypsometric formula (toSeaLevelHPa) before inHg conversion.
   Pressure diff remains in absolute hPa converted to inHg -- both sensors same elevation.
   Internal math remains in hPa throughout.

   --- Update July 6, 2026 ---
   Duplicate-row fix: alertFlag now gated to OFF transition only
   (if (!blowerPacket.on)) -- ON packets carry stale elapsed/daily values.

   Outside BME280 freshness (dead-node detection):
   globalTemp/globalHumidity/globalPressure initialized to NAN and cleared
   back to NAN after every Google Sheets post. If the BME280 node does not
   answer the alert poll during the current cycle, the values remain NAN and
   the Sheet/LittleFS log record "Offline" for outsideTemp, outsidePressure,
   and pressureDiff instead of fossilized data. Self-heals on next reply.
   Also fixes first-cycle 0.0 outside temp (HSM III bug) -- boot value is
   now NAN -> "Offline" until first real packet arrives.
   Sheets post-processing note: AVERAGE/charts skip text cells; guard any
   per-row formulas with ISNUMBER(); pandas: na_values=['Offline'].

   --- Update July 19, 2026 ---
   Cycle tracking merged in (was developed July 8 against the pre-NVS
   receiver and skipped during the Preferences work).
   Goal: find thermostat setpoint giving long ON cycles + long coast
   (hold) time -- data for the setpoint optimization study.
   - cyclesToday: OFF->ON transitions since midnight.
   - lastCoastMinutes: gap between previous OFF and this ON -- the
     "temperature held" time, measured via epoch time at the receiver.
   - avgCycleMinutes: dailyTotalMinutes / cyclesToday.
   Transitions detected against prevBlowerIsOn so a re-sent duplicate
   state packet cannot double-count a cycle. ON length itself already
   arrives from the blower node as elapsedMinutes at the OFF transition.
   cyclesToday and lastOffEpoch persist in NVS alongside dailyTotalMinutes
   (saveState/restoreState) -- survives power-on resets, matching the
   receiver's NVS-authority design. Midnight reset zeroes cycle stats and
   calls saveState() so a post-midnight power-cycle cannot restore
   yesterday's totals from stale NVS.
   New Sheets params: cyclesToday, coastMinutes, avgCycleMinutes.
   New LittleFS columns: CyclesToday, CoastMin, AvgCycleMin.
   Analysis: coast / (coast + elapsed) per row = duty cycle; compare
   across setpoint test blocks normalized by inside-outside delta-T.
   Also fixed: lastResetCheck initializer was -0.43 (paste artifact from
   the outside cal offset) -- now 0. Removed dead pre-offset globalTemp
   assignment in MSG_BME280 handler.
*/

#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <ESP32_NOW.h>
#include "FS.h"
#include <LittleFS.h>
#include <BME280I2C.h>
#include <FTPServer.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "rom/rtc.h"
#include "esp_system.h"   // esp_reset_reason() -- for reset_log.txt
#include <Preferences.h>

#define WRITE_LED_PIN 23  //LittleFS Status LED  ON = Writing

const char* ssid = "SSID";
const char* password = "PASSWORD";

//Flag to prevent reset
bool powerOnReset = false;

// ─── GOOGLE DEPLOYMENT ID ────────────────────────────────────────────────────
const String googleDeploymentID = "Removed for security";
const String googleURL          = "https://script.google.com/macros/s/" + googleDeploymentID + "/exec";

// ─────────────────────────────────────────────
// BME280 (Inside) — replaces MCP9808
// Default I2C address 0x76, same library as outside BME280 node
// ─────────────────────────────────────────────
BME280I2C bmeInside;

#define SDA 4
#define SCL 5


//Changed methodolgy of caluculating offset.  Set
//const float BME280_OUTSIDE_TEMP_CAL_OFFSET_F = 0;
//Note outside temperature.  Find nearby references
//used CWOP Stations, then averated reports --closet 
//to time of outside temperature reported n sketch.
//Find offset.


// ─────────────────────────────────────────────
// BME280 (Outside) Temperature Calibration
// NOTE: offset includes steady-state self-heating of current
// always-on DevKit node. RE-CALIBRATE after EoRa deep-sleep
// migration -- self-heating term will largely disappear.
// ─────────────────────────────────────────────
const float BME280_OUTSIDE_TEMP_CAL_OFFSET_F = -.43;

// ─────────────────────────────────────────────
// BME280 (Inside) Temperature Calibration
// Offset applied to BME280 (inside) temperature output
// ─────────────────────────────────────────────
const float BME280_INSIDE_TEMP_CAL_OFFSET_F = -7.11;

/*
//Thinking these were for Register sensor alert --not being used
const float LOW_TEMP_F  = 65.0;
const float HIGH_TEMP_F = 74.0;
const float LOW_TEMP_C  = (LOW_TEMP_F  - 32.0) / 1.8;
const float HIGH_TEMP_C = (HIGH_TEMP_F - 32.0) / 1.8;
*/

// ─── NTP / Time ──────────────────────────────────────────────────────────────
const char* udpAddress1 = "pool.ntp.org";
const char* udpAddress2 = "time.nist.gov";

int DOW, MONTH, DATE, YEAR, HOUR, MINUTE, SECOND;
String weekDay;
char strftime_buf[64];
String dtStamp    = "";


String lastUpdate = "";
time_t tnow;

// Replace Ticker variables with global millis trackers
unsigned long lastResetCheck = 0;   // was -0.43: paste artifact, see July 19 note
const unsigned long checkInterval = 1000; // Check every 1 second

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
  float insideHumidity;
  float thermostat;
  float lastEventMinutes;
  float dailyTotalMinutes;
  float lastRecordedMinute;
  float outsidePressure;
  float insidePressure;
  float pressureDiffHPa;   // outsidePressure - insidePressure
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
uint8_t senderBlowerMAC[] = { 0x9C, 0x13, 0x9E, 0xF2, 0x2A, 0xB4 };
uint8_t senderBmeMAC[]    = { 0xE4, 0x65, 0xB8, 0x25, 0x42, 0xF8 };
#define CHANNEL 0

HeatingSenderPeer blowerNode(senderBlowerMAC, CHANNEL, WIFI_IF_STA);
BME280Peer        bmeNode(senderBmeMAC,       CHANNEL, WIFI_IF_STA);

// ─── Global Tracking Registers ───────────────────────────────────────────────
bool blowerIsOn        = false;
bool alertFlag         = false;
bool googleSheetsSent  = false;

double elapsedMinutes                    = 0.0;
double dailyTotalMinutes   = 0.0;  // NVS-persisted; restored on power-on/brownout

Preferences statePrefs;
const char* STATE_NAMESPACE = "hsm4state";

// ─── Cycle Tracking (thermostat optimization study) ──────────────────────────
// Goal: find setpoint giving long ON cycles + long coast (hold) time.
// ON length comes from blower node's elapsedMinutes at OFF transition.
// Coast time = gap between last OFF and next ON, measured here via epoch time.
// cyclesToday and lastOffEpoch persist in NVS via saveState()/restoreState().
bool   prevBlowerIsOn   = false;   // for genuine-transition detection
int    cyclesToday      = 0;       // OFF->ON transitions since midnight
time_t lastOffEpoch     = 0;       // when blower last shut off
double lastCoastMinutes = 0.0;     // hold time preceding current cycle
double avgCycleMinutes  = 0.0;     // dailyTotalMinutes / cyclesToday

// Outside BME280 registers -- NAN = "no fresh reading this cycle".
// Written only by MSG_BME280; cleared back to NAN after every Sheets post.
// isnan() at post time => node did not answer this cycle's poll => "Offline".
float globalTemp         = NAN;
float globalHumidity     = NAN;
float globalPressure     = NAN;
float thermostatSetpoint = 75.0;

// ─── Forward Declarations ─────────────────────────────────────────────────────
// ─── Elevation / Sea-Level Pressure Conversion ───────────────────────────────
// Station elevation: 809 ft = 246.6 m
// Converts absolute BME280 pressure (hPa) to sea-level relative pressure (hPa)
// using temperature-compensated hypsometric formula.
#define STATION_ELEVATION_M 246.6

float toSeaLevelHPa(float absoluteHPa, float tempF) {
  float tempC = (tempF - 32.0) / 1.8;
  float tempK = tempC + 273.15;
  return absoluteHPa * exp(9.80665 * STATION_ELEVATION_M / (287.05 * tempK));
}

void sendGoogleSheetsData();
void sendDataToServer(String updateTime, float outT, float inT, float inHum,
                      float therm, double eventM, double dailyM,
                      float outP, float inP, float diffP);
void saveState();
bool restoreState();
void updateHeatingData();
void displayData();
void logData();
void disabledailyCooling();
void resetDailyStats();
void temperatureInterrupt();
void initNTP();
String getDateTime();
void logDailyTotal(String stamp, double dailyM, double eventM);
void logResetReason();
void initBME280Inside();
void readBME280Inside(float &tempF, float &humidity, float &pressureHPa);
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

// ─── BME280 (Inside) ─────────────────────────────────────────────────────────
void initBME280Inside() {
  while (!bmeInside.begin()) {
    Serial.println("Could not establish communication with inside BME280 sensor.");
    delay(1000);
  }
  Serial.println("Inside BME280 sensor initialized.");
}

void readBME280Inside(float &tempF, float &humidity, float &pressureHPa) {
  float temp = NAN, hum = NAN, pres = NAN;

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bmeInside.read(pres, temp, hum, tempUnit, presUnit);

  // Gate on temp/pressure only -- humidity is legitimately NaN on the
  // interim HW-611/BMP280 and must not block an otherwise valid reading.
  if (isnan(temp) || isnan(pres)) {
    Serial.println("Error reading inside BME280 sensor telemetry.");
    tempF       = NAN;
    humidity    = NAN;
    pressureHPa = NAN;
    return;
  }

  // Apply calibration offset — reference: HP-770HD DMM thermometer
  temp += BME280_INSIDE_TEMP_CAL_OFFSET_F;

  tempF       = temp;
  humidity    = hum;   // NAN on BMP280 (HW-611) until Monday's BME280 swap
  pressureHPa = pres;
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
        Serial.printf("\n[Radio Link] Blower Update Caught -> State: %s  Elapsed: %.2f min  Daily(blower-side): %.2f min\n",
                      blowerIsOn ? "ON" : "OFF",
                      blowerPacket.elapsedMinutes,
                      blowerPacket.dailyTotalMinutes);

        elapsedMinutes = blowerPacket.elapsedMinutes;
        sensordata.lastEventMinutes = blowerPacket.elapsedMinutes;

        // ── Cycle tracking: act only on genuine state transitions ─────────
        if (blowerPacket.on && !prevBlowerIsOn) {
          // OFF -> ON : a new cycle begins; measure the coast (hold) time
          cyclesToday++;
          if (lastOffEpoch > 0) {
            lastCoastMinutes = (double)(time(nullptr) - lastOffEpoch) / 60.0;
          } else {
            lastCoastMinutes = 0.0;   // first cycle after boot — no prior OFF
          }
          Serial.printf("[Cycle] #%d started. Coast (hold) before this cycle: %.1f min\n",
                        cyclesToday, lastCoastMinutes);
        }

        if (!blowerPacket.on && prevBlowerIsOn) {
          // ON -> OFF : cycle complete; stamp OFF time for coast measurement
          lastOffEpoch = time(nullptr);
        }

        if (!blowerPacket.on) {
          dailyTotalMinutes += blowerPacket.elapsedMinutes;   // receiver accumulates, own authority
          sensordata.dailyTotalMinutes = dailyTotalMinutes;
          if (cyclesToday > 0) {
            avgCycleMinutes = dailyTotalMinutes / (double)cyclesToday;
          }
          Serial.printf("[Cycle] #%d complete. ON: %.2f min  Avg cycle today: %.2f min\n",
                        cyclesToday, blowerPacket.elapsedMinutes, avgCycleMinutes);
          alertFlag = true;   // log to Sheet only at OFF
          saveState();   // <-- persist to NVS on every OFF-event accumulation
        }

        prevBlowerIsOn = blowerPacket.on;
      } else {
        Serial.printf("\n\n Size Mismatch — BlowerData: Expected %d got %d bytes\n",
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
        globalHumidity = bmePacket.humidity;      // fresh reading -- clears NAN sentinel
        globalPressure = bmePacket.pressure;
        globalTemp     = bmePacket.temperature + BME280_OUTSIDE_TEMP_CAL_OFFSET_F;

        Serial.printf("\n[Radio Link] BME280 Update -> Temp: %.2f F  Hum: %.1f%%  Pres: %.4f inHg\n",
                      globalTemp, globalHumidity, globalPressure * 0.02953);
      } else {
        Serial.printf("\n\n Size Mismatch — BmeData: Expected %d got %d bytes\n",
                      sizeof(BME280Data), len);
      }
      break;

    default:
      Serial.printf("\n\n Unknown packet type: %d\n", data[0]);
      break;
  }
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\nHeating System Monitor IV - ESP_NOW_Receiver_Preferences.ino\n");
  Serial.println("Build: " __DATE__ " " __TIME__ "\n");

  pinMode(WRITE_LED_PIN, OUTPUT);
  digitalWrite(WRITE_LED_PIN, LOW);

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

  // HW-394 replacement board: D4=SDA(GPIO4), D5=SCL(GPIO5)
  // Original board had defective 3.3V rail (2.04V), causing frozen I2C reads.
  Wire.begin(SDA,SCL);

  initNTP();
  initBME280Inside();

  bool fsok = LittleFS.begin(true);
  Serial.printf("FS init: %s\n", fsok ? "ok" : "fail!");

  /*
  Serial.println("Forcing full LittleFS format...");
  bool formatOk = LittleFS.format();
  Serial.printf("Format result: %s\n", formatOk ? "success" : "FAILED");
  */

  if (fsok) {
    getDateTime();
    logResetReason();

    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_POWERON || reason == ESP_RST_BROWNOUT) {
      if (restoreState()) {
        Serial.printf("Restored dailyTotalMinutes: %.2f  cyclesToday: %d\n",
                      dailyTotalMinutes, cyclesToday);
      } else {
        Serial.println("No valid saved state — starting dailyTotalMinutes at 0.");
      }
    }
  }

  ftpSrv.begin(F("admin"), F("admin"));

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

  unsigned long currentMillis = millis();

  // Non-blocking timer instead of Ticker interrupt
  if (currentMillis - lastResetCheck >= checkInterval) {
    lastResetCheck = currentMillis;
    getDateTime();
  }

  static bool didMidnightReset = false;

  // Window-based check (no SECOND == 0 dependency)
  if (HOUR == 23 && MINUTE == 58) {
    if (!didMidnightReset) {
      digitalWrite(WRITE_LED_PIN, HIGH);

      // Snapshot yesterday's final totals before the reset zeroes them
      double yesterdayDailyTotalMinutes = dailyTotalMinutes;
      double yesterdayElapsedMinutes    = elapsedMinutes;

      Serial.printf("Midnight snapshot -> Elapsed: %.2f min  Daily: %.2f min  Cycles: %d  AvgCycle: %.2f min\n",
                    yesterdayElapsedMinutes, yesterdayDailyTotalMinutes,
                    cyclesToday, avgCycleMinutes);

      logDailyTotal(dtStamp, yesterdayDailyTotalMinutes, yesterdayElapsedMinutes);

      dailyTotalMinutes = 0;
      cyclesToday       = 0;
      avgCycleMinutes   = 0.0;
      // lastOffEpoch intentionally NOT reset — first coast of the new day
      // spans midnight and is still a valid hold-time measurement.
      saveState();   // sync NVS so a post-midnight power-cycle can't restore stale totals
      Serial.println("Midnight reset: dailyTotalMinutes = 0, cyclesToday = 0 (NVS synced)");
      didMidnightReset = true;

      digitalWrite(WRITE_LED_PIN, LOW);
    }
  } else {
    didMidnightReset = false;
  }

  // ── Gatekeeper: alertFlag handler ────────────────────────────────────────
  if (alertFlag) {
    alertFlag = false;
    digitalWrite(WRITE_LED_PIN, HIGH);   // writes starting

    Serial.println("[ESP-NOW] Sending alert to BME280 node...");
    bool sent = bmeNode.sendAlert(true);
    Serial.printf("[ESP-NOW] Alert send: %s\n", sent ? "OK" : "FAILED");
    delay(1000);   // BME280 node reply lands here and refreshes globals

    float insideTempF = NAN, insideHumidity = NAN, insidePressureHPa = NAN;
    readBME280Inside(insideTempF, insideHumidity, insidePressureHPa);
    sensordata.insideTemp      = insideTempF;
    sensordata.insideHumidity  = insideHumidity;
    sensordata.outsideTemp     = globalTemp;        // NAN if node silent this cycle
    sensordata.thermostat      = thermostatSetpoint;
    sensordata.outsidePressure = globalPressure;    // NAN if node silent this cycle
    sensordata.insidePressure  = insidePressureHPa;
    sensordata.pressureDiffHPa = globalPressure - insidePressureHPa;  // NAN propagates

    if (isnan(sensordata.outsideTemp)) {
      Serial.println("[BME280 Outside] No reply this cycle -- posting Offline.");
    }

    Serial.printf("[BME280 Inside] Temp: %.2f °F  Hum: %.2f %%  Pres: %.4f inHg\n",
                  insideTempF, insideHumidity, insidePressureHPa * 0.02953);
    Serial.printf("[Pressure] Outside: %.4f inHg | Inside: %.4f inHg | Diff(out-in): %.4f inHg\n",
                  toSeaLevelHPa(sensordata.outsidePressure, sensordata.outsideTemp) * 0.02953,
                  toSeaLevelHPa(sensordata.insidePressure,  sensordata.insideTemp)  * 0.02953,
                  sensordata.pressureDiffHPa * 0.02953);  // diff stays absolute

    getDateTime();
    displayData();
    logData();
    sendGoogleSheetsData();
    googleSheetsSent = true;

    digitalWrite(WRITE_LED_PIN, LOW);    // writes done, safe to pull power
  }

  // ── Reset after pipeline ──────────────────────────────────────────────────
  if (googleSheetsSent) {
    elapsedMinutes   = 0;
    // Clear outside registers -- next post must be backed by a fresh
    // MSG_BME280 reply or it goes out as "Offline".
    globalTemp       = NAN;
    globalHumidity   = NAN;
    globalPressure   = NAN;
    googleSheetsSent = false;
  }
}

// ─── Google Sheets Bridge ─────────────────────────────────────────────────────
void sendGoogleSheetsData() {
  sendDataToServer(dtStamp,
                   sensordata.outsideTemp,
                   sensordata.insideTemp,
                   sensordata.insideHumidity,
                   thermostatSetpoint,
                   elapsedMinutes,
                   dailyTotalMinutes,
                   sensordata.outsidePressure,
                   sensordata.insidePressure,
                   sensordata.pressureDiffHPa);
}

void sendDataToServer(String updateTime, float outT, float inT, float inHum,
                      float therm, double eventM, double dailyM,
                      float outP, float inP, float diffP) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n\nSheet update blocked: Wi-Fi offline.");
    return;
  }

  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  HTTPClient http;

  // Outside node freshness: NAN => no reply to this cycle's poll => "Offline".
  // All three outside-derived columns go offline together (diff is
  // meaningless without a fresh outside pressure). Inside columns unaffected.
  bool bmeOffline = isnan(outT);

  String outTStr  = bmeOffline ? "Offline" : String(outT, 2);
  String outPStr  = bmeOffline ? "Offline" : String(toSeaLevelHPa(outP, outT) * 0.02953, 4);
  String diffPStr = bmeOffline ? "Offline" : String(diffP * 0.02953, 4);

  String data = "?lastUpdate="        + urlEncode(updateTime)
              + "&outsideTemp="       + outTStr
              + "&insideTemp="        + String(inT, 2)
              + "&insideHumidity="    + String(inHum, 2)
              + "&thermostat="        + String(therm, 2)
              + "&elapsedMinutes="    + String(eventM, 2)
              + "&dailyTotalMinutes=" + String(dailyM, 2)
              + "&outsidePressure="   + outPStr
              + "&insidePressure="    + String(toSeaLevelHPa(inP,  sensordata.insideTemp)  * 0.02953, 4)
              + "&pressureDiff="      + diffPStr   // diff stays absolute
              + "&cyclesToday="       + String(cyclesToday)
              + "&coastMinutes="      + String(lastCoastMinutes, 1)
              + "&avgCycleMinutes="   + String(avgCycleMinutes, 2);

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
    Serial.println("\n Secure connection failed.");
  }
}

// ─── Local File Logging ───────────────────────────────────────────────────────
void logData() {
  if (!LittleFS.exists("/log.txt")) {
    File setupFile = LittleFS.open("/log.txt", FILE_WRITE);
    if (!setupFile) return;
    setupFile.println("Timestamp, OutsideTemp, InsideTemp, InsideHumidity, Setpoint, EventDuration, TotalRuntime, OutsidePressure_inHg, InsidePressure_inHg, PressureDiff_inHg, CyclesToday, CoastMin, AvgCycleMin");
    setupFile.close();
  }
  File appendLog = LittleFS.open("/log.txt", FILE_APPEND);
  if (!appendLog) return;

  // Mirror the Sheet's Offline handling so LittleFS agrees with Google Sheets.
  bool bmeOffline = isnan(sensordata.outsideTemp);

  appendLog.print(dtStamp);
  appendLog.print(" , Outside: ");
  if (bmeOffline) appendLog.print("Offline");
  else            appendLog.print(sensordata.outsideTemp, 2);
  appendLog.print(" F, Inside: ");      appendLog.print(sensordata.insideTemp, 2);
  appendLog.print(" F, In.Humidity: "); appendLog.print(sensordata.insideHumidity, 2);
  appendLog.print(" %, Thermostat: ");  appendLog.print(thermostatSetpoint, 2);
  appendLog.print(" F, Event: ");       appendLog.print(sensordata.lastEventMinutes, 2);
  appendLog.print(", Total 24HR: ");    appendLog.print(sensordata.dailyTotalMinutes, 2);
  appendLog.print(", OutP: ");
  if (bmeOffline) appendLog.print("Offline");
  else            appendLog.print(toSeaLevelHPa(sensordata.outsidePressure, sensordata.outsideTemp) * 0.02953, 4);
  appendLog.print(" inHg, InP: ");      appendLog.print(toSeaLevelHPa(sensordata.insidePressure,  sensordata.insideTemp)  * 0.02953, 4);
  appendLog.print(" inHg, Diff: ");
  if (bmeOffline) appendLog.print("Offline");
  else            appendLog.print(sensordata.pressureDiffHPa * 0.02953, 4);
  appendLog.print(" inHg, Cycles: ");   appendLog.print(cyclesToday);
  appendLog.print(", Coast: ");         appendLog.print(lastCoastMinutes, 1);
  appendLog.print(" min, AvgCycle: ");  appendLog.print(avgCycleMinutes, 2);
  appendLog.print(" min\n");
  appendLog.close();
}

// ─── Local File Logging ───────────────────────────────────────────────────────
void logDailyTotal(String stamp, double dailyM, double eventM) {
  if (!LittleFS.exists("/daily_totals.csv")) {
    File setupFile = LittleFS.open("/daily_totals.csv", FILE_WRITE);
    if (!setupFile) return;
    setupFile.println("Timestamp,DailyTotalMinutes,LastEventMinutes,CyclesToday,AvgCycleMinutes");
    setupFile.close();
  }
  File appendLog = LittleFS.open("/daily_totals.csv", FILE_APPEND);
  if (!appendLog) return;
  appendLog.print(stamp);
  appendLog.print(",");
  appendLog.print(dailyM, 2);
  appendLog.print(",");
  appendLog.print(eventM, 2);
  appendLog.print(",");
  appendLog.print(cyclesToday);
  appendLog.print(",");
  appendLog.println(avgCycleMinutes, 2);
  appendLog.close();
}

// ─── State Persistence (poweron/brownout recovery) — NVS-based ─────────────
// Moved off LittleFS after a July 15, 2026 corruption event (Corrupted dir
// pair at {0x0,0x1}, mount fail -84) triggered by abrupt power-pull during
// a cal session. NVS (Preferences) uses a log-structured write scheme
// specifically tolerant of interrupted writes -- same rationale as the
// blower node's existing dailyTotalMinutes persistence.
// July 19: cycle-tracking state (cyclesToday, lastOffEpoch) added.
void saveState() {
  statePrefs.begin(STATE_NAMESPACE, false);   // read/write
  statePrefs.putDouble("elapsed", elapsedMinutes);
  statePrefs.putDouble("dailyTot", dailyTotalMinutes);
  statePrefs.putInt("cycles", cyclesToday);
  statePrefs.putLong64("lastOff", (int64_t)lastOffEpoch);
  statePrefs.end();
}

bool restoreState() {
  statePrefs.begin(STATE_NAMESPACE, true);    // read-only
  bool exists = statePrefs.isKey("dailyTot");
  if (exists) {
    elapsedMinutes    = statePrefs.getDouble("elapsed",  0.0);
    dailyTotalMinutes = statePrefs.getDouble("dailyTot", 0.0);
    cyclesToday       = statePrefs.getInt("cycles", 0);
    lastOffEpoch      = (time_t)statePrefs.getLong64("lastOff", 0);
    if (cyclesToday > 0) {
      avgCycleMinutes = dailyTotalMinutes / (double)cyclesToday;
    }
  }
  statePrefs.end();
  return exists;
}

// ─── Reset Reason Logging ───────────────────────────────────────────────────
// Distinguishes expected power-cycles (e.g. laptop -> wall wart swap after
// an offset update, which clears RTC_DATA_ATTR dailyTotalMinutes by design)
// from unexpected faults (brownout, watchdog, panic) that would otherwise
// look identical in the daily_totals.csv record -- a reset there, either way.
const char* getResetReasonStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "Power-on";        // normal power-cycle
    case ESP_RST_EXT:       return "External pin";
    case ESP_RST_SW:        return "Software reset";
    case ESP_RST_PANIC:     return "Exception/panic";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "Task watchdog";
    case ESP_RST_WDT:       return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep sleep wake";
    case ESP_RST_BROWNOUT:  return "Brownout";         // supply fault -- watch for this one
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "Unknown";
  }
}

void logResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  const char* reasonStr = getResetReasonStr(reason);

  if (!LittleFS.exists("/reset_log.txt")) {
    File setupFile = LittleFS.open("/reset_log.txt", FILE_WRITE);
    if (!setupFile) return;
    setupFile.println("Timestamp,ResetReason");
    setupFile.close();
  }

  File appendLog = LittleFS.open("/reset_log.txt", FILE_APPEND);
  if (!appendLog) {
    Serial.println("logResetReason: failed to open /reset_log.txt for append");
    return;
  }
  appendLog.print(dtStamp);
  appendLog.print(",");
  appendLog.println(reasonStr);
  appendLog.close();

  Serial.printf("Reset reason logged: %s , %s\n", dtStamp.c_str(), reasonStr);
}

// ─── Stubs ────────────────────────────────────────────────────────────────────
void updateHeatingData()    {}
void displayData()          {}
void disabledailyCooling()  {}
void resetDailyStats()      {}
void temperatureInterrupt() {}
