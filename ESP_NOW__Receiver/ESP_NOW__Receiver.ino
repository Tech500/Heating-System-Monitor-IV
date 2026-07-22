/* Heating System Monitor IV
   ESP_NOW_Receiver.ino with temperature Offset + LoRa WOR trigger
   July 19, 2026 (LoRa merge)
   ESP32-NOW, ESP32 Core 3.3.10
   Hub now runs on EoRa-S3-900TB (ESP32-S3 + onboard SX1262) -- same board
   family as the outside sensor node.

   Receives MSG_BLOWER_STATE from ESP_NOW_Blower node
   Receives MSG_BME280 from ESP_NOW_BME280 node (outside node's ESP-NOW reply)
   Sends LoRa WOR to wake the outside node (replaces the old ESP-NOW
   sendAlert()/delay(1000) poll, which only worked because that node used
   to be always-on and already listening).

   *** HARDWARE CONFLICT -- MUST RESOLVE BEFORE FLASHING ***
   Inside BME280 I2C is currently defined SDA=4, SCL=5. The EoRa-S3-900TB's
   onboard SX1262 has RADIO_SCLK_PIN fixed at GPIO5 (see pin block below).
   GPIO5 cannot be both the LoRa radio's SCLK and the inside BME280's I2C
   clock. Reassign SCL (and confirm SDA is actually free too) to pins that
   are genuinely unused by the radio/OLED/SD on your board -- check the
   EoRa Pi user manual pinout. Placeholder pins below are marked TODO and
   will NOT work as-is.

   --- Prior update history preserved from original file ---
   (BME280I2C library, HW-394 3.3V rail fix, hypsometric sea-level pressure,
   alertFlag OFF-transition gating, NAN/"Offline" freshness handling,
   cycle-tracking / coast-time study -- all unchanged, see inline comments.)

   --- LoRa merge, July 19, 2026 ---
   Hub's LoRa radio is TRANSMIT-ONLY -- it never listens over LoRa. The
   outside node is the one sitting in startReceiveDutyCycleAuto(); the
   hub only needs to wake its own sleeping radio (implicit SPI wake) and
   fire a WOR packet, then go straight back to radio.sleep(). The outside
   node's actual sensor reply comes back over ESP-NOW (MSG_BME280), same
   as before -- only the trigger mechanism changed, not the reply path.
   Link params (SF7 / BW500 / 2dBm) optimized for the real ~20ft link,
   MUST MATCH the outside node's radio.begin() exactly.
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
#include <FTPServer.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "rom/rtc.h"
#include "esp_system.h"   // esp_reset_reason() -- for reset_log.txt
#include <Preferences.h>

// ─── LoRa (EoRa-S3-900TB onboard SX1262) ─────────────────────────────────────
#define EoRa_PI_V1
#include <RadioLib.h>
#include <SPI.h>

// Local pin defines -- boards.h/utilities.h (Ebyte OEM files) intentionally
// NOT included here. Their initBoard()/init chain was the confirmed source
// of the check_i2c_driver_conflict crashes on the outside node -- same fix
// applied here for consistency, since both boards share the identical
// underlying conflict risk.
#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 3
#define RADIO_MOSI_PIN 6
#define RADIO_CS_PIN   7
#define RADIO_DIO1_PIN 33
#define RADIO_BUSY_PIN 34
#define RADIO_RST_PIN  8
#define BOARD_LED      37
#define LED_ON         HIGH
#define LED_OFF        LOW

float radioFreq = 915.0;
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

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

// I2C for inside BME280 -- explicit Wire.begin() on documented general-
// purpose header pins. GPIO17/18 (initBoard()'s I2C_SDA/I2C_SCL) are NOT
// on the main header per the pin mapping doc -- likely dedicated OLED
// connector pads, not reachable for external sensor wiring. GPIO42/41
// are confirmed general I/O, no radio/SD/strapping/RTC-wake conflicts.
// I2C for inside BME280 -- explicit setPins()/begin() on GPIO48/47.
// CONFIRMED WORKING via standalone BME280I2C test sketch: stable,
// consistent readings across many consecutive loops, no flicker.
// GPIO42/41 and GPIO40/39 were both intermittent on this board despite
// being valid, documented general-purpose pins.
#define BME_SDA 48
#define BME_SCL 47

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
unsigned long lastResetCheck = 0;
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

// BME280 node — bidirectional (ESP-NOW reply path only now; wake trigger
// moved to LoRa WOR, see sendOutsideWakeRequest())
class BME280Peer : public ESP_NOW_Peer {
  public:
    BME280Peer(const uint8_t *mac_addr, uint8_t channel,
               wifi_interface_t iface = WIFI_IF_STA,
               const uint8_t *lmk = NULL)
      : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

    bool add_to_system() { return ESP_NOW_Peer::add(); }

  protected:
    void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
      processIncomingPacket(data, (int)len);
    }
};

// ─── Hardware MAC Map ─────────────────────────────────────────────────────────
uint8_t senderBlowerMAC[] = { 0x9C, 0x13, 0x9E, 0xF2, 0x2A, 0xB4 };
// TODO: senderBmeMAC was the OLD always-on DevKit's WiFi MAC. The
// EoRa-S3-900TB outside node is different hardware with its own MAC --
// print WiFi.macAddress() on the new board and update this before the
// hub will recognize its ESP-NOW replies.
uint8_t senderBmeMAC[]    = { 0xD0, 0xCF, 0x13, 0x0A, 0x48, 0x90 };
#define CHANNEL 0

HeatingSenderPeer blowerNode(senderBlowerMAC, CHANNEL, WIFI_IF_STA);
BME280Peer        bmeNode(senderBmeMAC,       CHANNEL, WIFI_IF_STA);

// ─── Global Tracking Registers ───────────────────────────────────────────────
bool blowerIsOn        = false;
bool alertFlag         = false;
bool googleSheetsSent  = false;

double elapsedMinutes                    = 0.0;
double dailyTotalMinutes   = 0.0;

Preferences statePrefs;
const char* STATE_NAMESPACE = "hsm4state";

// ─── Cycle Tracking (thermostat optimization study) ──────────────────────────
bool   prevBlowerIsOn   = false;
int    cyclesToday      = 0;
time_t lastOffEpoch     = 0;
double lastCoastMinutes = 0.0;
double avgCycleMinutes  = 0.0;

// Outside BME280 registers -- NAN = "no fresh reading this cycle".
float globalTemp         = NAN;
float globalHumidity     = NAN;
float globalPressure     = NAN;
float thermostatSetpoint = 75.0;

// ─── Elevation / Sea-Level Pressure Conversion ───────────────────────────────
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
void setupLoRa();
void sendOutsideWakeRequest();

// ─── LoRa Setup (transmit-only on this node) ─────────────────────────────────
// Hub never listens over LoRa -- the outside node is the one sitting in
// startReceiveDutyCycleAuto(). All this node does is wake its own sleeping
// radio (implicit on any SPI transaction) and fire a WOR packet, then
// return to radio.sleep(). Link params MUST MATCH the outside node exactly.
void setupLoRa() {
  // initBoard() (Ebyte's boards.h) intentionally NOT called -- see note
  // above the pin defines. SPI needs to be brought up manually in its
  // place, same pins initBoard() would have used.
  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
  delay(1500);

  Serial.print(F("[SX126x] Initializing ... "));
  int state = radio.begin(
    radioFreq, 500.0, 7, 7,
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
    2, 512, 0.0, true
  );
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  radio.sleep();  // idle here between events -- hub is mains-powered so
                   // this isn't about battery life, just a clean resting
                   // state; SPI wakes it automatically on the next call.
}

// Wake the sleeping radio (implicit via SPI) and transmit the WOR that
// wakes the outside node's ESP32 via DIO1 -> EXT0. Outside node's actual
// sensor reply still comes back over ESP-NOW (MSG_BME280), unchanged.
void sendOutsideWakeRequest() {
  Serial.println("[LoRa] Waking radio and sending WOR to outside BME280 node...");

  pinMode(RADIO_BUSY_PIN, INPUT);
  Serial.printf("[LoRa] BUSY before standby: %d\n", digitalRead(RADIO_BUSY_PIN));
  int standbyState = radio.standby();
  Serial.printf("[LoRa] BUSY after standby: %d\n", digitalRead(RADIO_BUSY_PIN));
  if (standbyState != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] standby() failed, code %d\n", standbyState);
  }

  int state = radio.transmit("WOR");
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] WOR sent OK");
  } else {
    Serial.printf("[LoRa] WOR send failed, code %d\n", state);
  }

  radio.sleep();  // back to idle -- no LoRa listening needed on this node
}

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

  if (isnan(temp) || isnan(pres)) {
    Serial.println("Error reading inside BME280 sensor telemetry.");
    tempF       = NAN;
    humidity    = NAN;
    pressureHPa = NAN;
    return;
  }

  temp += BME280_INSIDE_TEMP_CAL_OFFSET_F;

  tempF       = temp;
  humidity    = hum;
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

        if (blowerPacket.on && !prevBlowerIsOn) {
          cyclesToday++;
          if (lastOffEpoch > 0) {
            lastCoastMinutes = (double)(time(nullptr) - lastOffEpoch) / 60.0;
          } else {
            lastCoastMinutes = 0.0;
          }
          Serial.printf("[Cycle] #%d started. Coast (hold) before this cycle: %.1f min\n",
                        cyclesToday, lastCoastMinutes);
        }

        if (!blowerPacket.on && prevBlowerIsOn) {
          lastOffEpoch = time(nullptr);
        }

        if (!blowerPacket.on) {
          dailyTotalMinutes += blowerPacket.elapsedMinutes;
          sensordata.dailyTotalMinutes = dailyTotalMinutes;
          if (cyclesToday > 0) {
            avgCycleMinutes = dailyTotalMinutes / (double)cyclesToday;
          }
          Serial.printf("[Cycle] #%d complete. ON: %.2f min  Avg cycle today: %.2f min\n",
                        cyclesToday, blowerPacket.elapsedMinutes, avgCycleMinutes);
          alertFlag = true;
          saveState();
        }

        prevBlowerIsOn = blowerPacket.on;
      } else {
        Serial.printf("\n\n Size Mismatch — BlowerData: Expected %d got %d bytes\n",
                      sizeof(BlowerData), len);
      }
      break;

    case MSG_ALERT_FLAG:
      // No longer sent by this hub (WOR replaces it) -- left in case the
      // outside node or a future node still emits one.
      if (len == sizeof(AlertFlag)) {
        AlertFlag alertPacket;
        memcpy(&alertPacket, data, sizeof(AlertFlag));
        Serial.printf("\n[Radio Link] AlertFlag received: %s\n",
                      alertPacket.alert ? "true" : "false");
      }
      break;

    case MSG_BME280:
      if (len == sizeof(BME280Data)) {
        BME280Data bmePacket;
        memcpy(&bmePacket, data, sizeof(BME280Data));
        globalHumidity = bmePacket.humidity;
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
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nHeating System Monitor IV - ESP_NOW_Receiver + LoRa WOR\n");
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

  // LoRa radio init first (SPI + radio.begin()). I2C for the inside
  // BME280 is set up separately right after, on GPIO48/47 -- no OLED-pin
  // bus to override now that initBoard() isn't called.
  setupLoRa();
  // Mirror the exact sequence proven working on the bench
  // (BME280I2CTest.ino): Wire.end() first, then setPins() + Wire.begin
  // (sda, scl) WITH the pin arguments -- the driver_ng crash was fixed
  // by adding Wire.end(), not by dropping the pin arguments.
  Wire.end();
  delay(50);
  Wire.setPins(BME_SDA, BME_SCL);
  if (!Wire.begin(BME_SDA, BME_SCL)) {
    Serial.println("Core 3.3.10 failed to allocate I2C peripheral instance!");
  }
  delay(50);

  initNTP();
  initBME280Inside();

  bool fsok = LittleFS.begin(true);
  Serial.printf("FS init: %s\n", fsok ? "ok" : "fail!");

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

  if (currentMillis - lastResetCheck >= checkInterval) {
    lastResetCheck = currentMillis;
    getDateTime();
  }

  static bool didMidnightReset = false;

  if (HOUR == 23 && MINUTE == 58) {
    if (!didMidnightReset) {
      digitalWrite(WRITE_LED_PIN, HIGH);

      double yesterdayDailyTotalMinutes = dailyTotalMinutes;
      double yesterdayElapsedMinutes    = elapsedMinutes;

      Serial.printf("Midnight snapshot -> Elapsed: %.2f min  Daily: %.2f min  Cycles: %d  AvgCycle: %.2f min\n",
                    yesterdayElapsedMinutes, yesterdayDailyTotalMinutes,
                    cyclesToday, avgCycleMinutes);

      logDailyTotal(dtStamp, yesterdayDailyTotalMinutes, yesterdayElapsedMinutes);

      dailyTotalMinutes = 0;
      cyclesToday       = 0;
      avgCycleMinutes   = 0.0;
      saveState();
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

    // LoRa WOR replaces the old ESP-NOW sendAlert()/delay(1000) poll --
    // the outside node is asleep between events now, so it can't answer
    // an ESP-NOW poll directly; it has to be woken via LoRa first.
    sendOutsideWakeRequest();

    // Outside node round trip: DIO1 wake -> ESP32 reboot -> setup() ->
    // radio init -> BME280 read -> WiFi reconnect -> ESP-NOW send. This
    // is meaningfully longer than the old always-on node's instant reply.
    // TODO: tune against measured round-trip time on the bench; this
    // blocks loop() (FTP, midnight check) for the full wait -- consider
    // converting to a non-blocking millis-based wait if that becomes a
    // problem in practice.
    delay(5000);   // starting point -- adjust once you've measured it

    float insideTempF = NAN, insideHumidity = NAN, insidePressureHPa = NAN;
    readBME280Inside(insideTempF, insideHumidity, insidePressureHPa);
    sensordata.insideTemp      = insideTempF;
    sensordata.insideHumidity  = insideHumidity;
    sensordata.outsideTemp     = globalTemp;
    sensordata.thermostat      = thermostatSetpoint;
    sensordata.outsidePressure = globalPressure;
    sensordata.insidePressure  = insidePressureHPa;
    sensordata.pressureDiffHPa = globalPressure - insidePressureHPa;

    if (isnan(sensordata.outsideTemp)) {
      Serial.println("[BME280 Outside] No reply this cycle -- posting Offline.");
    }

    Serial.printf("[BME280 Inside] Temp: %.2f °F  Hum: %.2f %%  Pres: %.4f inHg\n",
                  insideTempF, insideHumidity, insidePressureHPa * 0.02953);
    Serial.printf("[Pressure] Outside: %.4f inHg | Inside: %.4f inHg | Diff(out-in): %.4f inHg\n",
                  toSeaLevelHPa(sensordata.outsidePressure, sensordata.outsideTemp) * 0.02953,
                  toSeaLevelHPa(sensordata.insidePressure,  sensordata.insideTemp)  * 0.02953,
                  sensordata.pressureDiffHPa * 0.02953);

    getDateTime();
    displayData();
    logData();
    sendGoogleSheetsData();
    googleSheetsSent = true;

    digitalWrite(WRITE_LED_PIN, LOW);
  }

  if (googleSheetsSent) {
    elapsedMinutes   = 0;
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
              + "&pressureDiff="      + diffPStr
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
void saveState() {
  statePrefs.begin(STATE_NAMESPACE, false);
  statePrefs.putDouble("elapsed", elapsedMinutes);
  statePrefs.putDouble("dailyTot", dailyTotalMinutes);
  statePrefs.putInt("cycles", cyclesToday);
  statePrefs.putLong64("lastOff", (int64_t)lastOffEpoch);
  statePrefs.end();
}

bool restoreState() {
  statePrefs.begin(STATE_NAMESPACE, true);
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
const char* getResetReasonStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "Power-on";
    case ESP_RST_EXT:       return "External pin";
    case ESP_RST_SW:        return "Software reset";
    case ESP_RST_PANIC:     return "Exception/panic";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "Task watchdog";
    case ESP_RST_WDT:       return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep sleep wake";
    case ESP_RST_BROWNOUT:  return "Brownout";
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
