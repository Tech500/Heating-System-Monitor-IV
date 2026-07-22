/*
  EoRa Pi Foundation --Outside BME280 Sensor Node
  Wakes on incoming LoRa WOR from hub -> reads BME280 -> sends reading to
  hub via ESP-NOW -> sleeps.
  July 22, 2026

  CYCLE:
    1. Deep sleep, LoRa radio armed in startReceiveDutyCycleAuto() (SX1262
       keeps cycling listen windows in hardware while ESP32 core sleeps).
       LoRa's ONLY job on this node is the wake trigger -- it never
       carries sensor data.
    2. Hub's WOR packet arrives during a listen window -> DIO1 fires ->
       RTC_GPIO16 wakes ESP32 (EXT0) -> full reboot, setup() runs again.
    3. setup() re-inits LoRa (setupLoRa), reads BME280, sends the reading
       to the hub over ESP-NOW (WiFi radio, brief -- battery constraint
       reasoning from the design discussion: LoRa duty-cycle listen is
       cheap to hold continuously, ESP-NOW/WiFi is not, so WiFi only
       comes up for the short send).
    4. LoRa: radio.sleep() to cleanly end, then IMMEDIATELY re-arm
       radio.startReceiveDutyCycleAuto() before deep sleep -- required so
       the node stays wakeable by the hub's NEXT WOR.
    5. esp_deep_sleep_start().

  No command parsing / no load control on this node -- being woken IS
  the instruction. All it does every cycle is read and report.
*/

#include <RadioLib.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32_NOW.h>
#include <Wire.h>
#include <BME280I2C.h>          // Tyler Glenn / finitespace library -- matches hub
#include <SPI.h>
#include <FS.h>
#include <LittleFS.h>
#include <rom/rtc.h>
#include <driver/rtc_io.h>

int symbols = 512;

// EoRa Pi dev board pin configuration
#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 3
#define RADIO_MOSI_PIN 6
#define RADIO_CS_PIN 7
#define RADIO_DIO1_PIN 33
#define RADIO_BUSY_PIN 34
#define RADIO_RST_PIN 8
#define BOARD_LED 37

#define WAKE_PIN GPIO_NUM_16  // Inverted DIO1 signal for RTC wake-up

#define LED_ON HIGH
#define LED_OFF LOW

#define USING_SX1262_868M
#if defined(USING_SX1262_868M)
uint8_t txPower = 14;
float radioFreq = 915.0;
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

// I2C for BME280 -- explicit setPins()/begin() on GPIO48/47. CONFIRMED
// WORKING: stable, consistent readings on the bench, full end-to-end
// cycle (read -> ESP-NOW send -> sleep) verified. Also confirmed via
// your own EasyEDA schematic (INA226 already wired to these same two
// pins in the EoRa-PI-Foundation project). The actual root cause of
// earlier flakiness/crashes turned out to be Ebyte's initBoard()/
// boards.h chain conflicting with the newer I2C driver -- removing
// that OEM dependency entirely (not the pin choice) was the real fix.
#define BME_SDA 48
#define BME_SCL 47

BME280I2C bme;  // default constructor uses address 0x76

// ─── ESP-NOW: send reading to hub ────────────────────────────────────────────
enum MessageType : uint8_t {
  MSG_BME280       = 0,
  MSG_ALERT_FLAG   = 1,
  MSG_BLOWER_STATE = 2
};

struct __attribute__((packed)) BME280Data {
  MessageType type;
  float temperature;  // Fahrenheit
  float humidity;      // %RH
  float pressure;       // hPa -- NOT inHg. Hub applies its own
                          // toSeaLevelHPa()/inHg conversion downstream;
                          // sending inHg here would silently corrupt
                          // that math.
};

// Confirmed via Print_Hub_MAC_Channel.ino run on the hub board.
uint8_t hubMAC[] = { 0x1C, 0xDB, 0xD4, 0x85, 0x6E, 0x9C };

// Confirmed via Print_Hub_MAC_Channel.ino run on the hub board.
#define HUB_WIFI_CHANNEL 11

class HubPeer : public ESP_NOW_Peer {
  public:
    HubPeer(const uint8_t *mac_addr, uint8_t channel,
            wifi_interface_t iface = WIFI_IF_STA,
            const uint8_t *lmk = NULL)
      : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

    bool add_to_system() { return ESP_NOW_Peer::add(); }

    // Public wrapper -- ESP_NOW_Peer::send() is protected, can't be
    // called directly from outside the class (same pattern the hub's
    // own BME280Peer::sendAlert() uses).
    bool sendData(const uint8_t *data, size_t len) {
      return send(data, len);
    }

  protected:
    void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
      // Not expecting anything back from the hub on this node.
    }
};

HubPeer hub(hubMAC, HUB_WIFI_CHANNEL, WIFI_IF_STA);

void goToSleep(void) {
  Serial.println("=== PREPARING FOR DEEP SLEEP ===");

  // Re-arm duty cycle receive -- required so the node stays wakeable
  // by the hub's next WOR. Do NOT leave the radio in radio.sleep() here.
  radio.startReceiveDutyCycleAuto();

  rtc_gpio_pulldown_en(WAKE_PIN);
  esp_sleep_enable_ext0_wakeup(WAKE_PIN, RISING);

  digitalWrite(BOARD_LED, LED_OFF);

  Serial.println("Reading sent. Re-armed for next WOR. Deep sleep now...");
  Serial.flush();

  SPI.end();
  esp_deep_sleep_start();
}

void setupLoRa() {
  // initBoard() (Ebyte's boards.h) intentionally NOT called -- its I2C/
  // OLED/SD init chain was the actual source of the driver_ng crashes.
  // SPI is already brought up directly in setup(); radio.begin() below
  // doesn't need anything else initBoard() provided for this project.
  delay(1500);

  // Optimized for the actual ~20ft link: SF7 (fastest), BW500 (widest
  // standard bandwidth, shortest time-on-air), 2 dBm output. MUST MATCH
  // THE HUB'S SETTINGS EXACTLY.
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
    while (true);
  }
}

// Bring up WiFi/ESP-NOW just long enough to send the reading, then this
// gets torn back down before deep sleep (see setup()).
bool setupESPNOW() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // don't join the AP -- ESP-NOW doesn't need it

  esp_wifi_set_channel(HUB_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (!ESP_NOW.begin()) {
    Serial.println("ESP-NOW init failed");
    return false;
  }

  if (!hub.add_to_system()) {
    Serial.println("Failed to bind hub peer");
    return false;
  }

  Serial.println("ESP-NOW ready");
  return true;
}

bool readAndSendBME280() {
  // Mirror the exact sequence proven working on the bench
  // (BME280I2CTest.ino): Wire.end() first to tear down any implicit
  // default Wire state, then setPins() + Wire.begin(sda, scl) WITH the
  // pin arguments -- the driver_ng crash was fixed by adding Wire.end(),
  // not by dropping the pin arguments from begin() as originally assumed.
  Wire.end();
  delay(50);
  Wire.setPins(BME_SDA, BME_SCL);
  if (!Wire.begin(BME_SDA, BME_SCL)) {
    Serial.println("Core 3.3.10 failed to allocate I2C peripheral instance!");
  }
  delay(50);

  if (!bme.begin()) {
    Serial.println("BME280 not found -- check wiring/address");
    return false;
  }

  float tempF = NAN, humidity = NAN, pressureHPa = NAN;
  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);
  bme.read(pressureHPa, tempF, humidity, tempUnit, presUnit);

  if (isnan(tempF) || isnan(pressureHPa)) {
    Serial.println("Error reading BME280 telemetry.");
    return false;
  }

  // No explicit sleep-mode call needed -- BME280I2C's default
  // constructor uses forced mode, which takes one reading and returns
  // to sleep automatically. (setMode() doesn't exist on this library;
  // that was carried over incorrectly from the Adafruit API.)

  Serial.printf("BME280 -> Temp: %.2f F  Hum: %.2f %%  Pres: %.4f hPa\n",
                tempF, humidity, pressureHPa);

  if (!setupESPNOW()) {
    return false;
  }

  BME280Data pkt;
  pkt.type        = MSG_BME280;
  pkt.temperature = tempF;
  pkt.humidity    = humidity;
  pkt.pressure    = pressureHPa;   // hPa, matches hub's expected units

  bool sent = hub.sendData((uint8_t *)&pkt, sizeof(BME280Data));
  Serial.printf("[ESP-NOW] Send to hub: %s\n", sent ? "OK" : "FAILED");

  WiFi.mode(WIFI_OFF);  // done -- tear WiFi back down before deep sleep

  return sent;
}

// *** BENCH TEST ONLY -- REMOVE BEFORE FIELD DEPLOYMENT ***
// Forces the wake -> read -> ESP-NOW send path on every boot, regardless
// of actual wake reason. Lets you test the ESP-NOW leg independently of
// LoRa, which isn't proven yet on this corrected pin/library/core setup.
// Comment out (or delete) this line once LoRa WOR is verified working.
//#define BENCH_TEST_FORCE_WAKE

void setup() {
  Serial.begin(115200);
  while(!Serial){};
  delay(200);
  Serial.println("*** REACHED SETUP ***");
  Serial.flush();

  setCpuFrequencyMhz(80);

  bool fsok = LittleFS.begin(true);
  Serial.printf_P(PSTR("\nFS init: %s\n"), fsok ? PSTR("ok") : PSTR("fail!"));

  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

  // eora_s3_power_mgmt.h (Bluetooth/ADC disable helpers) removed along
  // with the other Ebyte OEM files. Bluetooth/ADC aren't used on this
  // node, so leaving them at default (not explicitly disabled) costs
  // a little idle power but isn't a functional problem. Worth revisiting
  // if the battery budget gets tight later.

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  setupLoRa();

#ifdef BENCH_TEST_FORCE_WAKE
  Serial.println("*** BENCH TEST MODE -- forcing wake/read/send path ***");
  digitalWrite(BOARD_LED, LED_ON);

  readAndSendBME280();

  radio.sleep();
  goToSleep();
  return;
#endif

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke from hub WOR -- reading and sending BME280 data");
    digitalWrite(BOARD_LED, LED_ON);

    readAndSendBME280();

    radio.sleep();     // cleanly end LoRa side
    goToSleep();        // re-arms LoRa duty cycle receive, then deep sleeps
    return;
  }

  // Power-on reset / cold boot -- arm and wait for first WOR
  Serial.println("Power-on reset -- arming duty cycle and going to sleep");
  goToSleep();
}

void loop() {
  // Intentionally empty -- this node does everything in setup() per
  // wake cycle and returns to deep sleep. No command parsing needed;
  // being woken is the only instruction it acts on.
}
