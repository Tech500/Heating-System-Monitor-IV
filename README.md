# Heating System Monitor IV

**Collaboration with myself, Claude (Anthropic's AI), and assistance from Gemini (Google's AI).**

No electrical connection to the heating system — monitors temperatures and blower runtime
using vibration detection only. Works in both cooling and heating seasons — year round!

---

## Why use Heating System Monitor IV?

**Answer questions your thermostat can't.** A thermostat knows the setpoint. HSM IV knows
what the system actually *did*: how many minutes the blower ran today, how long each cycle
lasted, how long the house held temperature between cycles, and what the indoor and outdoor
conditions were at the time. Over days and weeks, that data answers real questions:

- Is my system short-cycling?
- Which thermostat setpoint gives long, efficient cycles instead of frequent short ones?
- How does runtime track outdoor temperature? (Runtime per degree-day is your building's
  thermal signature — a sudden change flags a filter, refrigerant, or duct problem before
  the utility bill does.)
- Is the system degrading over time?

**No electrical connection to your HVAC system.** Blower detection is purely mechanical —
an MPU-6050 IMU sensing vibration from outside the blower enclosure. Nothing is wired into
the furnace or air handler. Nothing voids a warranty, nothing touches line voltage, and the
whole monitor installs and removes without a trace — ideal for renters and apartments.

**Owns its data.** Every event is logged twice: locally to the ESP32's flash (LittleFS,
retrievable over FTP) and to a perpetual Google Sheet that rolls month-to-month and
year-to-year automatically. No cloud subscription, no vendor account, no app.

**Survives real-world conditions.** Runtime totals persist through power failures via
NVS flash storage. A reset-reason log distinguishes normal power cycles from brownouts and
watchdog resets. If the outdoor sensor node goes silent, its columns log "Offline" instead
of silently repeating stale data.

---

## Overview

Heating System Monitor IV is a three-node ESP32 system using ESP-NOW for wireless
communication between nodes.

**How blower detection works (variance-based thresholding):**
Blower state is detected with an MPU-6050 6-axis IMU attached to the outside of the blower
enclosure. The sketch computes the statistical variance of accelerometer samples over a
short window. A running blower produces mechanical vibration (high variance); a stopped
blower produces almost none (low variance). Hysteresis between the ON and OFF thresholds
prevents chatter at the transitions. No electrical hookup to the heating/cooling system is
required, and no microphone is involved — it is immune to room noise.

**Logged data includes:** NTP timestamp, outside temperature, inside temperature, inside
humidity, thermostat setpoint, elapsed blower minutes (per cycle), daily total blower
minutes, outside pressure, inside pressure, pressure difference (out − in), cycles today,
coast minutes (hold time between cycles), and average cycle minutes.

Each record is written to both a local LittleFS log file and a perpetual Google Sheet
(month-to-month, year-to-year).

[Inspiration for the perpetual Google Sheet](https://iotdesignpro.com/articles/esp32-data-logging-to-google-sheets-with-google-scripts)

---

## Cycle tracking (thermostat optimization)

The receiver tracks blower ON/OFF transitions to build per-cycle statistics:

| Metric            | Meaning                                                        |
|-------------------|----------------------------------------------------------------|
| `cyclesToday`     | Blower starts since midnight                                   |
| `coastMinutes`    | How long the house held temperature before this cycle started  |
| `avgCycleMinutes` | Daily total runtime ÷ cycles today                             |

Comparing these across different thermostat setpoints (held for a few days each,
normalized by inside–outside temperature difference) reveals the setpoint that yields
long, lazy cycles — the efficient operating point — instead of frequent short-cycling.
`coastMinutes ÷ (coastMinutes + elapsedMinutes)` per row gives duty cycle directly.

---

## Hardware

### Receiver Node (`ESP_NOW__Receiver`)
- ESP32 DevKit-style module (HW-394)
- BME280 sensor — inside temperature, humidity, barometric pressure (I²C on GPIO4/GPIO5)
- Write-status LED on GPIO23 — ON while flash writes are in progress ("safe to pull power" indicator)
- Connects to Wi-Fi; posts each record to Google Sheets over HTTPS
- FTP server for retrieving LittleFS log files
- NVS (Preferences) state persistence: daily runtime and cycle count survive power loss
- Reset-reason logging to `/reset_log.txt`
- Daily totals appended to `/daily_totals.csv` at the nightly rollover

### BME280 Node (`ESP_NOW_BME280`)
- ESP32 DevKit V1 module
- BME280 sensor — outside temperature, humidity, barometric pressure
- Housed in a Stevenson screen; replies to receiver polls over ESP-NOW
- (Planned: migration to EoRa-S3-900TB LoRa board with deep sleep for battery operation)

### Blower Node (`ESP_NOW_Blower`)
- ESP32-S3 Super Mini module
- MPU-6050 IMU — vibration-based blower detection, mounted on the blower enclosure
- NVS persistence of daily total across power cycles

---

## ESP-NOW Configuration

| Parameter       | Value                                        |
|-----------------|----------------------------------------------|
| Channel         | 0 (follows the receiver's home Wi-Fi channel)|
| Blower Node MAC | E4:65:B8:20:20:A0                            |
| BME280 Node MAC | E4:65:B8:25:42:F8                            |
| ESP32 Core      | 3.3.10                                       |
| Wi-Fi Mode      | WIFI_MODE_APSTA                              |

> **Note:** MAC addresses shown are examples from this build. Replace them with the actual
> MAC addresses of your ESP32 modules. Each ESP32 has a unique MAC — run a brief sketch
> calling `WiFi.macAddress()` on each board to find them. Channel 0 means the ESP-NOW peers
> use whatever channel the receiver's Wi-Fi connection lands on, so all nodes stay in sync
> with the home router automatically.

---

## Google Apps Script Setup

1. In Google Sheets, create a new Google Sheet.
2. Note the **Sheet ID** from the URL:
   `https://docs.google.com/spreadsheets/d/`**`<YOUR_SHEET_ID>`**`/edit`
3. Open the Script Editor: **Extensions → Apps Script**.
4. Delete the default `myFunction()` stub entirely.
5. Copy the full text contents of `Code.gs` from this repository.
6. Paste into the Script Editor.
7. Replace the placeholder Sheet ID in the script with the Sheet ID noted in Step 2.
8. **Save** (Ctrl+S or the save icon).
9. Click **Deploy → New Deployment**.
10. Select type: **Web App**.
11. Set **Execute as:** Me.
12. Set **Who has access:** Anyone.
13. Click **Authorize** → **Advanced** → click your Gmail account → **Allow**.
14. Copy the deployment ID and paste it into the Receiver sketch as the Google Script endpoint.

> **Note:** If you redeploy after changes, create a **New Deployment** each time and update
> the deployment ID in the Receiver sketch to match. (Editing an existing deployment to a
> new version can preserve the ID, but a fresh deployment always works.)

Numeric fields pass through a `numOrText()` helper in `Code.gs`: numbers are stored as
numbers, and the receiver's "Offline" sentinel (sent when the outdoor node fails to reply)
is preserved as text. Sheets functions like AVERAGE skip text cells automatically; in
pandas, load with `na_values=['Offline']`.

[Live perpetuial Google Sheet](https://docs.google.com/spreadsheets/d/1I8YJg2rJ6niNyHXX-MTjp9aYh68fHtGoeyfJ15A9JZE/edit?usp=sharing)

---

## Logged Data Format

```
["MM/DD/YYYY HH:MM:SS", outsideTemp, insideTemp, insideHumidity, thermostat,
 elapsedMinutes, dailyTotalMinutes, outsidePressure, insidePressure, pressureDiff,
 cyclesToday, coastMinutes, avgCycleMinutes]
```

Pressures are sea-level corrected (temperature-compensated hypsometric formula) and
reported in inHg. The pressure difference is absolute (both sensors at the same elevation).

---

## Repository Contents

| Folder / File                 | Description                             |
|-------------------------------|-----------------------------------------|
| `ESP_NOW_Blower/`             | Blower detection node sketch            |
| `ESP_NOW_BME280/`             | BME280 environmental sensor node sketch |
| `ESP_NOW__Receiver/`          | Receiver / logger / Google Sheets node  |
| `Schematics/`                 | Node wiring of components               |
| `Code.gs`                     | Google Apps Script for Sheets logging   |
| `Heat System Monitor III.mp4` | Project demonstration video, download   |

---

## Dependencies

- Arduino ESP32 Core 3.3.10
- BME280 — [Tyler Glenn BME280I2C library](https://github.com/finitespace/BME280/tree/master)
- MPU-6050 — [Electronic Cats MPU6050 library](https://github.com/ElectronicCats/mpu6050)
- LittleFS and Preferences (built into ESP32 Arduino Core)

---

## License

MIT License — see `LICENSE` for details.
