# Heating System Monitor III

**Intensive collaboration with myself, Claude AI (Anthropic), and Gemini AI (Google).**

No electrical connection to the heating system — monitors temperatures and blower run time only via acoustical detection.
Works in both cooling and heating seasons --year round!

## Overview

Heating System Monitor III is a three-node ESP32 system using ESP-NOW for wireless communication.
The blower node acoustically detects HVAC blower on/off cycles using a KY-038 microphone module
and variance-based thresholding. No electrical hookup to the heating system is required.

What is variance-based thresholding?

Blower state is inferred acoustically by computing the statistical variance of ADC samples taken from the KY-038 microphone module's analog output. A running blower produces broadband mechanical noise, causing significant ADC sample fluctuation (high variance). A quiet room produces near-flat ADC output (low variance). The KY-038 onboard potentiometer sets the sensitivity threshold — adjusted once at installation with the blower off.

Logged data includes: timestamp, outside temperature, inside temperature, register temperature,
thermostat temperature, elapsed blower time, and daily total blower time — written to both a
local LittleFS log file and a perpetual Google Sheet (month-to-month, year-to-year).

[Inspiration for the perpetual Google Sheet](https://iotdesignpro.com/articles/esp32-data-logging-to-google-sheets-with-google-scripts)

---

## Hardware

### Receiver Node (HeatingMonitor_Receiver)
- ESP32 Dev Module
- MCP9808 temperature sensor (I2C address 0x18) — register/inside temperature
- Connects to Wi-Fi and Google Sheets via HTTP POST

### BME280 Node (ESP_NOW_BME280)
- ESP32 Dev Module
- BME280 sensor — temperature, humidity, barometric pressure

### Blower Node (ESP_NOW_Blower)
- ESP32 Dev Module
- KY-038 microphone module — acoustical blower detection

---

## ESP-NOW Configuration

| Parameter       | Value             |
|-----------------|-------------------|
| Channel         | 11                |
| Blower Node MAC | E4:65:B8:20:20:A0 |
| BME280 Node MAC | E4:65:B8:25:42:F8 |
| Arduino Core    | 3.3.10            |
| Wi-Fi Mode      | WIFI_MODE_APSTA   |

> **Note:** MAC addresses shown are examples from this build. Replace with the actual MAC addresses
> of your ESP32 modules. Each ESP32 has a unique MAC — use a brief sketch calling
> `WiFi.macAddress()` to discover yours.

---

## KY-038 Microphone — Threshold Adjustment

The KY-038 uses variance-based thresholding for blower detection. Proper pot adjustment
is critical for reliable, clean on/off transitions.

**Recommended: Adjust at the bench, away from the furnace room.** Working space around
the furnace is typically tight and dark, making the tiny pot screw difficult to access.

**Use a diddle stick (trimmer adjustment tool) if available** — it fits over the pot screw
and gives much better control than a standard screwdriver.

> **Caution:** The potentiometer is a multi-turn trimmer (10 or more turns). If it begins
> to tighten up, **stop turning immediately** — the wiper can be damaged if forced past
> its end stop.

**Orientation: Hold the KY-038 with the pin header to the right.**
- Counter-clockwise — reduces sensitivity
- Clockwise — increases sensitivity

**With the heating system blower OFF and the environment quiet:**

1. Power the system and observe the KY-038 digital output LED (LED2, adjacent to the potentiometer).
2. Rotate the pot until LED2 begins to flicker slowly.
3. Continue adjusting until LED2 just extinguishes and remains off.
4. This sets the detection threshold just above the ambient noise floor, ensuring reliable
   blower-on detection with a clean off-state baseline.

---

## Google Apps Script Setup

1. In Google Shhets, create a new Google Sheet.
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
14. Copy the deployment URL and paste it into the Receiver sketch as the Google Script id endpoint.

> **Note:** If you redeploy after changes, you must create a **New Deployment** each time —
> not "Manage existing." The Receiver sketch URL must be updated to match the new deployment URL.

---

## Logged Data Format

```
["MM/DD/YYYY HH:MM:SS", outsideTemp, insideTemp, registerTemp, thermostat, elapsedMin, dailyTotalMin]
```

---

## Repository Contents

| Folder / File                 | Description                             |
|-------------------------------|-----------------------------------------|
| `ESP_NOW_Blower/`             | Blower detection node sketch            |
| `ESP_NOW_BME280/`             | BME280 environmental sensor node sketch |
| `ESP_NOW__Receiver/`          | Receiver / logger / Google Sheets node  |
| `Code.gs`                     | Google Apps Script for Sheets logging   |
| `Heat System Monitor III.mp4` | Project demonstration video             |

---

## Dependencies

- Arduino ESP32 Core 3.3.10
- Tylor Glenn BME280I2C.h Library
- MCP9808 (direct Wire register reads — no library required)
- LittleFS (built into ESP32 Arduino Core)

---

## License

MIT License — see `LICENSE` for details.
