# Heating System Monitor IV  

**Collaboration with myself, Claude (Anthropic's AI), and asistance from Gemini (Google's AI).**

No electrical connection to the heating system — monitors temperatures and blower run time only via acoustical detection.
Works in both cooling and heating seasons --year round!

## Overview

Heating System Monitor IV is a three-node ESP32 system using ESP_NOW for wireless communication.
The blower node; HVAC blower on/off cycles are detected using a  MPU-6050; popular 6-axis Inertial Measurement Unit (IMU)  module
and variance-based thresholding. No electrical hookup to the heating/cooling system is required.

What is variance-based thresholding?

Blower state is detected using a MPU6050 IMU; by computing the statistical variance of ADC samples taken from the MPU6050 module's I²C output. A running blower produces mechanical vibration, causing significant vibration (high variance). A blower that is off; produces way less vibration (low variance). The MPU6050 IMU requires no threshoold adjustment.

Logged data includes: timestamp, outside temperature, inside temperature, register temperature,
thermostat temperature, elapsed blower time, and daily total blower, outside pressure, inside pressure, and difference presssure.  NTP time stamp is written to both a local LittleFS log file and a perpetual Google Sheet (month-to-month, year-to-year).

[Inspiration for the perpetual Google Sheet](https://iotdesignpro.com/articles/esp32-data-logging-to-google-sheets-with-google-scripts)

---

## Hardware

### Receiver Node (HeatingMonitor_Receiver)
- ESP32 DevKit V1 Module
- BME280 sensor — inside temperature, humidity, barometric pressure
- Connects to Wi-Fi and Google Sheets via HTTP POST

### BME280 Node (ESP_NOW_BME280)
- ESP32 DevKit V1v Module
- BME280 sensor — outside temperature, humidity, barometric pressure

### Blower Node (ESP_NOW_Blower)
- ESP32 DevKit V1 Module
- MPU6050 — IMU blower detection

---

## ESP-NOW Configuration

| Parameter       | Value             |
|-----------------|-------------------|
| Channel         | 11                |
| Blower Node MAC | E4:65:B8:20:20:A0 |
| BME280 Node MAC | E4:65:B8:25:42:F8 |
| ESP32 Core    | 3.3.10            |
| Wi-Fi Mode      | WIFI_MODE_APSTA   |

> **Note:** MAC addresses shown are examples from this build. Replace with the actual MAC addresses
> of your ESP32 modules. Each ESP32 has a unique MAC — use a brief sketch calling
> `Wis critical for reliable, clean on/off transitions.

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
["MM/DD/YYYY HH:MM:SS", outsideTemp, insideTemp, registerTemp, thermostat, elapsedMin, dailyTotalMin.outside presure, inside presure, differenc pressure]
```

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
- Tylor Glenn BME280I2C.h Library
- MCP6050 [ElectricCats MPU6050 Libreary](https://github.com/ElectronicCats/mpu6050)
- LittleFS (built into ESP32 Arduino Core)

---

## License

MIT License — see `LICENSE` for details.
