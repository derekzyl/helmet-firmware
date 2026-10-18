# Smart Helmet Unit Firmware (`helmet-firmware`)

Firmware for the ESP32 (38-pin DevKit) embedded in the Smart Helmet. Built on PlatformIO with the Arduino framework, FreeRTOS non-blocking task architecture, and hardware watchdog protection.

---

## Hardware Pinout (`include/pins.h`)

| Peripheral | ESP32 Pin | Type | Notes |
| :--- | :--- | :--- | :--- |
| **MQ-3 Alcohol Sensor** | GPIO 34 | Analog (ADC1_CH6) | Fed via 10k/20k voltage divider ($\le 3.3\text{V}$) |
| **Battery Voltage Sense** | GPIO 35 | Analog (ADC1_CH7) | 100k/100k resistor divider across Li-ion rail |
| **FC-51 Infrared IR Sensor** | GPIO 18 | Digital In | Active-LOW when worn; 1000ms debounce filter |
| **MPU-6050 6-Axis IMU** | GPIO 21 (SDA), 22 (SCL) | I2C | 400 kHz fast I2C bus; interrupt on GPIO 19 |
| **NEO-6M GPS Module** | GPIO 16 (RX), 17 (TX) | UART2 | 9600 baud, TinyGPSPlus NMEA parser |
| **SIM800L GSM/GPRS Modem** | GPIO 26 (RX), 25 (TX) | UART1 (Remapped) | Dedicated 4.0V/2A rail with 1000µF cap; RST on GPIO 23 |
| **Buzzer Actuator** | GPIO 27 | Digital Out | 5V active buzzer driven via 2N2222 transistor |
| **Status LEDs** | GPIO 32 (Grn), 33 (Yel), 14 (Red) | Digital Out | 330Ω current-limiting resistors |
| **Button A (Cancel/SOS)** | GPIO 4 | Digital In | `INPUT_PULLUP`, active-LOW, cancels false alarms |
| **Button B (Status/Prov)** | GPIO 13 | Digital In | `INPUT_PULLUP`, hold 5s for BLE provisioning |

---

## Core Algorithmic Subsystems

1. **`AlcoholProcessor` (`include/alcohol_processor.h`)**:
   - 20-second pre-heat warmup timer.
   - Clean air baseline auto-calibration ($R_0$).
   - Exponential moving average filter on raw ADC readings.
   - Computes $R_s / R_0$ to Blood Alcohol Concentration (BAC) equivalent.
   - Blow detection requires steady breath pressure while helmet is worn.

2. **`CollisionDetector` (`include/collision_detector.h`)**:
   - Multi-stage accident detection:
     - Peak acceleration impact $\ge 4.0\text{G}$.
     - Post-impact tilt verification $> 60^\circ$.
     - Motionlessness observation window ($\pm 0.35\text{G}$ and $< 20^\circ/\text{s}$ for 2.5s).
   - Enters `PRE_ALERT` with urgent buzzer beeps and 15-second cancel window via Button A.
   - Expired cancel window triggers `CONFIRMED_CRASH`, emergency SMS with Google Maps coordinates via SIM800L, and cloud telemetry event.

3. **`CryptoUtils` (`include/crypto_utils.h`)**:
   - Pure C++ portable SHA-256 and HMAC-SHA256 engine.
   - Authenticates local WebSocket heartbeats with 256-bit pre-shared key (PSK).
   - Monotonic sequence nonce and constant-time signature verification.

---

## Building and Testing

### 1. Run Native Logic Unit Tests (on host)
```bash
pio test -d helmet-firmware -e native
```
Tests crypto vectors, alcohol warm-up/calibration/thresholds, and collision detection without requiring physical hardware.

### 2. Compile Target ESP32 Firmware
```bash
pio run -d helmet-firmware
```

### 3. Flash to ESP32 DevKit
```bash
pio run -d helmet-firmware -t upload
```

### 4. Interactive Simulation / Bench Commands
Open serial monitor at 115200 baud (`pio device monitor -b 115200`):
- `SIM`: Toggle simulation mode (generates mock GPS, battery, and sensor dynamics).
- `CRASH`: Inject 5.5G simulated collision impact.
- `CANCEL`: Press Button A to cancel pre-alert alarm.
- `ALCOHOL`: Inject high BAC breath reading.
- `CLEAN`: Reset breath reading to sober/clean air.
- `STATUS`: Print current telemetry and ignition lock state.
