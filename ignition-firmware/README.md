# Smart Ignition Unit Firmware (`ignition-firmware`)

Safety-critical motorcycle ignition controller firmware for the ESP32 DevKit. Built on PlatformIO with the Arduino framework, FreeRTOS non-blocking supervisor, fail-safe solid-state relay (SSR) default-LOW architecture. **LCD is optional** — primary setup is the HelmGuard app over BLE (`SH-IGN-PROV`); LEDs show Wi‑Fi / SoftAP / BLE pairing status.

### LED link indicators (existing wiring)
| Pattern | Meaning |
| :--- | :--- |
| Red slow blink | Connecting to Wi‑Fi STA |
| Green slow blink | SoftAP up, waiting for helmet |
| Green/Red alternate (fast) | BLE provisioning (`SH-IGN-PROV`) |
| Solid green / red | Safety FSM unlocked / locked |

Hold **Button 2 for 5s** (or wait ~20s with no helmet link) to advertise BLE for app pairing. Serial `PROV` also works.

---

## Hardware Pinout (`include/pins.h`)

| Peripheral | ESP32 Pin | Type | Notes |
| :--- | :--- | :--- | :--- |
| **SSR Control Line** | GPIO 25 | Digital Out | **Safety Critical**: Initialized LOW on boot, external 10k pull-down |
| **Status LED Green** | GPIO 26 | Digital Out | Driven via 2N2222 transistor (Engine Unlocked / Latched) |
| **Status LED Red** | GPIO 27 | Digital Out | Driven via 2N2222 transistor (Locked / Link Failure / Alarm) |
| **Warning Buzzer** | GPIO 33 | Digital Out | Audible warning during mid-ride alert or accident cutoff |
| **12V Battery Sense** | GPIO 34 | Analog (ADC1_CH6) | 47k/10k voltage divider ($\text{ratio} = 5.7:1$) on ADC1 |
| **LCD 16x2 Backpack** | GPIO 21 (SDA), 22 (SCL) | I2C | PCF8574 I2C expander (auto-detects address `0x27` / `0x3F`) |
| **Button 1 (LCD Cycle)**| GPIO 4 | Digital In | `INPUT_PULLUP`, debounced, cycles 5 LCD telemetry pages |
| **Button 2 (Prov / Reset)**| GPIO 13 | Digital In | `INPUT_PULLUP`, hold 5s = BLE Prov, hold 15s = Factory Reset |

---

## Safety State Machine (`include/safety_fsm.h`)

The state machine implements a strict safety lifecycle:

```
[BOOT/REBOOT] ──> LOCKED ──(All 5 Pre-Ride Conditions)──> UNLOCKED ──(Engine Crank / Speed > 3km/h)──> RUNNING_LATCHED
                    ▲                     │                                                                  │
                    │                     └──(Helmet Removed / Link Lost BEFORE ride)──┘                     │
                    │                                                                                        │
                    └─────────────────────(Key Turned Off / Session Ended)───────────────────────────────────┘
                                                                                                             │
                                                                                 (Confirmed Crash Accident)  │
                                                                                                             ▼
                                                                                                           FAULT
```

### 1. Pre-Ride 5-Point Gate (`LOCKED -> UNLOCKED`)
The engine is unlocked **only** when all 5 conditions evaluate to TRUE:
1. **Fresh Authenticated Link**: Heartbeat received within last 2000ms.
2. **Helmet Worn**: Infrared proximity sensor stable $> 1000\text{ms}$.
3. **MQ-3 Warmed Up**: 20-second sensor heater warmup completed.
4. **Sober Breath**: Evaluated BAC $\le 0.05\%$.
5. **No Active Crash & Healthy Battery**: Impact flag false, helmet battery $> 15\%$.

### 2. Anti-Stall Mid-Ride Protection (`RUNNING_LATCHED`)
**Safety Invariant**: Once the motorcycle is in motion or latched, **THE ENGINE IS NEVER CUT MID-RIDE** due to temporary packet loss, link drops, or helmet adjustments. Instead:
- Warning buzzer pulses.
- LCD displays mid-ride warning.
- App and cloud receive immediate telemetry event.
- SSR remains firmly closed (HIGH) until the rider safely turns off the key ignition.

---

## 16x2 LCD Diagnostic Pages (Button 1)

1. **Page 0: System State & SSR**: Current FSM state (`LOCKED`, `UNLOCKED`, `RUNNING_LATCHED`), SSR status, or active lock reason.
2. **Page 1: Network & Link**: Infrastructure WiFi SSID, SoftAP fallback status, IP address.
3. **Page 2: Helmet Health & Alcohol**: Helmet battery percentage, evaluated breathalyzer BAC.
4. **Page 3: Motorcycle 12V Battery**: Real-time battery voltage and alternator charging status.
5. **Page 4: Diagnostics**: System uptime in seconds, authenticated sequence nonce counter.

---

## Building and Testing

### 1. Run Native Logic Unit Tests (on host)
```bash
pio test -d ignition-firmware -e native
```
Tests safety transitions, pre-ride validation, anti-stall invariants, and maintenance override.

### 2. Compile Target ESP32 Firmware
```bash
pio run -d ignition-firmware
```

### 3. Flash to ESP32 DevKit
```bash
pio run -d ignition-firmware -t upload
```

### 4. Interactive Simulation / Bench Commands (115200 baud)
- `UNLOCK`: Inject valid simulated sober rider heartbeat.
- `ALCOHOL`: Inject alcohol violation (0.09% BAC).
- `CRASH`: Inject emergency collision accident cutoff.
- `CLEAR`: Reset safety fault.
- `PAGE`: Cycle through the 5 LCD pages.
- `STATUS`: Print full state, SSR status, and 12V voltage reading.
