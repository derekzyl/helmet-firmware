#pragma once

// ==============================================================================
// SMART BIKE IGNITION UNIT — HARDWARE PIN MAPPING (ESP32 DevKit)
// ==============================================================================
// HARD ELECTRICAL RULES:
// 1. SSR Control Line (GPIO 25) MUST default LOW with an external 10k pull-down
//    resistor. Under NO circumstances should the SSR float HIGH on boot/reset.
// 2. 12V Battery Sense (GPIO 34) is on ADC1 (Input Only) via a 47k/10k divider
//    (ratio 5.7:1, 14.8V max translates to 2.59V at ADC, safely <= 3.3V).
// 3. Status LEDs are 12V driven via NPN transistors (2N2222 / BC547).
// 4. Buttons 1 and 2 are active-LOW with internal pull-up (`INPUT_PULLUP`).
// ==============================================================================

// --- Actuators & Outputs ---
#define PIN_SSR_CONTROL       25    // Solid-State Relay / MOSFET Gate (Default LOW, Pull-Down)
#define PIN_LED_GREEN         26    // Status Green: Unlocked / Engine Running (via transistor)
#define PIN_LED_RED           27    // Status Red: Locked / Link Error / Alarm (via transistor)
#define PIN_BUZZER            33    // Optional Audible Warning Buzzer (via transistor)

// --- Analog Inputs (ADC1 Only) ---
#define PIN_12V_SENSE         34    // ADC1_CH6: Motorcycle 12V Lead-Acid Battery Divider (47k / 10k)

// --- I2C Bus (LCD 16x2 PCF8574 Backpack) ---
#define PIN_I2C_SDA           21    // Wire SDA
#define PIN_I2C_SCL           22    // Wire SCL

// --- User Control Tactile Buttons ---
#define PIN_BUTTON_1          4     // Tactile Button 1: Cycle LCD Display Pages (INPUT_PULLUP)
#define PIN_BUTTON_2          13    // Tactile Button 2: Hold 5s = BLE Prov / Hold 15s = Reset (INPUT_PULLUP)
