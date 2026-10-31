#pragma once

// ==============================================================================
// SMART HELMET UNIT — HARDWARE PIN MAPPING (ESP32 38-Pin DevKit)
// ==============================================================================
// HARD ELECTRICAL RULES:
// 1. ESP32 + WiFi: ADC2 pins (0,2,4,12,13,14,15,25,26,27) CANNOT be used for
//    analog reads. All analog signals MUST be on ADC1: GPIO 32,33,34,35,36,39.
// 2. Pins 34-39 are input-only and lack internal pull-up/down resistors.
// 3. Avoid flash pins (GPIO 6-11) and treat 0,2,5,12,15 as strapping pins.
// 4. Keep GPIO 1/3 dedicated for USB-UART serial debugging.
// ==============================================================================

// --- Analog Inputs (ADC1 Only) ---
#define PIN_MQ3_AO            34    // ADC1_CH6: MQ-3 Analog Out (via 10k/20k voltage divider)
#define PIN_BATTERY_SENSE     35    // ADC1_CH7: Li-ion Battery Sense (via 100k/100k divider)
#define PIN_MQ3_DO            39    // ADC1_CH3 / SENSOR_VN: Optional digital threshold

// --- I2C Bus (MPU-6050 6-Axis IMU) ---
#define PIN_I2C_SDA           21    // Wire SDA (MPU-6050)
#define PIN_I2C_SCL           22    // Wire SCL (MPU-6050)
#define PIN_MPU_INT           19    // MPU-6050 Data Ready / Motion Interrupt

// --- GPS Hardware UART2 (NEO-6M) ---
#define PIN_GPS_RX            16    // ESP RX2 <- GPS TXD
#define PIN_GPS_TX            17    // ESP TX2 -> GPS RXD

// --- GSM/GPRS Hardware UART1 Remapped (SIM800L) ---
#define PIN_SIM_RX            26    // ESP RX1 <- SIM TXD (via level shifter)
#define PIN_SIM_TX            25    // ESP TX1 -> SIM RXD (via level shifter)
#define PIN_SIM_RST           23    // SIM800L Hardware Reset line (active-LOW pulse)

// --- Sensors & Digital Inputs ---
#define PIN_HELMET_IR         18    // FC-51 Infrared Proximity Sensor (Active-LOW when worn)
#define PIN_BUTTON_A          4     // Tactile Button A: SOS Cancel / False Alarm / OK (INPUT_PULLUP)
#define PIN_BUTTON_B          13    // Tactile Button B: Hold 3s for BLE Prov SH-HLM-PROV (INPUT_PULLUP)

// --- Actuators & Outputs ---
#define PIN_BUZZER            27    // Active 5V Piezo Buzzer (driven via 2N2222 transistor)
#define PIN_LED_GREEN         32    // System OK / Helmet Worn / Link Active (330R resistor)
#define PIN_LED_YELLOW        33    // Warning / Alcohol Test In Progress (330R resistor)
#define PIN_LED_RED           14    // Danger / Locked / Collision Alert (330R resistor)
