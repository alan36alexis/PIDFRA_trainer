#pragma once

#include <stdint.h>

#if defined(__cplusplus) ||                                                    \
    (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L)
#define PIDFRA_PIN_CONST static constexpr int
#else
#define PIDFRA_PIN_CONST static const int
#endif

// --- Entradas Analógicas (ADC1) ---
PIDFRA_PIN_CONST PIN_POT_KP = 36;       // ADC1_CH0 (Input only)
PIDFRA_PIN_CONST PIN_POT_KI = 39;       // ADC1_CH3 (Input only)
PIDFRA_PIN_CONST PIN_POT_KD = 34;       // ADC1_CH6 (Input only)
PIDFRA_PIN_CONST PIN_POT_SETPOINT = 35; // ADC1_CH7 (Input only)

// --- LCD Tinsharp TC802A-01(R) (Modo 4 bits, R/W fijo a GND) ---
PIDFRA_PIN_CONST PIN_LCD_RS = 32; // Pin 4 LCD
PIDFRA_PIN_CONST PIN_LCD_E = 33;  // Pin 6 LCD
PIDFRA_PIN_CONST PIN_LCD_D4 = 25; // Pin 11 LCD
PIDFRA_PIN_CONST PIN_LCD_D5 = 26; // Pin 12 LCD
PIDFRA_PIN_CONST PIN_LCD_D6 = 27; // Pin 13 LCD
PIDFRA_PIN_CONST PIN_LCD_D7 = 13; // Pin 14 LCD

// --- Encoder Incremental Motor (900 PPR -> 3600 cuentas/vuelta en X4) ---
PIDFRA_PIN_CONST PIN_ENC_MOTOR_A = 23; // PCNT Unit 0
PIDFRA_PIN_CONST PIN_ENC_MOTOR_B = 22; // PCNT Unit 0

// --- Driver de Motor L298N ---
PIDFRA_PIN_CONST PIN_MOTOR_ENA = 21; // LEDC PWM (Pull-down externo 10 kOhm)
PIDFRA_PIN_CONST PIN_MOTOR_IN1 = 19;
PIDFRA_PIN_CONST PIN_MOTOR_IN2 = 18;

// --- Encoder de Interfaz EC11 ---
PIDFRA_PIN_CONST PIN_EC11_CLK = 17; // PCNT Unit 1 / ISR (INPUT_PULLUP)
PIDFRA_PIN_CONST PIN_EC11_DT = 16;  // PCNT Unit 1 / ISR (INPUT_PULLUP)
PIDFRA_PIN_CONST PIN_EC11_SW = 4;   // INPUT_PULLUP
