#pragma once

// Orbit Drive STM32G431CBU6 — HALL_5 (analog halls + STSPIN233)
// PWM order matches the PCB (TIM1 CH1/CH2/CH3 → INU/INV/INW).

#define PIN_STATUS_LED PC6

#define PIN_HALL_U     PA0
#define PIN_HALL_V     PA1
#define PIN_HALL_W     PA2
#define PIN_TEMP_T1    PA3

#define PIN_IN1        PA8   // TIM1_CH1 → STSPIN INU
#define PIN_IN2        PA9   // TIM1_CH2 → STSPIN INV
#define PIN_IN3        PA10  // TIM1_CH3 → STSPIN INW
#define PIN_DRV_EN     PA4   // EN\FAULT, high = enabled

#define PIN_CAN_RX     PA11  // FDCAN1_RX → TJA1051 RXD
#define PIN_CAN_TX     PA12  // FDCAN1_TX → TJA1051 TXD

#define PIN_HOST_UART_TX  PB6
#define PIN_HOST_UART_RX  PB7
