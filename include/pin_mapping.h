#pragma once

// VnV V2.0 ESP32 pin mapping
//
// GPIO numbers are the IO numbers shown on the schematic.  EN is the
// ESP32 module enable/reset pin and is not a GPIO.

// Analog inputs and outputs
#define PIN_ADC_VIN         34      // ADC_VIN / ADC1_CH6
#define PIN_ADC_IOUT        35      // ADC_IOUT / ADC1_CH7
#define PIN_DAC             25      // DAC
#define PIN_LDAC            14      // LDAC
#define PIN_NTC             36      // SENSOR_VP / ADC1_CH0

// Digital interfaces
#define PIN_SCK             32      // SCK
#define PIN_SIO             33      // SIO
#define PIN_SCL             22      // I2C clock
#define PIN_SDA             21      // I2C data
#define PIN_CAN_RX          26      // CANRX
#define PIN_CAN_TX          27      // CANTX

// PWM outputs
#define PIN_PWM1            23
#define PIN_PWM2            19
#define PIN_PWM3            18
#define PIN_PWM4            5
#define PIN_PWM5            17
#define PIN_PWM6            16
