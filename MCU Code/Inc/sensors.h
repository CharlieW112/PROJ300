#ifndef __SENSORS_H
#define __SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "thermistor_mux.h"

#define TOTAL_NUM_THERMISTORS 18 // Total number of thermistors

typedef struct {
	float voltage_converted;
    uint16_t voltage_raw;
    bool warning_hv;
} voltage_sensor_t;

typedef struct {
	float current_setpoint;
    int16_t current_diff_raw;       // Raw ADC value for diff sensor
    float current_diff_converted;  // Converted current value for diff sensor
    uint16_t current_fine_raw;     // Raw ADC value for fine sensor (unused)
    uint16_t current_coarse_raw;   // Raw ADC value for coarse sensor
    float current_coarse_converted; // Converted current value for coarse sensor
    bool fine_range_enabled;       // Indicates if fine range is enabled
} current_sensor_t;

typedef struct {
    float thermistor[TOTAL_NUM_THERMISTORS];          // Temperature values
    const char* thermistor_names[TOTAL_NUM_THERMISTORS]; // Names for each thermistor
} thermistor_data_t;

extern volatile voltage_sensor_t voltage_data;
extern volatile current_sensor_t current_data;
extern volatile thermistor_data_t thermistor_data;

// Function prototypes
void sensors_init(void);
void sensors_update(void);
void sensors_debug_print(void);

// Encoder interface
void Encoder_Read(void);
int8_t Encoder_GetPosition(void);
uint8_t Encoder_GetButtonState(void);

// Toggle switch interface
void ToggleSwitch_Read(void);
uint8_t ToggleSwitch_GetState(void);

// Keypad interface
void Keypad_Scan(void);
void Keypad_GetState(uint8_t output[4][4]);

void DAC_Output_Update(uint16_t value);

// Conversion function prototypes
float convert_diff_sensor(int16_t counts);
float convert_coarse_sensor(uint16_t counts);
float convert_voltage_sensor(uint16_t counts);

#ifdef __cplusplus
}
#endif

#endif
