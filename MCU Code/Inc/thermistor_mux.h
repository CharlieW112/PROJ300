#ifndef __THERMISTOR_MUX_H
#define __THERMISTOR_MUX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define NUM_THERMISTORS 16

extern uint16_t mux0_values[NUM_THERMISTORS];
extern uint16_t mux1_values[NUM_THERMISTORS];


void thermistor_mux_init(void);
void thermistor_mux_tick(void);
void thermistor_mux_debug_print(void);
float voltage_to_temp(uint16_t adc_val);
float thermistor_get_temperature(uint8_t index);



#ifdef __cplusplus
}
#endif

#endif
