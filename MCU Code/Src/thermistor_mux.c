#include <stdio.h>
#include <math.h>

#include "thermistor_mux.h"
#include "main.h"

extern uint16_t adc1_dma_buf[];

uint16_t thermistor_adc_raw[NUM_THERMISTORS] = {0};

float thermistor_temp_offset[NUM_THERMISTORS] = {
    30.5,  // T00 offset
    30.5,  // T01
    30.5,  // T02
    30.5,  // T03
    30.5,  // T04
    30.5,  // T05
    30.5,  // T06
    30.5,  // T07
    30.5,  // T08
    30.5,  // T09
    30.5,  // T10
    30.5,  // T11
    30.5,  // T12
    30.5,  // T13
    30.5,  // T14
    30.5  // T15 offset
};

typedef enum {
    MUX_STATE_SET_ADDR,
    MUX_STATE_READ_ADC
} MuxState;

static uint8_t mux_index = 0;
static MuxState mux_state = MUX_STATE_SET_ADDR;


#define BETA 3435.0
#define R0   10000.0
#define T0   298.15   // in Kelvin
#define VREF 5.0
#define ADC_RESOLUTION 65535.0

float voltage_to_temp(uint16_t adc_val)
{
    float v = (adc_val / ADC_RESOLUTION) * VREF;
    if (v <= 0.0 || v >= VREF) return -999.9; // catch edge cases

    float r = (10000.0 * v) / (VREF - v);
    float inv_T = (1.0 / T0) + (1.0 / BETA) * log(r / R0);
    return (1.0 / inv_T) - 273.15;
}

void thermistor_mux_debug_print(void)
{
    printf("Thermistor Temperatures:\r\n");
    for (int i = 0; i < NUM_THERMISTORS; ++i)
    {
        float temp = voltage_to_temp(thermistor_adc_raw[i]);
        temp += thermistor_temp_offset[i];  // apply per-sensor correction
        printf("T%02d: %6.2f °C\r\n", i, temp);
    }
    printf("\r\n");
}


float thermistor_get_temperature(uint8_t index)
{
    if (index >= NUM_THERMISTORS)
        return -999.9f;

    float temp = voltage_to_temp(thermistor_adc_raw[index]);
    return temp + thermistor_temp_offset[index];
}



static void set_mux_address(uint8_t idx)
{
    HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, (idx & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, (idx & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, (idx & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void thermistor_mux_init(void)
{
    mux_index = 0;
    mux_state = MUX_STATE_SET_ADDR;
    set_mux_address(mux_index);
}

void thermistor_mux_tick(void)
{
    switch (mux_state)
    {
        case MUX_STATE_SET_ADDR:
            set_mux_address(mux_index);
            mux_state = MUX_STATE_READ_ADC;
            break;

        case MUX_STATE_READ_ADC:
            // Read results from the *previous* mux index
        	thermistor_adc_raw[mux_index + 8]       = adc1_dma_buf[5];  // Left mux → index 0–7
        	thermistor_adc_raw[mux_index]   = adc1_dma_buf[6];  // Right mux → index 8–15

            // Prepare for next one
            mux_index = (mux_index + 1) % (NUM_THERMISTORS / 2);
            mux_state = MUX_STATE_SET_ADDR;
            break;
    }
}


