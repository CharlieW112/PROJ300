#include "sensors.h"
#include "stm32h7xx_hal.h"
#include "fifo.h"
#include <math.h> // Add this for mathematical operations
#include "main.h"

#define FIFO_SIZE_LARGE 512

// Buffers for the FIFOs
static int32_t voltage_fifo_buffer[FIFO_SIZE_LARGE];
static int32_t current_diff_pos_fifo_buffer[FIFO_SIZE_LARGE]; // FIFO for positive end of diff signal
static int32_t current_diff_neg_fifo_buffer[FIFO_SIZE_LARGE]; // FIFO for negative end of diff signal
static int32_t current_coarse_fifo_buffer[FIFO_SIZE_LARGE];

// FIFO structures
static fifo_t voltage_fifo;
static fifo_t current_diff_pos_fifo;
static fifo_t current_diff_neg_fifo;
static fifo_t current_coarse_fifo;

extern uint16_t adc1_dma_buf[];
extern uint16_t adc2_dma_buf[];

extern DAC_HandleTypeDef hdac1;

// Public sensor structures
volatile voltage_sensor_t voltage_data = {0};
volatile current_sensor_t current_data = {0};
volatile thermistor_data_t thermistor_data = {0};

#define DEBUG_PRINT_SENSORS
#define DEBUG_PRINT_VOLTAGE
#define DEBUG_PRINT_CURRENT
#define DEBUG_PRINT_THERMISTORS
#define DEBUG_PRINT_INPUTS

// Input State Variables
static int8_t encoder_position = 0;
static uint8_t encoder_button_state = 0;
static uint8_t toggle_switch_state = 0;
static uint8_t keypad_state[4][4] = {0};

// Pin Macros
#define READ_PIN(port, pin) HAL_GPIO_ReadPin((port), (pin))
#define ENCODER_A READ_PIN(GPIOG, GPIO_PIN_4)
#define ENCODER_B READ_PIN(GPIOG, GPIO_PIN_5)
#define ENCODER_SW READ_PIN(GPIOG, GPIO_PIN_6)
#define FRONT_SW READ_PIN(GPIOD, GPIO_PIN_8)
GPIO_TypeDef* keypad_cols_port[4] = {GPIOB, GPIOB, GPIOB, GPIOB};
const uint16_t keypad_cols_pin[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
GPIO_TypeDef* keypad_rows_port[4] = {GPIOD, GPIOD, GPIOD, GPIOD};
const uint16_t keypad_rows_pin[4] = {GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_14, GPIO_PIN_15};

// Previous encoder state
static uint8_t last_encoder_state = 0;

// Conversion functions
float convert_diff_sensor(int16_t counts)
{
    return (counts - 0.45f) / 91.30f;
}

float convert_coarse_sensor(uint16_t counts)
{
    return (counts - 1047.33f) / 364.61f;
}

float convert_voltage_sensor(uint16_t counts)
{
	return counts * 0.005 - 5.2f;
}

void DAC_Output_Update(uint16_t value)
{
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

// DAC properties
#define DAC_MAX_VOLTAGE 3.3f
#define DAC_RESOLUTION  4096.0f

// Internal variables
static float current_setpoint = 0.0f; // Target current in amperes

/**
 * @brief Set the target current setpoint.
 * @param target_current Desired current in amperes.
 */
void set_target_current(float target_current)
{
    // Ensure the setpoint is within the valid range
    if (target_current < 0.0f) target_current = 0.0f;
    if (target_current > 100.0f) target_current = 100.0f;

    HAL_GPIO_WritePin(GPIO0_UNUSED_GPIO_Port, GPIO0_UNUSED_Pin, (target_current == 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    current_setpoint = target_current;
    current_data.current_setpoint = target_current;
}

/**
 * @brief PID control loop to adjust the DAC output for current control.
 */
#define PID_KP  0.6f
#define PID_KI  0.05f
#define CONTROL_INTERVAL_MS 5
#define RAMP_STEP 0.05f

void control_current(void)
{
    const float dt = CONTROL_INTERVAL_MS / 1000.0f;

    uint32_t now = HAL_GetTick();
    static uint32_t last = 0;
    if (now - last < CONTROL_INTERVAL_MS) return;
    last = now;

    static float ramped_setpoint = 0.0f;
    if (ramped_setpoint < current_setpoint)
        ramped_setpoint = fminf(ramped_setpoint + RAMP_STEP, current_setpoint);
    else if (ramped_setpoint > current_setpoint)
        ramped_setpoint = fmaxf(ramped_setpoint - RAMP_STEP, current_setpoint);

    float measured = current_data.current_diff_converted;
    float error = ramped_setpoint - measured;

    static float integral = 0;
    integral += error * dt;

    // Anti-windup clamp
    if (integral > 1.0f) integral = 1.0f;
    if (integral < -1.0f) integral = -1.0f;

    float output_voltage = PID_KP * error + PID_KI * integral;

    if (output_voltage > DAC_MAX_VOLTAGE) output_voltage = DAC_MAX_VOLTAGE;
    if (output_voltage < 0.0f) output_voltage = 0.0f;

    output_voltage = current_setpoint / 55;
    uint16_t dac_val = (uint16_t)((output_voltage / DAC_MAX_VOLTAGE) * (DAC_RESOLUTION - 1));
    DAC_Output_Update(dac_val);
}

void set_SSR(uint16_t state)
{
	HAL_GPIO_WritePin(SSR_EN_GPIO_Port, SSR_EN_Pin, (state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void sensors_init(void)
{
    fifo_init(&voltage_fifo, voltage_fifo_buffer, FIFO_SIZE_LARGE);
    fifo_init(&current_diff_pos_fifo, current_diff_pos_fifo_buffer, FIFO_SIZE_LARGE);
    fifo_init(&current_diff_neg_fifo, current_diff_neg_fifo_buffer, FIFO_SIZE_LARGE);
    fifo_init(&current_coarse_fifo, current_coarse_fifo_buffer, FIFO_SIZE_LARGE);

    for (int i = 0; i < TOTAL_NUM_THERMISTORS; ++i) {
        thermistor_data.thermistor_names[i] = "N/A"; // Default name
    }

    thermistor_data.thermistor_names[0] = "PWR R";
    thermistor_data.thermistor_names[1] = "PWR L";
    thermistor_data.thermistor_names[15] = "VCCS";
    thermistor_data.thermistor_names[16] = "MCU L";
    thermistor_data.thermistor_names[17] = "MCU R";
}

void Encoder_Read(void)
{
    uint8_t a = ENCODER_A;
    uint8_t b = ENCODER_B;
    uint8_t current_state = (a << 1) | b;

    if ((last_encoder_state == 0b00 && current_state == 0b01) ||
        (last_encoder_state == 0b01 && current_state == 0b11) ||
        (last_encoder_state == 0b11 && current_state == 0b10) ||
        (last_encoder_state == 0b10 && current_state == 0b00)) {
        encoder_position++;
        set_target_current(current_setpoint += 0.02);
    }
    else if ((last_encoder_state == 0b00 && current_state == 0b10) ||
             (last_encoder_state == 0b10 && current_state == 0b11) ||
             (last_encoder_state == 0b11 && current_state == 0b01) ||
             (last_encoder_state == 0b01 && current_state == 0b00)) {
        encoder_position--;
        set_target_current(current_setpoint -= 0.02);
    }

    last_encoder_state = current_state;
    encoder_button_state = (ENCODER_SW == GPIO_PIN_RESET) ? 1 : 0;
}

int8_t Encoder_GetPosition(void) {
    return encoder_position;
}

uint8_t Encoder_GetButtonState(void) {
    return encoder_button_state;
}

void ToggleSwitch_Read(void) {
    toggle_switch_state = (FRONT_SW == GPIO_PIN_RESET);
}

uint8_t ToggleSwitch_GetState(void) {
    return toggle_switch_state;
}

void Keypad_Scan(void)
{
    for (int col = 0; col < 4; col++) {
        HAL_GPIO_WritePin(keypad_cols_port[col], keypad_cols_pin[col], GPIO_PIN_RESET);

        for (int row = 0; row < 4; row++) {
            keypad_state[row][col] = (HAL_GPIO_ReadPin(keypad_rows_port[row], keypad_rows_pin[row]) == GPIO_PIN_RESET) ? 1 : 0;
        }

        HAL_GPIO_WritePin(keypad_cols_port[col], keypad_cols_pin[col], GPIO_PIN_SET);
    }
}

void Keypad_GetState(uint8_t output[4][4])
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            output[r][c] = keypad_state[r][c];
}

void sensors_update(void)
{
    fifo_push(&current_coarse_fifo, adc1_dma_buf[2]);

    current_data.current_coarse_raw = fifo_get_average(&current_coarse_fifo);

    current_data.current_diff_converted = convert_diff_sensor(current_data.current_diff_raw);
    current_data.current_coarse_converted = convert_coarse_sensor(current_data.current_coarse_raw);

    fifo_push(&voltage_fifo, adc2_dma_buf[0]);
    voltage_data.voltage_raw = fifo_get_average(&voltage_fifo);
    voltage_data.voltage_converted = convert_voltage_sensor(voltage_data.voltage_raw);

    for (int i = 0; i < 16; ++i) {
        thermistor_data.thermistor[i] = thermistor_get_temperature(i);
        if (thermistor_data.thermistor[i] < -100.0f || thermistor_data.thermistor[i] > 200.0f) {
            thermistor_data.thermistor[i] = -999.0f;
        }
    }

    thermistor_data.thermistor[16] = voltage_to_temp(adc2_dma_buf[1]);
    thermistor_data.thermistor[17] = voltage_to_temp(adc1_dma_buf[4]);

    Encoder_Read();
    ToggleSwitch_Read();
    Keypad_Scan();

    //set_target_current((float)encoder_position / 10.0f);
    set_SSR(toggle_switch_state);
}

void sensors_debug_print(void)
{
#ifdef DEBUG_PRINT_SENSORS
    char buf[1024];
    int len = 0;

#ifdef DEBUG_PRINT_VOLTAGE
    len += snprintf(buf + len, sizeof(buf) - len,
                    "[Voltage Sensor]\r\n"
                    "  Voltage Raw: %u\r\n"
                    "  HV Warning:  %s\r\n",
                    voltage_data.voltage_raw,
                    voltage_data.warning_hv ? "YES" : "no");
#endif

#ifdef DEBUG_PRINT_CURRENT
    len += snprintf(buf + len, sizeof(buf) - len,
                    "[Current Sensor]\r\n"
                    "  Diff Raw:    %d\r\n"
                    "  Diff Conv:   %.6f A\r\n"
                    "  Fine Raw:    %u\r\n"
                    "  Coarse Raw:  %u\r\n"
                    "  Coarse Conv: %.6f A\r\n"
                    "  Fine Enabled:%s\r\n",
                    current_data.current_diff_raw,
                    current_data.current_diff_converted,
                    current_data.current_fine_raw,
                    current_data.current_coarse_raw,
                    current_data.current_coarse_converted,
                    current_data.fine_range_enabled ? "YES" : "no");
#endif

#ifdef DEBUG_PRINT_THERMISTORS
    len += snprintf(buf + len, sizeof(buf) - len, "[Thermistors]\r\n");
    for (int i = 0; i < TOTAL_NUM_THERMISTORS; ++i)
    {
        if (thermistor_data.thermistor[i] != -999.0f && thermistor_data.thermistor[i] >= -50.0f && thermistor_data.thermistor[i] <= 120.0f)
        {
            len += snprintf(buf + len, sizeof(buf) - len,
                            "  %s: %6.2f C\r\n", thermistor_data.thermistor_names[i], thermistor_data.thermistor[i]);
        }
    }
#endif

#ifdef DEBUG_PRINT_INPUTS
    len += snprintf(buf + len, sizeof(buf) - len,
                    "\r\n[Inputs]\r\n"
                    "  Encoder Pos: %d\r\n"
                    "  Encoder Btn: %s\r\n"
                    "  Toggle Sw:   %s\r\n",
                    Encoder_GetPosition(),
                    Encoder_GetButtonState() ? "PRESSED" : "released",
                    ToggleSwitch_GetState() ? "ON" : "off");

    len += snprintf(buf + len, sizeof(buf) - len, "  Keypad:\r\n");
    for (int r = 0; r < 4; ++r) {
        len += snprintf(buf + len, sizeof(buf) - len, "   ");
        for (int c = 0; c < 4; ++c) {
            len += snprintf(buf + len, sizeof(buf) - len, "%c ", keypad_state[r][c] ? 'X' : '.');
        }
        len += snprintf(buf + len, sizeof(buf) - len, "\r\n");
    }
#endif

    snprintf(buf + len, sizeof(buf) - len, "\r\n");
    printf("%s", buf);
#endif
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        fifo_push(&current_diff_pos_fifo, (int16_t)adc1_dma_buf[0]);
        fifo_push(&current_diff_neg_fifo, (int16_t)adc1_dma_buf[1]);

        int32_t diff_pos_avg = fifo_get_average(&current_diff_pos_fifo);
        int32_t diff_neg_avg = fifo_get_average(&current_diff_neg_fifo);
        current_data.current_diff_raw = diff_pos_avg - diff_neg_avg;
    }
}
