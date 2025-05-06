/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "z_displ_ILI9XXX.h"
#include "z_touch_XPT2046.h"
#include "fonts.h"
#include "thermistor_mux.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERROR_LED_Pin GPIO_PIN_6
#define ERROR_LED_GPIO_Port GPIOE
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC
#define RTC_OSC_IN_Pin GPIO_PIN_14
#define RTC_OSC_IN_GPIO_Port GPIOC
#define RTC_OSC_OUT_Pin GPIO_PIN_15
#define RTC_OSC_OUT_GPIO_Port GPIOC
#define GPIO_WARNING_HV_Pin GPIO_PIN_5
#define GPIO_WARNING_HV_GPIO_Port GPIOF
#define GPIO_I_SENSE_ENABLE_Pin GPIO_PIN_7
#define GPIO_I_SENSE_ENABLE_GPIO_Port GPIOF
#define HSE_OSC_Pin GPIO_PIN_0
#define HSE_OSC_GPIO_Port GPIOH
#define ETH_CS_Pin GPIO_PIN_0
#define ETH_CS_GPIO_Port GPIOC
#define ETH_RST_Pin GPIO_PIN_3
#define ETH_RST_GPIO_Port GPIOC
#define MUX_TH1_Pin GPIO_PIN_0
#define MUX_TH1_GPIO_Port GPIOA
#define MUX_TH0_Pin GPIO_PIN_1
#define MUX_TH0_GPIO_Port GPIOA
#define ADC_ISENSEF_Pin GPIO_PIN_2
#define ADC_ISENSEF_GPIO_Port GPIOA
#define DTH3_Pin GPIO_PIN_3
#define DTH3_GPIO_Port GPIOA
#define DAC_VCCS_Pin GPIO_PIN_4
#define DAC_VCCS_GPIO_Port GPIOA
#define DAC0_UNUSED_Pin GPIO_PIN_5
#define DAC0_UNUSED_GPIO_Port GPIOA
#define DAC1_UNUSED_Pin GPIO_PIN_6
#define DAC1_UNUSED_GPIO_Port GPIOA
#define ADC0_UNUSED_Pin GPIO_PIN_7
#define ADC0_UNUSED_GPIO_Port GPIOA
#define ADC_VCCSSENSE_Pin GPIO_PIN_4
#define ADC_VCCSSENSE_GPIO_Port GPIOC
#define ADC1_UNUSED_Pin GPIO_PIN_5
#define ADC1_UNUSED_GPIO_Port GPIOC
#define ADC2_UNUSED_Pin GPIO_PIN_0
#define ADC2_UNUSED_GPIO_Port GPIOB
#define ADC_ISENSEC_Pin GPIO_PIN_1
#define ADC_ISENSEC_GPIO_Port GPIOB
#define ADC_DIFF_P_Pin GPIO_PIN_11
#define ADC_DIFF_P_GPIO_Port GPIOF
#define ADC_DIFF_N_Pin GPIO_PIN_12
#define ADC_DIFF_N_GPIO_Port GPIOF
#define ADC_VSENSE_Pin GPIO_PIN_13
#define ADC_VSENSE_GPIO_Port GPIOF
#define DTH2_Pin GPIO_PIN_14
#define DTH2_GPIO_Port GPIOF
#define SD_CARD_DETECT_Pin GPIO_PIN_10
#define SD_CARD_DETECT_GPIO_Port GPIOE
#define SD_CARD_DETECT_EXTI_IRQn EXTI15_10_IRQn
#define SD_CS_Pin GPIO_PIN_13
#define SD_CS_GPIO_Port GPIOE
#define ST_LINK_TX_Pin GPIO_PIN_10
#define ST_LINK_TX_GPIO_Port GPIOB
#define ST_LINK_RX_Pin GPIO_PIN_11
#define ST_LINK_RX_GPIO_Port GPIOB
#define MATC0_Pin GPIO_PIN_12
#define MATC0_GPIO_Port GPIOB
#define MATC1_Pin GPIO_PIN_13
#define MATC1_GPIO_Port GPIOB
#define MATC2_Pin GPIO_PIN_14
#define MATC2_GPIO_Port GPIOB
#define MATC3_Pin GPIO_PIN_15
#define MATC3_GPIO_Port GPIOB
#define FRONT_SW_Pin GPIO_PIN_8
#define FRONT_SW_GPIO_Port GPIOD
#define FRONT_SW_EXTI_IRQn EXTI9_5_IRQn
#define SSR_EN_Pin GPIO_PIN_9
#define SSR_EN_GPIO_Port GPIOD
#define MATR0_Pin GPIO_PIN_10
#define MATR0_GPIO_Port GPIOD
#define MATR1_Pin GPIO_PIN_11
#define MATR1_GPIO_Port GPIOD
#define PWM0_Pin GPIO_PIN_12
#define PWM0_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_13
#define PWM1_GPIO_Port GPIOD
#define MATR2_Pin GPIO_PIN_14
#define MATR2_GPIO_Port GPIOD
#define MATR3_Pin GPIO_PIN_15
#define MATR3_GPIO_Port GPIOD
#define MUX_A_Pin GPIO_PIN_2
#define MUX_A_GPIO_Port GPIOG
#define MUX_B_Pin GPIO_PIN_3
#define MUX_B_GPIO_Port GPIOG
#define ENCODER_A_Pin GPIO_PIN_4
#define ENCODER_A_GPIO_Port GPIOG
#define ENCODER_B_Pin GPIO_PIN_5
#define ENCODER_B_GPIO_Port GPIOG
#define ENCODER_SW_Pin GPIO_PIN_6
#define ENCODER_SW_GPIO_Port GPIOG
#define ENCODER_SW_EXTI_IRQn EXTI9_5_IRQn
#define MUX_C_Pin GPIO_PIN_7
#define MUX_C_GPIO_Port GPIOG
#define DISPL_LED_Pin GPIO_PIN_6
#define DISPL_LED_GPIO_Port GPIOC
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define PWM_BUZZER_Pin GPIO_PIN_10
#define PWM_BUZZER_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define ST_LINK_SWDIO_Pin GPIO_PIN_13
#define ST_LINK_SWDIO_GPIO_Port GPIOA
#define ST_LINK_SWCLK_Pin GPIO_PIN_14
#define ST_LINK_SWCLK_GPIO_Port GPIOA
#define TOUCH_CS_Pin GPIO_PIN_10
#define TOUCH_CS_GPIO_Port GPIOC
#define TOUCH_INT_Pin GPIO_PIN_11
#define TOUCH_INT_GPIO_Port GPIOC
#define TOUCH_INT_EXTI_IRQn EXTI15_10_IRQn
#define DISPL_DC_Pin GPIO_PIN_12
#define DISPL_DC_GPIO_Port GPIOC
#define GPIO15_UNUSED_Pin GPIO_PIN_0
#define GPIO15_UNUSED_GPIO_Port GPIOD
#define DISPL_RST_Pin GPIO_PIN_1
#define DISPL_RST_GPIO_Port GPIOD
#define ETH_INT_Pin GPIO_PIN_2
#define ETH_INT_GPIO_Port GPIOD
#define ETH_INT_EXTI_IRQn EXTI2_IRQn
#define ETH_SCK_Pin GPIO_PIN_3
#define ETH_SCK_GPIO_Port GPIOD
#define GPIO14_UNUSED_Pin GPIO_PIN_4
#define GPIO14_UNUSED_GPIO_Port GPIOD
#define GPIO13_UNUSED_Pin GPIO_PIN_5
#define GPIO13_UNUSED_GPIO_Port GPIOD
#define GPIO12_UNUSED_Pin GPIO_PIN_6
#define GPIO12_UNUSED_GPIO_Port GPIOD
#define GPIO11_UNUSED_Pin GPIO_PIN_7
#define GPIO11_UNUSED_GPIO_Port GPIOD
#define GPIO10_UNUSED_Pin GPIO_PIN_10
#define GPIO10_UNUSED_GPIO_Port GPIOG
#define DISPL_SCK_Pin GPIO_PIN_11
#define DISPL_SCK_GPIO_Port GPIOG
#define GPIO9_UNUSED_Pin GPIO_PIN_12
#define GPIO9_UNUSED_GPIO_Port GPIOG
#define GPIO8_UNUSED_Pin GPIO_PIN_13
#define GPIO8_UNUSED_GPIO_Port GPIOG
#define GPIO7_UNUSED_Pin GPIO_PIN_14
#define GPIO7_UNUSED_GPIO_Port GPIOG
#define GPIO6_UNUSED_Pin GPIO_PIN_15
#define GPIO6_UNUSED_GPIO_Port GPIOG
#define ST_LINK_SWO_Pin GPIO_PIN_3
#define ST_LINK_SWO_GPIO_Port GPIOB
#define DISPL_CS_Pin GPIO_PIN_4
#define DISPL_CS_GPIO_Port GPIOB
#define DISPL_MOSI_Pin GPIO_PIN_5
#define DISPL_MOSI_GPIO_Port GPIOB
#define GPIO5_UNUSED_Pin GPIO_PIN_6
#define GPIO5_UNUSED_GPIO_Port GPIOB
#define GPIO4_UNUSED_Pin GPIO_PIN_7
#define GPIO4_UNUSED_GPIO_Port GPIOB
#define GPIO3_UNUSED_Pin GPIO_PIN_8
#define GPIO3_UNUSED_GPIO_Port GPIOB
#define GPIO2_UNUSED_Pin GPIO_PIN_9
#define GPIO2_UNUSED_GPIO_Port GPIOB
#define GPIO1_UNUSED_Pin GPIO_PIN_0
#define GPIO1_UNUSED_GPIO_Port GPIOE
#define GPIO0_UNUSED_Pin GPIO_PIN_1
#define GPIO0_UNUSED_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define ADC1_DMA_BUF_LEN  8
#define ADC2_DMA_BUF_LEN  3

extern uint16_t adc1_dma_buf[ADC1_DMA_BUF_LEN];
extern uint16_t adc2_dma_buf[ADC2_DMA_BUF_LEN];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
