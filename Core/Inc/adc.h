/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc;

/* USER CODE BEGIN Private defines */
#ifndef ADC_H
#define ADC_H
// calibrated VDDA
extern uint32_t VDDA;

#endif

// the vrefint adc value is measured at exactly 3V.
// the formula Aref_cal * 3 / Aref_measured gives us VDDA. Which could be used for calibration purpose
#define AREFINT_CAL (*(volatile uint16_t*)0x1FF80078) // access the vrefint calibrated adc value

/* USER CODE END Private defines */

void MX_ADC_Init(void);

/* USER CODE BEGIN Prototypes */

void vdda_calibration(void);
uint32_t read_adc_channel(uint32_t channel);
uint32_t adc_to_voltage(uint32_t adc_value);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

