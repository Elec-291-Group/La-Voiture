/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "ir_rx.h"
#include "vl53l0x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//-------- proportional gain for line tracking control ---------------
// needed to be tuned !!!
#define KP 0.02f
#define KS 0.015f
//-------- base power of motor in auto mode -----------------
#define PB 65

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ── BT proxy (UART1 → UART2 → UART1) ──────────────────────────────────── */
#define U1_LINE_MAX  128u
static uint8_t  u1_line[U1_LINE_MAX];
static uint16_t u1_len;

enum path_tracking_states my_tracking_states = Running;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Set_Left_Motor(int speed);
void Set_Right_Motor(int speed);
void handle_intersection_encountered(void);
void handle_intersection_turning(void);
void path_tracking(void);
void handle_line_tracking(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Route printf → UART1 via the Newlib __io_putchar hook in syscalls.c */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1u, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //starting PWM generation
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  
  // Configure ADC channel
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  vdda_calibration();

  MX_I2C1_Init();
  //MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Configure TIM6 for a 263 µs periodic interrupt (IR FSM tick).
     PSC=15 already set by CubeMX → 1 MHz clock. Override ARR: 262 → 263 µs. */
  htim6.Instance->ARR = 262u;
  htim6.Instance->EGR = 0x01U;   /* UG: latch new ARR immediately */
  IR_RX_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  printf("IR RX Ready\r\n");
  if (VL53L0X_Init()) {
    printf("VL53L0X OK\r\n");
  } else {
    printf("VL53L0X FAIL\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    path_tracking();
   
    /* Wait for two consecutive valid frames (address 0x0B already verified
       inside the FSM).  Frame 1 = x_byte, frame 2 = y_byte.               */
    if (IR_RX_Available() >= 2) {
      uint8_t x_byte, y_byte;
      IR_RX_GetFrame(&x_byte);
      IR_RX_GetFrame(&y_byte);
      printf("X=%3u Y=%3u\r\n", x_byte, y_byte);
    }

    /* ── VL53L0X distance read every 200 ms ─────────────────────────────── */
    /*
    {
      static uint32_t last_dist_ms;
      if (HAL_GetTick() - last_dist_ms >= 200u) {
        last_dist_ms = HAL_GetTick();
        uint16_t dist = VL53L0X_ReadDistance();
        if (dist != 0xFFFFu) {
          printf("D=%4u mm\r\n", dist);
        }
      }
    }
     */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// LEFT MOTOR (TIM2)
// CH1 (PA15) = Forward Pin | CH2 (PB3) = Reverse Pin
void Set_Left_Motor(int speed){
  if(speed > 100) speed = 100;
  if(speed < -100) speed = -100;
  
  if(speed == 0) { // STOP
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  }
  else if(speed > 0) { // FORWARD: CH1 gets PWM, CH2 stays 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed * 10); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  }
  else { // REVERSE: CH1 stays 0, CH2 gets PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (-speed) * 10);
  }
}

// RIGHT MOTOR (TIM2)
// CH3 (PA2) = Forward Pin | CH4 (PA3) = Reverse Pin
void Set_Right_Motor(int speed){
  if(speed > 100) speed = 100;
  if(speed < -100) speed = -100;
  
  if(speed == 0) { // STOP
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  }
  else if(speed > 0) { // FORWARD: CH3 gets PWM, CH4 stays 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed * 10); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  }
  else { // REVERSE: CH3 stays 0, CH4 gets PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (-speed) * 10);
  }
}

void path_tracking(void){
  switch(my_tracking_states){
    case Running:
      handle_line_tracking();
      break;
    
    case Intersection_encountered:
      handle_intersection_encountered();
      break;

    case Intersection_turning:
      handle_intersection_turning();
      break;
      
  }
}

void handle_line_tracking(void){
  uint32_t adc0;
  uint32_t v_left;
  uint32_t adc1;
  uint32_t v_right;
  uint32_t adc9;
  uint32_t v_front;
  int base_power;
  int error;
  int left_power;
  int right_power;

  adc0 = read_adc_channel(ADC_CHANNEL_0);
  adc1 = read_adc_channel(ADC_CHANNEL_1);
  adc9 = read_adc_channel(ADC_CHANNEL_9);

  // left, right, front and adc mapping depend on wiring
  v_left = adc_to_voltage(adc0);
  v_right = adc_to_voltage(adc9);
  v_front = adc_to_voltage(adc1);

  error = (int)v_left - (int)v_right;
  base_power = PB - KS * abs(error);
  //base_power = PB;
  left_power = base_power - KP * error;
  right_power = base_power + KP * error;

  if(v_front > 1000){
    my_tracking_states = Intersection_encountered;
  }

  printf("left: %d; right: %d; front: %d\n", v_left, v_right, v_front);
  //printf("left_power: %d; right_power: %d\n", left_power, right_power);
  Set_Left_Motor(left_power);
  Set_Right_Motor(right_power);
}

void handle_intersection_encountered(void){
  Set_Left_Motor(0);
  Set_Right_Motor(0);
  my_tracking_states = Intersection_turning;
}

void handle_intersection_turning(void){
  Set_Left_Motor(-20); 
  Set_Right_Motor(20);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
