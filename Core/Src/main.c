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
#include "ir_rx.h"
#include "vl53l0x.h"
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* ── IR command state (written in ISR via HandleCommand) ─────────────────── */
volatile uint8_t ir_joystick_x = 128u;   /* centred */
volatile uint8_t ir_joystick_y = 128u;
volatile uint8_t ir_mode       = 0u;     /* 0 = auto, 1 = remote */
volatile uint8_t ir_running    = 0u;     /* 1 after start, 0 after pause/reset */
volatile uint8_t ir_path       = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Set_Left_Motor(int speed);
void Set_Right_Motor(int speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  uint32_t adc0;
  uint32_t v0;
  uint32_t adc1;
  uint32_t adc9;

  char buffer[20];

  MX_I2C1_Init();
  //MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* IR receiver: configures TIM6 as free-running 1 µs counter
     and enables EXTI on PA7 (both edges) for input-capture decoding. */
  IR_RX_Init();
  VL53L0X_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

    //test sequence
    Set_Left_Motor(100);  // Left motor full forward
    Set_Right_Motor(100); // Right motor full forward

    adc0 = read_adc_channel(ADC_CHANNEL_1);
    v0 = adc_to_voltage(adc0);
   
    /* ── IR FSM timeout watchdog ────────────────────────────────────────── */
    IR_RX_Update();

    /* ── Print joystick values when updated ─────────────────────────────── */
    {
      static uint8_t last_x = 128u, last_y = 128u;
      uint8_t x = ir_joystick_x, y = ir_joystick_y;
      if (x != last_x || y != last_y) {
        last_x = x;
        last_y = y;
        printf("X=%3u Y=%3u\r\n", x, y);
      }
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

/* ── IR command handler — called from ISR context (TIM6 tick) ───────────── */
/* Keep this function short: no blocking calls, no printf.                    */
void HandleCommand(uint8_t cmd_name, uint8_t data)
{
  switch (cmd_name)
  {
    case IR_CMD_START:
      ir_running = 1u;
      break;

    case IR_CMD_PAUSE:
      ir_running = 0u;
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      break;

    case IR_CMD_RESET:
      ir_running    = 0u;
      ir_joystick_x = 128u;
      ir_joystick_y = 128u;
      ir_mode       = IR_MODE_AUTO;
      ir_path       = IR_PATH_1;
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      break;

    case IR_CMD_MODE:
      ir_mode = data;   /* IR_MODE_AUTO or IR_MODE_REMOTE */
      break;

    case IR_CMD_PATH:
      ir_path = data;   /* IR_PATH_1 / IR_PATH_2 / IR_PATH_3 */
      break;

    case IR_CMD_JOYSTICK_X:
      ir_joystick_x = data;
      if (ir_mode == IR_MODE_REMOTE && ir_running) {
        /* Map 0-255 → -100..+100: centre 128 → 0 */
        int spd = ((int)data - 128) * 100 / 128;
        Set_Left_Motor(spd);
      }
      break;

    case IR_CMD_JOYSTICK_Y:
      ir_joystick_y = data;
      if (ir_mode == IR_MODE_REMOTE && ir_running) {
        int spd = ((int)data - 128) * 100 / 128;
        Set_Right_Motor(spd);
      }
      break;

    default:
      break;
  }
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
