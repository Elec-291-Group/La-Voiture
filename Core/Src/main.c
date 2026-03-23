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
#include <stdio.h>
#include "ir_rx.h"
#include "vl53l0x.h"
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Configure TIM2 for a 263 µs periodic interrupt used by the IR FSM.
     T = 10 / 38000 Hz = 263.16 µs.
     HSI = 16 MHz → PSC = 15 → timer clock = 1 MHz → ARR = 262 → 263 µs. */
  htim2.Instance->PSC = 15u;
  htim2.Instance->ARR = 262u;
  htim2.Instance->EGR = 0x01U;   /* UG: latch new PSC/ARR immediately      */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  IR_RX_Init();
  HAL_TIM_Base_Start_IT(&htim2);
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
    /* Wait for two consecutive valid frames (address 0x0B already verified
       inside the FSM).  Frame 1 = x_byte, frame 2 = y_byte.               */
    if (IR_RX_Available() >= 2) {
      uint8_t x_byte, y_byte;
      IR_RX_GetFrame(&x_byte);
      IR_RX_GetFrame(&y_byte);
      printf("X=%3u Y=%3u\r\n", x_byte, y_byte);
    }

    /* ── VL53L0X distance read every 200 ms ─────────────────────────────── */
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

    /* ── BT proxy: forward "B:<payload>\r\n" from UART1 to JDY-23 on UART2,
       then echo the module's reply back to UART1. ──────────────────────── */
    {
      uint8_t ch;
      if (HAL_UART_Receive(&huart1, &ch, 1u, 1u) == HAL_OK) {
        if (ch == '\r' || ch == '\n') {
          if (u1_len > 0u) {
            u1_line[u1_len] = '\0';
            if (u1_line[0] == 'B' && u1_line[1] == ':' && u1_len > 2u) {
              /* Send payload + CRLF to BT module */
              const uint8_t crlf[2] = {'\r', '\n'};
              HAL_UART_Transmit(&huart2, u1_line + 2u, u1_len - 2u, HAL_MAX_DELAY);
              HAL_UART_Transmit(&huart2, (uint8_t *)crlf, 2u, HAL_MAX_DELAY);
              /* Collect reply: 500 ms for first byte, 50 ms between bytes */
              uint8_t  reply[256];
              uint16_t rlen = 0u;
              while (rlen < sizeof(reply)) {
                uint32_t tmo = (rlen == 0u) ? 500u : 50u;
                if (HAL_UART_Receive(&huart2, &reply[rlen], 1u, tmo) != HAL_OK) break;
                rlen++;
              }
              if (rlen > 0u) {
                HAL_UART_Transmit(&huart1, reply, rlen, HAL_MAX_DELAY);
              }
            }
            u1_len = 0u;
          }
        } else if (u1_len < U1_LINE_MAX - 1u) {
          u1_line[u1_len++] = ch;
        }
      }
    }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
