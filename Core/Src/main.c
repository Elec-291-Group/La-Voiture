/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "ir_rx.h"
#include "vl53l0x.h"
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// IMU Registers 
#define LSM6DS33_ADDR_HIGH        0x6B
#define LSM6DS33_ADDR_LOW         0x6A

#define LSM6DS33_WHO_AM_I_REG     0x0F
#define LSM6DS33_CTRL1_XL         0x10
#define LSM6DS33_CTRL2_G          0x11
#define LSM6DS33_CTRL3_C          0x12

#define LSM6DS33_OUTX_L_G         0x22
#define LSM6DS33_OUTX_L_XL        0x28

#define LSM6DS33_WHO_AM_I_VAL     0x69

// IMU Values
#define ACCEL_SENS_G_PER_LSB      0.000061f
#define GYRO_SENS_DPS_PER_LSB     0.00875f

#define GYRO_CAL_SAMPLES          200
#define RAD_TO_DEG                57.2957795f

// Vehicle Control 
// needed to be tuned !!!
#define KP 0.02f
#define KS 0.015f
//-------- base power of motor in auto mode -----------------
#define PB 65
#define TURN_PRESCALAR 2 // power prescalar to reduce turn speed

//For sensor
#define VL53L0X_I2C_ADDR 0x52 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// IMU Variables
static uint8_t imu_addr = 0;
static uint8_t gyro_buf[6];
static uint8_t accel_buf[6];

static int16_t gyro_x = 0;
static int16_t gyro_y = 0;
static int16_t gyro_z = 0;

static int16_t accel_x = 0;
static int16_t accel_y = 0;
static int16_t accel_z = 0;

static float ax_g = 0.0f;
static float ay_g = 0.0f;
static float az_g = 0.0f;

static float gx_dps = 0.0f;
static float gy_dps = 0.0f;
static float gz_dps = 0.0f;

static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;

static float roll_deg = 0.0f;
static float pitch_deg = 0.0f;

static uint8_t init_ctrl1 = 0x40; // 01000000
static uint8_t init_ctrl3 = 0x44; // 01000100

// Vehicle Control 
enum path_tracking_states my_tracking_states = Running;
volatile uint8_t ir_joystick_x = 128u;   /* centred */
volatile uint8_t ir_joystick_y = 128u;
volatile uint8_t ir_mode       = 0u;     /* 0 = auto, 1 = remote */
volatile uint8_t ir_running    = 0u;     /* 1 after start, 0 after pause/reset */
volatile uint8_t ir_path       = 0u;

// Inductor readings
uint32_t adc0;
uint32_t v_left;
uint32_t adc1;
uint32_t v_right;
uint32_t adc9;
uint32_t v_front;

// Controller State Machine 
typedef enum 
{
    STATE_CONFIG, 
    STATE_PATH,
    STATE_MODE,
    STATE_DRIVE,
    STATE_PAUSE
} ControllerState;

typedef enum
{
  CAR_STATE_IDLE = 0,
  CAR_STATE_PAUSED,
  CAR_STATE_RUNNING_AUTO,
  CAR_STATE_RUNNING_REMOTE
} CarState;

volatile CarState car_state    = CAR_STATE_IDLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Vehicle Controls 
void Set_Left_Motor(int speed);
void Set_Right_Motor(int speed);
void handle_intersection_encountered(void);
void handle_intersection_turning(void);
void path_tracking(void);
void handle_line_tracking(void);
void motor_remote_control(uint8_t, uint8_t);
void Set_Car_Speed(int speed);

// IMU 
void uart_send_text(const char *s);
void uart_send_line(const char *s);

uint8_t probe_addr(uint8_t addr);
void i2c_scan(void);

uint8_t imu_write_reg(uint8_t reg, uint8_t val);
uint8_t imu_read_reg(uint8_t reg, uint8_t *val);
uint8_t imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t MiniMU_FindIMU(void);
uint8_t MiniMU_Init(void);
uint8_t MiniMU_ReadGyro(void);
uint8_t MiniMU_ReadAccel(void);
void MiniMU_UpdateScaled(void);
void MiniMU_UpdateAngles(void);
void MiniMU_PrintScaledData(void);
void MiniMU_PrintAngles(void);
uint8_t MiniMU_CalibrateGyro(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1u, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // Starting PWM generation
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // Initializing IMU
  uart_send_line("");
  uart_send_line("Booting...");
  uart_send_line("USART1 ready");
  HAL_Delay(300);

  ControllerState current_state = STATE_CONFIG;
  int input = 0; // Example input variable

  if (!MiniMU_Init())
  {
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
          uart_send_line("MiniMU init failed");
          HAL_Delay(1000);
      }
  }

  if (!MiniMU_CalibrateGyro())
  {
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
          uart_send_line("Gyro calibration failed");
          HAL_Delay(1000);
      }
  }

  uart_send_line("Streaming accel + gyro + angles");
  
  // Configure ADC channel
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  vdda_calibration();

  MX_I2C1_Init();
  //MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Configure TIM2 for a 263 µs periodic interrupt used by the IR FSM.
     T = 10 / 38000 Hz = 263.16 µs.
     HSI = 16 MHz → PSC = 15 → timer clock = 1 MHz → ARR = 262 → 263 µs. */
  htim6.Instance->ARR = 262u;
  htim6.Instance->EGR = 0x01U; 
  IR_RX_Init();
  VL53L0X_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  printf("IR RX Ready\r\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* LQFP32 PINOUT
              ----------
        VDD -|1       32|- VSS
       PC14 -|2       31|- BOOT0
       PC15 -|3       30|- PB7 (SDA)
       NRST -|4       29|- PB6 (SCL)
       VDDA -|5       28|- PB5
  (LED) PA0 -|6       27|- PB4
        PA1 -|7       26|- PB3
        PA2 -|8       25|- PA15 (PWM output channel 1 of TIM2)
        PA3 -|9       24|- PA14
        PA4 -|10      23|- PA13
        PA5 -|11      22|- PA12
        PA6 -|12      21|- PA11
        PA7 -|13      20|- PA10 (Reserved for RXD)
        PB0 -|14      19|- PA9  (Reserved for TXD)
        PB1 -|15      18|- PA8
        VSS -|16      17|- VDD
              ----------
  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //path_tracking();
   
    /* ── IR FSM timeout watchdog ────────────────────────────────────────── */
    IR_RX_Update();

    /* ── Drain IR ring buffer ────────────────────────────────────────────── */
    {
      uint8_t cmd, dat;
      while (IR_RX_GetFrame(&cmd, &dat)) {
        printf("RAW cmd=0x%X data=%u\r\n", cmd, dat);  /* DEBUG: remove later */
        HandleCommand(cmd, dat);
      }
    }
    /* ── VL53L0X Distance Read & Collision Override ─────────────────────── */
    static uint32_t last_dist_ms = 0;
    static uint16_t current_distance = 8190; // Default to max range

    if (HAL_GetTick() - last_dist_ms >= 100u) {
      last_dist_ms = HAL_GetTick();
      
      uint16_t reading = VL53L0X_ReadDistance();
      if (reading != 0xFFFFu) { // 0xFFFF means error or sensor not ready
          current_distance = reading;
      } else {
          current_distance = 8190; // Default to max range on error i think
      }
    }

    /* 3. COLLISION DETECTION OVERRIDE */
    if (current_distance < 100) { 
      // EMERGENCY STOP: If bject is closer than 10cm
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      printf("OBSTACLE DETECTED! STOPPING.\r\n");
    } 
    else {
      // PATH CLEAR: Run behavior from top-level drive FSM state.
      switch (car_state) {
        case CAR_STATE_RUNNING_AUTO:
          path_tracking();
          break;

        case CAR_STATE_RUNNING_REMOTE:
          motor_remote_control(ir_joystick_x, ir_joystick_y);
          break;

        case CAR_STATE_IDLE:
        case CAR_STATE_PAUSED:
        default:
          Set_Left_Motor(0);
          Set_Right_Motor(0);
          break;
      }
    }

  /* USER CODE END 3 */

    //motor_remote_control(ir_joystick_x, ir_joystick_y);

    /* ── Print joystick values every loop ───────────────────────────────── */
    //printf("X=%3u Y=%3u\r\n", ir_joystick_x, ir_joystick_y);

    
  /* USER CODE END 3 */
  static uint32_t last_imu_print = 0;

  if (HAL_GetTick() - last_imu_print >= 200)
  {
    last_imu_print = HAL_GetTick();

    if (!MiniMU_ReadGyro())
    {
        uart_send_line("Gyro read failed");
    }
    else if (!MiniMU_ReadAccel())
    {
        uart_send_line("Accel read failed");
    }
    else
    {
        MiniMU_UpdateScaled();
        MiniMU_UpdateAngles();
        MiniMU_PrintScaledData();
        MiniMU_PrintAngles();
        uart_send_line("");
    }
  }
  }
  /* USER CODE END WHILE */
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
// LEFT MOTOR (TIM2) |  CH1 (PA15) = Forward Pin | CH2 (PB3) = Reverse Pin
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

// RIGHT MOTOR (TIM2) | CH3 (PA2) = Forward Pin | CH4 (PA3) = Reverse Pin
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
  adc0 = read_adc_channel(ADC_CHANNEL_0);
  adc1 = read_adc_channel(ADC_CHANNEL_1);
  adc9 = read_adc_channel(ADC_CHANNEL_9);
  // left, right, front and adc mapping depend on wiring
  v_left = adc_to_voltage(adc0);
  v_right = adc_to_voltage(adc9);
  v_front = adc_to_voltage(adc1);
  
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
  int base_power;
  int error;
  int left_power;
  int right_power;

  error = (int)v_left - (int)v_right;
  base_power = PB - KS * abs(error);
  //base_power = PB;
  left_power = base_power - KP * error;
  right_power = base_power + KP * error;

  if(v_front > 1000){
    my_tracking_states = Intersection_encountered;
  }
  else{
    my_tracking_states = Running;
  }

  printf("left: %ld; right: %d; front: %d\n", v_left, v_right, v_front);
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
  Set_Left_Motor(-50); 
  Set_Right_Motor(50);
  printf("intersection encountered, turning!\n");
  printf("left: %d; right: %d; front: %d\n", v_left, v_right, v_front);
  my_tracking_states = Intersection_turning;  
}

void motor_remote_control(uint8_t x, uint8_t y){
  int x_in;
  int y_in;
  int left_power;
  int right_power;
  
  if(x >= 165){
    x_in = (((int)x - 165) * 100) / 90;
  }
  else{
    x_in = (((int)x - 165) * 100) / 165;
  }
  
  if(y >= 170){
    y_in = (((int)y - 170) * 100) / 85;
  }
  else{
    y_in = (((int)y - 170) * 100) / 170;
  }
  
  /* Dead zone for joy stick */
  if(x_in < 20 && x_in > -20){
    x_in = 0;
  }
  if(y_in < 20 && y_in > -20){
    y_in = 0;
  }

  if(x_in == 0){
    left_power = y_in;
    right_power = -y_in;
  }
  else{
    left_power = x_in + y_in / TURN_PRESCALAR;
    right_power = x_in - y_in / TURN_PRESCALAR;
  }
  
  Set_Left_Motor(left_power);
  Set_Right_Motor(right_power);
}

/* ── IR command handler — called from ISR context (TIM6 tick) ───────────── */
/* Keep this function short: no blocking calls, no printf.                    */
void HandleCommand(uint8_t cmd_name, uint8_t data)
{
  switch (cmd_name)
  {
    case IR_CMD_START:
      ir_running = 1u;
      car_state = (ir_mode == IR_MODE_REMOTE) ? CAR_STATE_RUNNING_REMOTE : CAR_STATE_RUNNING_AUTO;
      break;

    case IR_CMD_PAUSE:
      ir_running = 0u;
      car_state = CAR_STATE_PAUSED;
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      break;

    case IR_CMD_RESET:
      ir_running    = 0u;
      ir_joystick_x = 128u;
      ir_joystick_y = 128u;
      ir_mode       = IR_MODE_AUTO;
      ir_path       = IR_PATH_1;
      car_state     = CAR_STATE_IDLE;
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      break;

    case IR_CMD_MODE:
      if ((data == IR_MODE_AUTO) || (data == IR_MODE_REMOTE)) {
        ir_mode = data;   /* IR_MODE_AUTO or IR_MODE_REMOTE */
        if (ir_running) {
          car_state = (ir_mode == IR_MODE_REMOTE) ? CAR_STATE_RUNNING_REMOTE : CAR_STATE_RUNNING_AUTO;
        }
      }
      break;

    case IR_CMD_PATH:
      if ((data == IR_PATH_1) || (data == IR_PATH_2) || (data == IR_PATH_3)) {
        ir_path = data;   /* IR_PATH_1 / IR_PATH_2 / IR_PATH_3 */
      }
      break;

    case IR_CMD_JOYSTICK_X:
      ir_joystick_x = data;
      break;

    case IR_CMD_JOYSTICK_Y:
      ir_joystick_y = data;
      break;

    default:
      break;
  }
}

void Set_Car_Speed(int speed){
  //speed should be between 0 and 100
  if(speed > 100) 
    speed = 100;
  if(speed < -100) 
    speed = -100;
  //STOP
  if(speed == 0) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  }
  //FORWARD
  else if(speed > 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    // Multiply by 10 (e.g., 100% * 10 = 1000 ARR)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed * 10); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed * 10);
  }
  //BACKWARD
  else{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    // Multiply by -10 (e.g., -100% * -10 = 1000 ARR)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (-speed) * 10); 
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (-speed) * 10);
  }
}

void uart_send_text(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

void uart_send_line(const char *s)
{
    uart_send_text(s);
    uart_send_text("\r\n");
}

uint8_t probe_addr(uint8_t addr)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 3, 100) == HAL_OK) ? 1u : 0u;
}

void i2c_scan(void)
{
    char msg[64];
    uint8_t found = 0;
    uint8_t addr;

    uart_send_line("I2C scan start");

    for (addr = 0x08; addr < 0x78; addr++)
    {
        if (probe_addr(addr))
        {
            snprintf(msg, sizeof(msg), "I2C device found at 0x%02X", addr);
            uart_send_line(msg);
            found = 1;
        }
    }

    if (!found)
    {
        uart_send_line("No I2C devices found");
    }

    uart_send_line("I2C scan end");
}

uint8_t imu_write_reg(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(imu_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK) ? 1u : 0u;
}

uint8_t imu_read_reg(uint8_t reg, uint8_t *val)
{
    return (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(imu_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100) == HAL_OK) ? 1u : 0u;
}

uint8_t imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(imu_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) ? 1u : 0u;
}

uint8_t MiniMU_FindIMU(void)
{
    if (probe_addr(LSM6DS33_ADDR_HIGH))
    {
        imu_addr = LSM6DS33_ADDR_HIGH;
        return 1;
    }

    if (probe_addr(LSM6DS33_ADDR_LOW))
    {
        imu_addr = LSM6DS33_ADDR_LOW;
        return 1;
    }

    imu_addr = 0;
    return 0;
}

uint8_t MiniMU_Init(void)
{
    uint8_t who = 0;
    uint8_t ctrl1 = 0;
    uint8_t ctrl2 = 0;
    uint8_t ctrl3 = 0;
    char msg[96];

    i2c_scan();

    if (!MiniMU_FindIMU())
    {
        uart_send_line("Could not find LSM6DS33 at 0x6B or 0x6A");
        return 0;
    }

    snprintf(msg, sizeof(msg), "LSM6DS33 addr = 0x%02X", imu_addr);
    uart_send_line(msg);

    if (!imu_read_reg(LSM6DS33_WHO_AM_I_REG, &who))
    {
        uart_send_line("WHO_AM_I read failed");
        return 0;
    }

    snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X", who);
    uart_send_line(msg);

    if (who != LSM6DS33_WHO_AM_I_VAL)
    {
        uart_send_line("WHO_AM_I mismatch");
        return 0;
    }

    if (!imu_write_reg(LSM6DS33_CTRL3_C, 0x01))
    {
        uart_send_line("CTRL3_C reset write failed");
        return 0;
    }

    if (!imu_write_reg(LSM6DS33_CTRL3_C, 0x44))
    {
        uart_send_line("CTRL3_C config write failed");
        return 0;
    }

    if (!imu_write_reg(LSM6DS33_CTRL1_XL, 0x40))
    {
        uart_send_line("CTRL1_XL write failed");
        return 0;
    }

    if (!imu_write_reg(LSM6DS33_CTRL2_G, 0x40))
    {
        uart_send_line("CTRL2_G write failed");
        return 0;
    }

    HAL_Delay(20);

    if (!imu_read_reg(LSM6DS33_CTRL1_XL, &ctrl1))
    {
        uart_send_line("CTRL1_XL readback failed");
        return 0;
    }

    if (!imu_read_reg(LSM6DS33_CTRL2_G, &ctrl2))
    {
        uart_send_line("CTRL2_G readback failed");
        return 0;
    }

    if (!imu_read_reg(LSM6DS33_CTRL3_C, &ctrl3))
    {
        uart_send_line("CTRL3_C readback failed");
        return 0;
    }

    snprintf(msg, sizeof(msg),
             "CTRL1_XL=0x%02X CTRL2_G=0x%02X CTRL3_C=0x%02X",
             ctrl1, ctrl2, ctrl3);
    uart_send_line(msg);

    if ((ctrl1 == 0x00) && (ctrl2 == 0x00))
    {
        uart_send_line("Sensor config did not stick");
        return 0;
    }

    uart_send_line("MiniMU init OK");
    return 1;
}

uint8_t MiniMU_ReadGyro(void)
{
    if (!imu_read_regs(LSM6DS33_OUTX_L_G, gyro_buf, 6))
    {
        return 0;
    }

    gyro_x = (int16_t)((((uint16_t)gyro_buf[1]) << 8) | gyro_buf[0]);
    gyro_y = (int16_t)((((uint16_t)gyro_buf[3]) << 8) | gyro_buf[2]);
    gyro_z = (int16_t)((((uint16_t)gyro_buf[5]) << 8) | gyro_buf[4]);

    return 1;
}

uint8_t MiniMU_ReadAccel(void)
{
    if (!imu_read_regs(LSM6DS33_OUTX_L_XL, accel_buf, 6))
    {
        return 0;
    }

    accel_x = (int16_t)((((uint16_t)accel_buf[1]) << 8) | accel_buf[0]);
    accel_y = (int16_t)((((uint16_t)accel_buf[3]) << 8) | accel_buf[2]);
    accel_z = (int16_t)((((uint16_t)accel_buf[5]) << 8) | accel_buf[4]);

    return 1;
}

void MiniMU_UpdateScaled(void)
{
    ax_g = ((float)accel_x) * ACCEL_SENS_G_PER_LSB;
    ay_g = ((float)accel_y) * ACCEL_SENS_G_PER_LSB;
    az_g = ((float)accel_z) * ACCEL_SENS_G_PER_LSB;

    gx_dps = ((float)gyro_x) * GYRO_SENS_DPS_PER_LSB - gx_bias;
    gy_dps = ((float)gyro_y) * GYRO_SENS_DPS_PER_LSB - gy_bias;
    gz_dps = ((float)gyro_z) * GYRO_SENS_DPS_PER_LSB - gz_bias;
}

void MiniMU_UpdateAngles(void)
{
    float denom_pitch;

    denom_pitch = sqrtf((ax_g * ax_g) + (az_g * az_g));

    if (denom_pitch < 0.0001f)
    {
        denom_pitch = 0.0001f;
    }

    roll_deg = atan2f(ay_g, az_g) * RAD_TO_DEG;
    pitch_deg = atan2f(-ax_g, denom_pitch) * RAD_TO_DEG;
}

void MiniMU_PrintScaledData(void)
{
    char msg[220];

    int ax_milli = (int)(ax_g * 1000.0f);
    int ay_milli = (int)(ay_g * 1000.0f);
    int az_milli = (int)(az_g * 1000.0f);

    int gx_milli = (int)(gx_dps * 1000.0f);
    int gy_milli = (int)(gy_dps * 1000.0f);
    int gz_milli = (int)(gz_dps * 1000.0f);

    snprintf(msg, sizeof(msg),
             "ACC[g] X:%d.%03d Y:%d.%03d Z:%d.%03d | GYRO[dps] X:%d.%03d Y:%d.%03d Z:%d.%03d",
             ax_milli / 1000, (ax_milli < 0 ? -ax_milli : ax_milli) % 1000,
             ay_milli / 1000, (ay_milli < 0 ? -ay_milli : ay_milli) % 1000,
             az_milli / 1000, (az_milli < 0 ? -az_milli : az_milli) % 1000,
             gx_milli / 1000, (gx_milli < 0 ? -gx_milli : gx_milli) % 1000,
             gy_milli / 1000, (gy_milli < 0 ? -gy_milli : gy_milli) % 1000,
             gz_milli / 1000, (gz_milli < 0 ? -gz_milli : gz_milli) % 1000);

    uart_send_line(msg);
}

void MiniMU_PrintAngles(void)
{
    char msg[120];

    int roll_hundredths = (int)(roll_deg * 100.0f);
    int pitch_hundredths = (int)(pitch_deg * 100.0f);

    snprintf(msg, sizeof(msg),
             "ANGLE roll:%d.%02d deg | pitch:%d.%02d deg",
             roll_hundredths / 100, (roll_hundredths < 0 ? -roll_hundredths : roll_hundredths) % 100,
             pitch_hundredths / 100, (pitch_hundredths < 0 ? -pitch_hundredths : pitch_hundredths) % 100);

    uart_send_line(msg);
}

uint8_t MiniMU_CalibrateGyro(void)
{
    uint32_t i;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    char msg[128];

    uart_send_line("Gyro calibration starting. Keep IMU still.");

    for (i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        if (!MiniMU_ReadGyro())
        {
            uart_send_line("Gyro calibration read failed");
            return 0;
        }

        sum_x += gyro_x;
        sum_y += gyro_y;
        sum_z += gyro_z;

        HAL_Delay(10);
    }

    gx_bias = ((float)sum_x / (float)GYRO_CAL_SAMPLES) * GYRO_SENS_DPS_PER_LSB;
    gy_bias = ((float)sum_y / (float)GYRO_CAL_SAMPLES) * GYRO_SENS_DPS_PER_LSB;
    gz_bias = ((float)sum_z / (float)GYRO_CAL_SAMPLES) * GYRO_SENS_DPS_PER_LSB;

    {
        int bx = (int)(gx_bias * 1000.0f);
        int by = (int)(gy_bias * 1000.0f);
        int bz = (int)(gz_bias * 1000.0f);

        snprintf(msg, sizeof(msg),
                 "Gyro bias[dps] X:%d.%03d Y:%d.%03d Z:%d.%03d",
                 bx / 1000, (bx < 0 ? -bx : bx) % 1000,
                 by / 1000, (by < 0 ? -by : by) % 1000,
                 bz / 1000, (bz < 0 ? -bz : bz) % 1000);
        uart_send_line(msg);
    }

    uart_send_line("Gyro calibration done");
    return 1;
}

void controller_State_Machine (void) {
  
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    HAL_Delay(150);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
