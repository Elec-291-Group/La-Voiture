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
#include "ir_tx.h"
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
#define GYRO_SENS_DPS_PER_LSB     0.00875f

#define GYRO_CAL_SAMPLES          200
#define RAD_TO_DEG                57.2957795f

// Vehicle Control 
// needed to be tuned !!!
#define KP 0.020f
#define KS 0.015f
#define KF 0.025f // decrease speed before approaching the intersection
//-------- base power of motor in auto mode -----------------
#define PB 65
#define STOP_VF 1500

//For sensor
#define VL53L0X_I2C_ADDR 0x52 

// ms it takes for the car to rotate one full circle at 60% power
#define IMU_UPDATE_PERIOD_MS      20u
#define POSE_STREAM_PERIOD_MS     100u
#define PATH_MAX_WAYPOINTS        32u
#define PATH_REACHED_TOLERANCE_CM 8.0f
#define DRIVE_SPEED_SCALE_CM_S    0.35f
#define PATH_FORWARD_SPEED        40
#define PATH_APPROACH_SPEED       25
#define PATH_STEER_GAIN           1.0f
#define PATH_MAX_STEER_CMD        55
#define MOTOR_TEST_SPEED          40
#define IR_JOYSTICK_CENTER_X      165u
#define IR_JOYSTICK_CENTER_Y      170u
#define GUIDEWIRE_FRONT_THRESHOLD_MV    1000u
#define GUIDEWIRE_BALANCE_TOLERANCE_MV   150u
#define GUIDEWIRE_LOCK_SAMPLES_REQUIRED    5u
#define GUIDEWIRE_SAMPLE_PERIOD_MS        20u

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
static float yaw_deg = 0.0f;
static float yaw_zero_deg = 0.0f;

static float pose_x_cm = 0.0f;
static float pose_y_cm = 0.0f;
static int current_left_motor_cmd = 0;
static int current_right_motor_cmd = 0;
static int current_drive_cmd = 0;
static uint32_t last_imu_update_ms = 0u;
static uint32_t last_pose_stream_ms = 0u;
static uint32_t last_motor_debug_ms = 0u;
static uint8_t origin_sent = 0u;
static uint32_t last_guidewire_sample_ms = 0u;
static uint8_t guidewire_origin_locked = 0u;
static uint8_t guidewire_lock_streak = 0u;

uint8_t rx_line_buf[96];
uint16_t rx_line_len = 0u;
uint8_t rx_pending_buf[96];
volatile uint16_t rx_pending_len = 0u;
volatile uint8_t rx_line_ready = 0u;
uint8_t uart_rx_byte = 0u;

float path_x[PATH_MAX_WAYPOINTS];
float path_y[PATH_MAX_WAYPOINTS];
uint8_t path_count = 0u;
uint8_t path_current_index = 0u;
uint8_t path_receiving = 0u;
uint8_t path_loaded = 0u;
uint8_t path_ready = 0u;
uint8_t motor_test_active = 0u;

// Vehicle Control 
enum path_tracking_states my_tracking_states = Running;
volatile uint8_t ir_joystick_x = IR_JOYSTICK_CENTER_X;
volatile uint8_t ir_joystick_y = IR_JOYSTICK_CENTER_Y;


volatile uint8_t ir_mode       = 0u;     
volatile uint8_t ir_running    = 0u;    
volatile uint8_t ir_path       = 0u;   
volatile uint8_t ir_reset      = 0u; 

// Inductor readings
uint16_t adc0;
uint16_t v_left;
uint16_t adc1;
uint16_t v_right;
uint16_t adc9;
uint16_t v_front;
uint16_t v_front_last;

//float intersection_encountered_yaw_angle;
float intersection_turn_current_yaw_angle;
//uint32_t intersection_encountered_time;
uint8_t intersection_number = 0;
//enum intersection_directions path1[] = {Forward, Left, Left, Forward, Right, Left, Right, Stop};
enum intersection_directions path1[] = {Left, Left, Left, Right, Right, Left, Right, Stop};
uint8_t front_inductor_ready = 1;
uint32_t intersection_leave_time;
uint32_t last_intersection_turning_time;

// Time
uint32_t now_ms;
float dt_s;

// Controller State Machine 
typedef enum 
{
  STATE_CONFIG, 
  STATE_DRIVE,
  STATE_PAUSE
} ControllerState;

// belongs to DRIVE state in ControllerState
typedef enum
{
  STATE_FIELD_TRACKING,
  STATE_REMOTE,
  STATE_PATH_TRACKING
} CarState;

ControllerState controller_state = STATE_CONFIG;
CarState car_state = STATE_FIELD_TRACKING;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void IR_Debug_Update(void);
void Set_Left_Motor(int speed);
void Set_Right_Motor(int speed);
void handle_intersection_encountered(void);
void handle_intersection_turning(void);
void path_tracking(void);
void handle_line_tracking(void);
void motor_remote_control(uint8_t, uint8_t);
void Set_Car_Speed(int speed);
void handle_intersection_stop(void);

// IMU 
void uart_send_text(const char *s);
void uart_send_line(const char *s);
void uart_send_uint32(uint32_t value);
void uart_send_path_tracking_debug(void);
void uart_send_origin(void);
// void process_uart_command(char *line);
void process_pathfinder_control(float dt_s);
void sample_guidewire_sensors(void);
void update_guidewire_origin_reference(void);
void reset_pose_origin(void);
float wrap_angle_deg(float angle_deg);
int clamp_motor_speed(int speed);
int parse_int_simple(const char *s, int *out);

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

void IMUUpdate(void); //MAIN IMU FUNCTION

// State Machine Functions
void ControllerStateMachine(void);
void CarStateMachine(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UART_StartRxIT(void)
{
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
}

float wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

int clamp_motor_speed(int speed)
{
    if (speed > 100)
    {
        return 100;
    }

    if (speed < -100)
    {
        return -100;
    }

    return speed;
}

int parse_int_simple(const char *s, int *out)
{
    int sign = 1;
    int value = 0;

    if ((s == NULL) || (out == NULL) || (*s == '\0'))
    {
        return 0;
    }

    if (*s == '-')
    {
        sign = -1;
        s++;
    }

    if (*s == '\0')
    {
        return 0;
    }

    while (*s != '\0')
    {
        if ((*s < '0') || (*s > '9'))
        {
            return 0;
        }

        value = (value * 10) + (*s - '0');
        s++;
    }

    *out = sign * value;
    return 1;
}

void reset_pose_origin(void)
{
    pose_x_cm = 0.0f;
    pose_y_cm = 0.0f;
    yaw_zero_deg = yaw_deg;
    current_drive_cmd = 0;
    origin_sent = 0u;
    guidewire_origin_locked = 0u;
    guidewire_lock_streak = 0u;
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
  UART_StartRxIT();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  htim6.Instance->ARR = 262u;
  htim6.Instance->EGR = 0x01U;
  IR_RX_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  IR_TX_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  
  reset_pose_origin();
  last_imu_update_ms = HAL_GetTick();
  last_pose_stream_ms = last_imu_update_ms;
  last_guidewire_sample_ms = last_imu_update_ms;
  uart_send_origin();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    now_ms = HAL_GetTick();
    IR_Debug_Update(); // IR debug function
    /*--------------------------------------------------------------------------*/
    // Collision Detection
    /*--------------------------------------------------------------------------*/
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

    if (current_distance < 100u)
    {
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      current_drive_cmd = 0;
    }
    else
    {
      ControllerStateMachine();
    }
    /*--------------------------------------------------------------------------*/
  /* USER CODE END 3 */
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
  speed = clamp_motor_speed(speed);
  current_left_motor_cmd = speed;
  
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
  speed = clamp_motor_speed(speed);
  current_right_motor_cmd = speed;
  
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


/* TX addr=0x7 continuously, print any received addr=0x6 frame. */
void IR_Debug_Update(void)
{
    /* TX: keep sending addr=0x7 frames every 100 ms */
    static uint32_t last_tx_ms = 0;
    static uint8_t  tx_reg     = 0;

    if (!IR_TX_Busy() && (HAL_GetTick() - last_tx_ms) >= 100u) {
        last_tx_ms = HAL_GetTick();
        IR_Send_IMU(tx_reg, 0xABCD);
        tx_reg = (tx_reg + 1u) % IMU_REG_COUNT;
    }
    /* RX: print every frame received with addr=0x6 */
    if (ir_rx_ready) {
        ir_rx_ready = 0u;
        printf("[RX] addr=0x%X  cmd=%u  val=0x%04X\r\n",
               ir_rx_frame.addr, ir_rx_frame.cmd, ir_rx_frame.val);
    }
}

/* ── IR command handler ─────────────────────────────────────────────────── */
void path_tracking(void){
  sample_guidewire_sensors();
  
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

    case Stop:
      handle_intersection_stop();
      break;
  }
}

void sample_guidewire_sensors(void)
{
  adc0 = read_adc_channel(ADC_CHANNEL_0);
  adc1 = read_adc_channel(ADC_CHANNEL_1);
  adc9 = read_adc_channel(ADC_CHANNEL_9);

  // left, right, front and adc mapping depend on wiring
  v_left = adc_to_voltage(adc0);
  v_right = adc_to_voltage(adc9);
  v_front = adc_to_voltage(adc1);
}

void update_guidewire_origin_reference(void)
{
  uint32_t lateral_error_mv;

  if (guidewire_origin_locked)
  {
    return;
  }

  lateral_error_mv = (v_left > v_right) ? (v_left - v_right) : (v_right - v_left);

  if ((v_front >= GUIDEWIRE_FRONT_THRESHOLD_MV) &&
      (lateral_error_mv <= GUIDEWIRE_BALANCE_TOLERANCE_MV))
  {
    if (guidewire_lock_streak < GUIDEWIRE_LOCK_SAMPLES_REQUIRED)
    {
      guidewire_lock_streak++;
    }
  }
  else
  {
    guidewire_lock_streak = 0u;
  }

  if (guidewire_lock_streak >= GUIDEWIRE_LOCK_SAMPLES_REQUIRED)
  {
    reset_pose_origin();
    guidewire_origin_locked = 1u;
    uart_send_origin();
    uart_send_pose();
  }
}

void handle_line_tracking(void){
  int base_power;
  int error;
  int left_power;
  int right_power;
  uint32_t line_tracking_current_time = HAL_GetTick();
  if(line_tracking_current_time - intersection_leave_time > 3000){
    front_inductor_ready = 1;
  }

  error = (int)v_left - (int)v_right;
  base_power = PB - KS * abs(error);
  
  /*
  if(v_front > 700){
    base_power = base_power - KF * v_front;
    if (base_power < 45) {
      base_power = 45;
    }
  }
  */
  if(v_front > 700 && v_front > v_front_last){
    base_power = PB - 0.1 * (v_front - v_front_last);
  }
  
  left_power = base_power - KP * error;
  right_power = base_power + KP * error;

  if(front_inductor_ready == 1 && v_front > STOP_VF){
    my_tracking_states = Intersection_encountered;
  }
  else{
    my_tracking_states = Running;
  }

  //printf("left_power: %d; right_power: %d\n", left_power, right_power);
  uart_send_path_tracking_debug();
  Set_Left_Motor(left_power);
  Set_Right_Motor(right_power);

  v_front_last = v_front;
}

void handle_intersection_encountered(void){
  Set_Left_Motor(0);
  Set_Right_Motor(0);
  intersection_number += 1;
  my_tracking_states = Intersection_turning;
  uart_send_line("START_TURN");
  //intersection_encountered_yaw_angle = yaw_deg;
  //intersection_turn_current_yaw_angle = yaw_deg;
  last_intersection_turning_time = HAL_GetTick();
  intersection_turn_current_yaw_angle = 0;
  front_inductor_ready = 0;
}

void handle_intersection_turning(void){
  //uint32_t intersection_turning_current_time = HAL_GetTick();
  float dt_s = (HAL_GetTick() - last_intersection_turning_time) / 1000.0f;
  last_intersection_turning_time = HAL_GetTick();
  intersection_turn_current_yaw_angle = intersection_turn_current_yaw_angle + gz_dps * dt_s;
  enum intersection_directions current_direction = path1[intersection_number-1];
  uart_send_path_tracking_debug();
  
  switch(current_direction){
    case Forward:
      my_tracking_states = Running;
      uart_send_line("END_TURN");
      intersection_leave_time = HAL_GetTick();
      break;

    case Left:
      if(intersection_turn_current_yaw_angle > -67){
        Set_Left_Motor(-65); 
        Set_Right_Motor(65);
        my_tracking_states = Intersection_turning;  
      }
      else{
        my_tracking_states = Running;
        uart_send_line("END_TURN");
        intersection_leave_time = HAL_GetTick();
      }
      break;
    
    case Right:
      if(intersection_turn_current_yaw_angle < 90){
        Set_Left_Motor(65);
        Set_Right_Motor(-65);
        my_tracking_states = Intersection_turning;  
      }
      else{
        my_tracking_states = Running;
        uart_send_line("END_TURN");
        intersection_leave_time = HAL_GetTick();
      }
      break;

    case Stop:
      my_tracking_states = Stop;
      intersection_leave_time = HAL_GetTick();
      break;
  }  
}

void handle_intersection_stop(void){
  Set_Left_Motor(0);
  Set_Right_Motor(0);
  my_tracking_states = Stop;
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
    left_power = x_in + y_in / 2;
    right_power = x_in - y_in / 2;
  }

  current_drive_cmd = 0;
  
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
      motor_test_active = 0u;
      ir_running = 1u;
      break;

    case IR_CMD_PAUSE:
      motor_test_active = 0u;
      ir_running = 0u;
      path_ready = 0u;
      break;

    case IR_CMD_RESET:
      motor_test_active = 0u;
      ir_running    = 0u;
      path_ready    = 0u;
      ir_joystick_x = IR_JOYSTICK_CENTER_X;
      ir_joystick_y = IR_JOYSTICK_CENTER_Y;
      ir_mode       = IR_MODE_FIELD;
      ir_path       = IR_PATH_1;
      break;

    case IR_CMD_MODE:
      ir_mode = data;   /* IR_MODE_FIELD / IR_MODE_REMOTE / IR_MODE_PATH */
      break;

    case IR_CMD_PATH:
      ir_path = data;   /* IR_PATH_1 / IR_PATH_2 / IR_PATH_3 */
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

void uart_send_uint32(uint32_t value)
{
    char buf[10];
    int i = 0;

    if (value == 0u)
    {
        uart_send_text("0");
        return;
    }

    while ((value > 0u) && (i < (int)sizeof(buf)))
    {
        buf[i++] = (char)('0' + (value % 10u));
        value /= 10u;
    }

    while (i > 0)
    {
        i--;
        HAL_UART_Transmit(&huart1, (uint8_t *)&buf[i], 1u, HAL_MAX_DELAY);
    }
}

void uart_send_path_tracking_debug(void)
{
    static uint32_t last_path_debug_ms = 0u;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_path_debug_ms) < 100u)
    {
        return;
    }

    last_path_debug_ms = now_ms;
    uart_send_text("v_front=");
    uart_send_uint32(v_front);
    uart_send_text(" v_left=");
    uart_send_uint32(v_left);
    uart_send_text(" v_right=");
    uart_send_uint32(v_right);
    uart_send_text("\r\n");
}

void uart_send_origin(void)
{
    uart_send_line("ORIGIN,0,0");
    origin_sent = 1u;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t i;

    if (huart->Instance != USART1)
    {
        return;
    }

    if ((uart_rx_byte == '\r') || (uart_rx_byte == '\n'))
    {
        if ((rx_line_len > 0u) && !rx_line_ready)
        {
            for (i = 0u; i < rx_line_len; i++)
            {
                rx_pending_buf[i] = rx_line_buf[i];
            }
            rx_pending_len = rx_line_len;
            rx_line_len = 0u;
            rx_line_ready = 1u;
        }
        else
        {
            rx_line_len = 0u;
        }
    }
    else if (rx_line_len < (sizeof(rx_line_buf) - 1u))
    {
        rx_line_buf[rx_line_len++] = uart_rx_byte;
    }
    else
    {
        rx_line_len = 0u;
    }

    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    rx_line_len = 0u;
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
}

void process_pathfinder_control(float dt_s)
{
    float target_x;
    float target_y;
    float dx;
    float dy;
    float distance_cm;
    float current_heading_deg;
    float target_heading_deg;
    float heading_error_deg;
    float forward_scale;
    int drive_cmd;
    int steer_cmd;
    int left_cmd;
    int right_cmd;

    if (!path_ready || (path_count == 0u))
    {
        return;
    }

    if (path_current_index >= path_count)
    {
        path_ready = 0u;
        current_drive_cmd = 0;
        Set_Left_Motor(0);
        Set_Right_Motor(0);
        return;
    }

    target_x = path_x[path_current_index];
    target_y = path_y[path_current_index];
    dx = target_x - pose_x_cm;
    dy = target_y - pose_y_cm;
    distance_cm = sqrtf((dx * dx) + (dy * dy));
    current_heading_deg = wrap_angle_deg(yaw_deg - yaw_zero_deg);
    target_heading_deg = atan2f(dy, dx) * RAD_TO_DEG;
    heading_error_deg = wrap_angle_deg(target_heading_deg - current_heading_deg);

    if (distance_cm <= PATH_REACHED_TOLERANCE_CM)
    {
        path_current_index++;
        if (path_current_index >= path_count)
        {
            path_ready = 0u;
            current_drive_cmd = 0;
            Set_Left_Motor(0);
            Set_Right_Motor(0);
        }
        return;
    }

    drive_cmd = PATH_FORWARD_SPEED;
    if (distance_cm < 20.0f)
    {
        drive_cmd = PATH_APPROACH_SPEED;
    }

    if (fabsf(heading_error_deg) > 60.0f)
    {
        drive_cmd = PATH_APPROACH_SPEED;
    }

    steer_cmd = (int)(heading_error_deg * PATH_STEER_GAIN);

    if (steer_cmd > PATH_MAX_STEER_CMD)
    {
        steer_cmd = PATH_MAX_STEER_CMD;
    }
    else if (steer_cmd < -PATH_MAX_STEER_CMD)
    {
        steer_cmd = -PATH_MAX_STEER_CMD;
    }

    forward_scale = cosf(fabsf(heading_error_deg) / RAD_TO_DEG);
    if (forward_scale < 0.0f)
    {
        forward_scale = 0.0f;
    }

    current_drive_cmd = (int)(drive_cmd * forward_scale);
    left_cmd = clamp_motor_speed(drive_cmd + steer_cmd);
    right_cmd = clamp_motor_speed(drive_cmd - steer_cmd);

    Set_Left_Motor(left_cmd);
    Set_Right_Motor(right_cmd);

    (void)dt_s;
}

uint8_t probe_addr(uint8_t addr)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 3, 100) == HAL_OK) ? 1u : 0u;
}

void i2c_scan(void)
{
    uint8_t addr;

    for (addr = 0x08; addr < 0x78; addr++)
    {
        if (probe_addr(addr))
        {
            uart_send_line("I2C device found");
        }
    }
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
        return 0;
    }

    imu_write_reg(LSM6DS33_CTRL1_XL, 0x40);
    imu_write_reg(LSM6DS33_CTRL2_G, 0x40);
    imu_write_reg(LSM6DS33_CTRL3_C, 0x44);


    return 1;
}

uint8_t MiniMU_ReadGyro(void)
{
    imu_read_regs(LSM6DS33_OUTX_L_G, gyro_buf, 6);

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
    ax_g = ((float)accel_x) *  0.000061f;
    ay_g = ((float)accel_y) *  0.000061f;
    az_g = ((float)accel_z) *  0.000061f;

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
    /* Detailed IMU telemetry formatting disabled to reduce flash usage. */
}

void MiniMU_PrintAngles(void)
{
    /* Detailed angle telemetry formatting disabled to reduce flash usage. */
}

uint8_t MiniMU_CalibrateGyro(void)
{
    uint32_t i;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    for (i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        if (!MiniMU_ReadGyro())
        {
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

    return 1;
}

void IMUUpdate(void)
{
  if ((now_ms - last_imu_update_ms) >= IMU_UPDATE_PERIOD_MS)
    {
      dt_s = (float)(now_ms - last_imu_update_ms) / 1000.0f;
      last_imu_update_ms = now_ms;

      if (!MiniMU_ReadGyro()) {}
      else if (!MiniMU_ReadAccel()) {}
      else
      {
        MiniMU_UpdateScaled();
        MiniMU_UpdateAngles();

        yaw_deg = wrap_angle_deg(yaw_deg + gz_dps * dt_s);
        pose_x_cm += ((float)current_drive_cmd * DRIVE_SPEED_SCALE_CM_S * dt_s) * cosf((yaw_deg - yaw_zero_deg) / RAD_TO_DEG);
        pose_y_cm += ((float)current_drive_cmd * DRIVE_SPEED_SCALE_CM_S * dt_s) * sinf((yaw_deg - yaw_zero_deg) / RAD_TO_DEG);
      }
    }

    if ((now_ms - last_pose_stream_ms) >= POSE_STREAM_PERIOD_MS)
    {
      last_pose_stream_ms = now_ms;
      /* Serial debug mode: suppress periodic origin/pose telemetry. */
    }
}
// MAIN CONTROLLER STATE MACHINE
void ControllerStateMachine(void) 
{
  switch (controller_state) 
  {
    case STATE_CONFIG:
      current_drive_cmd = 0;
      ir_reset = 0;
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      if (ir_running)
      {
        controller_state = STATE_DRIVE;
      }
      break;

    case STATE_DRIVE:
      IMUUpdate();
      if (!ir_running)
      {
        controller_state = STATE_PAUSE;
      }
      else
      {
        
        CarStateMachine();
      }
      break;

    case STATE_PAUSE:
      current_drive_cmd = 0;
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      IMUUpdate();
      if (ir_running)
      {
        controller_state = STATE_DRIVE;
      }
      else if (ir_reset)
      {
        controller_state = STATE_CONFIG;
      }
      break;

    default:
      controller_state = STATE_CONFIG;
      break;
  }
}

void CarStateMachine(void) 
{
  if (ir_mode == IR_MODE_FIELD)
  {
    car_state = STATE_FIELD_TRACKING;
  }
  else if (ir_mode == IR_MODE_REMOTE)
  {
    car_state = STATE_REMOTE;
  }
  else if (ir_mode == IR_MODE_PATH)
  {
    car_state = STATE_PATH_TRACKING;
  }
  else
  {
    car_state = STATE_FIELD_TRACKING;
  }

  switch (car_state)
  {
    case STATE_FIELD_TRACKING:
      path_tracking();
      break;

    case STATE_REMOTE:
      motor_remote_control(ir_joystick_x, ir_joystick_y);
      break;

    case STATE_PATH_TRACKING:
      process_pathfinder_control((float)(HAL_GetTick() - last_imu_update_ms) / 1000.0f);
      break;

    default:
      current_drive_cmd = 0;
      Set_Left_Motor(0);
      Set_Right_Motor(0);
      car_state = STATE_FIELD_TRACKING;
      break;
  }
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