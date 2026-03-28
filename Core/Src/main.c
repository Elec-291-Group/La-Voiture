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
#include "datatypes.h"
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

// path tracking variables
uint8_t path_x[PATH_MAX_WAYPOINTS];
uint8_t path_y[PATH_MAX_WAYPOINTS];
uint8_t path_count = 0u;
uint8_t path_current_index = 0u;
uint8_t path_receiving = 0u;
uint8_t path_loaded = 0u;

// Vehicle Control 
enum path_tracking_states my_tracking_states = Running;
volatile uint16_t ir_joystick_x = IR_JOYSTICK_CENTER_X;
volatile uint16_t ir_joystick_y = IR_JOYSTICK_CENTER_Y;

volatile uint16_t ir_mode      = 0u;
volatile uint8_t  ir_running   = 0u;
volatile uint16_t ir_path      = 0u;
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
uint32_t intersection_encountered_time;
uint8_t intersection_number = 0;
enum intersection_directions path1[] = {Forward, Left, Left, Forward, Right, Left, Right, Stop};
//enum intersection_directions path1[] = {Left, Left, Left, Right, Right, Left, Right, Stop};
enum intersection_directions path2[] = {Left, Right, Left, Right, Forward, Forward, Stop};
enum intersection_directions path3[] = {Right, Forward, Right, Left, Right, Left, Forward, Stop};
uint8_t front_inductor_ready = 1;
uint32_t intersection_leave_time;
uint32_t last_intersection_turning_time;
uint32_t intersection_turn_settle_start_time;

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
void motor_remote_control(uint16_t, uint16_t);
void Set_Car_Speed(int speed);
void handle_intersection_stop(void);
void handle_intersection_compensation(void);
void handle_intersection_turn_settle(void);

// IMU 
void uart_send_text(const char *s);
void uart_send_line(const char *s);
void uart_send_uint32(uint32_t value);
void uart_send_pose(void);
void uart_send_origin(void);
void process_uart_command(char *line);
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
void HandleCommand(uint8_t cmd_name, uint16_t val);

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

static uint8_t text_equals(const char *a, const char *b)
{
    if ((a == NULL) || (b == NULL))
    {
        return 0u;
    }

    while ((*a != '\0') && (*b != '\0'))
    {
        if (*a != *b)
        {
            return 0u;
        }
        a++;
        b++;
    }

    return ((*a == '\0') && (*b == '\0')) ? 1u : 0u;
}

static int split_csv_fields(char *line, char **fields, int max_fields)
{
    int count = 0;

    if ((line == NULL) || (fields == NULL) || (max_fields <= 0))
    {
        return 0;
    }

    fields[count++] = line;

    while (*line != '\0')
    {
        if (*line == ',')
        {
            *line = '\0';
            if (count < max_fields)
            {
                fields[count++] = line + 1;
            }
        }
        line++;
    }

    return count;
}

void process_uart_command(char *line)
{
    char original_line[sizeof(rx_pending_buf)];
    char *tokens[4];
    int token_count = 0;
    int parsed_value;
    uint16_t i = 0u;

    if ((line == NULL) || (*line == '\0'))
    {
        return;
    }

    while ((i < (uint16_t)(sizeof(original_line) - 1u)) && (line[i] != '\0'))
    {
        original_line[i] = line[i];
        i++;
    }
    original_line[i] = '\0';

    token_count = split_csv_fields(line, tokens, (int)(sizeof(tokens) / sizeof(tokens[0])));

    if (token_count <= 0)
    {
        return;
    }

    if (text_equals(tokens[0], "PATH_BEGIN"))
    {
        if ((token_count >= 2) &&
            parse_int_simple(tokens[1], &parsed_value) &&
            (parsed_value >= 0) &&
            (parsed_value <= (int)PATH_MAX_WAYPOINTS))
        {
            path_count = (uint8_t)parsed_value;
            path_current_index = 0u;
            path_receiving = 1u;
            path_loaded = 0u;
            uart_send_line("PATH_ACK,BEGIN");
        }
        return;
    }

    if (text_equals(tokens[0], "WPT"))
    {
        int idx;
        int x_cm;
        int y_cm;

        if ((token_count >= 4) &&
            path_receiving &&
            parse_int_simple(tokens[1], &idx) &&
            parse_int_simple(tokens[2], &x_cm) &&
            parse_int_simple(tokens[3], &y_cm) &&
            (idx >= 0) &&
            (idx < (int)PATH_MAX_WAYPOINTS) &&
            (x_cm >= 0) &&
            (x_cm <= 255) &&
            (y_cm >= 0) &&
            (y_cm <= 255))
        {
            path_x[idx] = (uint8_t)x_cm;
            path_y[idx] = (uint8_t)y_cm;
            if ((uint8_t)(idx + 1) > path_count)
            {
                path_count = (uint8_t)(idx + 1);
            }
            uart_send_text("RX_CMD,");
            uart_send_line(original_line);
        }
        return;
    }

    if (text_equals(tokens[0], "PATH_END"))
    {
        if (path_receiving)
        {
            path_receiving = 0u;
            path_loaded = (path_count > 0u) ? 1u : 0u;
            path_current_index = 0u;
            uart_send_line("PATH_ACK,LOADED");
        }
        return;
    }

    if (text_equals(tokens[0], "ZERO_YAW"))
    {
        yaw_zero_deg = yaw_deg;
        uart_send_line("ZERO_YAW_ACK,OK");
    }
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

  reset_pose_origin();
  last_imu_update_ms = HAL_GetTick();
  last_pose_stream_ms = last_imu_update_ms;
  last_guidewire_sample_ms = last_imu_update_ms;
  uart_send_origin();

  // Configure ADC channel
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  vdda_calibration();

  // i2c init
  VL53L0X_Init();

  // IR init
  htim6.Instance->ARR = 262u;
  htim6.Instance->EGR = 0x01U;
  IR_RX_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  IR_TX_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  
  // IMU init
  if (!MiniMU_Init())
  {
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
          HAL_Delay(1000);
      }
  }

  if (!MiniMU_CalibrateGyro())
  {
      while (1)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
          HAL_Delay(1000);
      }
  }

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

    if (ir_rx_ready) {
        ir_rx_ready = 0u;
        HandleCommand(ir_rx_frame.cmd, ir_rx_frame.val);
    }

    if (rx_line_ready)
    {
        char line_buf[sizeof(rx_pending_buf)];
        uint16_t copy_len = rx_pending_len;
        uint16_t i;

        if (copy_len >= sizeof(line_buf))
        {
            copy_len = (uint16_t)(sizeof(line_buf) - 1u);
        }

        for (i = 0u; i < copy_len; i++)
        {
            line_buf[i] = (char)rx_pending_buf[i];
        }
        line_buf[copy_len] = '\0';
        rx_pending_len = 0u;
        rx_line_ready = 0u;
        process_uart_command(line_buf);
    }

    IR_RX_Update();

    /*--------------------------------------------------------------------------*/
    // Collision Detection
    /*--------------------------------------------------------------------------*/
    static uint32_t last_dist_ms = 0;
    static uint16_t current_distance = 8190; // Default to max range

    if (HAL_GetTick() - last_dist_ms >= 100u) {
      last_dist_ms = HAL_GetTick();
      uint16_t reading = VL53L0X_ReadDistance();
      if (reading != 0xFFFFu) {
          current_distance = reading;
      } else {
          current_distance = 8190;
      }
      //printf("[DIST] raw=%u  used=%u\r\n", reading, current_distance);
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
    /* RX: decode every received frame and dispatch to HandleCommand */
    if (ir_rx_ready) {
        ir_rx_ready = 0u;
        /* RX debug print intentionally disabled to save flash. */
        HandleCommand(ir_rx_frame.cmd, ir_rx_frame.val);
    }
}

/* ── IR command handler ─────────────────────────────────────────────────── */
void path_tracking(void){
  sample_guidewire_sensors();
  //printf("%d\n", my_tracking_states);
  
  switch(my_tracking_states){
    case Running:
      handle_line_tracking();
      break;
    
    case Intersection_encountered:
      handle_intersection_encountered();
      break;

    case Intersection_compensation:
      handle_intersection_compensation();
      break;

    case Intersection_turning:
      handle_intersection_turning();
      break;

    case Intersection_turn_settle:
      handle_intersection_turn_settle();
      break;

    case Intersection_stop:
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
  Set_Left_Motor(left_power);
  Set_Right_Motor(right_power);

  v_front_last = v_front;
}

void handle_intersection_encountered(void){
  Set_Left_Motor(0);
  Set_Right_Motor(0);
  intersection_number += 1;
  //my_tracking_states = Intersection_turning;
  my_tracking_states = Intersection_compensation;
  intersection_encountered_time = HAL_GetTick();
  //last_intersection_turning_time = HAL_GetTick();
  //intersection_turn_current_yaw_angle = 0;
  //front_inductor_ready = 0;
}

void handle_intersection_compensation(void){
  if(HAL_GetTick() - intersection_encountered_time < INTERSECTION_COMPENSATION_TIME){
    Set_Left_Motor(50);
    Set_Right_Motor(50);
    my_tracking_states = Intersection_compensation;
  }
  else{
    Set_Left_Motor(0);
    Set_Right_Motor(0);
    my_tracking_states = Intersection_turning;
    last_intersection_turning_time = HAL_GetTick();
    intersection_turn_current_yaw_angle = 0;
    front_inductor_ready = 0;
  }
}

void handle_intersection_turning(void){
  //uint32_t intersection_turning_current_time = HAL_GetTick();
  float dt_s = (HAL_GetTick() - last_intersection_turning_time) / 1000.0f;
  last_intersection_turning_time = HAL_GetTick();
  intersection_turn_current_yaw_angle = intersection_turn_current_yaw_angle + gz_dps * dt_s;
  enum intersection_directions current_direction;
  switch(ir_path){
    case IR_PATH_1:
      current_direction = path1[intersection_number-1];
      break;
    
    case IR_PATH_2:
      current_direction = path2[intersection_number-1];
      break;

    case IR_PATH_3:
      current_direction = path3[intersection_number-1];
      break;
    
    default:
      current_direction = Stop;
      break;
  }
  
  switch(current_direction){
    case Forward:
      my_tracking_states = Running;
      intersection_leave_time = HAL_GetTick();
      break;

    case Left:
      if(intersection_turn_current_yaw_angle > LEFT_TURN_DEGREE){
        Set_Left_Motor(-65); 
        Set_Right_Motor(65);
        my_tracking_states = Intersection_turning;  
      }
      else{
        Set_Left_Motor(0);
        Set_Right_Motor(0);
        intersection_turn_settle_start_time = HAL_GetTick();
        my_tracking_states = Intersection_turn_settle;
      }
      break;
    
    case Right:
      if(intersection_turn_current_yaw_angle < RIGHT_TURN_DEGREE){
        Set_Left_Motor(65);
        Set_Right_Motor(-65);
        my_tracking_states = Intersection_turning;  
      }
      else{
        Set_Left_Motor(0);
        Set_Right_Motor(0);
        intersection_turn_settle_start_time = HAL_GetTick();
        my_tracking_states = Intersection_turn_settle;
      }
      break;

    case Stop:
      my_tracking_states = Intersection_stop;
      intersection_leave_time = HAL_GetTick();
      break;
  }  
}

void handle_intersection_turn_settle(void){
  Set_Left_Motor(0);
  Set_Right_Motor(0);

  if(HAL_GetTick() - intersection_turn_settle_start_time < INTERSECTION_TURN_SETTLE_TIME){
    my_tracking_states = Intersection_turn_settle;
  }
  else{
    my_tracking_states = Running;
    intersection_leave_time = HAL_GetTick();
  }
}

void handle_intersection_stop(void){
  Set_Left_Motor(0);
  Set_Right_Motor(0);
  my_tracking_states = Intersection_stop;
}

void motor_remote_control(uint16_t x, uint16_t y){
  int x_in;
  int y_in;
  int left_power;
  int right_power;

  if(x >= 165u){
    x_in = (((int)x - 165) * 100) / 90;
  }
  else{
    x_in = (((int)x - 165) * 100) / 165;
  }

  if(y >= 170u){
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

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1u, HAL_MAX_DELAY);
    return ch;
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
    static uint32_t last_path_control_debug_ms = 0u;
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

    if (path_current_index >= path_count)
    {
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

    if ((HAL_GetTick() - last_path_control_debug_ms) >= 200u)
    {
        last_path_control_debug_ms = HAL_GetTick();
        printf("[PATH_CTRL] mode=%u state=%u idx=%u/%u\r\n",
               ir_mode, car_state, path_current_index, path_count);
        printf("[PATH_CTRL] pose=(%.2f,%.2f) yaw=%.2f zero=%.2f\r\n",
               pose_x_cm, pose_y_cm, yaw_deg, yaw_zero_deg);
        printf("[PATH_CTRL] target=(%.2f,%.2f) dx=%.2f dy=%.2f dist=%.2f\r\n",
               target_x, target_y, dx, dy, distance_cm);
        printf("[PATH_CTRL] heading=%.2f target=%.2f err=%.2f steer=%d\r\n",
               current_heading_deg, target_heading_deg, heading_error_deg, steer_cmd);
        printf("[PATH_CTRL] drive=%d scaled=%d left=%d right=%d\r\n",
               drive_cmd, current_drive_cmd, left_cmd, right_cmd);
    }

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
        if (!IR_TX_Busy())
        {
          static uint8_t imu_tx_idx = 0u;
          uint16_t val;
          uint8_t  cmd;
          switch (imu_tx_idx)
          {
            case 0:  cmd = IR_CMD_ACCEL_X; val = (uint16_t)accel_x; break;
            case 1:  cmd = IR_CMD_ACCEL_Y; val = (uint16_t)accel_y; break;
            case 2:  cmd = IR_CMD_ACCEL_Z; val = (uint16_t)accel_z; break;
            case 3:  cmd = IR_CMD_GYRO_X;  val = (uint16_t)gyro_x;  break;
            case 4:  cmd = IR_CMD_GYRO_Y;  val = (uint16_t)gyro_y;  break;
            default: cmd = IR_CMD_GYRO_Z;  val = (uint16_t)gyro_z;  break;
          }
          IR_Send_Cmd(cmd, val);
          if (++imu_tx_idx >= 6u) imu_tx_idx = 0u;
        }
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

/* ── IR command handler — called from main loop on each decoded IR frame ─── */
/* Keep this function short: no blocking calls, no printf.                    */
void HandleCommand(uint8_t cmd_name, uint16_t val)
{
  switch (cmd_name)
  {
    case IR_CMD_START:
      ir_running = 1u;
      break;

    case IR_CMD_PAUSE:
      ir_running = 0u;
      break;

    case IR_CMD_RESET:
      ir_reset = 1u;
      ir_joystick_x = IR_JOYSTICK_CENTER_X;
      ir_joystick_y = IR_JOYSTICK_CENTER_Y;
      ir_mode       = IR_MODE_FIELD;
      ir_path       = IR_PATH_1;
      break;

    case IR_CMD_MODE:
      ir_mode = val;
      break;

    case IR_CMD_PATH:
      ir_path = val;
      break;

    case IR_CMD_JOYSTICK_X:
      ir_joystick_x = val;
      break;

    case IR_CMD_JOYSTICK_Y:
      ir_joystick_y = val;
      break;

    default:
      /* path waypoint: cmd = PATH_CMD_BASE + index (7..38) */
      if (cmd_name >= PATH_CMD_BASE && cmd_name < PATH_CMD_BASE + PATH_MAX_WAYPOINTS)
      {
        uint8_t idx = cmd_name - PATH_CMD_BASE;
        path_x[idx] = (uint8_t)((val >> 8) & 0xFFu);  /* high byte = x_cm */
        path_y[idx] = (uint8_t)(val & 0xFFu);           /* low byte  = y_cm */
        if (idx + 1u > path_count)
        {
          path_count = idx + 1u;
        }
        if (!IR_TX_Busy() && (path_x[idx] != 0u || path_y[idx] != 0u))
        {
          IR_Send_Cmd(IR_CMD_WAYPOINTS_ARRIVED, 0x0000u);
        }
      }
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
