#ifndef CONFIG_H
#define CONFIG_H

// ---- System clock / UART / ADC constants ----
#define SYSCLK   72000000L
#define BAUDRATE  115200L
#define SARCLK   18000000L

// ---- ADC reference voltage ----
#define VDD 4.637f // Measured VDD in volts

// IR addresses
#define IR_ADDR1           0x6   // EFM8 TX address (received by STM32)
#define IR_ADDR2           0x7   // STM32 TX address (received by EFM8)
#define IR_ADDR            IR_ADDR1

// IR TX Command bytes (EFM8 TX -> STM32 RX)
#define IR_CMD_START      0   // val = 0x0000
#define IR_CMD_PAUSE      1   // val = 0x0000
#define IR_CMD_RESET      2   // val = 0x0000
#define IR_CMD_MODE       3   // val: 0x0000=auto, 0x0001=remote, 0x0002=pathfind
#define IR_CMD_PATH       4   // val: 0x0001=path1, 0x0002=path2, 0x0003=path3
#define IR_CMD_JOYSTICK_X 5   // val = joystick_x byte
#define IR_CMD_JOYSTICK_Y    6   // val = joystick_y byte
#define IR_CMD_PATH_WPT_BASE     7   // cmd 7+i: path_active[i], val = (x_cm_byte<<8)|y_cm_byte
#define IR_CMD_ZERO_YAW          39  // val = 0x0012
#define IR_CMD_CROSSING_DECISION 40  // val: (intersection_idx<<8)|direction (0=fwd,1=left,2=right,3=stop)


// IR RX Command bytes (STM32 TX -> EFM8 RX)
#define IR_RX_CMD_DATA_RECEIVED   25   // val: 0x00=field tracking, 0x01=auto, 0x02=path tracking
#define IR_RX_CMD_CROSSING_ACTION 26   // val: 0x00=straight, 0x01=left, 0x02=right, 0x03=stop
#define IR_RX_CMD_LEFT_POWER      27  // val: high byte=positive part, low byte=negative part (abs)
#define IR_RX_CMD_RIGHT_POWER     28  // val: high byte=positive part, low byte=negative part (abs)

// ---- IMU register array indices (0-17 → imu_regs[]) ----
#define IMU_CMD_BASE        7   /* command byte offset for IMU registers */
#define IMU_REG_ACCEL_X     0   /* Raw accelerometer X   → cmd 7        */
#define IMU_REG_ACCEL_Y     1   /* Raw accelerometer Y   → cmd 8        */
#define IMU_REG_ACCEL_Z     2   /* Raw accelerometer Z   → cmd 9        */
#define IMU_REG_GYRO_X      3   /* Raw gyroscope X       → cmd 10       */
#define IMU_REG_GYRO_Y      4   /* Raw gyroscope Y       → cmd 11       */
#define IMU_REG_GYRO_Z      5   /* Raw gyroscope Z       → cmd 12       */
#define IMU_REG_COUNT       18

#define CHARS_PER_LINE 16
#define JOYSTICK_X     QFP32_MUX_P1_5      // P1.5 = ADC0MX channel 13
#define JOYSTICK_Y     QFP32_MUX_P1_4      // P1.4 = ADC0MX channel 12
#define IR_IN          P0_7

#define JoyStick_SW    P2_3
#define PB_PAUSE       P2_6
#define PB_START       P2_5
#define PB_RESET       P2_4
#define PB_TXCMD       P3_0

extern unsigned char crossing_action;  // 0=straight, 1=left, 2=right, 3=stop

#endif
