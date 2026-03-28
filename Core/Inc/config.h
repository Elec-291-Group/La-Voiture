#ifndef CONFIG_H
#define CONFIG_H

/* ── IR Protocol — 28-bit pulse-distance ─────────────────────────────────── */

/* Unit period T = 263 µs (10 cycles of 38 kHz carrier)                      */
#define IR_T_US             263u

/* Frame addresses (4-bit nibble) */
#define IR_ADDR_RX          0x6u   /* EFM8 → STM32  (frames we accept)       */
#define IR_ADDR_TX          0x7u   /* STM32 → EFM8  (frames we send)         */

/* Command byte values (cmd field [7:0])                                      */
#define IR_CMD_START        0u
#define IR_CMD_PAUSE        1u
#define IR_CMD_RESET        2u
#define IR_CMD_MODE         3u
#define IR_CMD_PATH         4u
#define IR_CMD_JOYSTICK_X   5u
#define IR_CMD_JOYSTICK_Y   6u
/* Commands 7–12: IMU telemetry TX (STM32 → EFM8)                            */
#define IR_CMD_ACCEL_X      7u
#define IR_CMD_ACCEL_Y      8u
#define IR_CMD_ACCEL_Z      9u
#define IR_CMD_GYRO_X       10u
#define IR_CMD_GYRO_Y       11u
#define IR_CMD_GYRO_Z       12u
/* Commands 13–24 reserved for future IMU register TX (see ir_tx.h)          */

/* Data values for IR_CMD_MODE */
#define IR_MODE_FIELD       0x00u
#define IR_MODE_REMOTE      0x01u
#define IR_MODE_PATH        0x02u

/* Data values for IR_CMD_PATH */
#define IR_PATH_1           0x00u
#define IR_PATH_2           0x01u
#define IR_PATH_3           0x02u

/* ── RX decoder timing (1 µs timer ticks, SYSCLK=16 MHz, PSC=15) ─────────── */

/* Start burst minimum width: 3.5T = 920 µs                                  */
#define IR_LEADER_MIN_US    920u

/* Falling-to-falling threshold: 3.5T separates Bit-0 (3T) from Bit-1 (4T)  */
#define IR_F2F_THRESH_US    920u

/* Valid interval bounds: 2T–5T */
#define IR_F2F_MIN_US       526u   /* 2T  – anything shorter is an error      */
#define IR_F2F_MAX_US      1315u   /* 5T  – anything longer  is an error      */

/* Timeout: reset FSM if no edge for > 20 ms                                  */
#define IR_TIMEOUT_MS       20u

/* ── IMU register bank ───────────────────────────────────────────────────── */
#define IMU_REG_COUNT       18u    /* 18 two-byte registers                   */
#define IMU_CMD_BASE        7u     /* cmd = reg_index + IMU_CMD_BASE          */

/* LSM6DS33 base register for burst read (gyro + accel + extras)             */
#define IMU_REG_BASE        0x22u  /* OUTX_L_G; 36 consecutive bytes = 18×2  */

#endif /* CONFIG_H */
