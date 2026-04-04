#ifndef CONFIG_H
#define CONFIG_H

/* ── IR Protocol — 28-bit pulse-distance ─────────────────────────────────── */

/* Unit period T = 184 µs (7 cycles of 38 kHz carrier)                       */
#define IR_T_US             184u

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
#define IR_CMD_ZERO_YAW     39u
#define IR_CMD_MANUAL_PATH  40u   /* 16-bit val: 8 intersections × 2 bits    */
#define IR_CMD_NOP          41u   /* No-op: keeps ping-pong alive            */

/* Data values for IR_CMD_MODE */
#define IR_MODE_FIELD       0x00u
#define IR_MODE_REMOTE      0x01u
#define IR_MODE_PATH        0x02u

/* Data values for IR_CMD_PATH */
#define IR_PATH_1           0x01u
#define IR_PATH_2           0x02u
#define IR_PATH_3           0x03u
#define IR_PATH_MANUAL      0x04u

/* ── RX decoder timing (1 µs timer ticks, SYSCLK=16 MHz, PSC=15) ─────────── */

/* Start burst minimum width: 1.5T = 276 µs (leader is 2T burst)            */
#define IR_LEADER_MIN_US    276u

/* Falling-to-falling threshold: 3.5T separates Bit-0 (3T) from Bit-1 (4T)  */
#define IR_F2F_THRESH_US    644u

/* Valid interval bounds: 2T–5T */
#define IR_F2F_MIN_US       368u   /* 2T  – anything shorter is an error      */
#define IR_F2F_MAX_US       920u   /* 5T  – anything longer  is an error      */

/* Timeout: reset FSM if no edge for > 20 ms                                  */
#define IR_TIMEOUT_MS       20u

/* ── IMU register bank ───────────────────────────────────────────────────── */
#define IMU_REG_COUNT       18u    /* 18 two-byte registers                   */
#define IMU_CMD_BASE        7u     /* cmd = reg_index + IMU_CMD_BASE          */

/* LSM6DS33 base register for burst read (gyro + accel + extras)             */
#define IMU_REG_BASE        0x22u  /* OUTX_L_G; 36 consecutive bytes = 18×2  */

#endif /* CONFIG_H */
