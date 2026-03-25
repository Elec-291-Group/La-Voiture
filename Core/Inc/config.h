#ifndef CONFIG_H
#define CONFIG_H

/* ── IR Protocol Configuration ───────────────────────────────────────────── */

/* Expected address nibble in every valid frame */
#define IR_ADDR             0xBu

/* Command name nibble values */
#define IR_CMD_START        0x0u
#define IR_CMD_PAUSE        0x1u
#define IR_CMD_RESET        0x2u
#define IR_CMD_MODE         0x3u
#define IR_CMD_PATH         0x4u
#define IR_CMD_JOYSTICK_X   0x5u
#define IR_CMD_JOYSTICK_Y   0x6u

/* Data values for IR_CMD_MODE */
#define IR_MODE_AUTO        0x00u
#define IR_MODE_REMOTE      0x01u

/* Data values for IR_CMD_PATH */
#define IR_PATH_1           0x00u
#define IR_PATH_2           0x01u
#define IR_PATH_3           0x02u

/* ── Input-Capture decoder timing ────────────────────────────────────────── */
/* Unit period T = 263 µs (10 cycles of 38 kHz carrier) */
#define IR_T_US             263u

/* Leader pulse: 4T burst, ±15% tolerance */
#define IR_LEADER_MIN_US    ((4u * IR_T_US * 85u) / 100u)   /*  894 µs */
#define IR_LEADER_MAX_US    ((4u * IR_T_US * 115u) / 100u)  /* 1209 µs */

/* F2F decision threshold: 3.5T separates '0' (3T) from '1' (4T) */
#define IR_F2F_THRESH_US    (IR_T_US * 7u / 2u)             /*  920 µs */

/* Timeout: reset FSM if no edge arrives within 5 ms */
#define IR_TIMEOUT_MS       5u
#define IR_TIMEOUT_US       5000u    /* same value in µs, for ISR use */

#endif /* CONFIG_H */
