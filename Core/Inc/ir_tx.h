#ifndef IR_TX_H
#define IR_TX_H

#include <stdint.h>
#include "config.h"

/*
 * IR TX: sends 28-bit pulse-distance frames to EFM8 (addr nibble = 0x7).
 *
 * Hardware:
 *   TIM22 CH1 (PA6, AF5) — 38 kHz carrier PWM (PSC=0, ARR=420)
 *   TIM21               — envelope tick every T=263 µs (PSC=15, ARR=262)
 *
 * Non-blocking: IR_Send_IMU() loads the frame and returns immediately.
 * The TIM21 ISR (IR_TX_TIM21_IRQHandler) drives the output.
 */

/* IMU register bank — populate from sensor before calling IR_Send_IMU().   */
extern uint16_t imu_regs[IMU_REG_COUNT];

/* Initialise TIM22 carrier PWM and TIM21 envelope timer; starts both.
   Call after MX_TIM21_Init() and MX_TIM22_Init(), before the main loop.   */
void IR_TX_Init(void);

/* Queue one 28-bit frame for transmission.
   reg_index : 0–17  → cmd byte = reg_index + IMU_CMD_BASE (7–24)
   val       : 16-bit payload
   Address nibble is always IR_ADDR_TX (0x7).
   Safe to call only when IR_TX_Busy() == 0.                               */
void IR_Send_IMU(uint8_t reg_index, uint16_t val);

/* Send an arbitrary cmd+val frame (addr always IR_ADDR_TX = 0x7).
   Safe to call only when IR_TX_Busy() == 0.                               */
void IR_Send_Cmd(uint8_t cmd, uint16_t val);

#define IR_CMD_DATA_RECEIVED      0x00u  /* cmd=0, val=mode (0x00/0x01/0x02)*/
#define IR_CMD_CROSSING_ACTION    0x01u  /* cmd=1, val=direction action     */

/* Returns 1 while a transmission is in progress, 0 when idle.             */
uint8_t IR_TX_Busy(void);

/* HAL_TIM_PeriodElapsedCallback is implemented in ir_tx.c.
   No manual ISR wiring needed — TIM21_IRQHandler calls HAL_TIM_IRQHandler. */

#endif /* IR_TX_H */
