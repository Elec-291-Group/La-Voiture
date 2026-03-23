#ifndef IR_RX_H
#define IR_RX_H

#include <stdint.h>

/* SIRC repeat period expressed in T = 263 µs units.
   45 ms / 263 µs ≈ 171 ticks. */
#define SIRC_PERIOD_UNITS  171u

/* Initialise (or reset) the IR FSM and frame buffer.
   Call once before starting the 263 µs timer interrupt. */
void IR_RX_Init(void);

/* Advance the FSM by one T = 263 µs tick.
   Called from HAL_TIM_PeriodElapsedCallback inside ir_rx.c. */
void IR_RX_Tick(void);

/* Return the number of complete, validated frames waiting in the ring buffer. */
int  IR_RX_Available(void);

/* Consume the oldest frame from the ring buffer.
   Returns 1 and writes the command byte to *cmd_out.
   Returns 0 (and leaves *cmd_out unchanged) when the buffer is empty. */
int  IR_RX_GetFrame(uint8_t *cmd_out);

#endif /* IR_RX_H */
