#ifndef IR_RX_H
#define IR_RX_H

#include <stdint.h>

/* Initialise the IR receiver.
   Reconfigures TIM6 as a free-running 1 µs counter and enables the
   EXTI interrupt on PA7 (IR_Receiver_Pin) for both edges.
   Call once after MX_TIM6_Init(), before the main loop. */
void IR_RX_Init(void);

/* Timeout watchdog — call from the main loop.
   Resets the FSM to IDLE if no IR edge has been seen for > IR_TIMEOUT_MS. */
void IR_RX_Update(void);

/* Returns the number of decoded frames waiting in the ring buffer. */
int IR_RX_Available(void);

/* Pops the oldest frame from the ring buffer.
   Returns 1 and writes cmd/data on success, 0 if the buffer is empty. */
int IR_RX_GetFrame(uint8_t *cmd_out, uint8_t *data_out);

/* Application-defined command handler — implement in main.c.
   Called from the main loop (NOT from ISR) for each valid frame.
   cmd_name : IR_CMD_* nibble (see config.h)
   data     : 8-bit payload */
void HandleCommand(uint8_t cmd_name, uint8_t data);

#endif /* IR_RX_H */
