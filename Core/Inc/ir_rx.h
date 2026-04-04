#ifndef IR_RX_H
#define IR_RX_H

#include <stdint.h>

/*
 * 28-bit pulse-distance frame:  addr[27:24] | val[23:8] | cmd[7:0]
 * Transmitted MSB first.
 */
typedef struct {
    uint8_t  cmd;    /* 0=start 1=pause 2=reset 3=mode 4=path 5=joy_x 6=joy_y */
    uint16_t val;    /* 16-bit payload                                          */
    uint8_t  addr;   /* 4-bit address nibble (must equal IR_ADDR_RX = 0x6)     */
} IR_Frame_t;

/* Set by the EXTI ISR on a valid accepted frame; cleared by the main loop.   */
extern volatile uint8_t  ir_rx_ready;
extern IR_Frame_t        ir_rx_frame;


/* Initialise: reconfigures TIM6 as free-running 1 µs counter, resets FSM.
   Call after MX_TIM6_Init(), before the main loop.                          */
void IR_RX_Init(void);

/* Watchdog — call from the main loop; resets FSM on 20 ms silence.          */
void IR_RX_Update(void);

/* Returns 1 while the RX FSM is actively decoding a frame, 0 when idle.
   Used by TX to avoid transmitting during reception (half-duplex).          */
uint8_t IR_RX_Active(void);

#endif /* IR_RX_H */
