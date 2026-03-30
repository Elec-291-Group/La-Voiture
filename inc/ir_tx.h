#ifndef IR_TX_H
#define IR_TX_H

// Portable integer types (C51 does not provide <stdint.h>)
typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;

// Timer 0: 38 kHz carrier
// Half-period = SYSCLK / 38000 / 2 = 72000000 / 38000 / 2 ≈ 947 counts
// RELOAD = 65536 - 947 = 64589  (original 64588 also fine, kept as-is)
#define TIMER0_RELOAD   64588U

// Timer 2: one SIRC "T" unit = 10 carrier cycles at 38 kHz = 263.16 µs
// Counts = 72e6 * 10/38000 = 18947 → RELOAD = 65536 - 18947 = 46589
#define TIMER2_RELOAD   46589U

// FSM states
#define FSM_IDLE        0x00   // waiting for next send_ir_packet()
#define FSM_SENDING     0x01   // stepping through unit array

// Frame: start(6) + 8-bit cmd(max 32) + 16-bit data(max 64) + 4-bit addr(max 16) + stop(1) = 119 worst case
// Bit 0 = [1,0,0] (3 units), Bit 1 = [1,0,0,0] (4 units), MSB first
#define SIRC_MAX_UNITS      119U
// 100 ms repeat period: 100 ms / 263.16 µs ≈ 380 units
#define SIRC_PERIOD_UNITS   380U

extern volatile unsigned char fsm_state;

void TIMER0_Init(void);
void TIMER2_Init(void);

// Non-blocking 28-bit packet: 8-bit cmd | 16-bit data | 4-bit addr.
// Frame: 4T+2T start | 8-bit cmd | 16-bit val | 4-bit addr | 1T stop
// Returns immediately; Timer 2 ISR drives the actual transmission.
// Drops silently if FSM is busy (fsm_state != FSM_IDLE).
void send_ir_packet(uint8_t cmd, uint16_t val, uint8_t addr);

#endif
