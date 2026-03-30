#ifndef IR_RX_H
#define IR_RX_H

/*
 * ir_rx.h — IR receiver driver for EFM8LB1
 *
 * Hardware:
 *   P0.7  — TSOP33x demodulated output (active-low), via Port Match interrupt
 *   Timer 3 — free-running 1-tick ≈ 0.167 µs counter (SYSCLK/12 = 6 MHz)
 *
 * Protocol: Pulse-Distance, MSB first, active-low
 *   Start : 4T LOW burst  (~1052 µs) + 2T space, detected on rising edge
 *   Bit 0 : 3T falling-to-falling (~789  µs)
 *   Bit 1 : 4T falling-to-falling (~1052 µs)
 *   Stop  : 1T carrier burst
 *   Threshold 3.5T (~921 µs) separates 0 from 1
 *
 * Frame (28 bits, MSB first):
 *   [27:20] 8-bit command label    [19:4] 16-bit IMU register data    [3:0] 4-bit addr
 */

/* Decoded IR frame */
typedef struct {
    unsigned char cmd;    /* 8-bit command label       [27:20] */
    unsigned int  val;    /* 16-bit IMU register data  [19:4]  */
    unsigned char addr;   /* 4-bit address             [ 3:0]  */
} IR_Frame_t;

/* Circular buffer depth — must be a power of 2 */
#define IR_RX_BUF_SIZE  8

/* Live IMU register bank — updated by IR_RX_get() on every popped frame.
 * Index with IMU_REG_* defines from config.h. */
extern volatile unsigned int xdata imu_regs[18];

/* Configure P0.7, Timer 3, and Port Match interrupt.
 * Call once before enabling interrupts (EA=1 handled internally). */
void IR_RX_init(void);

/* Pop one frame from the circular buffer, write it into imu_regs[], and
 * optionally fill *out (may be NULL).  Returns 1 if a frame was available,
 * 0 if the buffer was empty.  Safe to call from the main loop. */
unsigned char IR_RX_get(IR_Frame_t *out);

/* Read the current Timer 3 value (6 MHz free-running, 1 tick ≈ 0.167 µs). */
unsigned int IR_RX_read_t3(void);

#endif /* IR_RX_H */
