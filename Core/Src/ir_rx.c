/*
 * ir_rx.c — interrupt-driven IR receiver FSM
 *
 * Protocol (TSOP38238, active-low demodulated output):
 *   T = 263 µs  (one sampling tick = 10 cycles of the 38 kHz carrier)
 *   Start  : 4T LOW  + 1T HIGH
 *   Bit  0 : 1T LOW  + 1T HIGH
 *   Bit  1 : 2T LOW  + 1T HIGH  (burst length distinguishes the bit value)
 *   Frame  : 8 command bits (LSB first) + 4 address bits (LSB first)
 *   Address: always 0x0B — frames with any other address are silently dropped
 *   Update : two consecutive frames  →  frame 1 = x_byte, frame 2 = y_byte
 *
 * IR_RX_Tick() must be called every 263 µs from HAL_TIM_PeriodElapsedCallback.
 */

#include "ir_rx.h"
#include "main.h"   /* IR_Receiver_GPIO_Port, IR_Receiver_Pin            */
#include "tim.h"    /* htim2                                              */

/* Expected address field in every valid frame */
#define IR_ADDR_EXPECTED  0x0Bu

/* ── FSM ─────────────────────────────────────────────────────────────────── */

typedef enum { ST_IDLE, ST_BURST, ST_SPACE } ir_state_t;

static ir_state_t  state;
static uint8_t     burst_cnt;   /* consecutive LOW ticks in current burst  */
static uint8_t     space_cnt;   /* consecutive HIGH ticks in current space */
static uint16_t    shift_reg;   /* accumulates decoded bits, LSB first     */
static uint8_t     bit_cnt;     /* bits received so far (0-12)             */
static uint8_t     started;     /* 1 once a valid 4T start burst is seen   */

/* Silence threshold: >3T of HIGH without a burst = inter-frame gap.
   Worst intra-frame space is 1T, so 4T guarantees we are between frames. */
#define SILENCE_TICKS  4u

static void reset_fsm(void)
{
    state     = ST_IDLE;
    burst_cnt = 0u;
    space_cnt = 0u;
    shift_reg = 0u;
    bit_cnt   = 0u;
    started   = 0u;
}

/* ── Frame ring-buffer ───────────────────────────────────────────────────── */
/* Written from ISR context (IR_RX_Tick), read from main-loop context.
   Single-producer single-consumer: head moves in ISR, tail moves in main.
   Both indices are volatile; no other lock is needed on Cortex-M0+.        */

#define FRAME_BUF_LEN  8u

static volatile uint8_t frame_buf[FRAME_BUF_LEN];
static volatile uint8_t frame_head;   /* next write position (ISR)  */
static volatile uint8_t frame_tail;   /* next read  position (main) */

static void push_frame(uint8_t cmd)
{
    uint8_t next = (uint8_t)((frame_head + 1u) % FRAME_BUF_LEN);
    if (next != frame_tail) {          /* drop silently when full */
        frame_buf[frame_head] = cmd;
        frame_head = next;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void IR_RX_Init(void)
{
    reset_fsm();
    frame_head = 0u;
    frame_tail = 0u;
}

int IR_RX_Available(void)
{
    return (int)((frame_head - frame_tail + FRAME_BUF_LEN) % FRAME_BUF_LEN);
}

int IR_RX_GetFrame(uint8_t *cmd_out)
{
    if (frame_tail == frame_head) return 0;
    *cmd_out   = frame_buf[frame_tail];
    frame_tail = (uint8_t)((frame_tail + 1u) % FRAME_BUF_LEN);
    return 1;
}

/* ── FSM tick — called every T = 263 µs ─────────────────────────────────── */

void IR_RX_Tick(void)
{
    /* Active-low output: GPIO LOW means 38 kHz carrier is present (burst). */
    int is_burst = (HAL_GPIO_ReadPin(IR_Receiver_GPIO_Port, IR_Receiver_Pin)
                    == GPIO_PIN_RESET);

    switch (state)
    {
    /* ── IDLE: waiting for the first burst after silence ───────────────── */
    case ST_IDLE:
        if (is_burst) {
            burst_cnt = 1u;
            state     = ST_BURST;
        }
        break;

    /* ── BURST: counting consecutive LOW ticks ──────────────────────────── */
    case ST_BURST:
        if (is_burst) {
            burst_cnt++;
        } else {
            /* Burst just ended — evaluate its length. */
            if (!started) {
                /* We are waiting for the 4T start burst. */
                if (burst_cnt == 4u) {
                    started   = 1u;
                    bit_cnt   = 0u;
                    shift_reg = 0u;
                } else {
                    /* Invalid start burst — discard and wait for silence. */
                    reset_fsm();
                    break;
                }
            } else {
                /* Data burst: 1T = bit 0, ≥2T = bit 1 (LSB first). */
                uint8_t bit = (burst_cnt >= 2u) ? 1u : 0u;
                shift_reg  |= (uint16_t)((uint16_t)bit << bit_cnt);
                bit_cnt++;

                if (bit_cnt == 12u) {
                    /* Frame complete: bits [0..7] = command, [8..11] = address */
                    uint8_t cmd  = (uint8_t)(shift_reg & 0xFFu);
                    uint8_t addr = (uint8_t)((shift_reg >> 8) & 0x0Fu);
                    if (addr == IR_ADDR_EXPECTED) {
                        push_frame(cmd);
                    }
                    reset_fsm();
                    return;   /* skip the state = ST_SPACE below */
                }
            }
            space_cnt = 1u;
            state     = ST_SPACE;
        }
        break;

    /* ── SPACE: counting consecutive HIGH ticks ─────────────────────────── */
    case ST_SPACE:
        if (!is_burst) {
            space_cnt++;
            if (space_cnt >= SILENCE_TICKS) {
                /* Inter-frame silence — any incomplete frame is discarded. */
                reset_fsm();
            }
        } else {
            /* Next burst begins. */
            burst_cnt = 1u;
            state     = ST_BURST;
        }
        break;
    }
}

/* ── HAL timer callback (overrides the weak HAL default) ────────────────── */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        IR_RX_Tick();
    }
}
