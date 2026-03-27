/*
 * ir_tx.c — 28-bit pulse-distance IR transmitter
 *
 * Frame format (28 bits, LSB first — cmd sent first):
 *   [7:0] cmd  [23:8] val (16-bit)  [27:24] addr=0x7  (transmitted bit 0 → bit 27)
 *
 * Unit buffer layout:
 *   Each entry represents one T-period (263 µs).
 *   Value 1 = carrier burst, 0 = space (no carrier).
 *
 * Max buffer depth:
 *   Start: 4+2=6   Data: 28×(1+3)=112   Stop: 1   → 119 entries
 *
 * Hardware:
 *   TIM22 CH1 (PA6, AF5) — 38 kHz carrier, PSC=0, ARR=420
 *     CCR1 = 210 → 50% duty (burst)
 *     CCR1 = 0   → 0% duty  (space)
 *   TIM21 — envelope tick, PSC=15, ARR=262, UPDATE interrupt every 263 µs
 */

#include "stm32l0xx_hal.h"
#include "ir_tx.h"
#include "config.h"

/* ── External timer handles (declared/initialised in tim.c) ─────────────── */
extern TIM_HandleTypeDef htim21;
extern TIM_HandleTypeDef htim22;

/* ── Carrier duty-cycle values ───────────────────────────────────────────── */
/* ARR=420 → period=421 counts; 50% = 210 counts.                           */
#define CARRIER_ON_CCR   210u
#define CARRIER_OFF_CCR    0u

/* ── TX buffer ───────────────────────────────────────────────────────────── */
#define TX_BUF_SIZE  120u

typedef enum { TX_FSM_IDLE = 0, TX_FSM_SENDING, TX_FSM_COOLDOWN } tx_state_t;

#define TX_COOLDOWN_MS  100u

static volatile tx_state_t tx_state     = TX_FSM_IDLE;
static volatile uint8_t    tx_buf[TX_BUF_SIZE];
static volatile uint8_t    tx_len       = 0u;
static volatile uint8_t    tx_pos       = 0u;
static volatile uint32_t   tx_done_tick = 0u;

/* ── IMU register bank ───────────────────────────────────────────────────── */
uint16_t imu_regs[IMU_REG_COUNT];

/* ── Internal helpers ────────────────────────────────────────────────────── */
static inline void carrier_on(void)
{
    __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_1, CARRIER_ON_CCR);
}

static inline void carrier_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_1, CARRIER_OFF_CCR);
}

/* Pack 28-bit frame word.  EFM8 layout: cmd[27:20] | val[19:4] | addr[3:0] */
static uint32_t pack_frame(uint8_t cmd, uint16_t val, uint8_t addr)
{
    return ((uint32_t)(cmd)          << 20)
         | ((uint32_t)(val)          <<  4)
         | ((uint32_t)(addr & 0xFu));
}

/* Fill tx_buf[] with T-unit phases and return the count.
 *
 * Encoding (each entry = one T period):
 *   Start : 4× burst, 2× space
 *   Bit 0 : 1× burst, 2× space   (3T total)
 *   Bit 1 : 1× burst, 3× space   (4T total)
 *   Stop  : 1× burst
 */
static uint8_t build_frame(uint32_t frame28, uint8_t *buf)
{
    uint8_t pos = 0u;

    /* Start symbol: 4T burst + 2T space */
    buf[pos++] = 1u; buf[pos++] = 1u; buf[pos++] = 1u; buf[pos++] = 1u;
    buf[pos++] = 0u; buf[pos++] = 0u;

    /* 28 data bits, MSB first (cmd[7:0] then val[15:0] then addr[3:0] on wire) */
    for (int8_t b = 27; b >= 0; b--) {
        buf[pos++] = 1u;  /* 1T burst */
        uint8_t spaces = ((frame28 >> b) & 1u) ? 3u : 2u;
        for (uint8_t s = 0u; s < spaces; s++) {
            buf[pos++] = 0u;
        }
    }

    /* Stop symbol: 1T burst */
    buf[pos++] = 1u;

    return pos;
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void IR_TX_Init(void)
{
    carrier_off();
    HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);  /* carrier timer running, CCR=0 */
    HAL_TIM_Base_Start_IT(&htim21);              /* envelope ticker              */
}

void IR_Send_IMU(uint8_t reg_index, uint16_t val)
{
    if (tx_state != TX_FSM_IDLE) return;  /* caller must check IR_TX_Busy()    */

    uint8_t  cmd   = reg_index + IMU_CMD_BASE;
    uint32_t frame = pack_frame(cmd, val, IR_ADDR_TX);

    tx_len   = build_frame(frame, (uint8_t *)tx_buf);
    tx_pos   = 0u;
    tx_state = TX_FSM_SENDING;

    /* Drive the first T-unit now so the ISR only needs to handle pos >= 1.  */
    if (tx_buf[0u]) carrier_on(); else carrier_off();
    tx_pos = 1u;
}

uint8_t IR_TX_Busy(void)
{
    if (tx_state == TX_FSM_COOLDOWN) {
        if ((HAL_GetTick() - tx_done_tick) >= TX_COOLDOWN_MS) {
            tx_state = TX_FSM_IDLE;
        }
    }
    return (tx_state != TX_FSM_IDLE) ? 1u : 0u;
}

/* ── HAL timer period-elapsed callback ───────────────────────────────────── */
/* HAL_TIM_IRQHandler (called from TIM21_IRQHandler in stm32l0xx_it.c) clears
   the UPDATE flag and then calls this weak callback.  TIM6 also routes here
   (every 65.5 ms) so we gate on the instance.                               */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM21) return;

    if (tx_state != TX_FSM_SENDING) return;

    if (tx_pos >= tx_len) {
        carrier_off();
        tx_done_tick = HAL_GetTick();
        tx_state = TX_FSM_COOLDOWN;
        return;
    }

    if (tx_buf[tx_pos]) carrier_on(); else carrier_off();
    tx_pos++;
}
