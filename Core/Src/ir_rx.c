/*
 * ir_rx.c — 28-bit pulse-distance IR decoder
 *
 * Hardware:
 *   PA7   — EXTI line 7, both edges (IR_Receiver_Pin), pull-up
 *            TSOP active-low: LOW = carrier burst, HIGH = space
 *   TIM6  — free-running 1 µs counter (PSC=15, ARR=0xFFFF, no overflow IRQ)
 *
 * Frame (28 bits MSB first):
 *   [27:24] addr (4-bit)  [23:8] val (16-bit)  [7:0] cmd (8-bit)
 *
 * Timing at 1 µs resolution:
 *   Start burst  ≥ 3.5T (920 µs) — detected on rising edge
 *   Bit period threshold: 3.5T (920 µs) → 0 below, 1 at-or-above
 *   Error: period < 2T (526 µs) or > 5T (1315 µs) → reset
 *   Timeout: no edge for > 20 ms → reset
 *
 * CubeMX requirements (already in project):
 *   PA7  → GPIO_EXTI7, both edges, pull-up, label IR_Receiver
 *   NVIC → EXTI4_15_IRQn enabled
 *   TIM6 → base timer, PSC=15, no interrupt (IR_RX_Init overrides ARR)
 */

#include "stm32l0xx_hal.h"
#include "ir_rx.h"
#include "config.h"
#include "main.h"   /* IR_Receiver_Pin, IR_Receiver_GPIO_Port */
#include "tim.h"    /* htim6 */

/* ── Pin alias ───────────────────────────────────────────────────────────── */
#define IR_PIN   IR_Receiver_Pin
#define IR_PORT  IR_Receiver_GPIO_Port

/* ── State machine ───────────────────────────────────────────────────────── */
typedef enum {
    ST_IDLE,          /* waiting for first falling edge (start burst begin)   */
    ST_LEADER_CHECK,  /* start burst in progress; confirm on rising edge      */
    ST_REF_EDGE,      /* start confirmed; waiting for first data falling edge */
    ST_DATA_COLLECT,  /* accumulating 28 bits by falling-to-falling interval  */
} ir_state_t;

static volatile ir_state_t state       = ST_IDLE;
static volatile uint16_t   t_last_fall = 0;   /* TIM6 CNT at last falling edge */
static volatile uint32_t   rx_bits     = 0;   /* MSB-first shift register       */
static volatile uint8_t    bit_cnt     = 0;   /* decoded bits so far (0–28)     */
static volatile uint32_t   last_edge_ms = 0;  /* HAL_GetTick() at last ISR call */

/* ── Public state ────────────────────────────────────────────────────────── */
volatile uint8_t ir_rx_ready = 0;
IR_Frame_t       ir_rx_frame;

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static inline uint16_t tim6_us(void)
{
    return (uint16_t)htim6.Instance->CNT;
}

static void fsm_reset(void)
{
    state   = ST_IDLE;
    rx_bits = 0u;
    bit_cnt = 0u;
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void IR_RX_Init(void)
{
    /* Reconfigure TIM6 as free-running 1 µs counter.
       PSC=15: 16 MHz / 16 = 1 MHz → 1 µs per tick.
       ARR=0xFFFF: 65.5 ms full range before wrap (>3× frame period).       */
    htim6.Instance->CR1  &= ~TIM_CR1_CEN;
    htim6.Instance->PSC   = 15u;
    htim6.Instance->ARR   = 0xFFFFu;
    htim6.Instance->DIER &= ~TIM_DIER_UIE;  /* no overflow interrupt          */
    htim6.Instance->EGR   = TIM_EGR_UG;     /* latch PSC/ARR immediately      */
    htim6.Instance->SR   &= ~TIM_SR_UIF;
    htim6.Instance->CNT   = 0u;
    htim6.Instance->CR1  |= TIM_CR1_CEN;

    fsm_reset();
    last_edge_ms = 0u;
}

void IR_RX_Update(void)
{
    /* Called from main loop. Resets FSM on 20 ms silence.                   */
    if (state != ST_IDLE) {
        if ((HAL_GetTick() - last_edge_ms) > IR_TIMEOUT_MS) {
            fsm_reset();
        }
    }
}

/* ── EXTI callback ───────────────────────────────────────────────────────── */
/* Called by HAL_GPIO_EXTI_IRQHandler from EXTI4_15_IRQHandler.              */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IR_PIN) return;

    uint16_t      now = tim6_us();
    GPIO_PinState pin = HAL_GPIO_ReadPin(IR_PORT, IR_PIN);

    last_edge_ms = HAL_GetTick();

    /*
     * Active-low TSOP output:
     *   Falling edge (GPIO_PIN_RESET) = burst start
     *   Rising  edge (GPIO_PIN_SET)   = burst end
     */
    switch (state)
    {
    /* ────────────────────────────────────────────────────────────────────── */
    case ST_IDLE:
        if (pin == GPIO_PIN_RESET) {       /* falling → start burst begins    */
            t_last_fall = now;
            state = ST_LEADER_CHECK;
        }
        break;

    /* ────────────────────────────────────────────────────────────────────── */
    case ST_LEADER_CHECK:
        if (pin == GPIO_PIN_SET) {         /* rising → measure burst width    */
            uint16_t pw = (uint16_t)(now - t_last_fall);
            if (pw >= IR_LEADER_MIN_US) {
                /* Valid 4T start burst — wait for first data falling edge.   */
                state = ST_REF_EDGE;
            } else {
                fsm_reset();
            }
        } else {
            /* Another falling edge during leader — glitch, restart.         */
            t_last_fall = now;
        }
        break;

    /* ────────────────────────────────────────────────────────────────────── */
    case ST_REF_EDGE:
        if (pin == GPIO_PIN_RESET) {       /* falling → reference edge (bit 0 start) */
            t_last_fall = now;
            rx_bits     = 0u;
            bit_cnt     = 0u;
            state       = ST_DATA_COLLECT;
        }
        break;

    /* ────────────────────────────────────────────────────────────────────── */
    case ST_DATA_COLLECT:
        if (pin == GPIO_PIN_RESET) {       /* falling edge only                */
            uint16_t f2f = (uint16_t)(now - t_last_fall);

            if (f2f < IR_F2F_MIN_US || f2f > IR_F2F_MAX_US) {
                /* Out-of-range interval: could be a fresh start burst.
                   If it looks like a leader, restart from here.             */
                fsm_reset();
                if (f2f > IR_F2F_MAX_US) {
                    t_last_fall = now;
                    state = ST_LEADER_CHECK;
                }
                break;
            }

            /* Decode: below threshold → bit 0 (3T period),
                       at/above      → bit 1 (4T period)                     */
            uint8_t bit = (f2f >= IR_F2F_THRESH_US) ? 1u : 0u;
            rx_bits = (rx_bits << 1u) | bit;   /* MSB first, matches EFM8 TX */
            bit_cnt++;
            t_last_fall = now;

            if (bit_cnt == 28u) {
                /* Full frame received — decode fields (EFM8 layout).
                 * [27:20] cmd  [19:4] val  [3:0] addr                        */
                uint8_t addr = (uint8_t)(rx_bits & 0xFu);
                if (addr == IR_ADDR_RX) {
                    ir_rx_frame.cmd  = (uint8_t) ((rx_bits >> 20) & 0xFFu);
                    ir_rx_frame.val  = (uint16_t)((rx_bits >>  4) & 0xFFFFu);
                    ir_rx_frame.addr = addr;
                    ir_rx_ready = 1u;
                }
                fsm_reset();
            }
        }
        /* Rising edges during data collection are not needed.               */
        break;

    default:
        fsm_reset();
        break;
    }
}
