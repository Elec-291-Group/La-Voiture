/*
 * ir_rx.c — IR receiver using EXTI input-capture on PA7
 *
 * Protocol (TSOP331-style, active-low demodulated output):
 *   T = 263 µs  (10 cycles of the 38 kHz carrier)
 *   Modulation : Pulse-Distance, MSB first
 *
 *   Signal is ACTIVE-LOW: LOW = carrier burst, HIGH = space (idle).
 *
 *   Start  : 4T burst  (~1052 µs LOW pulse), detected on rising edge
 *   Bit '0': 3T Falling-to-Falling interval (~789 µs)
 *   Bit '1': 4T Falling-to-Falling interval (~1052 µs)
 *   Threshold: 3.5T (~920 µs) separates 0 from 1
 *
 *   Frame (16 bits, MSB first):
 *     [15:12] 4-bit Command name
 *     [11: 4] 8-bit Data
 *     [ 3: 0] 4-bit Address  (must be IR_ADDR = 0xB)
 *
 * Hardware resources:
 *   TIM6  — reconfigured as a free-running 1 µs counter (no interrupt)
 *   PA7   — EXTI line 7, both edges (IR_Receiver_Pin / IR_Receiver_GPIO_Port)
 *
 * CubeMX setup required:
 *   PA7  → GPIO_EXTI7, both edges (Rising+Falling), pull-up, label IR_Receiver
 *   NVIC → EXTI4_15_IRQn enabled (CubeMX generates EXTI4_15_IRQHandler automatically)
 */

#include "stm32l0xx_hal.h"   /* GPIO_TypeDef, TIM_TypeDef, GPIOA, etc. */
#include "ir_rx.h"
#include "config.h"
#include "main.h"            /* IR_Receiver_Pin, IR_Receiver_GPIO_Port  */
#include "tim.h"             /* htim6                                   */

/* ── Pin alias ───────────────────────────────────────────────────────────── */
#define IR_PIN   IR_Receiver_Pin
#define IR_PORT  IR_Receiver_GPIO_Port

/* ── State machine ───────────────────────────────────────────────────────── */
typedef enum {
    ST_IDLE,          /* waiting for first falling edge (leader start)       */
    ST_LEADER_CHECK,  /* leader burst in progress; confirm width on rising    */
    ST_DATA_COLLECT,  /* collecting 16 data bits via F2F intervals            */
} ir_state_t;

static volatile ir_state_t state;
static volatile uint16_t   t_last_fall;  /* TIM6 CNT at last falling edge (µs) */
static volatile uint16_t   frame;        /* MSB-first shift register            */
static volatile uint8_t    bit_cnt;      /* 0 = ref edge pending, 1-16 = bits   */
static volatile uint32_t   last_edge_ms; /* HAL_GetTick() at last ISR entry     */

/* ── Ring buffer (ISR writes, main loop reads) ───────────────────────────── */
#define IR_BUF_SIZE  8u   /* must be a power of 2 */

typedef struct { uint8_t cmd; uint8_t data; } ir_frame_t;

static volatile ir_frame_t ir_buf[IR_BUF_SIZE];
static volatile uint8_t    ir_head;   /* written by ISR  */
static volatile uint8_t    ir_tail;   /* read  by main   */

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static inline uint16_t tim6_us(void)
{
    return (uint16_t)htim6.Instance->CNT;
}

static void fsm_reset(void)
{
    state     = ST_IDLE;
    frame     = 0u;
    bit_cnt   = 0u;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void IR_RX_Init(void)
{
    /* Reconfigure TIM6 as a free-running 1 µs counter (no overflow IRQ).
       PSC=15 is already set by CubeMX (16 MHz / 16 = 1 MHz → 1 µs/tick). */
    htim6.Instance->CR1  &= ~TIM_CR1_CEN;
    htim6.Instance->PSC   = 15u;
    htim6.Instance->ARR   = 0xFFFFu;
    htim6.Instance->DIER &= ~TIM_DIER_UIE;   /* no overflow interrupt        */
    htim6.Instance->EGR   = TIM_EGR_UG;      /* latch PSC / ARR immediately  */
    htim6.Instance->SR   &= ~TIM_SR_UIF;     /* clear pending update flag     */
    htim6.Instance->CNT   = 0u;
    htim6.Instance->CR1  |= TIM_CR1_CEN;

    /* PA7 EXTI and NVIC are configured by CubeMX / MX_GPIO_Init(). */
    fsm_reset();
    last_edge_ms = 0u;
}

void IR_RX_Update(void)
{
    /* Called from the main loop. Resets the FSM if no edge arrived recently. */
    if (state != ST_IDLE) {
        if ((HAL_GetTick() - last_edge_ms) > IR_TIMEOUT_MS) {
            fsm_reset();
        }
    }
}

int IR_RX_Available(void)
{
    return ir_head != ir_tail;
}

int IR_RX_GetFrame(uint8_t *cmd_out, uint8_t *data_out)
{
    if (ir_head == ir_tail) return 0;
    *cmd_out  = ir_buf[ir_tail].cmd;
    *data_out = ir_buf[ir_tail].data;
    ir_tail   = (uint8_t)((ir_tail + 1u) & (IR_BUF_SIZE - 1u));
    return 1;
}

/* ── EXTI callback (called by HAL from CubeMX-generated EXTI4_15_IRQHandler) */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IR_PIN) return;

    uint16_t      now = tim6_us();
    GPIO_PinState pin = HAL_GPIO_ReadPin(IR_PORT, IR_PIN);

    last_edge_ms = HAL_GetTick();

    /* Active-low: GPIO_PIN_RESET (LOW) = burst, GPIO_PIN_SET (HIGH) = space.
       Falling edge → burst start.  Rising edge → burst end.               */

    switch (state)
    {
    /* ── IDLE: arm on first falling edge (leader burst start) ──────────── */
    case ST_IDLE:
        if (pin == GPIO_PIN_RESET) {       /* falling edge */
            t_last_fall = now;
            state = ST_LEADER_CHECK;
        }
        break;

    /* ── LEADER_CHECK: measure leader pulse width on rising edge ─────── */
    case ST_LEADER_CHECK:
        if (pin == GPIO_PIN_SET) {         /* rising edge: burst just ended */
            uint16_t pw = (uint16_t)(now - t_last_fall);
            if (pw >= IR_LEADER_MIN_US && pw <= IR_LEADER_MAX_US) {
                /* Valid 4T leader confirmed. Next: collect 16 data bits.
                   bit_cnt=0 means we still need the first data falling
                   edge to set the F2F reference (no bit decoded yet).    */
                frame   = 0u;
                bit_cnt = 0u;
                state   = ST_DATA_COLLECT;
            } else {
                fsm_reset();               /* wrong pulse width → discard  */
            }
        } else {                           /* another falling edge → glitch */
            t_last_fall = now;             /* restart leader measurement    */
        }
        break;

    /* ── DATA_COLLECT: decode bits by F2F interval on each falling edge ─ */
    case ST_DATA_COLLECT:
        if (pin == GPIO_PIN_RESET) {       /* falling edge only            */
            uint16_t f2f = (uint16_t)(now - t_last_fall);

            /* Timeout: gap > 5 ms means this edge belongs to the next frame,
               not the stalled current one. Restart FSM using this edge as
               the new leader falling edge so it is not wasted.            */
            if (f2f > IR_TIMEOUT_US) {
                fsm_reset();
                t_last_fall = now;
                state = ST_LEADER_CHECK;
                break;
            }

            t_last_fall  = now;

            if (bit_cnt == 0u) {
                /* First data falling edge: sets the F2F reference.
                   No bit decoded from this edge alone.                   */
                bit_cnt = 1u;
                break;
            }

            /* Decode: below threshold → '0' (3T F2F), at/above → '1' (4T) */
            uint8_t bit = (f2f >= IR_F2F_THRESH_US) ? 1u : 0u;
            frame = (uint16_t)((frame << 1u) | bit);
            bit_cnt++;

            if (bit_cnt == 17u) {          /* 1 ref edge + 16 decoded bits */
                uint8_t addr     = (uint8_t)( frame        & 0x0Fu);
                uint8_t cmd_name = (uint8_t)((frame >> 12) & 0x0Fu);
                uint8_t data     = (uint8_t)((frame >>  4) & 0xFFu);
                if (addr == IR_ADDR) {
                    /* Push to ring buffer; drop silently if full. */
                    uint8_t next = (uint8_t)((ir_head + 1u) & (IR_BUF_SIZE - 1u));
                    if (next != ir_tail) {
                        ir_buf[ir_head].cmd  = cmd_name;
                        ir_buf[ir_head].data = data;
                        ir_head = next;
                    }
                }
                fsm_reset();
            }
        }
        /* Rising edges during data collection are ignored. */
        break;

    default:
        fsm_reset();
        break;
    }
}
