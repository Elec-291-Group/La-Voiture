#include <EFM8LB1.h>
#include "ir_tx.h"
#include "config.h"

// FSM state
volatile unsigned char fsm_state = FSM_IDLE;

// Trigger flag set by IR_Send(), cleared in ISR
static volatile bit fsm_trigger = 0;

// SIRC unit buffer in XDATA (119 bytes worst-case for 28-bit IMU frame)
static unsigned char xdata sirc_units[SIRC_MAX_UNITS];
static unsigned char sirc_unit_count = 0;   // filled by PrepareFrame()
static unsigned char sirc_unit_idx   = 0;   // current position in ISR
static unsigned int  sirc_tick_count = 0;   // ticks elapsed this frame

// ---- Carrier helpers ------------------------------------------------

static void enable_carrier(void)
{
	TH0 = (TIMER0_RELOAD >> 8) & 0xFF;
	TL0 = TIMER0_RELOAD & 0xFF;
	TF0 = 0;
	ET0 = 1;
	TR0 = 1;
}

static void disable_carrier(void)
{
	TR0  = 0;
	ET0  = 0;
	P2_1 = 0;
}

// ---- Append one encoded bit (pulse-distance, MSB-first) -------------
// Bit "0" : [1,0,0]    1T burst + 2T space  (3T period)
// Bit "1" : [1,0,0,0]  1T burst + 3T space  (4T period)
static void append_bit(unsigned char val)
{
	sirc_units[sirc_unit_count++] = 1;   // 1T burst (always)
	sirc_units[sirc_unit_count++] = 0;   // 2T space
	sirc_units[sirc_unit_count++] = 0;
	if (val)
		sirc_units[sirc_unit_count++] = 0; // extra 1T space for bit "1"
}

void send_ir_packet(uint8_t cmd, uint16_t val, uint8_t addr)
{
	uint8_t i;

	if (fsm_state != FSM_IDLE) return;   /* drop if previous frame still sending */

	sirc_unit_count = 0;

	/* Start sign: 4T burst + 2T space --------------------------------- */
	sirc_units[sirc_unit_count++] = 1;
	sirc_units[sirc_unit_count++] = 1;
	sirc_units[sirc_unit_count++] = 1;
	sirc_units[sirc_unit_count++] = 1;
	sirc_units[sirc_unit_count++] = 0;
	sirc_units[sirc_unit_count++] = 0;

	/* 8-bit command label, MSB first ----------------------------------- */
	i = 8;
	do { i--; append_bit((cmd >> i) & 1); } while (i > 0);

	/* 16-bit register value, MSB first --------------------------------- */
	i = 16;
	do { i--; append_bit((val >> i) & 1); } while (i > 0);

	/* 4-bit address, MSB first ----------------------------------------- */
	i = 4;
	do { i--; append_bit((addr >> i) & 1); } while (i > 0);

	/* Stop sign: 1T burst ---------------------------------------------- */
	sirc_units[sirc_unit_count++] = 1;

	fsm_trigger = 1;   /* ISR picks this up on the next Timer 2 tick */
}

// ---- Timer 0 — 38 kHz carrier ---------------------------------------

void TIMER0_Init(void)
{
	TR0 = 0;
	TMOD &= 0b_1111_0000;       // clear Timer 0 mode bits
	TMOD |= 0b_0000_0001;       // Mode 1: 16-bit timer
	CKCON0 &= 0b_1111_1000;     // clear Timer 0 clock select
	CKCON0 |= 0b_0000_0100;     // Timer 0 clocked by SYSCLK

	TH0 = (TIMER0_RELOAD >> 8) & 0xFF;
	TL0 = TIMER0_RELOAD & 0xFF;
	TF0 = 0;

	ET0 = 0;   // disabled until FSM enables it
	EA  = 1;
	TR0 = 0;
}

void Timer0_ISR(void) __interrupt (1)
{
	TR0 = 0;
	TH0 = (TIMER0_RELOAD >> 8) & 0xFF;
	TL0 = TIMER0_RELOAD & 0xFF;
	P2_1 = !P2_1;   // toggle IR output → 38 kHz
	TR0 = 1;
}

// ---- Timer 2 — 263 µs envelope tick (10 × 38 kHz cycles) -----------

void TIMER2_Init(void)
{
	TR2 = 0;
	CKCON0 |= 0b_0011_0000;     // Timer 2 clocked by SYSCLK
	T2SPLIT = 0;                 // 16-bit auto-reload mode
	TMR2RLH = (TIMER2_RELOAD >> 8) & 0xFF;
	TMR2RLL = TIMER2_RELOAD & 0xFF;
	TMR2H   = (TIMER2_RELOAD >> 8) & 0xFF;
	TMR2L   = TIMER2_RELOAD & 0xFF;

	ET2 = 1;
	EA  = 1;
	TR2 = 1;
}

// Called every 263 µs (one SIRC T unit).
// State machine:
//   FSM_IDLE    — wait for fsm_trigger, then start frame
//   FSM_SENDING — step through sirc_units[], pad silence to 171 units (45 ms)
void Timer2_ISR(void) __interrupt (5)
{
	TR2  = 0;
	ET2  = 0;
	TF2H = 0;

	P2_2 = !P2_2;   // debug toggle

	// Check for new frame trigger while idle
	if (fsm_state == FSM_IDLE && fsm_trigger)
	{
		fsm_trigger      = 0;
		sirc_unit_idx    = 0;
		sirc_tick_count  = 0;
		fsm_state        = FSM_SENDING;
	}

	if (fsm_state == FSM_SENDING)
	{
		sirc_tick_count++;

		if (sirc_unit_idx < sirc_unit_count)
		{
			// Drive carrier according to current unit
			if (sirc_units[sirc_unit_idx++])
				enable_carrier();
			else
				disable_carrier();
		}
		else
		{
			// Frame payload exhausted — hold carrier off (inter-frame silence)
			disable_carrier();
		}

		// Return to idle after the full 45 ms period (171 T units)
		if (sirc_tick_count >= SIRC_PERIOD_UNITS)
		{
			disable_carrier();
			fsm_state = FSM_IDLE;
		}
	}

	ET2 = 1;
	TR2 = 1;
}
