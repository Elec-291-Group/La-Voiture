#include <EFM8LB1.h>
#include "timer.h"
#include "config.h"

// ---- Software busy-wait delay ---------------------------------------
// Timer 3 is reserved by ir_rx as a free-running timestamp counter.
// All four hardware timers are allocated: T0=carrier, T1=UART0,
// T2=IR envelope, T3=IR RX timestamp.  Use a software loop instead.
// At SYSCLK=72 MHz, the volatile inner loop ≈ 4 cycles ≈ 56 ns per
// iteration, so 18 iterations ≈ 1 µs (conservative — slightly long).

void Timer3us(unsigned char us)
{
	volatile unsigned char j;
	unsigned char i;
	for (i = 0; i < us; i++)
		for (j = 0; j < 18; j++);
}

void waitms(unsigned int ms)
{
	unsigned int  j;
	unsigned char k;
	for (j = 0; j < ms; j++)
		for (k = 0; k < 4; k++) Timer3us(250);
}
