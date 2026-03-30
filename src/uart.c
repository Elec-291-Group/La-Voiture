/*
 * uart.c – Interrupt-driven ring-buffer UART driver for EFM8LB12F64G
 *
 * UART0 : P0.4 TX / P0.5 RX  – SFR page 0x00, baud via Timer 1
 * UART1 : P0.2 TX / P0.3 RX  – SFR page 0x20, baud via dedicated BRG
 *
 * Both channels: BAUDRATE baud (see config.h).
 *
 * Design notes:
 *  - Every function/ISR that changes SFRPAGE saves and restores it so that
 *    callers always get back the page they were on (critical on EFM8).
 *  - Ring-buffer sizes are powers of 2; head/tail wrap uses bitmasking
 *    (cheaper than modulo on an 8051 with no hardware divider).
 *  - SCON1 is not bit-addressable, so RI1/TI1 are accessed via byte masks.
 */

#include <EFM8LB1.h>
#include "config.h"
#include "uart.h"

/* Advance a ring-buffer index with power-of-2 wrap */
#define BUF_NEXT(idx, mask)   (((idx) + 1u) & (mask))

// UART0  (SFR page 0x00)
#define UART0_PAGE  0x00u
#define TX0_SIZE    64u                     /* must be a power of 2 */
#define RX0_SIZE    64u
#define TX0_MASK    (TX0_SIZE - 1u)
#define RX0_MASK    (RX0_SIZE - 1u)

static xdata char    tx0_buf[TX0_SIZE];
static unsigned char xdata tx0_head = 0;
static unsigned char xdata tx0_tail = 0;
static bit           tx0_busy = 0;

static xdata char    rx0_buf[RX0_SIZE];
static unsigned char xdata rx0_head = 0;
static unsigned char xdata rx0_tail = 0;

void UART0_init(void)
{
    unsigned char saved_page = SFRPAGE;
    SFRPAGE = UART0_PAGE;

#if (((SYSCLK / BAUDRATE) / (2L * 12L)) > 0xFFL)
#error Timer1 reload overflow: (SYSCLK/BAUDRATE)/(2*12) > 0xFF
#endif

    /* Timer 1: mode 2 (8-bit auto-reload), baud-rate clock source */
    TMOD = (TMOD & 0x0Fu) | 0x20u;
    TH1  = (unsigned char)(0x100u - (SYSCLK / BAUDRATE) / (2L * 12L));
    TL1  = TH1;
    TR1  = 1;

    /* UART0: 8-bit UART (mode 1), receiver enabled */
    SCON0 = 0x10u;

    ES0 = 1;    /* enable UART0 interrupt */
    EA  = 1;    /* global interrupt enable */

    SFRPAGE = saved_page;
}

void UART0_ISR(void) __interrupt(4)
{
    unsigned char saved_page = SFRPAGE;
    SFRPAGE = UART0_PAGE;

    if (RI) {
        RI = 0;
        rx0_buf[rx0_head] = SBUF0;
        rx0_head = BUF_NEXT(rx0_head, RX0_MASK);
    }

    if (TI) {
        TI = 0;
        if (tx0_tail != tx0_head) {
            SBUF0    = tx0_buf[tx0_tail];
            tx0_tail = BUF_NEXT(tx0_tail, TX0_MASK);
        } else {
            tx0_busy = 0;
        }
    }

    SFRPAGE = saved_page;
}

void UART0_send_char(char c)
{
    unsigned char saved_page = SFRPAGE;
    SFRPAGE = UART0_PAGE;

    /* Spin until space is available (interrupts stay on to drain buffer) */
    while (BUF_NEXT(tx0_head, TX0_MASK) == tx0_tail);

    EA = 0;
    tx0_buf[tx0_head] = c;
    tx0_head = BUF_NEXT(tx0_head, TX0_MASK);
    if (!tx0_busy) {
        tx0_busy = 1;
        TI = 1;     /* kick-start TX ISR */
    }
    EA = 1;

    SFRPAGE = saved_page;
}

void UART0_send_string(const char *s)
{
    while (*s)
        UART0_send_char(*s++);
}

bit UART0_available(void)
{
    return (rx0_head != rx0_tail);
}

char UART0_read(void)
{
    char c;
    while (!UART0_available());
    c        = rx0_buf[rx0_tail];
    rx0_tail = BUF_NEXT(rx0_tail, RX0_MASK);
    return c;
}

#define UART1_PAGE      0x20u
#define TX1_SIZE        64u                 /* must be a power of 2 */
#define RX1_SIZE        64u
#define TX1_MASK        (TX1_SIZE - 1u)
#define RX1_MASK        (RX1_SIZE - 1u)

/* SCON1 flag bit masks (byte access – SCON1 is not bit-addressable) */
#define SCON1_RI_BIT    0x01u               /* receive  interrupt flag */
#define SCON1_TI_BIT    0x02u               /* transmit interrupt flag */

static xdata char    tx1_buf[TX1_SIZE];
static unsigned char xdata tx1_head = 0;
static unsigned char xdata tx1_tail = 0;
static bit           tx1_busy = 0;

static xdata char    rx1_buf[RX1_SIZE];
static unsigned char xdata rx1_head = 0;
static unsigned char xdata rx1_tail = 0;

void UART1_init(void)
{
    unsigned char saved_page = SFRPAGE;
    SFRPAGE = UART1_PAGE;

    /* UART1: 8-bit UART (mode 1), receiver enabled */
    SCON1  = 0x10u;

    /* Dedicated baud-rate generator: ~9600 baud @ 72 MHz SYSCLK (JDY-23 default) */
    SMOD1  = 0x0Cu;     /* SDL[3:2]=11 = 8-bit data length (matches reset default) */
    SBRLH1 = 0xF1u;
    SBRLL1 = 0x9Au;
    SBCON1 = 0x43u;     /* enable baud-rate generator (BCLKEN|SBRUN|BGCLK=SYSCLK) */

    /* EIE2 lives on page 0x00 */
    SFRPAGE = UART0_PAGE;
    EIE2 |= 0x01u;      /* enable UART1 interrupt vector */
    /* EA already set by UART_init() */

    SFRPAGE = saved_page;
}

void UART1_ISR(void) __interrupt(15)
{
    unsigned char saved_page = SFRPAGE;
    SFRPAGE = UART1_PAGE;

    if (SCON1 & SCON1_RI_BIT) {
        SCON1 &= ~SCON1_RI_BIT;
        rx1_buf[rx1_head] = SBUF1;
        rx1_head = BUF_NEXT(rx1_head, RX1_MASK);
    }

    if (SCON1 & SCON1_TI_BIT) {
        SCON1 &= ~SCON1_TI_BIT;
        if (tx1_tail != tx1_head) {
            SBUF1    = tx1_buf[tx1_tail];
            tx1_tail = BUF_NEXT(tx1_tail, TX1_MASK);
        } else {
            tx1_busy = 0;
        }
    }

    SFRPAGE = saved_page;
}

void UART1_send_char(char c)
{
    unsigned char saved_page = SFRPAGE;

    while (BUF_NEXT(tx1_head, TX1_MASK) == tx1_tail); /* wait for ring buffer space */

    EA = 0;
    tx1_buf[tx1_head] = c;
    tx1_head = BUF_NEXT(tx1_head, TX1_MASK);
    if (!tx1_busy) {
        tx1_busy = 1;
        SFRPAGE = UART1_PAGE;
        SBUF1 = tx1_buf[tx1_tail];      /* write first byte to TX FIFO — hardware will set TI when done */
        tx1_tail = BUF_NEXT(tx1_tail, TX1_MASK);
    }
    EA = 1;

    SFRPAGE = saved_page;
}

void UART1_send_string(const char *s)
{
    while (*s)
        UART1_send_char(*s++);
}

bit UART1_available(void)
{
    return (rx1_head != rx1_tail);
}

char UART1_read(void)
{
    char c;
    while (!UART1_available());
    c        = rx1_buf[rx1_tail];
    rx1_tail = BUF_NEXT(rx1_tail, RX1_MASK);
    return c;
}
