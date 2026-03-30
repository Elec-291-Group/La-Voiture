#ifndef UART_H
#define UART_H

// UART0: P0.4 = TX, P0.5 = RX
void UART0_init(void);
void UART0_send_char(char c);
void UART0_send_string(const char *s);
char UART0_read(void);
bit  UART0_available(void);

// UART1: P0.2 = TX, P0.3 = RX
void UART1_init(void);
void UART1_send_char(char c);
void UART1_send_string(const char *s);
char UART1_read(void);
bit  UART1_available(void);

#endif