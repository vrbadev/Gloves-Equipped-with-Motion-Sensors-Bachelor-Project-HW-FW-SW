#ifndef UART_H__
#define UART_H__

#include "stdio.h"
#include "BlueNRG1_conf.h"

void uart_init(void);
void uart_send(uint8_t tx_data);
void uart_sendstr(char* str);

#endif
