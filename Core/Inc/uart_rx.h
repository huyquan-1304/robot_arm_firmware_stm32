#ifndef UART_RX_H
#define UART_RX_H

#include <stdint.h>

/* Nhan USART1 bang ngat vao bo dem vong 512 byte.
 *
 * Ly do: truoc day cmd_uart doc thang thanh ghi RDR trong vong lap chinh. Moi lenh CAN
 * cho tra loi toi vai ms, trong khi 115200 baud chi co 87 us cho moi byte, nen khi Pi gui
 * goi nhi phan lien tuc 50 Hz se bi tran (ORE) va mat byte. Ngat lay tung byte ngay khi den.
 *
 * Ham USART1_IRQHandler nam trong uart_rx.c. Khong bat ngat USART1 trong CubeMX,
 * neu bat thi CubeMX sinh them mot USART1_IRQHandler trong stm32g4xx_it.c va bi trung ten. */

void     uart_rx_init(void);
int      uart_rx_get(uint8_t *c);       /* 1 = lay duoc 1 byte, 0 = bo dem rong */
uint32_t uart_rx_overflow_count(void);  /* so byte bi bo vi bo dem day */

#endif /* UART_RX_H */
