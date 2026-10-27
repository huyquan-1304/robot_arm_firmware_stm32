#include "uart_rx.h"
#include "main.h"

extern UART_HandleTypeDef huart1;   /* do CubeMX tao trong main.c */

#define RX_BUF_SIZE 512u            /* luy thua cua 2 */

static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /* ngat ghi */
static volatile uint16_t rx_tail = 0;   /* vong lap chinh doc */
static volatile uint32_t rx_overflow = 0;

void uart_rx_init(void)
{
    rx_head = rx_tail = 0;
    /* Bo cac byte va co loi con sot tu luc khoi dong */
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        (void)(huart1.Instance->RDR);
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

int uart_rx_get(uint8_t *c)
{
    if (rx_tail == rx_head) return 0;
    *c = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & (RX_BUF_SIZE - 1u));
    return 1;
}

uint32_t uart_rx_overflow_count(void)
{
    return rx_overflow;
}

void USART1_IRQHandler(void)
{
    USART_TypeDef *u = huart1.Instance;

    /* Xoa loi tran / khung / nhieu / chan le de ngat khong bi lap mai */
    if (u->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }

    while (u->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t  c    = (uint8_t)(u->RDR & 0xFFu);
        uint16_t next = (uint16_t)((rx_head + 1u) & (RX_BUF_SIZE - 1u));
        if (next != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next;
        } else {
            rx_overflow++;
        }
    }
}
