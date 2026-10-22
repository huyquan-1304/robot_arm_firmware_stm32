#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include <stdint.h>

/* Lop thap nhat: gui / nhan frame CAN mo rong (29 bit) qua FDCAN1.
 * FDCAN1 duoc CubeMX khoi tao (PD0 = RX, PD1 = TX, Classic, 500 kbps). */

HAL_StatusTypeDef bsp_can_start(void);

/* Gui 1 frame ID mo rong, len = 0..8 byte */
HAL_StatusTypeDef bsp_can_send(uint32_t ext_id, const uint8_t *data, uint8_t len);

/* Nhan 1 frame neu co (khong cho). Tra ve 1 neu nhan duoc, 0 neu FIFO rong */
int bsp_can_recv(uint32_t *ext_id, uint8_t *data, uint8_t *len);

/* Bo het frame dang nam trong FIFO */
void bsp_can_flush(void);

#endif /* BSP_CAN_H */
