#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include <stdint.h>

/* Lop thap nhat: gui / nhan frame CAN mo rong (29 bit) qua FDCAN1.
 * FDCAN1 duoc CubeMX khoi tao (PD0 = RX, PD1 = TX, Classic, 500 kbps).
 *
 * Tu thoat bus-off: khi bo FDCAN tu khoa (bus-off, sau qua nhieu loi truyen),
 * moi lan gui se xoa bit INIT de bo CAN tu noi lai bus (sau 129 x 11 bit lan),
 * khong can reset STM32. Moi lan gui cung ghi nhan loai loi gan nhat de chan doan. */

HAL_StatusTypeDef bsp_can_start(void);

/* Gui 1 frame ID mo rong, len = 0..8 byte */
HAL_StatusTypeDef bsp_can_send(uint32_t ext_id, const uint8_t *data, uint8_t len);

/* Nhan 1 frame neu co (khong cho). Tra ve 1 neu nhan duoc, 0 neu FIFO rong */
int bsp_can_recv(uint32_t *ext_id, uint8_t *data, uint8_t *len);

/* Bo het frame dang nam trong FIFO */
void bsp_can_flush(void);

/* Kiem tra trang thai bus, ghi nhan loi, tu thoat bus-off. Goi duoc bat ky luc nao. */
void bsp_can_service(void);

/* So lieu chan doan (lenh 'can' tren console) */
typedef struct {
    uint8_t  tec;           /* bo dem loi truyen hien tai (0..255) */
    uint8_t  rec;           /* bo dem loi nhan hien tai (0..127) */
    uint8_t  passive;       /* 1 = error-passive (dem loi >= 128) */
    uint8_t  busoff_now;    /* 1 = dang bus-off */
    uint32_t busoff_count;  /* so lan bus-off tu luc khoi dong / xoa */
    uint32_t err_total;     /* tong so loi da dem (tu bo dem CEL cua FDCAN) */
    uint32_t lec[8];        /* so lan thay tung loai loi: 1 stuff, 2 form, 3 ack, 4 bit1, 5 bit0, 6 crc */
} bsp_can_diag_t;

void bsp_can_get_diag(bsp_can_diag_t *d);
void bsp_can_clear_diag(void);

#endif /* BSP_CAN_H */
