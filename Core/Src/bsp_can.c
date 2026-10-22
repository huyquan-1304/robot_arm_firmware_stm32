#include "bsp_can.h"

extern FDCAN_HandleTypeDef hfdcan1;   /* do CubeMX tao trong main.c */

/* Ma DLC cho 0..8 byte (dung macro cua HAL de chay dung o moi phien ban HAL) */
static const uint32_t dlc_table[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
};

HAL_StatusTypeDef bsp_can_start(void)
{
    /* Nhan moi frame (ca ID chuan va ID mo rong) vao RX FIFO0, khong can bo loc */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_Start(&hfdcan1);
}

HAL_StatusTypeDef bsp_can_send(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    if (len > 8) return HAL_ERROR;

    /* Cho cho trong TX FIFO toi da 10 ms */
    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
        if (HAL_GetTick() - t0 > 10) return HAL_TIMEOUT;
    }

    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = ext_id;
    tx.IdType              = FDCAN_EXTENDED_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = dlc_table[len];
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)data);
}

int bsp_can_recv(uint32_t *ext_id, uint8_t *data, uint8_t *len)
{
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) return 0;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t buf[8];
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, buf) != HAL_OK) return 0;

    /* HAL moi: DataLength = so byte. HAL cu: ma DLC dich trai 16 bit */
    uint32_t n = (rx.DataLength > 0xFU) ? (rx.DataLength >> 16) : rx.DataLength;
    if (n > 8) n = 8;

    for (uint32_t i = 0; i < n; i++) data[i] = buf[i];
    *len    = (uint8_t)n;
    *ext_id = rx.Identifier;
    return 1;
}

void bsp_can_flush(void)
{
    uint32_t id;
    uint8_t  d[8];
    uint8_t  n;
    while (bsp_can_recv(&id, d, &n)) {
        /* bo qua */
    }
}
