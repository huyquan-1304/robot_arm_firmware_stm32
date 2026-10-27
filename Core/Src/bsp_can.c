#include "bsp_can.h"

extern FDCAN_HandleTypeDef hfdcan1;   /* do CubeMX tao trong main.c */

/* Ma DLC cho 0..8 byte (dung macro cua HAL de chay dung o moi phien ban HAL) */
static const uint32_t dlc_table[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
};

/* ---- chan doan ---- */
static volatile uint32_t busoff_count = 0;
static volatile uint32_t err_total    = 0;
static volatile uint32_t lec_count[8] = {0};
static uint8_t was_busoff = 0;          /* dem bus-off theo lan vao, khong theo lan doc */

void bsp_can_service(void)
{
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;

    /* Doc PSR lam LEC ve 7 ("khong doi"), nen moi lan doc chi thay loi moi */
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) == HAL_OK) {
        /* Trong luc tu hoi phuc bus-off, FDCAN dat LEC = 5 moi khi thay 11 bit lan:
         * do khong phai loi that, khong dem */
        uint8_t recovering = (uint8_t)(ps.BusOff || READ_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT));
        if (ps.LastErrorCode >= 1u && ps.LastErrorCode <= 6u &&
            !(recovering && ps.LastErrorCode == 5u)) lec_count[ps.LastErrorCode]++;
        if (!ps.BusOff) was_busoff = 0;
        if (ps.BusOff) {
            if (!was_busoff) { busoff_count++; was_busoff = 1; }
            /* Bus-off: phan cung da dat INIT = 1. Xoa INIT de bo CAN tu noi lai bus
             * sau 129 lan 11 bit lan (khoang 3 ms o 500 kbps). */
            CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
        }
    }
    /* Doc ECR lam CEL ve 0: cong don vao tong */
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) == HAL_OK) {
        err_total += ec.ErrorLogging;
    }
}

void bsp_can_get_diag(bsp_can_diag_t *d)
{
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;

    bsp_can_service();
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    if (ps.LastErrorCode >= 1u && ps.LastErrorCode <= 6u && !(ps.BusOff && ps.LastErrorCode == 5u))
        lec_count[ps.LastErrorCode]++;
    err_total += ec.ErrorLogging;

    d->tec          = (uint8_t)ec.TxErrorCnt;
    d->rec          = (uint8_t)ec.RxErrorCnt;
    d->passive      = (uint8_t)(ps.ErrorPassive ? 1u : 0u);
    d->busoff_now   = (uint8_t)(ps.BusOff ? 1u : 0u);
    d->busoff_count = busoff_count;
    d->err_total    = err_total;
    for (int i = 0; i < 8; i++) d->lec[i] = lec_count[i];
}

void bsp_can_clear_diag(void)
{
    bsp_can_service();
    busoff_count = 0;
    err_total    = 0;
    for (int i = 0; i < 8; i++) lec_count[i] = 0;
}

/* ---- gui / nhan ---- */

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

    bsp_can_service();
    /* Dang bus-off (vua xoa INIT, chua noi lai xong): bao loi ngay, khong cho 10 ms vo ich */
    if (READ_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT)) return HAL_ERROR;

    /* Cho cho trong TX FIFO toi da 10 ms */
    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
        bsp_can_service();                /* lay mau loai loi trong luc frame dang gui lai */
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
    /* Ham nay duoc goi lien tuc khi cho tra loi: lay mau loai loi CAN tai day de bat duoc
     * ca chuoi loi xay ra trong luc mot frame bi gui lai nhieu lan */
    bsp_can_service();
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
