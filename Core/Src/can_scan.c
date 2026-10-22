/*
 * can_scan.c - Tu do chan FDCAN tren board STM32G474VE (DevEBox)
 *
 * Nguyen ly:
 *   IC transceiver luon dua tin hieu tren bus nguoc ve chan RX.
 *   - Dung cap chan: chip "nghe" thay chinh minh. Neu driver nghe duoc
 *     thi driver tra loi. Neu driver chua nghe duoc thi loi ACK.
 *   - Sai cap chan: chan RX chi co dien tro keo len, chip khong thay bit 0
 *     minh vua gui, nen bao loi BIT0.
 *
 * Yeu cau:
 *   - Clock he thong: HSI 16 MHz, khong dung PLL. Kernel clock FDCAN = PCLK1 = 16 MHz.
 *   - Toc do CAN: 500 kbps (16 MHz / 1 / (1 + 25 + 6) = 500 kHz).
 *   - Driver ZDT: P_Serial = CAN1_MAP, CAN_Baud = 500000, ID_Addr = 1, Checksum = 0x6B.
 */

#include "main.h"
#include "can_scan.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    FDCAN_GlobalTypeDef *inst;
    const char          *name;
    GPIO_TypeDef        *rx_port;
    uint16_t             rx_pin;
    const char          *rx_name;
    GPIO_TypeDef        *tx_port;
    uint16_t             tx_pin;
    const char          *tx_name;
    uint8_t              af;      /* so alternate function, tra trong datasheet Table 13 */
} can_pair_t;

/* 7 cap chan CAN co the co tren G474VE (LQFP100) */
static const can_pair_t pairs[] = {
    { FDCAN1, "FDCAN1", GPIOA, GPIO_PIN_11, "PA11", GPIOA, GPIO_PIN_12, "PA12",  9 },
    { FDCAN1, "FDCAN1", GPIOB, GPIO_PIN_8,  "PB8",  GPIOB, GPIO_PIN_9,  "PB9",   9 },
    { FDCAN1, "FDCAN1", GPIOD, GPIO_PIN_0,  "PD0",  GPIOD, GPIO_PIN_1,  "PD1",   9 },
    { FDCAN2, "FDCAN2", GPIOB, GPIO_PIN_5,  "PB5",  GPIOB, GPIO_PIN_6,  "PB6",   9 },
    { FDCAN2, "FDCAN2", GPIOB, GPIO_PIN_12, "PB12", GPIOB, GPIO_PIN_13, "PB13",  9 },
    { FDCAN3, "FDCAN3", GPIOA, GPIO_PIN_8,  "PA8",  GPIOA, GPIO_PIN_15, "PA15", 11 },
    { FDCAN3, "FDCAN3", GPIOB, GPIO_PIN_3,  "PB3",  GPIOB, GPIO_PIN_4,  "PB4",  11 },
};
#define NUM_PAIRS (sizeof(pairs) / sizeof(pairs[0]))

static FDCAN_HandleTypeDef hcan_scan;

/* Dua tat ca chan ung vien ve input co keo len (trang thai an toan) */
static void pins_idle(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint32_t i = 0; i < NUM_PAIRS; i++) {
        g.Pin = pairs[i].rx_pin;
        HAL_GPIO_Init(pairs[i].rx_port, &g);
        g.Pin = pairs[i].tx_pin;
        HAL_GPIO_Init(pairs[i].tx_port, &g);
    }
}

static void pin_af(GPIO_TypeDef *port, uint16_t pin, uint8_t af, uint32_t pull)
{
    GPIO_InitTypeDef g = {0};
    g.Pin       = pin;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = pull;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = af;
    HAL_GPIO_Init(port, &g);
}

static void fdcan_kernel_clock_pclk1(void)
{
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&clk);
}

static HAL_StatusTypeDef can_start(const can_pair_t *p)
{
    /* Reset sach toan bo khoi FDCAN (xoa bo dem loi tu lan thu truoc) */
    __HAL_RCC_FDCAN_FORCE_RESET();
    __HAL_RCC_FDCAN_RELEASE_RESET();
    fdcan_kernel_clock_pclk1();
    __HAL_RCC_FDCAN_CLK_ENABLE();

    memset(&hcan_scan, 0, sizeof(hcan_scan));
    hcan_scan.Instance                  = p->inst;
    hcan_scan.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hcan_scan.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hcan_scan.Init.Mode                 = FDCAN_MODE_NORMAL;
    hcan_scan.Init.AutoRetransmission   = DISABLE;   /* moi lan thu chi gui 1 lan */
    hcan_scan.Init.TransmitPause        = DISABLE;
    hcan_scan.Init.ProtocolException    = DISABLE;
    hcan_scan.Init.NominalPrescaler     = 1;         /* 16 MHz / 1 = 16 MHz */
    hcan_scan.Init.NominalSyncJumpWidth = 6;
    hcan_scan.Init.NominalTimeSeg1      = 25;        /* 1 + 25 + 6 = 32 tq -> 500 kbps */
    hcan_scan.Init.NominalTimeSeg2      = 6;         /* diem lay mau 81% */
    hcan_scan.Init.DataPrescaler        = 1;
    hcan_scan.Init.DataSyncJumpWidth    = 1;
    hcan_scan.Init.DataTimeSeg1         = 1;
    hcan_scan.Init.DataTimeSeg2         = 1;
    hcan_scan.Init.StdFiltersNbr        = 0;
    hcan_scan.Init.ExtFiltersNbr        = 0;
    hcan_scan.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hcan_scan) != HAL_OK) {
        return HAL_ERROR;
    }

    /* HAL_FDCAN_Init co the goi MspInit do CubeMX sinh ra (doi clock, doi chan).
     * Dat lai clock va chan theo cap dang thu. */
    fdcan_kernel_clock_pclk1();
    pins_idle();
    pin_af(p->rx_port, p->rx_pin, p->af, GPIO_PULLUP);
    pin_af(p->tx_port, p->tx_pin, p->af, GPIO_NOPULL);

    /* Nhan moi frame (khong loc) vao RX FIFO0 */
    HAL_FDCAN_ConfigGlobalFilter(&hcan_scan,
                                 FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    return HAL_FDCAN_Start(&hcan_scan);
}

static const char *lec_text(uint32_t lec)
{
    switch (lec) {
    case FDCAN_PROTOCOL_ERROR_NONE:      return "khong loi";
    case FDCAN_PROTOCOL_ERROR_STUFF:     return "STUFF";
    case FDCAN_PROTOCOL_ERROR_FORM:      return "FORM";
    case FDCAN_PROTOCOL_ERROR_ACK:       return "ACK";
    case FDCAN_PROTOCOL_ERROR_BIT1:      return "BIT1";
    case FDCAN_PROTOCOL_ERROR_BIT0:      return "BIT0";
    case FDCAN_PROTOCOL_ERROR_CRC:       return "CRC";
    case FDCAN_PROTOCOL_ERROR_NO_CHANGE: return "khong co hoat dong";
    default:                             return "?";
    }
}

/* Tra ve 1 neu cap chan nay la cap dung */
static int test_pair(const can_pair_t *p)
{
    int correct = 0;
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;

    printf("Thu %s  RX=%-4s TX=%-4s ... ", p->name, p->rx_name, p->tx_name);

    if (can_start(p) != HAL_OK) {
        printf("loi khoi tao FDCAN\r\n");
        goto done;
    }
    HAL_Delay(10);                                   /* cho bus ranh (11 bit lien tiep muc 1) */
    HAL_FDCAN_GetProtocolStatus(&hcan_scan, &ps);    /* doc 1 lan de xoa ma loi cu */

    /* Lenh doc phien ban firmware ZDT: dia chi 1 -> ID mo rong 0x100, du lieu 1F 6B */
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = 0x100;
    tx.IdType              = FDCAN_EXTENDED_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_2;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    uint8_t txd[2] = { 0x1F, 0x6B };

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hcan_scan, &tx, txd) != HAL_OK) {
        printf("khong dua duoc frame vao hang doi\r\n");
        goto done;
    }

    /* Cho tra loi toi da 100 ms */
    FDCAN_RxHeaderTypeDef rx;
    uint8_t rxd[8] = {0};
    int got = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 100) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hcan_scan, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hcan_scan, FDCAN_RX_FIFO0, &rx, rxd) == HAL_OK) {
                got = 1;
                break;
            }
        }
    }

    HAL_FDCAN_GetProtocolStatus(&hcan_scan, &ps);
    HAL_FDCAN_GetErrorCounters(&hcan_scan, &ec);
    uint32_t free_slots = HAL_FDCAN_GetTxFifoFreeLevel(&hcan_scan);

    if (got) {
        /* Ban HAL moi: DataLength la so byte. Ban cu: ma DLC dich trai 16 bit */
        uint32_t len = (rx.DataLength > 0xFU) ? (rx.DataLength >> 16) : rx.DataLength;
        if (len > 8) len = 8;
        printf("DRIVER TRA LOI  ID=0x%lX  data:", (unsigned long)rx.Identifier);
        for (uint32_t i = 0; i < len; i++) printf(" %02X", rxd[i]);
        printf("   <== DUNG CHAN\r\n");
        correct = 1;
    } else if (ps.LastErrorCode == FDCAN_PROTOCOL_ERROR_ACK) {
        printf("loi ACK: DUNG CHAN, nhung driver chua nghe (kiem tra CAN1_MAP, 500k, day CTX/CRX)\r\n");
        correct = 1;
    } else if (ps.LastErrorCode == FDCAN_PROTOCOL_ERROR_NONE) {
        printf("gui thanh cong, co ACK, nhung khong co tra loi: DUNG CHAN (kiem tra dia chi driver)\r\n");
        correct = 1;
    } else if (ps.LastErrorCode == FDCAN_PROTOCOL_ERROR_BIT0 ||
               ps.LastErrorCode == FDCAN_PROTOCOL_ERROR_BIT1) {
        printf("loi %s: sai chan\r\n", lec_text(ps.LastErrorCode));
    } else if (free_slots < 3) {
        printf("khong gui duoc (RX bi keo muc 0?): sai chan\r\n");
    } else {
        printf("loi %s, TEC=%lu: chua ro\r\n", lec_text(ps.LastErrorCode),
               (unsigned long)ec.TxErrorCnt);
    }

done:
    HAL_FDCAN_Stop(&hcan_scan);
    HAL_FDCAN_DeInit(&hcan_scan);
    pins_idle();
    return correct;
}

void can_scan_run(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    pins_idle();

    printf("\r\n===== DO CHAN CAN (HSI 16 MHz, 500 kbps, lenh 1F 6B toi dia chi 1) =====\r\n");

    int found = -1;
    for (uint32_t i = 0; i < NUM_PAIRS; i++) {
        if (test_pair(&pairs[i]) && found < 0) {
            found = (int)i;
        }
        HAL_Delay(50);
    }

    printf("-----------------------------------------------------------------------\r\n");
    if (found >= 0) {
        printf("KET QUA: %s  RX = %s, TX = %s\r\n",
               pairs[found].name, pairs[found].rx_name, pairs[found].tx_name);
    } else {
        printf("KET QUA: khong cap nao dung. Kiem tra nguon 3V3 board, day L/H, GND chung.\r\n");
    }
    printf("Bam nut RST tren board de chay lai.\r\n");
}
