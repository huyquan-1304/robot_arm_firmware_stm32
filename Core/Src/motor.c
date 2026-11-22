#include "motor.h"
#include "link.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Handle do CubeMX tao trong main.c */
extern I2C_HandleTypeDef hi2c3;   /* 2 module MKE-M18 */
extern TIM_HandleTypeDef htim2;   /* encoder FL (32 bit), PA0/PA1 */
extern TIM_HandleTypeDef htim4;   /* encoder FR (16 bit) */
extern TIM_HandleTypeDef htim20;  /* encoder RR (16 bit), PE2/PE3 */
extern TIM_HandleTypeDef htim1;   /* encoder RL (16 bit), PC0/PC1 */
extern TIM_HandleTypeDef htim6;   /* nhip PID 200 Hz */

#define DT_S        0.005f        /* chu ky ngat TIM6 */
#define DT_MS       5u
#define HIST_N      4u            /* toc do = trung binh 4 chu ky (20 ms) */
#define POS_TOL_REV (1.0f / 360.0f)   /* toi dich khi sai so < 1 do */

/* Giao thuc MKE-M18: [0x05][mode][payload 4 byte big-endian], payload = (thoi_gian_ms << 16) | toc_do */
#define MKE_REGION        0x05u
#define MKE_CMD_SET_ADDR  1u
#define MKE_CMD_GET_ADDR  10u
#define MKE_CMD_FREQ      104u
#define MKE_CMD_A_CW      106u
#define MKE_CMD_A_CCW     107u
#define MKE_CMD_B_CW      108u
#define MKE_CMD_B_CCW     109u
#define MKE_CMD_A_STOP    110u
#define MKE_CMD_B_STOP    111u
#define MKE_CMD_GET_VIN   112u
#define I2C_TIMEOUT_MS    5u
#define I2C_STUCK_MS      20u      /* 1 dot gui qua 20 ms chua xong -> coi la treo, phuc hoi bus */

/* I2C3: PC8 = SCL, PC9 = SDA (dung khi phuc hoi bus) */
#define I2C_PORT    GPIOC
#define I2C_SCL     GPIO_PIN_8
#define I2C_SDA     GPIO_PIN_9

typedef enum { MODE_OFF = 0, MODE_PWM, MODE_VEL, MODE_POS } mot_mode_t;

/* Su kien de motor_task() in ra (ISR khong printf) */
#define EVT_TIMEOUT   0x01u
#define EVT_STALL     0x02u
#define EVT_DONE      0x04u
#define EVT_REVERSE   0x08u

typedef struct {
    /* cau hinh */
    const char        *name;
    uint8_t            i2c_addr;
    uint8_t            chan_b;     /* 0 = kenh A, 1 = kenh B cua module */
    TIM_HandleTypeDef *enc_tim;
    uint8_t            enc32;      /* 1 = timer 32 bit */
    int8_t             dir;        /* dau lenh -> lan ve truoc */
    int8_t             enc_dir;    /* dau encoder -> lan ve truoc */

    /* trang thai (ISR ghi, main doc) */
    volatile mot_mode_t mode;
    uint32_t           enc_last;
    volatile int32_t   counts;
    int32_t            hist[HIST_N];
    uint8_t            hidx;
    volatile float     rpm;
    volatile float     duty;       /* % dang gui, -100..100 */

    /* lenh */
    float              duty_cmd;   /* MODE_PWM */
    float              v_ref;      /* MODE_VEL, RPM */
    float              integ;

    /* che do vi tri, don vi vong banh va vong/giay */
    float              p_target, p_ref, prof_v, prof_vmax, prof_acc;
    volatile uint8_t   done;

    volatile uint32_t  ms_left;    /* tu dung (0 = khong hen gio) */
    uint8_t            off_at_zero;
    uint32_t           stall_ms, rev_ms;
    volatile uint8_t   fault;
    volatile uint8_t   evt;
} mot_t;

static mot_t mt[MOT_COUNT] = {
    { "FL", MOT_I2C_ADDR_LEFT,  0, &htim2, 1, MOT_DIR_FL, MOT_ENC_FL },   /* module TRAI 0x40, kenh A */
    { "FR", MOT_I2C_ADDR_RIGHT, 0, &htim4, 0, MOT_DIR_FR, MOT_ENC_FR },   /* module PHAI 0x41, kenh A */
    { "RL", MOT_I2C_ADDR_LEFT,  1, &htim1, 0, MOT_DIR_RL, MOT_ENC_RL },   /* module TRAI 0x40, kenh B */
    { "RR", MOT_I2C_ADDR_RIGHT, 1, &htim20, 0, MOT_DIR_RR, MOT_ENC_RR },   /* module PHAI 0x41, kenh B */
};

static float kff  = MOT_KFF_DEFAULT;
static float kp   = MOT_KP_DEFAULT;
static float ki   = MOT_KI_DEFAULT;
static float kpos = MOT_KPOS_DEFAULT;

static volatile uint8_t started = 0;

/* Lenh vi tri xep hang cho nhom (mot_group_begin / go) */
static uint8_t grp_open = 0;
static struct { uint8_t on; float deg, vmax, acc; } staged[MOT_COUNT];

/* ---- Gui I2C theo ngat: 4 goi lien tiep, goi sau bat dau khi goi truoc xong ---- */
static uint8_t           txbuf[MOT_COUNT][6];
static volatile int8_t   tx_idx   = -1;      /* -1 = ranh */
static volatile uint32_t tx_t0    = 0;
static volatile uint8_t  i2c_pause = 0;      /* 1 = main dang dung bus (doc Vin, doi dia chi...) */
static volatile uint8_t  i2c_need_recover = 0;
static uint8_t           tick_div = 0;

/* Dem loi de xem nhieu */
static volatile uint32_t n_sent = 0, n_nack = 0, n_buserr = 0, n_skip = 0, n_recover = 0;

/* ------------------------------------------------------------------ */

static float absf(float x) { return (x < 0.0f) ? -x : x; }
static float clampf(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

static float sqrt_f(float x)                  /* tranh phu thuoc libm */
{
    if (x <= 0.0f) return 0.0f;
    float r = (x > 1.0f) ? x : 1.0f;
    for (int i = 0; i < 12; i++) r = 0.5f * (r + x / r);
    return r;
}

/* In so thuc 1 chu so thap phan (printf khong ho tro %f) */
static void pf1(float v)
{
    int32_t x = (int32_t)(v * 10.0f + ((v < 0.0f) ? -0.5f : 0.5f));
    if (x < 0) { putchar('-'); x = -x; }
    printf("%ld.%ld", (long)(x / 10), (long)(x % 10));
}

static void pack(uint8_t *b, uint8_t mode, uint32_t payload)
{
    b[0] = MKE_REGION;
    b[1] = mode;
    b[2] = (uint8_t)(payload >> 24);
    b[3] = (uint8_t)(payload >> 16);
    b[4] = (uint8_t)(payload >> 8);
    b[5] = (uint8_t)payload;
}

/* Dong goi lenh cho 1 banh tu duty % (duong = lan ve truoc) */
static void pack_motor(int i)
{
    mot_t *m = &mt[i];
    float d = (m->mode == MODE_OFF) ? 0.0f : m->duty * (float)m->dir;
    uint32_t spd = (uint32_t)(absf(d) * 255.0f / 100.0f + 0.5f);
    if (spd > 255u) spd = 255u;

    uint8_t mode;
    if (spd == 0u) mode = m->chan_b ? MKE_CMD_B_STOP : MKE_CMD_A_STOP;
    else if (d > 0.0f) mode = m->chan_b ? MKE_CMD_B_CW : MKE_CMD_A_CW;
    else mode = m->chan_b ? MKE_CMD_B_CCW : MKE_CMD_A_CCW;

    uint32_t payload = (spd == 0u) ? 0u : (((uint32_t)MOT_CMD_HOLD_MS << 16) | spd);
    pack(txbuf[i], mode, payload);
}

static void tx_start(int i)
{
    tx_idx = (int8_t)i;
    if (HAL_I2C_Master_Transmit_IT(&hi2c3, (uint16_t)(mt[i].i2c_addr << 1), txbuf[i], 6) != HAL_OK) {
        tx_idx = -1;                                  /* HAL ban: bo dot nay, dot sau gui lai */
        n_skip++;
    }
}

/* Goi tu ngat TIM6 moi 10 ms */
static void tx_batch(void)
{
    if (i2c_pause || i2c_need_recover) return;
    if (tx_idx >= 0) { n_skip++; return; }           /* dot truoc chua xong */
    for (int i = 0; i < MOT_COUNT; i++) pack_motor(i);
    tx_t0 = HAL_GetTick();
    tx_start(0);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c3 || tx_idx < 0) return;
    n_sent++;
    int next = tx_idx + 1;
    if (next < MOT_COUNT) tx_start(next);
    else tx_idx = -1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c3) return;
    uint32_t e = HAL_I2C_GetError(hi2c);
    if (e & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_TIMEOUT)) {
        n_buserr++;                                   /* loi bus (nhieu): bo dot nay, phuc hoi bus */
        i2c_need_recover = 1;
        tx_idx = -1;
        return;
    }
    if (e & HAL_I2C_ERROR_AF) n_nack++;               /* module khong tra loi (chua cam / sai dia chi) */
    int next = tx_idx + 1;                            /* van gui tiep cho cac banh con lai */
    if (tx_idx >= 0 && next < MOT_COUNT) tx_start(next);
    else tx_idx = -1;
}

/* Phuc hoi bus: tat I2C, phat 9 xung SCL de giai phong SDA, tao STOP, khoi dong lai */
static void i2c_delay(void) { for (volatile int k = 0; k < 40; k++) { } }   /* ~5 us o 16 MHz */

static void i2c_bus_recover(void)
{
    HAL_I2C_DeInit(&hi2c3);
    GPIO_InitTypeDef g = {0};
    g.Pin   = I2C_SCL | I2C_SDA;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(I2C_PORT, I2C_SCL | I2C_SDA, GPIO_PIN_SET);
    HAL_GPIO_Init(I2C_PORT, &g);
    for (int k = 0; k < 9; k++) {
        HAL_GPIO_WritePin(I2C_PORT, I2C_SCL, GPIO_PIN_RESET); i2c_delay();
        HAL_GPIO_WritePin(I2C_PORT, I2C_SCL, GPIO_PIN_SET);   i2c_delay();
    }
    HAL_GPIO_WritePin(I2C_PORT, I2C_SDA, GPIO_PIN_RESET); i2c_delay();   /* STOP: SDA len khi SCL cao */
    HAL_GPIO_WritePin(I2C_PORT, I2C_SDA, GPIO_PIN_SET);   i2c_delay();
    HAL_GPIO_DeInit(I2C_PORT, I2C_SCL | I2C_SDA);
    HAL_I2C_Init(&hi2c3);                                  /* MspInit gan lai chan AF */
    n_recover++;
}

/* Main dung bus (doc / ghi lenh cham): dung gui theo ngat, cho dot dang chay xong */
static void bus_take(void)
{
    i2c_pause = 1;
    uint32_t t = HAL_GetTick();
    while (tx_idx >= 0 && (HAL_GetTick() - t) < 30u) { }
}
static void bus_give(void) { i2c_pause = 0; }

static HAL_StatusTypeDef mke_write(uint8_t addr, uint8_t mode, uint32_t payload)
{
    uint8_t b[6];
    pack(b, mode, payload);
    return HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)(addr << 1), b, 6, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mke_read(uint8_t addr, uint8_t mode, uint32_t *val)
{
    uint8_t r[4];
    if (mke_write(addr, mode, 0u) != HAL_OK) return HAL_ERROR;
    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < 2u) { }                   /* module can >= 200 us de chuan bi */
    if (HAL_I2C_Master_Receive(&hi2c3, (uint16_t)(addr << 1), r, 4, I2C_TIMEOUT_MS) != HAL_OK) return HAL_ERROR;
    *val = ((uint32_t)r[0] << 24) | ((uint32_t)r[1] << 16) | ((uint32_t)r[2] << 8) | r[3];
    return HAL_OK;
}

/* PID toc do -> duty %. Chong bao hoa tich phan */
static float vel_ctrl(mot_t *m, float v_target)
{
    float e   = v_target - m->rpm;
    float ff  = kff * v_target;
    float in2 = m->integ + e * DT_S;
    float u2  = ff + kp * e + ki * in2;
    if (!((u2 > 100.0f && e > 0.0f) || (u2 < -100.0f && e < 0.0f))) m->integ = in2;
    return clampf(ff + kp * e + ki * m->integ, -100.0f, 100.0f);
}

static void go_off(mot_t *m)
{
    m->mode = MODE_OFF;
    m->integ = 0.0f;
    m->ms_left = 0;
    m->duty = 0.0f;
}

/* ------------------------------------------------------------------ */
/* NGAT TIM6, 200 Hz */

static void motor_isr(void)
{
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];

        /* 1. Doc encoder */
        uint32_t raw = m->enc_tim->Instance->CNT;
        int32_t  d   = m->enc32 ? (int32_t)(raw - m->enc_last)
                                : (int32_t)(int16_t)(uint16_t)(raw - m->enc_last);
        m->enc_last = raw;
        d *= m->enc_dir;
        m->counts += d;
        m->hist[m->hidx] = d;
        m->hidx = (uint8_t)((m->hidx + 1u) % HIST_N);
        int32_t sum = 0;
        for (unsigned k = 0; k < HIST_N; k++) sum += m->hist[k];
        m->rpm = (float)sum * 60.0f / (MOT_COUNTS_PER_REV * DT_S * (float)HIST_N);

        /* 2. Hen gio tu dung */
        if (m->ms_left) {
            if (m->ms_left <= DT_MS) {
                m->ms_left = 0;
                if (m->mode == MODE_PWM) go_off(m);
                else if (m->mode == MODE_VEL) { m->v_ref = 0.0f; m->off_at_zero = 1; }
                m->evt |= EVT_TIMEOUT;
            } else m->ms_left -= DT_MS;
        }

        /* 3. Dieu khien */
        float duty = 0.0f;
        switch (m->mode) {
        case MODE_PWM:
            duty = m->duty_cmd;
            break;
        case MODE_VEL:
            if (m->v_ref == 0.0f && absf(m->rpm) < 1.0f) {
                m->integ = 0.0f;                          /* duty 0 -> lenh STOP */
                if (m->off_at_zero) { m->off_at_zero = 0; go_off(m); }
            }
            else duty = vel_ctrl(m, m->v_ref);
            break;
        case MODE_POS: {
            float pos = (float)m->counts / MOT_COUNTS_PER_REV;
            float rem = m->p_target - m->p_ref;
            float a   = m->prof_acc;                    /* vong/s^2 */
            if (rem != 0.0f) {
                float sgn  = (rem > 0.0f) ? 1.0f : -1.0f;
                float vdes = sgn * sqrt_f(2.0f * a * absf(rem));
                if (absf(vdes) > m->prof_vmax) vdes = sgn * m->prof_vmax;
                float dv = vdes - m->prof_v;
                float dvmax = a * DT_S;
                m->prof_v += clampf(dv, -dvmax, dvmax);
                float step = m->prof_v * DT_S;
                if ((sgn > 0.0f && step >= rem) || (sgn < 0.0f && step <= rem) || absf(rem) < 1e-6f) {
                    m->p_ref = m->p_target; m->prof_v = 0.0f;
                } else m->p_ref += step;
            }
            float err = m->p_target - pos;
            if (m->p_ref == m->p_target && absf(err) < POS_TOL_REV) {
                if (!m->done) { m->done = 1; m->evt |= EVT_DONE; }
                m->integ = 0.0f;                          /* toi dich: STOP, PID keo lai neu bi day lech */
            } else {
                float v_cmd = m->prof_v * 60.0f + kpos * (m->p_ref - pos);
                duty = vel_ctrl(m, clampf(v_cmd, -MOT_RPM_MAX, MOT_RPM_MAX));
            }
            break;
        }
        default:
            break;
        }

        /* 4. Bao ve: ket truc, hoac encoder nguoc chieu (motor chay vot) */
        if (m->mode == MODE_VEL || m->mode == MODE_POS) {
            if (absf(duty) > MOT_STALL_DUTY && absf(m->rpm) < MOT_STALL_RPM) m->stall_ms += DT_MS;
            else m->stall_ms = 0;
            if (absf(duty) > 50.0f && duty * m->rpm < 0.0f && absf(m->rpm) > 5.0f) m->rev_ms += DT_MS;
            else m->rev_ms = 0;
            if (m->stall_ms >= MOT_STALL_MS) { go_off(m); m->fault = 1; m->evt |= EVT_STALL; continue; }
            if (m->rev_ms >= 300u)          { go_off(m); m->fault = 1; m->evt |= EVT_REVERSE; continue; }
        } else { m->stall_ms = 0; m->rev_ms = 0; }

        m->duty = (m->mode == MODE_OFF) ? 0.0f : duty;
    }

    /* 5. Gui lenh cho 2 module moi 10 ms */
    if (++tick_div >= 2u) { tick_div = 0; tx_batch(); }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6 && started) motor_isr();
}

/* ------------------------------------------------------------------ */

/* Bat dien tro keo len cho 8 chan encoder (trong .ioc dang de No pull).
 * Ghi thang thanh ghi PUPDR nen Generate Code lai cung khong mat */
static void enc_pullups(void)
{
    static const struct { GPIO_TypeDef *port; uint8_t pin; } enc_pins[] = {
        { GPIOA, 0 }, { GPIOA, 1 },     /* TIM2 FL */
        { GPIOD, 12 }, { GPIOD, 13 },   /* TIM4 FR */
        { GPIOC, 0 }, { GPIOC, 1 },     /* TIM1 RL */
        { GPIOE, 2 }, { GPIOE, 3 },     /* TIM20 RR */
    };
    for (unsigned i = 0; i < sizeof(enc_pins) / sizeof(enc_pins[0]); i++) {
        uint32_t sh = 2u * enc_pins[i].pin;
        MODIFY_REG(enc_pins[i].port->PUPDR, 3u << sh, 1u << sh);   /* 01 = pull-up */
    }
}

void motor_init(void)
{
    enc_pullups();
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];
        /* Bo loc dau vao kenh 2 = 10 (phong khi CubeMX de 0) */
        MODIFY_REG(m->enc_tim->Instance->CCMR1, TIM_CCMR1_IC2F, 10u << TIM_CCMR1_IC2F_Pos);
        HAL_TIM_Encoder_Start(m->enc_tim, TIM_CHANNEL_ALL);
        m->enc_last = m->enc_tim->Instance->CNT;
        go_off(m);
    }
    /* Dung ca 2 module ngay khi khoi dong */
    (void)mke_write(MOT_I2C_ADDR_LEFT, MKE_CMD_A_STOP, 0u);
    (void)mke_write(MOT_I2C_ADDR_LEFT, MKE_CMD_B_STOP, 0u);
    (void)mke_write(MOT_I2C_ADDR_RIGHT, MKE_CMD_A_STOP, 0u);
    (void)mke_write(MOT_I2C_ADDR_RIGHT, MKE_CMD_B_STOP, 0u);
    started = 1;
    HAL_TIM_Base_Start_IT(&htim6);
}

void mot_stop_all(void)
{
    __disable_irq();
    for (int i = 0; i < MOT_COUNT; i++) { go_off(&mt[i]); mt[i].done = 1; }
    __enable_irq();
}

static void stop_one(mot_t *m)
{
    __disable_irq();
    go_off(m);
    m->done = 1;
    __enable_irq();
}

void mot_set_rpm(mot_id_t id, float rpm)
{
    if (id >= MOT_COUNT) return;
    mot_t *m = &mt[id];
    if (m->fault) return;
    rpm = clampf(rpm, -MOT_RPM_MAX, MOT_RPM_MAX);
    __disable_irq();
    if (m->mode != MODE_VEL) { m->integ = 0.0f; m->mode = MODE_VEL; }
    m->v_ref = rpm;
    m->off_at_zero = 0;
    m->ms_left = 0;
    m->done = 1;
    __enable_irq();
}

static void start_pos(mot_t *m, float deg, float rpm_max, float acc)
{
    float pos = (float)m->counts / MOT_COUNTS_PER_REV;
    m->p_target  = pos + deg / 360.0f;
    m->p_ref     = pos;
    m->prof_v    = 0.0f;
    m->prof_vmax = clampf(absf(rpm_max), 0.1f, MOT_RPM_MAX) / 60.0f;
    m->prof_acc  = ((acc > 0.1f) ? acc : 0.1f) / 60.0f;
    m->integ     = 0.0f;
    m->ms_left   = 0;
    m->done      = 0;
    m->mode      = MODE_POS;
}

void mot_move_rel(mot_id_t id, float deg, float rpm_max, float acc)
{
    if (id >= MOT_COUNT) return;
    if (grp_open) {
        staged[id].on = 1; staged[id].deg = deg; staged[id].vmax = rpm_max; staged[id].acc = acc;
        return;
    }
    if (mt[id].fault) return;
    __disable_irq();
    start_pos(&mt[id], deg, rpm_max, acc);
    __enable_irq();
}

void mot_group_begin(void)
{
    memset(staged, 0, sizeof(staged));
    grp_open = 1;
}

void mot_group_go(void)
{
    grp_open = 0;
    __disable_irq();                          /* 4 banh bat dau trong cung 1 chu ky */
    for (int i = 0; i < MOT_COUNT; i++)
        if (staged[i].on && !mt[i].fault) start_pos(&mt[i], staged[i].deg, staged[i].vmax, staged[i].acc);
    __enable_irq();
}

int   mot_move_done(mot_id_t id) { return (id < MOT_COUNT) ? (mt[id].mode != MODE_POS || mt[id].done) : 1; }
float mot_get_rpm(mot_id_t id)   { return (id < MOT_COUNT) ? mt[id].rpm : 0.0f; }
int32_t mot_get_counts(mot_id_t id) { return (id < MOT_COUNT) ? mt[id].counts : 0; }
float mot_get_deg(mot_id_t id)   { return (id < MOT_COUNT) ? (float)mt[id].counts * 360.0f / MOT_COUNTS_PER_REV : 0.0f; }
int   mot_fault(mot_id_t id)     { return (id < MOT_COUNT) ? mt[id].fault : 0; }
const char *mot_name(mot_id_t id){ return (id < MOT_COUNT) ? mt[id].name : "?"; }

void mot_clear_fault(mot_id_t id)
{
    if (id >= MOT_COUNT) return;
    __disable_irq();
    mt[id].fault    = 0;
    mt[id].stall_ms = 0;
    mt[id].rev_ms   = 0;
    __enable_irq();
}

uint32_t mot_i2c_errors(void) { return n_nack + n_buserr; }

void mot_zero(mot_id_t id)
{
    if (id >= MOT_COUNT) return;
    __disable_irq();
    mt[id].counts = 0;
    if (mt[id].mode == MODE_POS) { mt[id].p_target = 0.0f; mt[id].p_ref = 0.0f; mt[id].prof_v = 0.0f; }
    __enable_irq();
}

/* ------------------------------------------------------------------ */
/* THONG BAO TU ISR, PHUC HOI I2C */

void motor_task(void)
{
    /* Bus treo: co bao loi, hoac 1 dot gui qua lau chua xong */
    if (!i2c_pause) {
        if (tx_idx >= 0 && (HAL_GetTick() - tx_t0) > I2C_STUCK_MS) {
            i2c_need_recover = 1;                       /* dat truoc de ngat TIM6 khong gui dot moi */
            HAL_I2C_Master_Abort_IT(&hi2c3, (uint16_t)(mt[tx_idx].i2c_addr << 1));
            tx_idx = -1;
        }
        if (i2c_need_recover && tx_idx < 0) {
            i2c_bus_recover();
            i2c_need_recover = 0;
        }
    }

    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];
        uint8_t e = m->evt;
        if (!e) continue;
        __disable_irq(); m->evt = 0; __enable_irq();
        if (e & EVT_STALL)   printf("\r\n[Banh %s: KET TRUC (PWM cao ma khong quay), da ngat. Kiem tra roi go 'm %s c']\r\n> ", m->name, m->name);
        if (e & EVT_REVERSE) printf("\r\n[Banh %s: ENCODER NGUOC CHIEU motor, da ngat. Go 'm %s daoe' roi 'm %s c']\r\n> ", m->name, m->name, m->name);
        if (e & EVT_TIMEOUT) printf("\r\n[Banh %s: het gio, da dung]\r\n> ", m->name);
    }
}

/* ------------------------------------------------------------------ */
/* CONSOLE "m ..." */

static int parse_wheel(const char *s)
{
    if (!s) return -1;
    for (int i = 0; i < MOT_COUNT; i++) {
        const char *n = mt[i].name;
        if ((s[0] == n[0] || s[0] == n[0] + 32) && (s[1] == n[1] || s[1] == n[1] + 32) && s[2] == '\0') return i;
    }
    return -1;
}

static const char *mode_str(mot_mode_t md)
{
    switch (md) { case MODE_PWM: return "PWM"; case MODE_VEL: return "VAN TOC"; case MODE_POS: return "VI TRI"; default: return "TAT"; }
}

static void show_all(void)
{
    printf("Banh | module | che do  | PWM %%  | toc do RPM | vi tri do | xung     | loi\r\n");
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_t *m = &mt[i];
        printf("  %s | 0x%02X %c | %-7s | ", m->name, m->i2c_addr, m->chan_b ? 'B' : 'A', mode_str(m->mode));
        pf1(m->duty); printf(" | "); pf1(m->rpm); printf(" | "); pf1(mot_get_deg((mot_id_t)i));
        printf(" | %ld | %s\r\n", (long)m->counts, m->fault ? "KET/NGUOC" : "-");
    }
    printf("I2C: gui %lu goi | NACK %lu | loi bus %lu | bo qua %lu | phuc hoi %lu\r\n",
           (unsigned long)n_sent, (unsigned long)n_nack, (unsigned long)n_buserr,
           (unsigned long)n_skip, (unsigned long)n_recover);
}

static void show_enc(void)
{
    for (int i = 0; i < MOT_COUNT; i++) {
        printf("  %s: %ld xung = ", mt[i].name, (long)mt[i].counts);
        pf1(mot_get_deg((mot_id_t)i)); printf(" do = ");
        pf1(mot_get_deg((mot_id_t)i) / 360.0f); printf(" vong\r\n");
    }
}

static void watch(void)
{
    printf("Theo doi toc do / vi tri 4 banh moi 0.2 s. Bam phim bat ky de thoat.\r\n");
    uint32_t next = HAL_GetTick();
    while (!link_any_key()) {
        link_task();
        motor_task();
        if ((int32_t)(HAL_GetTick() - next) >= 0) {
            next += 200u;
            for (int i = 0; i < MOT_COUNT; i++) {
                printf("%s ", mt[i].name); pf1(mt[i].rpm); printf(" RPM ");
                pf1(mot_get_deg((mot_id_t)i)); printf(" do  ");
            }
            printf("| loi I2C %lu\r\n", (unsigned long)(n_nack + n_buserr));
        }
    }
    printf("Thoat theo doi\r\n");
}

static void show_pid(void)
{
    printf("PID toc do: kp "); pf1(kp); printf(" | ki "); pf1(ki); printf(" | kff ");
    printf("%ld.%02ld", (long)(kff * 100.0f) / 100, (long)(kff * 100.0f) % 100);
    printf(" | kpos "); pf1(kpos); printf("   (don vi %% PWM / RPM)\r\n");
}

/* Quet bus I2C, in dia chi co tra loi */
static void i2c_scan(void)
{
    mot_stop_all();
    bus_take();
    printf("Quet I2C3 dia chi 1..126...\r\n");
    int n = 0;
    for (uint8_t a = 1; a <= 126; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(a << 1), 2, 3) == HAL_OK) {
            uint32_t vin = 0;
            printf("  0x%02X", a);
            if (mke_read(a, MKE_CMD_GET_VIN, &vin) == HAL_OK) printf("  Vin %lu mV", (unsigned long)vin);
            printf("\r\n");
            n++;
        }
    }
    printf("Tim thay %d thiet bi. Can co 0x%02X (TRAI: FL, RL) va 0x%02X (PHAI: FR, RR)\r\n", n, MOT_I2C_ADDR_LEFT, MOT_I2C_ADDR_RIGHT);
    bus_give();
}

static void show_vin(void)
{
    bus_take();
    const uint8_t ad[2] = { MOT_I2C_ADDR_LEFT, MOT_I2C_ADDR_RIGHT };
    for (int k = 0; k < 2; k++) {
        uint32_t v = 0;
        if (mke_read(ad[k], MKE_CMD_GET_VIN, &v) == HAL_OK)
            printf("  Module 0x%02X: Vin %lu.%02lu V\r\n", ad[k], (unsigned long)(v / 1000u), (unsigned long)((v % 1000u) / 10u));
        else printf("  Module 0x%02X: khong tra loi\r\n", ad[k]);
    }
    bus_give();
}

static void set_addr(const char *so, const char *sn)
{
    if (!so || !sn) { printf("Vd: m addr 40 41   (doi module 0x40 thanh 0x41, so hex)\r\n"); return; }
    uint8_t o = (uint8_t)strtoul(so, NULL, 16), n = (uint8_t)strtoul(sn, NULL, 16);
    if (o < 1 || o > 126 || n < 1 || n > 126) { printf("Dia chi phai trong 01..7E (hex)\r\n"); return; }
    mot_stop_all();
    bus_take();
    if (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(n << 1), 2, 3) == HAL_OK) {
        printf("Da co thiet bi o 0x%02X, khong doi de tranh trung\r\n", n); bus_give(); return;
    }
    HAL_StatusTypeDef s = mke_write(o, MKE_CMD_SET_ADDR, n);
    uint32_t t = HAL_GetTick(); while ((HAL_GetTick() - t) < 50u) { }
    int ok = (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(n << 1), 2, 3) == HAL_OK);
    printf("Doi 0x%02X -> 0x%02X: %s%s\r\n", o, n, (s == HAL_OK) ? "da gui" : "LOI GUI",
           ok ? ", module da tra loi o dia chi moi" : ". Chua thay o dia chi moi: tat bat lai module roi 'm scan'");
    bus_give();
}

void motor_print_help(void)
{
    printf("  --- MOTOR BANH (2 module MKE-M18 qua I2C). <banh> = fl, fr, rl, rr. KE XE LEN ---\r\n");
    printf("  m                  trang thai 4 banh + dem loi I2C\r\n");
    printf("  m scan             quet I2C, tim 2 module (0x40 trai, 0x41 phai)\r\n");
    printf("  m addr <cu> <moi>  doi dia chi module, so hex (vd: m addr 40 41). Chi cam 1 module\r\n");
    printf("  m vin              doc dien ap nguon motor cua 2 module\r\n");
    printf("  m freq <hz>        dat tan so PWM cua 2 module\r\n");
    printf("  m e                so xung encoder 4 banh (quay tay de kiem tra)\r\n");
    printf("  m w                theo doi toc do / vi tri lien tuc\r\n");
    printf("  m <banh> pwm <%%> [giay]  chay PWM KHONG PID (-100..100), tu dung sau 2 s\r\n");
    printf("  m <banh> v <rpm> [giay]  PID toc do, tu dung sau 5 s (0 = khong hen gio)\r\n");
    printf("  m <banh> q <vong>  quay dung so vong\r\n");
    printf("  m <banh> z / m z   dat vi tri = 0\r\n");
    printf("  m <banh> dao       dao chieu MOTOR (PWM duong ma banh lan lui)\r\n");
    printf("  m <banh> daoe      dao chieu ENCODER (lan toi ma so xung giam)\r\n");
    printf("  m <banh> c / m c   xoa loi ket truc\r\n");
    printf("  m pid [kp ki [kff [kpos]]]  xem / dat PID\r\n");
    printf("  m <banh> x / m x   dung 1 banh / DUNG CA 4\r\n");
}

void motor_cmd(int argc, char **argv)
{
    const char *a = (argc > 1) ? argv[1] : NULL;
    const char *b = (argc > 2) ? argv[2] : NULL;
    const char *c = (argc > 3) ? argv[3] : NULL;
    const char *d = (argc > 4) ? argv[4] : NULL;
    const char *e = (argc > 5) ? argv[5] : NULL;

    if (!a)                 { show_all(); return; }
    if (!strcmp(a, "e"))    { show_enc(); return; }
    if (!strcmp(a, "w"))    { watch(); return; }
    if (!strcmp(a, "scan")) { i2c_scan(); return; }
    if (!strcmp(a, "vin"))  { show_vin(); return; }
    if (!strcmp(a, "addr")) { set_addr(b, c); return; }
    if (!strcmp(a, "freq")) {
        if (!b) { printf("Vd: m freq 20000\r\n"); return; }
        uint32_t f = (uint32_t)strtoul(b, NULL, 10);
        bus_take();
        HAL_StatusTypeDef s1 = mke_write(MOT_I2C_ADDR_LEFT, MKE_CMD_FREQ, f);
        HAL_StatusTypeDef s2 = mke_write(MOT_I2C_ADDR_RIGHT, MKE_CMD_FREQ, f);
        bus_give();
        printf("Tan so PWM %lu Hz: module trai %s, module phai %s\r\n", (unsigned long)f,
               (s1 == HAL_OK) ? "OK" : "LOI", (s2 == HAL_OK) ? "OK" : "LOI");
        return;
    }
    if (!strcmp(a, "x"))    { mot_stop_all(); printf("Da dung ca 4 banh\r\n"); return; }
    if (!strcmp(a, "z"))    { for (int i = 0; i < MOT_COUNT; i++) mot_zero((mot_id_t)i); printf("Vi tri 4 banh = 0\r\n"); return; }
    if (!strcmp(a, "c"))    { for (int i = 0; i < MOT_COUNT; i++) mt[i].fault = 0; printf("Da xoa loi 4 banh\r\n"); return; }
    if (!strcmp(a, "pid")) {
        if (b && c) {
            kp = strtof(b, NULL); ki = strtof(c, NULL);
            if (d) kff = strtof(d, NULL);
            if (e) kpos = strtof(e, NULL);
            for (int i = 0; i < MOT_COUNT; i++) mt[i].integ = 0.0f;
        }
        show_pid();
        return;
    }

    int w = parse_wheel(a);
    if (w < 0) { printf("Khong hieu 'm %s'. Banh: fl, fr, rl, rr. Go h de xem lenh.\r\n", a); return; }
    mot_t *m = &mt[w];

    if (!b) {
        printf("%s (module 0x%02X kenh %c): %s, PWM ", m->name, m->i2c_addr, m->chan_b ? 'B' : 'A', mode_str(m->mode));
        pf1(m->duty); printf(" %%, ");
        pf1(m->rpm); printf(" RPM, "); pf1(mot_get_deg((mot_id_t)w)); printf(" do, %ld xung, dau motor %+d, dau encoder %+d%s\r\n",
               (long)m->counts, m->dir, m->enc_dir, m->fault ? ", DANG LOI" : "");
        return;
    }
    if (!strcmp(b, "x")) { stop_one(m); printf("%s: dung\r\n", m->name); return; }
    if (!strcmp(b, "z")) { mot_zero((mot_id_t)w); printf("%s: vi tri = 0\r\n", m->name); return; }
    if (!strcmp(b, "c")) { m->fault = 0; printf("%s: da xoa loi\r\n", m->name); return; }
    if (!strcmp(b, "dao"))  { stop_one(m); m->dir = (int8_t)-m->dir;
        printf("%s: dau MOTOR = %+d. Nho sua MOT_DIR_%s trong motor.h\r\n", m->name, m->dir, m->name); return; }
    if (!strcmp(b, "daoe")) { stop_one(m); m->enc_dir = (int8_t)-m->enc_dir;
        printf("%s: dau ENCODER = %+d. Nho sua MOT_ENC_%s trong motor.h\r\n", m->name, m->enc_dir, m->name); return; }

    if (m->fault) { printf("%s dang loi. Go 'm %s c' truoc\r\n", m->name, m->name); return; }

    if (!strcmp(b, "pwm")) {
        if (!c) { printf("Vd: m %s pwm 30\r\n", m->name); return; }
        float p = clampf(strtof(c, NULL), -100.0f, 100.0f);
        uint32_t s = d ? (uint32_t)atoi(d) : 2u;
        if (s == 0u || s > 30u) s = 2u;
        __disable_irq();
        m->duty_cmd = p; m->mode = MODE_PWM; m->ms_left = s * 1000u; m->done = 1;
        __enable_irq();
        printf("%s: PWM ", m->name); pf1(p); printf(" %% trong %lu s (khong PID)\r\n", (unsigned long)s);
    }
    else if (!strcmp(b, "v")) {
        if (!c) { printf("Vd: m %s v 20\r\n", m->name); return; }
        float v = strtof(c, NULL);
        uint32_t s = d ? (uint32_t)atoi(d) : 5u;
        mot_set_rpm((mot_id_t)w, v);
        __disable_irq(); m->ms_left = s * 1000u; __enable_irq();
        printf("%s: toc do ", m->name); pf1(clampf(v, -MOT_RPM_MAX, MOT_RPM_MAX)); printf(" RPM");
        if (s) printf(", tu dung sau %lu s\r\n", (unsigned long)s); else printf(", KHONG hen gio ('m x' de dung)\r\n");
    }
    else if (!strcmp(b, "q")) {
        if (!c) { printf("Vd: m %s q 1\r\n", m->name); return; }
        float rev = strtof(c, NULL);
        if (rev == 0.0f || absf(rev) > 20.0f) { printf("So vong sai (toi da 20)\r\n"); return; }
        mot_move_rel((mot_id_t)w, rev * 360.0f, 30.0f, 60.0f);
        printf("%s: quay ", m->name); pf1(rev); printf(" vong o 30 RPM. Xong go 'm %s' xem vi tri\r\n", m->name);
    }
    else printf("Khong hieu 'm %s %s'\r\n", a, b);
}
