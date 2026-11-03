#include "wrist.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;   /* do CubeMX tao trong main.c, 1 tick = 1 us, chu ky 20 ms */

/* ===================== HIEU CHUAN (do bang lenh "wu <us>" va thuoc do goc) =====================
 *   US_ZERO : xung lam gripper nam dung goc 0
 *   US_P90  : xung lam gripper xoay toi +90 do (chieu duong theo URDF)
 *   US_M90  : xung lam gripper xoay toi -90 do
 * Neu chieu duong cua servo nguoc URDF thi US_P90 se NHO hon US_ZERO, khong can them dau. */
#define WRIST_US_ZERO    1500
#define WRIST_US_P90      500
#define WRIST_US_M90     2500

#define WRIST_MIN_X10    (-900)   /* gioi han goc, nho hon diem chan ben trong servo 2..3 do */
#define WRIST_MAX_X10      900
#define WRIST_DPS          120    /* toc do xoay (do/s), MG90S toi da khoang 600 do/s */
#define WRIST_US_ABS_MIN   400    /* chan xung tho cho lenh wu */
#define WRIST_US_ABS_MAX  2600

/* ------------------------------------------------------------------ */

static int32_t  cur_x10    = 0;   /* goc dang phat */
static int32_t  tgt_x10    = 0;   /* goc dich */
static uint16_t cur_us     = 0;   /* 0 = chua phat xung */
static uint8_t  moving     = 0;
static uint32_t last_tick  = 0;

static uint16_t x10_to_us(int32_t x10)
{
    int32_t us;
    if (x10 >= 0) us = WRIST_US_ZERO + (int32_t)(WRIST_US_P90 - WRIST_US_ZERO) * x10 / 900;
    else          us = WRIST_US_ZERO + (int32_t)(WRIST_US_ZERO - WRIST_US_M90) * x10 / 900;
    if (us < WRIST_US_ABS_MIN) us = WRIST_US_ABS_MIN;
    if (us > WRIST_US_ABS_MAX) us = WRIST_US_ABS_MAX;
    return (uint16_t)us;
}

static void write_us(uint16_t us)
{
    cur_us = us;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, us);
}

void wrist_start(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);   /* chua phat xung: servo khong giat khi bat nguon */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    cur_us = 0;
    moving = 0;
}

uint8_t wrist_move_x10(int32_t deg_x10)
{
    if (deg_x10 < WRIST_MIN_X10 || deg_x10 > WRIST_MAX_X10) return 0;
    tgt_x10 = deg_x10;
    if (cur_us == 0) {                 /* chua biet servo dang o dau: nhay thang toi dich */
        cur_x10 = deg_x10;
        write_us(x10_to_us(deg_x10));
        moving = 0;
        return 1;
    }
    moving    = (tgt_x10 != cur_x10);
    last_tick = HAL_GetTick();
    return 1;
}

void wrist_update(void)
{
    if (!moving) return;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 20) return;
    uint32_t dt = now - last_tick;
    last_tick = now;

    int32_t step = (int32_t)(WRIST_DPS * 10L * (int32_t)dt / 1000L);   /* 0.1 do moi lan cap nhat */
    if (step < 1) step = 1;
    int32_t d = tgt_x10 - cur_x10;
    if (d >  step) d =  step;
    if (d < -step) d = -step;
    cur_x10 += d;
    write_us(x10_to_us(cur_x10));
    if (cur_x10 == tgt_x10) moving = 0;
}

void wrist_stop(void)
{
    tgt_x10 = cur_x10;
    moving  = 0;
}

void wrist_release(void)
{
    moving = 0;
    write_us(0);
}

void wrist_set_us(uint16_t us)
{
    moving = 0;
    if (us != 0 && us < WRIST_US_ABS_MIN) us = WRIST_US_ABS_MIN;
    if (us > WRIST_US_ABS_MAX) us = WRIST_US_ABS_MAX;
    write_us(us);
    /* xung tho: goc khong con dung, lenh goc ke tiep se nhay thang toi dich */
    if (us == 0) return;
    cur_x10 = tgt_x10 = (us >= WRIST_US_ZERO)
        ? (int32_t)(us - WRIST_US_ZERO) * 900 / ((WRIST_US_P90 - WRIST_US_ZERO) ? (WRIST_US_P90 - WRIST_US_ZERO) : 1)
        : -(int32_t)(WRIST_US_ZERO - us) * 900 / ((WRIST_US_ZERO - WRIST_US_M90) ? (WRIST_US_ZERO - WRIST_US_M90) : 1);
}

int32_t  wrist_get_x10(void) { return cur_x10; }
uint16_t wrist_get_us(void)  { return cur_us; }
uint8_t  wrist_busy(void)    { return moving; }
