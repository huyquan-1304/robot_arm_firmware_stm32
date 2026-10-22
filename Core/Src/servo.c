#include "servo.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;   /* do CubeMX tao trong main.c */

static uint16_t current_us = 0;   /* 0 = chua phat xung */
static uint16_t target_us  = 0;   /* dich cua chuyen dong cham */
static uint16_t step_us    = 0;   /* buoc moi 20 ms, 0 = khong chay cham */
static uint32_t last_tick  = 0;

static void write_us(uint16_t us)
{
    current_us = us;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}

static uint16_t clamp_us(uint16_t us)
{
    if (us == 0) return 0;
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    return us;
}

/* Khoi dong PWM nhung CHUA phat xung (compare = 0): servo dung yen, khong giat khi bat nguon.
 * Servo chi bat dau chay khi goi servo_set_us() hoac servo_set_deg(). */
void servo_start(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

/* us = 0: ngat xung, servo tha long (khong giu luc, khong nong khi dang kep vat cung) */
void servo_set_us(uint16_t us)
{
    us = clamp_us(us);
    step_us   = 0;            /* huy chuyen dong cham neu dang chay */
    target_us = us;
    write_us(us);
}

/* Chay cham toi 'us', moi 20 ms tien 'step' micro giay.
 * Neu servo dang tha long (chua biet vi tri) thi nhay thang toi dich. */
void servo_move_to(uint16_t us, uint16_t step)
{
    us = clamp_us(us);
    if (current_us == 0 || step == 0 || us == 0) { servo_set_us(us); return; }
    target_us = us;
    step_us   = step;
    last_tick = HAL_GetTick();
}

/* Goi lien tuc trong while(1). Tra ve 1 khi vua toi dich. */
uint8_t servo_update(void)
{
    if (step_us == 0) return 0;
    if (HAL_GetTick() - last_tick < 20) return 0;
    last_tick += 20;

    uint16_t c = current_us;
    if (c < target_us) c = (uint16_t)((target_us - c > step_us) ? c + step_us : target_us);
    else               c = (uint16_t)((c - target_us > step_us) ? c - step_us : target_us);
    write_us(c);

    if (c == target_us) { step_us = 0; return 1; }
    return 0;
}

void servo_set_deg(uint16_t deg)
{
    if (deg > 180) deg = 180;
    servo_set_us((uint16_t)(SERVO_US_MIN + ((uint32_t)deg * (SERVO_US_MAX - SERVO_US_MIN)) / 180U));
}

uint16_t servo_get_us(void)
{
    return current_us;
}
