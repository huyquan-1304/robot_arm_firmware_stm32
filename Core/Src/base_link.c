#include "base_link.h"
#include "ros_link.h"
#include "motor.h"
#include "mecanum.h"
#include "main.h"

#define TWO_PI_F        6.28318531f
#define MRADS_PER_RPM   (TWO_PI_F * 1000.0f / 60.0f)   /* 104.72 mrad/s cho 1 RPM */
#define RAMP_DT_MAX_MS  50u                            /* vong lap bi cham (vd home): khong nhay buoc lon */

static uint8_t  enabled   = 0;
static uint8_t  wd_trip   = 0;
static uint8_t  saturated = 0;
static uint8_t  estop     = 0;
static uint16_t last_seq  = 0;
static uint32_t last_cmd_tick   = 0;
static uint32_t last_state_tick = 0;
static uint32_t last_ramp_tick  = 0;
static uint32_t i2c_err_seen    = 0;

/* Toc do dich (sau khi gioi han) va toc do dang ra lenh cho PID (sau bo tang toc), RPM */
static float tgt[MOT_COUNT];
static float cur[MOT_COUNT];

static float absf(float x) { return (x < 0.0f) ? -x : x; }

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_i32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t  get_i16(const uint8_t *p) { return (int16_t)get_u16(p); }

static int16_t sat_i16(float x)
{
    if (x > 32767.0f)  return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)(x + ((x < 0.0f) ? -0.5f : 0.5f));
}

/* Dung ngay, khong qua bo tang toc (ESTOP, tat, thoat che do ROS) */
static void hard_stop(void)
{
    mecanum_stop();
    for (int i = 0; i < MOT_COUNT; i++) tgt[i] = cur[i] = 0.0f;
}

/* ------------------------------------------------------------------ */

void base_link_reset(void)
{
    hard_stop();
    enabled = wd_trip = saturated = estop = 0;
    last_seq = 0;
    last_cmd_tick = last_state_tick = last_ramp_tick = HAL_GetTick();
    i2c_err_seen = mot_i2c_errors();
}

void base_link_stop(void)
{
    hard_stop();
    enabled = 0;
}

void base_link_estop(void)
{
    hard_stop();
    enabled = 0;
    estop   = 1;
}

static void on_vel(const uint8_t *p)
{
    last_seq = get_u16(&p[0]);
    if (!enabled) return;

    float rpm[MOT_COUNT], rmax = 0.0f;
    for (int i = 0; i < MOT_COUNT; i++) {
        rpm[i] = (float)get_i16(&p[2 + 2 * i]) / MRADS_PER_RPM;
        if (absf(rpm[i]) > rmax) rmax = absf(rpm[i]);
    }
    /* Vuot gioi han: thu nho DEU ca 4 banh de giu dung huong chuyen dong cua xe */
    float k = 1.0f;
    saturated = 0;
    if (rmax > BASE_RPM_LIMIT) { k = BASE_RPM_LIMIT / rmax; saturated = 1; }

    for (int i = 0; i < MOT_COUNT; i++) tgt[i] = rpm[i] * k;   /* bo tang toc trong base_link_task dua cur toi tgt */
    last_cmd_tick = HAL_GetTick();
    wd_trip = 0;
}

static void on_enable(uint8_t on)
{
    if (!on) { base_link_stop(); return; }
    for (int i = 0; i < MOT_COUNT; i++) {
        mot_clear_fault((mot_id_t)i);
        tgt[i] = cur[i] = 0.0f;
        mot_set_rpm((mot_id_t)i, 0.0f);   /* bat dau tu dung yen, PID giu 0 */
    }
    estop = wd_trip = saturated = 0;
    last_cmd_tick = last_ramp_tick = HAL_GetTick();
    enabled = 1;
}

int base_link_handle(uint8_t type, const uint8_t *p, uint8_t len)
{
    switch (type) {
    case T_CMD_BASE_VEL:
        if (len == LEN_CMD_BASE_VEL) on_vel(p);
        return 1;
    case T_CMD_BASE_ENABLE:
        if (len == 1u) on_enable(p[0]);
        return 1;
    case T_CMD_BASE_PING:
        return 1;
    default:
        return 0;
    }
}

/* Bo tang toc: dua cur toi tgt voi gia toc banh toi da BASE_ACC_RPM_S.
 * Ca 4 banh thay doi CUNG TI LE (banh can doi nhieu nhat dung gia toc toi da, cac banh khac cham hon theo ti le),
 * nen trong luc tang / giam toc, xe van di dung huong cua lenh. */
static void ramp_step(uint32_t now)
{
    uint32_t dt_ms = now - last_ramp_tick;
    last_ramp_tick = now;
    if (dt_ms == 0u) return;
    if (dt_ms > RAMP_DT_MAX_MS) dt_ms = RAMP_DT_MAX_MS;

    float d[MOT_COUNT], dmax = 0.0f;
    for (int i = 0; i < MOT_COUNT; i++) {
        d[i] = tgt[i] - cur[i];
        if (absf(d[i]) > dmax) dmax = absf(d[i]);
    }
    if (dmax == 0.0f) return;

    float step = BASE_ACC_RPM_S * (float)dt_ms / 1000.0f;
    float k = (dmax > step) ? step / dmax : 1.0f;
    for (int i = 0; i < MOT_COUNT; i++) {
        cur[i] = (k < 1.0f) ? cur[i] + d[i] * k : tgt[i];
        mot_set_rpm((mot_id_t)i, cur[i]);
    }
}

static void send_state(uint8_t i2c_err)
{
    uint8_t  b[LEN_STATE_BASE];
    uint16_t st = 0;

    put_u16(&b[0], last_seq);
    for (int i = 0; i < MOT_COUNT; i++) {
        put_i32(&b[2 + 4 * i], mot_get_counts((mot_id_t)i));
        put_u16(&b[18 + 2 * i], (uint16_t)sat_i16(mot_get_rpm((mot_id_t)i) * MRADS_PER_RPM));
        if (mot_fault((mot_id_t)i)) st |= (uint16_t)(BASE_ST_FAULT_FL << i);
    }
    if (enabled)   st |= BASE_ST_ENABLED;
    if (wd_trip)   st |= BASE_ST_WATCHDOG;
    if (saturated) st |= BASE_ST_SATURATED;
    if (i2c_err)   st |= BASE_ST_I2C_ERROR;
    if (estop)     st |= BASE_ST_ESTOP;
    put_u16(&b[26], st);

    ros_link_send(T_STATE_BASE, b, LEN_STATE_BASE);
}

void base_link_task(void)
{
    uint32_t now = HAL_GetTick();

    if (enabled) {
        /* Watchdog: Pi ngung gui lenh toc do -> giam toc ve 0 theo bo tang toc */
        if (!wd_trip && (now - last_cmd_tick) > BASE_WATCHDOG_MS) {
            for (int i = 0; i < MOT_COUNT; i++) tgt[i] = 0.0f;
            wd_trip   = 1;
            saturated = 0;
        }
        ramp_step(now);
    } else {
        last_ramp_tick = now;
    }

    if ((now - last_state_tick) < BASE_CYCLE_MS) return;
    last_state_tick = now;

    uint32_t e = mot_i2c_errors();
    uint8_t  i2c_err = (e != i2c_err_seen);
    i2c_err_seen = e;

    send_state(i2c_err);
}
