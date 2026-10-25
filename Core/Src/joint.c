#include "joint.h"
#include <stddef.h>

/* ===================== BANG THONG SO KHOP =====================
 * Chinh o day khi thay doi co khi.
 * Toc do / gia toc tinh tai DAU RA khop. Khop 2 tai nang nen de cham va gia toc nho.
 * Quy doi sang dong co: RPM = (do/giay) * ti_so / 6   (vi 1 RPM = 6 do/giay)
 *
 *  Khop | Ti so | Gioi han (do) | Toc do khop | Gia toc khop | -> Dong co
 *   1   |  4    |  -90 .. +90   |  30 do/s    |  60 do/s2    |  20 RPM
 *   2   |  27   |    0 .. 90    |  15 do/s    |  15 do/s2    |  67.5 RPM
 *   3   |  14   |    0 .. 90    |  20 do/s    |  30 do/s2    |  46.7 RPM
 *   4   |  1    |    0 .. 90    |  45 do/s    |  90 do/s2    |  7.5 RPM
 */
static joint_t joints[JOINT_COUNT] = {
    /* addr ratio dir   min    max   dps_x10 acc_x10 offset target */
    {  1,   12,   +1,  -900,   900,   800,    1200,    0,     0 },
    {  2,   27,  +1,   -900,   900,    80,    100,    0,     0 },
    {  3,   14,  +1,   -900,   900,    250,     350,    0,     0 },
    {  4,   1,   +1,   -900,   900,    800,    2000,    0,     0 },
};

static zdt_status_t last_err = ZDT_OK;

/* ------------------------------------------------------------------ */

static int32_t joint_to_motor(const joint_t *j, int32_t deg_x10)
{
    return j->offset_x10 + (int32_t)j->dir * (int32_t)j->ratio * deg_x10;
}

/* chia lam tron ve so nguyen gan nhat (ca so am) */
static int32_t div_round(int32_t a, int32_t b)
{
    return (a >= 0) ? (a + b / 2) / b : -((-a + b / 2) / b);
}

static int32_t motor_to_joint(const joint_t *j, int32_t motor_x10)
{
    return div_round((motor_x10 - j->offset_x10) * j->dir, (int32_t)j->ratio);
}

/* do/s (x10) cua khop -> RPM (x10) cua dong co */
static uint16_t dps_to_rpm_x10(const joint_t *j, float dps)
{
    float rpm_x10 = dps * (float)j->ratio / 6.0f * 10.0f;
    if (rpm_x10 < 1.0f)     rpm_x10 = 1.0f;
    if (rpm_x10 > 30000.0f) rpm_x10 = 30000.0f;
    return (uint16_t)(rpm_x10 + 0.5f);
}

/* do/s^2 cua khop -> RPM/s cua dong co */
static uint16_t acc_to_rpm_s(const joint_t *j, float dps2)
{
    float a = dps2 * (float)j->ratio / 6.0f;
    if (a < 1.0f)     a = 1.0f;
    if (a > 65535.0f) a = 65535.0f;
    return (uint16_t)(a + 0.5f);
}

static joint_status_t drv(zdt_status_t s)
{
    last_err = s;
    return (s == ZDT_OK) ? JOINT_OK : JOINT_ERR_DRIVER;
}

/* ------------------------------------------------------------------ */

void joint_init(void)
{
    for (int i = 0; i < JOINT_COUNT; i++) joints[i].target_x10 = 0;
}

joint_t *joint_get(uint8_t n)
{
    if (n < 1 || n > JOINT_COUNT) return NULL;
    return &joints[n - 1];
}

const char *joint_status_str(joint_status_t s)
{
    switch (s) {
    case JOINT_OK:         return "OK";
    case JOINT_ERR_ARG:    return "so khop sai (1..4)";
    case JOINT_ERR_LIMIT:  return "ngoai gioi han goc";
    case JOINT_ERR_DRIVER: return zdt_status_str(last_err);
    default:               return "loi khac";
    }
}

zdt_status_t joint_last_driver_error(void) { return last_err; }

joint_status_t joint_move(uint8_t n, int32_t deg_x10)
{
    joint_t *j = joint_get(n);
    if (!j) return JOINT_ERR_ARG;
    if (deg_x10 < j->min_x10 || deg_x10 > j->max_x10) return JOINT_ERR_LIMIT;

    zdt_status_t s = zdt_move(j->addr, joint_to_motor(j, deg_x10),
                              dps_to_rpm_x10(j, j->dps_x10 / 10.0f),
                              acc_to_rpm_s(j, j->acc_x10 / 10.0f),
                              acc_to_rpm_s(j, j->acc_x10 / 10.0f),
                              ZDT_MOVE_ABSOLUTE);
    if (s == ZDT_OK) j->target_x10 = deg_x10;
    return drv(s);
}

joint_status_t joint_move_rel(uint8_t n, int32_t delta_x10)
{
    joint_t *j = joint_get(n);
    if (!j) return JOINT_ERR_ARG;
    return joint_move(n, j->target_x10 + delta_x10);
}

/* Dong bo thoi gian: moi khop i di quang duong d_i voi toc do v_i, gia toc a_i.
 *   t_a = max(v_i / a_i)        thoi gian tang toc chung (lay khop tang toc lau nhat)
 *   T   = max(d_i / v_i) + t_a  tong thoi gian chung (lay khop cham nhat)
 *   v_i' = d_i / (T - t_a)      toc do moi, luon <= v_i
 *   a_i' = v_i' / t_a           gia toc moi, luon <= a_i
 * => tat ca khop bat dau, tang toc, giam toc va toi noi cung luc, khong khop nao vuot gioi han toc do. */
joint_status_t joint_move_all(const int32_t deg_x10[JOINT_COUNT])
{
    float d[JOINT_COUNT], t_a = 0.0f, t_move = 0.0f;

    for (int i = 0; i < JOINT_COUNT; i++) {
        joint_t *j = &joints[i];
        if (deg_x10[i] < j->min_x10 || deg_x10[i] > j->max_x10) return JOINT_ERR_LIMIT;
        int32_t diff = deg_x10[i] - j->target_x10;
        d[i] = (float)((diff < 0) ? -diff : diff) / 10.0f;
        if (d[i] <= 0.0f) continue;
        float v = j->dps_x10 / 10.0f, a = j->acc_x10 / 10.0f;
        if (v / a > t_a)     t_a = v / a;
        if (d[i] / v > t_move) t_move = d[i] / v;
    }
    if (t_move <= 0.0f) return JOINT_OK;          /* khong khop nao can di chuyen */

    for (int i = 0; i < JOINT_COUNT; i++) {
        if (d[i] <= 0.0f) continue;
        joint_t *j = &joints[i];
        float v = d[i] / t_move;                   /* = d_i / (T - t_a) */
        float a = v / t_a;
        zdt_status_t s = zdt_move_ex(j->addr, joint_to_motor(j, deg_x10[i]),
                                     dps_to_rpm_x10(j, v), acc_to_rpm_s(j, a), acc_to_rpm_s(j, a),
                                     ZDT_MOVE_ABSOLUTE, 1);
        if (s != ZDT_OK) {
            joint_stop_all();                      /* huy cac lenh da cho de khong chay nham sau nay */
            return drv(s);
        }
    }
    zdt_status_t s = zdt_sync_trigger();
    if (s == ZDT_OK) {
        for (int i = 0; i < JOINT_COUNT; i++) joints[i].target_x10 = deg_x10[i];
    }
    return drv(s);
}

joint_status_t joint_read(uint8_t n, int32_t *deg_x10)
{
    joint_t *j = joint_get(n);
    if (!j) return JOINT_ERR_ARG;
    int32_t m;
    zdt_status_t s = zdt_read_position(j->addr, &m);
    if (s == ZDT_OK) *deg_x10 = motor_to_joint(j, m);
    return drv(s);
}

joint_status_t joint_set_speed(uint8_t n, uint16_t dps_x10, uint16_t acc_x10)
{
    joint_t *j = joint_get(n);
    if (!j || dps_x10 == 0) return JOINT_ERR_ARG;
    j->dps_x10 = dps_x10;
    if (acc_x10 != 0) j->acc_x10 = acc_x10;
    return JOINT_OK;
}

joint_status_t joint_zero_here(uint8_t n)
{
    joint_t *j = joint_get(n);
    if (!j) return JOINT_ERR_ARG;
    zdt_status_t s = zdt_zero_position(j->addr);
    if (s == ZDT_OK) { j->offset_x10 = 0; j->target_x10 = 0; }
    return drv(s);
}

joint_status_t joint_enable(uint8_t n, int on)
{
    if (n == 0) {
        joint_status_t r = JOINT_OK;
        for (uint8_t i = 1; i <= JOINT_COUNT; i++) {
            if (joint_enable(i, on) != JOINT_OK) r = JOINT_ERR_DRIVER;
        }
        return r;
    }
    joint_t *j = joint_get(n);
    if (!j) return JOINT_ERR_ARG;
    return drv(zdt_enable(j->addr, on));
}

void joint_stop_all(void)
{
    for (int i = 0; i < JOINT_COUNT; i++) (void)zdt_stop(joints[i].addr);
}

joint_status_t joint_home(uint8_t n)
{
    (void)n;
    /* TODO khi co cong tac hanh trinh:
     *  1. chay cham ve phia cong tac (che do toc do hoac lenh ve 0 cua driver)
     *  2. khi cong tac tac dong: dung, zdt_zero_position(), dat offset neu cong tac khong nam o 0 do
     *  3. lui ra mot doan nho */
    return JOINT_ERR_ARG;
}
