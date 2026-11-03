#include "home.h"
#include "zdt_x42s.h"
#include "main.h"
#include <stdio.h>

/* ===================== BANG CAU HINH HOME BANG DRIVER =====================
 *   en       : 0 = khong cho home khop nay
 *   dir      : chieu goc khop di TOI cam bien tu tu the lam viec (+1 goc tang, -1 goc giam). Kiem tra bang "lsj"
 *   cw_pos   : +1 = lenh CW cua driver lam vi tri dong co TANG, -1 = GIAM, 0 = CHUA DO. Do bang "hcw <n>"
 *   home_x10 : goc khop (0.1 do) tai cho DRIVER DUNG khi home theo chieu dir, tinh tu GIUA vung kim loai.
 *              Do bang "hcal <n>". Xap xi = -dir * band / 2
 *   band_x10 : be rong vung kim loai (0.1 do). Do bang "hcal <n>"
 *   rpm      : toc do home cua driver, tinh tai TRUC DONG CO (RPM). Toc do khop = rpm * 6 / ti_so (do/s)
 *   fast     : toc do STM32 tim cam bien (0.1 do/s tai khop), dung khi HOME_USE_DRIVER = 0
 *   slow     : toc do STM32 nhich cham (0.1 do/s tai khop): ra khoi cam bien, do be rong, vao giua
 *   travel   : quang tim theo chieu dir toi da (0.1 do tai khop)
 *   rev      : khong thay thi tim nguoc toi da bay nhieu, tinh tu cho bat dau (0.1 do). 0 = khong tim nguoc
 */
typedef struct {
    uint8_t  enabled;
    int8_t   dir;
    int8_t   cw_pos;
    int32_t  home_x10;
    uint16_t band_x10;
    uint16_t rpm;
    uint16_t fast_dps_x10;
    uint16_t slow_dps_x10;
    uint16_t travel_x10;
    uint16_t rev_x10;
} home_cfg_t;

static const home_cfg_t cfg[JOINT_COUNT] = {
    /* dir, band: giu tu lan do hcal 2026-10-24. home_x10 TAM = -dir * band / 2, PHAI DO LAI bang hcal.
     * cw_pos = 0: CHUA DO, chay "hcw <n>" truoc. */
    /* home_x10 = -dir * band / 2: mep gan cach giua vung kim loai nua be rong (so do hcal 2026-10-24) */
    /* en dir cw  home_x10 band rpm fast slow travel rev */
    {  1, +1,  0,    -33,   66,  16,  80,  30,  400,  400 },   /* khop 1: tim 8 do/s   */
    {  1, -1,  0,     27,   53,  14,  30,  20,  500,  100 },   /* khop 2: tim 3 do/s   */
    {  1, +1,  0,    -33,   66,  19,  80,  30,  500,  300 },   /* khop 3: tim 8 do/s   */
    {  1, +1,  0,    -50,  101,   2, 100,  50,  400,  400 },   /* khop 4: tim 10 do/s  */
};

/* Cach home:
 *   0 = MAC DINH. Menu driver: O_Mode = Nearest (KHONG de EndStop: EndStop lam lenh 3D doc sai chan En).
 *   2 = DRIVER TU HOME THEO THONG SO TREN MENU. STM32 chi gui lenh 9A, KHONG gui 4C.
 *       Menu tung driver: O_Mode = EndStop, O_Dir = chieu TOI cam bien, O_Vel = 30, O_POT_En = Disable.
 *       Cot rpm trong bang = O_Vel tren menu (chi dung de tinh thoi gian cho). Khong can hcw, khong doc 3D.
 *   1 = driver home, STM32 gui thong so qua 4C (can cw_pos). Driver khong chay o 16 RPM.
 *   0 = STM32 tu tim, doc chan En qua lenh 3D. Lenh 3D doc khong tin cay tren driver nay. */
#define HOME_MODE         0
#define HOME_USE_DRIVER   (HOME_MODE == 1)

/* Thu tu home tat ca: tu co tay vao goc, de luc khop 1 xoay thi canh tay da thu gon */
static const uint8_t HOME_ORDER[JOINT_COUNT] = { 4, 3, 2, 1 };

#define EN_ACTIVE_LOW     1      /* chan En = 0 V nghia la cam bien tac dong (driver cung hieu nhu vay) */
#define BAND_MAX_X10      200    /* be rong toi da cho phep cua vung kim loai (0.1 do) */
#define EXIT_MARGIN_X10   20     /* ra khoi cam bien roi lui them 2 do truoc khi home */
#define POLL_MS           30     /* chu ky doc co home cua driver */
#define STALL_SKIP_MS     2000
#define STALL_WINDOW_MS   500
#define STOP_ACC_RPM_S    1000
#define HOME_SPEED_ACC    300

/* ------------------------------------------------------------------ */

static home_abort_fn abort_cb = 0;
static uint8_t homed[JOINT_COUNT];

void home_init(home_abort_fn abort_fn)
{
    abort_cb = abort_fn;
    for (uint8_t i = 0; i < JOINT_COUNT; i++) homed[i] = 0;
}

void home_sensor_update(void) { }

/* Doc 1 lan muc chan En. Tra ve 1 neu doc duoc */
static int read_en(uint8_t n, uint8_t *level)
{
    uint8_t io;
    if (zdt_read_io(joint_get(n)->addr, &io) != ZDT_OK) return 0;
    *level = (io & ZDT_IO_EN_PIN) ? 1 : 0;   /* bit 3D = muc chan En (ban test luc 23:27: luc bat nguon bao tu do) */
    return 1;
}

static uint8_t level_to_active(uint8_t level) { return EN_ACTIVE_LOW ? !level : level; }

/* Doc trang thai cam bien 2 lan cach 2 ms, 2 lan giong nhau moi tin. *act = 1 tac dong */
static int read_sensor(uint8_t n, uint8_t *act)
{
    uint8_t a, b;
    for (uint8_t k = 0; k < 3; k++) {
        if (!read_en(n, &a)) return 0;
        HAL_Delay(2);
        if (!read_en(n, &b)) return 0;
        if (a == b) { *act = level_to_active(a); return 1; }
    }
    return 0;
}

uint8_t home_sensor(uint8_t n)
{
    uint8_t act;
    if (n < 1 || n > JOINT_COUNT) return 0;
    return read_sensor(n, &act) ? act : 0;
}

uint8_t home_pin_level(uint8_t n)
{
    uint8_t lv;
    if (n < 1 || n > JOINT_COUNT) return 0;
    return read_en(n, &lv) ? lv : 0;
}

uint8_t home_is_homed(uint8_t n)
{
    return (n >= 1 && n <= JOINT_COUNT) ? homed[n - 1] : 0;
}

const char *home_status_str(home_status_t s)
{
    switch (s) {
    case HOME_OK:             return "OK";
    case HOME_ERR_ARG:        return "so khop sai";
    case HOME_ERR_NOT_CONFIG: return "khop chua cau hinh (en = 0, hoac cw_pos = 0: chay hcw)";
    case HOME_ERR_DRIVER:     return "driver khong tra loi / tu choi lenh (kiem tra P_Pul = ESI_RCO)";
    case HOME_ERR_NOT_FOUND:  return "het quang duong ma khong thay cam bien (sai chieu? sai cw_pos? dut day cam bien NO?)";
    case HOME_ERR_STUCK_ON:   return "cam bien luon tac dong (cam bien NC noi thang vao En? dut day? lap sai?)";
    case HOME_ERR_STALL:      return "dong co khong nhuc nhich (ket co khi?)";
    case HOME_ERR_TIMEOUT:    return "qua thoi gian";
    case HOME_ERR_ABORT:      return "da huy";
    default:                  return "loi khac";
    }
}

/* ------------------------------------------------------------------ */

static int user_abort(void) { return abort_cb ? abort_cb() : 0; }
static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

static int32_t div_round(int32_t a, int32_t b)
{
    return (a >= 0) ? (a + b / 2) / b : -((-a + b / 2) / b);
}

static int32_t rpm_x10_of(const joint_t *j, uint16_t dps_x10)
{
    int32_t v = ((int32_t)dps_x10 * j->ratio + 3) / 6;
    if (v < 1) v = 1;
    if (v > 30000) v = 30000;
    return v;
}

static uint16_t acc_of(const joint_t *j)
{
    int32_t a = ((int32_t)j->acc_x10 * j->ratio + 30) / 60;
    if (a < HOME_SPEED_ACC) a = HOME_SPEED_ACC;
    if (a > 65535) a = 65535;
    return (uint16_t)a;
}

static int32_t joint_of(const joint_t *j, int32_t m)
{
    return div_round((m - j->offset_x10) * j->dir, (int32_t)j->ratio);
}

static int wait_ms(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        if (user_abort()) return 0;
    }
    return 1;
}

static void stop_joint(const joint_t *j)
{
    (void)zdt_speed(j->addr, 0, STOP_ACC_RPM_S);
    (void)zdt_stop(j->addr);
    HAL_Delay(150);
}

/* STM32 tu chay che do toc do, doc cam bien qua CAN, DUNG khi cam bien == want.
 * Chi dung cho doan cham (ra khoi cam bien, do be rong, lsj). */
static home_status_t seek_dir(uint8_t n, int8_t joint_dir, uint16_t dps_x10, uint8_t want,
                              uint32_t max_x10, int32_t *m_hit)
{
    joint_t *j = joint_get(n);
    int32_t m0, m, m_win, m_first = 0;
    uint8_t lv, hits = 0;

    if (zdt_read_position(j->addr, &m0) != ZDT_OK) return HOME_ERR_DRIVER;

    int32_t  limit_m  = (int32_t)max_x10 * j->ratio;
    int32_t  expect_m = (int32_t)dps_x10 * j->ratio * STALL_WINDOW_MS / 1000 / 4;
    uint32_t timeout  = (uint32_t)max_x10 * 1000u / dps_x10 * 2u + 3000u;
    int32_t  v        = rpm_x10_of(j, dps_x10) * j->dir * joint_dir;

    if (zdt_speed(j->addr, v, acc_of(j)) != ZDT_OK) return HOME_ERR_DRIVER;

    uint32_t t0 = HAL_GetTick(), t_win = t0;
    m_win = m0;

    for (;;) {
        if (zdt_read_position(j->addr, &m) != ZDT_OK || !read_en(n, &lv)) {
            stop_joint(j);
            return HOME_ERR_DRIVER;
        }
        if (level_to_active(lv) == want) {
            if (hits == 0) m_first = m;          /* lay vi tri o lan doc dau tien thay doi */
            if (++hits >= 2) {
                stop_joint(j);
                *m_hit = m_first;
                return HOME_OK;
            }
        } else {
            hits = 0;
        }
        if (user_abort()) { stop_joint(j); return HOME_ERR_ABORT; }

        uint32_t now = HAL_GetTick();
        if (now - t0 > timeout) { stop_joint(j); return HOME_ERR_TIMEOUT; }
        if (iabs(m - m0) > limit_m) {
            stop_joint(j);
            return want ? HOME_ERR_NOT_FOUND : HOME_ERR_STUCK_ON;
        }
        if (now - t0 > STALL_SKIP_MS && now - t_win >= STALL_WINDOW_MS) {
            if (iabs(m - m_win) < expect_m) { stop_joint(j); return HOME_ERR_STALL; }
            m_win = m;
            t_win = now;
        }
    }
}

/* Chay khop toi goc joint_x10 theo offset hien tai, khong kiem tra gioi han joint.c, cho toi noi */
static home_status_t move_to(uint8_t n, int32_t joint_x10, uint16_t dps_x10)
{
    joint_t *j = joint_get(n);
    int32_t mt = j->offset_x10 + (int32_t)j->dir * (int32_t)j->ratio * joint_x10;
    int32_t m0, m;
    if (zdt_read_position(j->addr, &m0) != ZDT_OK) return HOME_ERR_DRIVER;
    if (zdt_move(j->addr, mt, (uint16_t)rpm_x10_of(j, dps_x10), acc_of(j), acc_of(j),
                 ZDT_MOVE_ABSOLUTE) != ZDT_OK) return HOME_ERR_DRIVER;

    int32_t  tol     = (int32_t)j->ratio * 3;
    if (tol < 10) tol = 10;
    uint32_t dist    = (uint32_t)iabs(mt - m0) / j->ratio;
    uint32_t timeout = dist * 1000u / dps_x10 * 2u + 3000u;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout) {
        if (!wait_ms(20)) { stop_joint(j); return HOME_ERR_ABORT; }
        if (zdt_read_position(j->addr, &m) != ZDT_OK) return HOME_ERR_DRIVER;
        if (iabs(m - mt) <= tol) { j->target_x10 = joint_x10; return HOME_OK; }
    }
    stop_joint(j);
    return HOME_ERR_TIMEOUT;
}

/* Dang dung tren cam bien: nhich cham nguoc chieu dir toi khi nha, roi lui them EXIT_MARGIN */
static home_status_t leave_sensor(uint8_t n, int8_t dir)
{
    const home_cfg_t *c = &cfg[n - 1];
    joint_t *j = joint_get(n);
    uint8_t act;
    int32_t m;

    if (!read_sensor(n, &act)) return HOME_ERR_DRIVER;
    if (!act) return HOME_OK;

    printf("  Dang tren cam bien, nhich ra...\r\n");
    uint32_t lim = c->band_x10 ? (uint32_t)c->band_x10 + 50u : BAND_MAX_X10;
    home_status_t st = seek_dir(n, (int8_t)-dir, c->slow_dps_x10, 0, lim, &m);
    if (st != HOME_OK) return st;
    return move_to(n, joint_of(j, m) - (int32_t)dir * EXIT_MARGIN_X10, c->slow_dps_x10);
}

/* Cho driver home theo chieu joint_dir, gioi han max_x10. *m_stop = vi tri dong co luc driver dung */
static home_status_t driver_home(uint8_t n, int8_t joint_dir, uint32_t max_x10, int32_t *m_stop)
{
    const home_cfg_t *c = &cfg[n - 1];
    joint_t *j = joint_get(n);
    int32_t  m0, m;
    uint8_t  reply, f, seen_busy = 0;

    /* chieu dong co can quay = dir cua joint.c x chieu khop. Doi ra CW/CCW theo cw_pos */
    int8_t  motor_sign = (int8_t)(j->dir * joint_dir);
    uint8_t ccw        = (motor_sign == c->cw_pos) ? 0 : 1;

    /* thoi gian = quang duong tai truc dong co / toc do, nhan 1.5 cong 2 s */
    uint32_t t_need  = (uint32_t)max_x10 * j->ratio * 100u / ((uint32_t)c->rpm * 6u);    /* ms */
    uint32_t drv_tmo = t_need * 3u / 2u + 2000u;

    if (zdt_read_position(j->addr, &m0) != ZDT_OK) return HOME_ERR_DRIVER;
    if (zdt_home_set_params(j->addr, ZDT_HOME_MODE_ENDSTOP, ccw, c->rpm, drv_tmo, 0) != ZDT_OK)
        return HOME_ERR_DRIVER;
    if (zdt_home_trigger(j->addr, ZDT_HOME_MODE_ENDSTOP, &reply) != ZDT_OK) return HOME_ERR_DRIVER;
    if (reply == 0x12 || reply == 0x22) return HOME_ERR_STUCK_ON;

    uint32_t t0 = HAL_GetTick();
    int32_t  limit_m = (int32_t)(max_x10 + 50u) * j->ratio;   /* STM32 tu chan them neu driver chay qua */

    for (;;) {
        if (!wait_ms(POLL_MS)) { (void)zdt_home_abort(j->addr); stop_joint(j); return HOME_ERR_ABORT; }
        if (zdt_read_home_flags(j->addr, &f) != ZDT_OK) { (void)zdt_home_abort(j->addr); return HOME_ERR_DRIVER; }
        if (zdt_read_position(j->addr, &m) != ZDT_OK)   { (void)zdt_home_abort(j->addr); return HOME_ERR_DRIVER; }

        if (f & ZDT_HOME_BUSY) seen_busy = 1;
        uint32_t el = HAL_GetTick() - t0;

        if (iabs(m - m0) > limit_m) { (void)zdt_home_abort(j->addr); stop_joint(j); return HOME_ERR_NOT_FOUND; }
        if (el > drv_tmo + 2000u)   { (void)zdt_home_abort(j->addr); stop_joint(j); return HOME_ERR_TIMEOUT; }

        if (!(f & ZDT_HOME_BUSY) && (seen_busy || el > 300u)) {
            if (f & ZDT_HOME_FAIL) return HOME_ERR_NOT_FOUND;
            HAL_Delay(100);                                  /* cho dong co dung han roi doc lai */
            if (zdt_read_position(j->addr, &m) != ZDT_OK) return HOME_ERR_DRIVER;
            *m_stop = m;
            return HOME_OK;
        }
    }
}

/* ------------------------------------------------------------------ */

home_status_t home_probe_cw(uint8_t n, int8_t *cw_pos)
{
    joint_t *j = joint_get(n);
    if (!j) return HOME_ERR_ARG;
    const home_cfg_t *c = &cfg[n - 1];
    int32_t m0, m1;
    uint8_t act, reply;

    if (!read_sensor(n, &act)) return HOME_ERR_DRIVER;
    if (act) return HOME_ERR_STUCK_ON;

    if (zdt_read_position(j->addr, &m0) != ZDT_OK) return HOME_ERR_DRIVER;
    if (zdt_home_set_params(j->addr, ZDT_HOME_MODE_ENDSTOP, 0, c->rpm, 3000, 0) != ZDT_OK) return HOME_ERR_DRIVER;
    if (zdt_home_trigger(j->addr, ZDT_HOME_MODE_ENDSTOP, &reply) != ZDT_OK) return HOME_ERR_DRIVER;
    if (reply == 0x12 || reply == 0x22) return HOME_ERR_STUCK_ON;

    int ok = wait_ms(500);
    (void)zdt_home_abort(j->addr);
    stop_joint(j);
    if (!ok) return HOME_ERR_ABORT;
    if (zdt_read_position(j->addr, &m1) != ZDT_OK) return HOME_ERR_DRIVER;

    int32_t d = m1 - m0;
    if (iabs(d) < 5) return HOME_ERR_STALL;          /* duoi 0.5 do tai truc dong co */
    *cw_pos = (d > 0) ? 1 : -1;
    j->target_x10 = joint_of(j, m1);
    return HOME_OK;
}

home_status_t home_jog_to_sensor(uint8_t n, int8_t joint_dir, int32_t *joint_x10)
{
    joint_t *j = joint_get(n);
    if (!j || (joint_dir != 1 && joint_dir != -1)) return HOME_ERR_ARG;
    const home_cfg_t *c = &cfg[n - 1];
    int32_t m_hit;
    uint8_t act;

    if (!read_sensor(n, &act)) return HOME_ERR_DRIVER;
    if (act) return HOME_ERR_STUCK_ON;

    uint16_t v = (uint16_t)((uint32_t)c->rpm * 60u / j->ratio);   /* toc do khop tuong duong, 0.1 do/s */
    if (v < 10) v = 10;
    home_status_t st = seek_dir(n, joint_dir, v, 1, c->travel_x10, &m_hit);
    if (st != HOME_OK) return st;
    *joint_x10 = joint_of(j, m_hit);
    j->target_x10 = *joint_x10;
    return HOME_OK;
}

/* hcal: driver home theo dir (diem dung S), STM32 di tiep cham qua vung kim loai toi khi nha (E).
 * Giua vung kim loai = (S + E) / 2. home_x10 = goc cua S tinh tu giua. Khong doi offset. */
home_status_t home_measure(uint8_t n, int8_t dir_override, int32_t *home_x10, int32_t *band_x10)
{
    joint_t *j = joint_get(n);
    if (!j) return HOME_ERR_ARG;
    const home_cfg_t *c = &cfg[n - 1];
    int8_t dir = (dir_override == 1 || dir_override == -1) ? dir_override : c->dir;
    int32_t m_s, m_e;
    home_status_t st;

#if !HOME_USE_DRIVER
    /* STM32 tu do: mep gan (tu ngoai vao), di qua vung kim loai, quay lai tim mep xa (tu phia kia vao) */
    int32_t m_out, m_far;
    st = leave_sensor(n, dir);
    if (st != HOME_OK) return st;
    printf("  1/3 Tim mep gan...\r\n");
    st = seek_dir(n, dir, c->fast_dps_x10, 1, c->travel_x10, &m_s);
    if (st != HOME_OK) return st;
    printf("  2/3 Di qua vung kim loai...\r\n");
    st = seek_dir(n, dir, c->slow_dps_x10, 0, BAND_MAX_X10, &m_out);
    if (st != HOME_OK) return st;
    if (!wait_ms(200)) return HOME_ERR_ABORT;
    printf("  3/3 Quay lai tim mep xa...\r\n");
    st = seek_dir(n, (int8_t)-dir, c->slow_dps_x10, 1, BAND_MAX_X10, &m_far);
    if (st != HOME_OK) return st;
    {
        int32_t m_mid = m_s + (m_far - m_s) / 2;
        *home_x10 = div_round((m_s - m_mid) * j->dir, (int32_t)j->ratio);
        *band_x10 = div_round(iabs(m_far - m_s), (int32_t)j->ratio);
        j->target_x10 = joint_of(j, m_far);
    }
    return HOME_OK;
#endif

    if (c->cw_pos == 0) return HOME_ERR_NOT_CONFIG;

    st = leave_sensor(n, dir);
    if (st != HOME_OK) return st;

    printf("  1/2 Driver home toi cam bien...\r\n");
    st = driver_home(n, dir, c->travel_x10, &m_s);
    if (st != HOME_OK) return st;
    if (!wait_ms(200)) return HOME_ERR_ABORT;

    printf("  2/2 Di cham qua vung kim loai...\r\n");
    st = seek_dir(n, dir, c->slow_dps_x10, 0, BAND_MAX_X10, &m_e);
    if (st != HOME_OK) return st;

    int32_t m_mid = m_s + (m_e - m_s) / 2;
    *home_x10 = div_round((m_s - m_mid) * j->dir, (int32_t)j->ratio);
    *band_x10 = div_round(iabs(m_e - m_s), (int32_t)j->ratio);
    j->target_x10 = joint_of(j, m_e);
    return HOME_OK;
}

/* HOME_MODE 2: driver tu home theo thong so menu. Chi gui 9A, theo doi co 3B va quang duong */
static home_status_t driver_home_menu(uint8_t n, uint32_t max_x10, int32_t *m_stop, uint8_t *on_sensor)
{
    const home_cfg_t *c = &cfg[n - 1];
    joint_t *j = joint_get(n);
    int32_t  m0, m;
    uint8_t  reply, f, seen_busy = 0;
    uint32_t t_need = (uint32_t)max_x10 * j->ratio * 100u / ((uint32_t)c->rpm * 6u);   /* ms */
    uint32_t tmo    = t_need * 3u / 2u + 4000u;

    *on_sensor = 0;
    if (zdt_read_position(j->addr, &m0) != ZDT_OK) return HOME_ERR_DRIVER;
    if (zdt_home_trigger(j->addr, ZDT_HOME_MODE_ENDSTOP, &reply) != ZDT_OK) return HOME_ERR_DRIVER;
    if (reply == 0x12 || reply == 0x22) { *on_sensor = 1; return HOME_OK; }

    uint32_t t0 = HAL_GetTick();
    int32_t  limit_m = (int32_t)(max_x10 + 50u) * j->ratio;
    for (;;) {
        if (!wait_ms(POLL_MS)) { (void)zdt_home_abort(j->addr); stop_joint(j); return HOME_ERR_ABORT; }
        if (zdt_read_home_flags(j->addr, &f) != ZDT_OK) { (void)zdt_home_abort(j->addr); return HOME_ERR_DRIVER; }
        if (zdt_read_position(j->addr, &m) != ZDT_OK)   { (void)zdt_home_abort(j->addr); return HOME_ERR_DRIVER; }
        if (f & ZDT_HOME_BUSY) seen_busy = 1;
        uint32_t el = HAL_GetTick() - t0;
        /* driver co the dat lai vi tri ve 0 khi home xong: chi xet quang duong khi dang home */
        if ((f & ZDT_HOME_BUSY) && iabs(m - m0) > limit_m) {
            (void)zdt_home_abort(j->addr); stop_joint(j); return HOME_ERR_NOT_FOUND;
        }
        if (el > tmo) { (void)zdt_home_abort(j->addr); stop_joint(j); return HOME_ERR_TIMEOUT; }
        if (iabs(m - m0) > 5) seen_busy = 1;                /* home xong qua nhanh, truoc lan doc dau */
        if (!(f & ZDT_HOME_BUSY) && (seen_busy || el > 500u)) {
            if (f & ZDT_HOME_FAIL) return HOME_ERR_NOT_FOUND;
            if (!seen_busy) return HOME_ERR_STALL;          /* driver khong chay home */
            HAL_Delay(150);
            if (zdt_read_position(j->addr, &m) != ZDT_OK) return HOME_ERR_DRIVER;
            *m_stop = m;
            return HOME_OK;
        }
    }
}

home_status_t home_joint(uint8_t n)
{
    joint_t *j = joint_get(n);
    if (!j) return HOME_ERR_ARG;
    const home_cfg_t *c = &cfg[n - 1];
    if (!c->enabled) return HOME_ERR_NOT_CONFIG;
#if HOME_MODE == 2
    {
        homed[n - 1] = 0;
        printf("Home khop %u (driver, thong so menu):\r\n", n);
        int32_t m_stop = 0, m;
        uint8_t on;
        home_status_t st = driver_home_menu(n, c->travel_x10, &m_stop, &on);
        if (st == HOME_OK && on) {
            /* Driver bao dang o tren cam bien: lui nguoc chieu dir ra khoi vung kim loai roi home lai */
            printf("  Dang tren cam bien, lui ra...\r\n");
            if (zdt_read_position(j->addr, &m) != ZDT_OK) return HOME_ERR_DRIVER;
            st = move_to(n, joint_of(j, m) - (int32_t)c->dir * ((int32_t)c->band_x10 + 50), c->slow_dps_x10);
            if (st != HOME_OK) return st;
            st = driver_home_menu(n, c->travel_x10, &m_stop, &on);
            if (st == HOME_OK && on) return HOME_ERR_STUCK_ON;
        }
        if (st != HOME_OK) return st;

        /* Cho driver dung = mep gan cua vung kim loai, goc khop tai do = home_x10 */
        j->offset_x10 = m_stop - (int32_t)j->dir * (int32_t)j->ratio * c->home_x10;
        j->target_x10 = c->home_x10;
        printf("  Ve goc 0 (giua cam bien)...\r\n");
        st = move_to(n, 0, c->slow_dps_x10);
        if (st != HOME_OK) return st;
        homed[n - 1] = 1;
        return HOME_OK;
    }
#endif
#if HOME_USE_DRIVER
    if (c->cw_pos == 0) return HOME_ERR_NOT_CONFIG;
#endif

    homed[n - 1] = 0;
    printf("Home khop %u:\r\n", n);

    int32_t m_stop;
    uint8_t far = 0;
    home_status_t st;
    uint8_t act;

    if (!read_sensor(n, &act)) return HOME_ERR_DRIVER;
    if (act) {
        /* Dang tren kim loai: nhich cham nguoc chieu dir toi khi nha = mep gan */
        printf("  Dang tren cam bien, nhich ra mep...\r\n");
        uint32_t lim = c->band_x10 ? (uint32_t)c->band_x10 + 50u : BAND_MAX_X10;
        st = seek_dir(n, (int8_t)-c->dir, c->slow_dps_x10, 0, lim, &m_stop);
    } else {
        printf("  Tim theo chieu dir (toi da %lu do)...\r\n", (unsigned long)(c->travel_x10 / 10));
#if HOME_USE_DRIVER
        st = driver_home(n, c->dir, c->travel_x10, &m_stop);
#else
        st = seek_dir(n, c->dir, c->fast_dps_x10, 1, c->travel_x10, &m_stop);
#endif
        if (st == HOME_ERR_NOT_FOUND && c->rev_x10 > 0) {
            printf("  Khong thay, tim chieu nguoc...\r\n");
#if HOME_USE_DRIVER
            st = driver_home(n, (int8_t)-c->dir, (uint32_t)c->travel_x10 + c->rev_x10, &m_stop);
#else
            st = seek_dir(n, (int8_t)-c->dir, c->fast_dps_x10, 1, (uint32_t)c->travel_x10 + c->rev_x10, &m_stop);
#endif
            far = 1;
        }
    }
    if (st != HOME_OK) return st;

    /* Goc khop tai cho driver dung: mep gan = home_x10, mep xa = home_x10 + dir * band */
    int32_t at = c->home_x10;
    if (far) at += (int32_t)c->dir * c->band_x10;
    j->offset_x10 = m_stop - (int32_t)j->dir * (int32_t)j->ratio * at;
    j->target_x10 = at;

    /* Dua ve goc 0 = giua vung kim loai */
    printf("  Ve goc 0 (giua cam bien)...\r\n");
    st = move_to(n, 0, c->slow_dps_x10);
    if (st != HOME_OK) return st;
    if (!home_sensor(n)) printf("  CANH BAO: o goc 0 ma cam bien khong tac dong, chay lai hcal %u\r\n", n);

    homed[n - 1] = 1;
    return HOME_OK;
}

home_status_t home_all(void)
{
    for (uint8_t k = 0; k < JOINT_COUNT; k++) {
        uint8_t n = HOME_ORDER[k];
        if (!cfg[n - 1].enabled) { printf("Bo qua khop %u (en = 0)\r\n", n); continue; }
        home_status_t st = home_joint(n);
        printf("Home khop %u: %s\r\n", n, home_status_str(st));
        if (st != HOME_OK) { joint_stop_all(); return st; }
    }
    return HOME_OK;
}
