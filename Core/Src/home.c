#include "home.h"
#include "zdt_x42s.h"
#include "main.h"
#include <stdio.h>

/* ===================== BANG CAU HINH HOMING =====================
 *   act      : 1 = muc cao tren chan GPIO nghia la tac dong, 0 = muc thap (qua module opto PC817 la 0)
 *   en       : 0 = chua cho phep "home" (van dung duoc "hcal", "lsj"). Dat 1 sau khi da co home_x10
 *   dir      : chieu goc khop di TOI cam bien tu tu the goc (+1 = goc tang, -1 = goc giam). Do bang "lsj"
 *   home_x10 : goc khop (0.1 do) tai MEP GAN cua vung kim loai (mep gap khi di theo chieu dir). Do bang "hcal"
 *   band_x10 : be rong vung kim loai (0.1 do). Do bang "hcal". Dung khi cham tu phia ben kia.
 *              0 = chua do: van home duoc nhung neu cham tu phia ben kia thi goc lech toi be rong vung kim loai
 *   fast     : toc do tim (0.1 do/s)
 *   slow     : toc do nhich ra khi luc bat dau dang dung tren kim loai (0.1 do/s)
 *   travel   : tim theo chieu dir toi da bay nhieu, tinh tu cho bat dau (0.1 do)
 *   rev      : khong thay thi tim nguoc toi da bay nhieu, tinh tu cho bat dau (0.1 do). 0 = khong tim nguoc
 *
 * Quy trinh home: GAP CAM BIEN LA DUNG NGAY, khong lui, khong tiep can lai.
 * Toc do tinh tai DAU RA khop.
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       act_high;
    uint8_t       enabled;
    int8_t        dir;
    int32_t       home_x10;
    uint16_t      band_x10;
    uint16_t      fast_dps_x10;
    uint16_t      slow_dps_x10;
    uint16_t      travel_x10;
    uint16_t      rev_x10;
} home_cfg_t;

/* Chan cam bien: lay tu CubeMX (nhan LSW1..LSW4 trong .ioc, sinh ra trong main.h).
 * Neu .ioc chua khai bao thi dung PB12..PB15 va code tu cau hinh trong home_init(). */
#if defined(LSW1_Pin) && defined(LSW2_Pin) && defined(LSW3_Pin) && defined(LSW4_Pin)
  #define LSW_FROM_CUBEMX 1
  #define LSW1 LSW1_GPIO_Port, LSW1_Pin
  #define LSW2 LSW2_GPIO_Port, LSW2_Pin
  #define LSW3 LSW3_GPIO_Port, LSW3_Pin
  #define LSW4 LSW4_GPIO_Port, LSW4_Pin
#else
  #define LSW_FROM_CUBEMX 0
  #define LSW1 GPIOB, GPIO_PIN_12
  #define LSW2 GPIOB, GPIO_PIN_13
  #define LSW3 GPIOB, GPIO_PIN_14
  #define LSW4 GPIOB, GPIO_PIN_15
#endif

static const home_cfg_t cfg[JOINT_COUNT] = {
    /* home_x10 TAM tinh: khop 1, 4 tu lsj. Khop 2, 3 = so lsj cu x 0.9 (truoc khi tat S_PosTDP).
     * band_x10 = 0: CHUA DO. Chay "hcal <n>" de lay ca home_x10 va band_x10. */
    /* fast = toc do tim, de THAP de luc cham dung ngay ma khong giat ca tay
     * (nang luong phai trieu tieu khi dung ty le binh phuong toc do). */
    /* GOC 0 = GIUA VUNG KIM LOAI CUA MOI CAM BIEN (tu the 4 den opto deu sang) = home trong MoveIt.
     * Vi vay home_x10 = -dir * band / 2 (mep gan cach giua nua be rong).
     * dir, band: do bang hcal ngay 2026-10-24. Cam bien khop 3 o PB11. */
    /* port, pin act en dir home_x10 band fast slow travel rev */
    { LSW1,      0,  1, +1,   -33,   66,   80,  30,  400,  400 },   /* khop 1: tim 8 do/s */
    { LSW2,      0,  1, -1,    32,   64,   30,  20,  500,  100 },   /* khop 2: tim 3 do/s, lam viec phia goc duong nen rev nho */
    { LSW3,      0,  1, +1,   -33,   66,   80,  30,  500,  300 },   /* khop 3: tim 8 do/s */
    { LSW4,      0,  1, +1,   -50,  101,  100,  50,  400,  400 },   /* khop 4: tim 10 do/s */
};

/* Thu tu home tat ca: tu ngon tay vao goc (gap co tay, khuyu, vai roi moi xoay de)
 * de luc khop 1 xoay thi canh tay da thu gon. Doi lai neu co khi cua ban khac. */
static const uint8_t HOME_ORDER[JOINT_COUNT] = { 4, 3, 2, 1 };

#define BAND_MAX_X10      200    /* be rong toi da cho phep cua vung kim loai (0.1 do) */
#define DEBOUNCE_SAMPLES  3      /* so mau 1 ms lien tiep giong nhau moi doi trang thai */
#define STALL_SKIP_MS     2000   /* bo qua kiem tra ket trong luc tang toc */
#define STALL_WINDOW_MS   500    /* cua so kiem tra ket */
#define STOP_ACC_RPM_S    1000   /* gia toc dung khi cham cam bien (RPM/s tai truc dong co) */
#define HOME_SPEED_ACC    300    /* gia toc toi thieu (RPM/s) khi chay che do toc do luc home */

/* ------------------------------------------------------------------ */

static home_abort_fn abort_cb = 0;
static uint8_t  stable[JOINT_COUNT], cnt[JOINT_COUNT], homed[JOINT_COUNT];
static uint32_t last_tick;

static uint8_t raw_active(uint8_t i)
{
    uint8_t high = (HAL_GPIO_ReadPin(cfg[i].port, cfg[i].pin) == GPIO_PIN_SET);
    return cfg[i].act_high ? high : !high;
}

void home_sensor_update(void)
{
    uint32_t t = HAL_GetTick();
    if (t == last_tick) return;
    last_tick = t;
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        uint8_t r = raw_active(i);
        if (r != stable[i]) {
            if (++cnt[i] >= DEBOUNCE_SAMPLES) { stable[i] = r; cnt[i] = 0; }
        } else {
            cnt[i] = 0;
        }
    }
}

void home_init(home_abort_fn abort_fn)
{
    abort_cb = abort_fn;
#if !LSW_FROM_CUBEMX
    /* .ioc chua khai bao chan: tu cau hinh. Neu da khai bao, MX_GPIO_Init() trong main.c lam viec nay */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        g.Pin = cfg[i].pin;
        HAL_GPIO_Init(cfg[i].port, &g);
    }
#endif
    HAL_Delay(2);
    for (uint8_t i = 0; i < JOINT_COUNT; i++) { stable[i] = raw_active(i); cnt[i] = 0; homed[i] = 0; }
    last_tick = HAL_GetTick();
}

uint8_t home_sensor(uint8_t n)
{
    if (n < 1 || n > JOINT_COUNT) return 0;
    home_sensor_update();
    return stable[n - 1];
}

uint8_t home_pin_level(uint8_t n)
{
    if (n < 1 || n > JOINT_COUNT) return 0;
    return HAL_GPIO_ReadPin(cfg[n - 1].port, cfg[n - 1].pin) == GPIO_PIN_SET;
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
    case HOME_ERR_NOT_CONFIG: return "khop chua bat home (en = 0 trong home.c)";
    case HOME_ERR_DRIVER:     return "driver khong tra loi / tu choi lenh";
    case HOME_ERR_NOT_FOUND:  return "het quang duong ma khong thay cam bien (sai chieu? tang travel?)";
    case HOME_ERR_STUCK_ON:   return "cam bien luon tac dong (dut day? sai cuc tinh? vung kim loai qua rong?)";
    case HOME_ERR_STALL:      return "dong co khong nhuc nhich (ket co khi?)";
    case HOME_ERR_TIMEOUT:    return "qua thoi gian";
    case HOME_ERR_ABORT:      return "da huy bang phim";
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

/* 0.1 do/s cua khop -> 0.1 RPM cua dong co:  dps_x10/10 * ratio / 6 * 10 */
static int32_t rpm_x10_of(const joint_t *j, uint16_t dps_x10)
{
    int32_t v = ((int32_t)dps_x10 * j->ratio + 3) / 6;
    if (v < 1) v = 1;
    if (v > 30000) v = 30000;
    return v;
}

/* gia toc cua khop (0.1 do/s2) -> RPM/s cua dong co, khong nho hon HOME_SPEED_ACC */
static uint16_t acc_of(const joint_t *j)
{
    int32_t a = ((int32_t)j->acc_x10 * j->ratio + 30) / 60;
    if (a < HOME_SPEED_ACC) a = HOME_SPEED_ACC;
    if (a > 65535) a = 65535;
    return (uint16_t)a;
}

/* goc khop (0.1 do) tu vi tri dong co (0.1 do) theo offset hien tai */
static int32_t joint_of(const joint_t *j, int32_t m)
{
    return div_round((m - j->offset_x10) * j->dir, (int32_t)j->ratio);
}

/* Cho ms mili giay, van cap nhat cam bien. Tra ve 0 neu bi huy */
static int wait_ms(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        home_sensor_update();
        if (user_abort()) return 0;
    }
    return 1;
}

static void stop_joint(const joint_t *j)
{
    /* Lenh toc do 0 voi gia toc lon de dung ngay, sau do lenh dung FE 98 cho chac */
    (void)zdt_speed(j->addr, 0, STOP_ACC_RPM_S);
    (void)zdt_stop(j->addr);
    HAL_Delay(150);
}

/* Chay che do toc do theo chieu GOC KHOP joint_dir (+1 tang, -1 giam).
 * DUNG NGAY khi cam bien == want. *m_hit = vi tri dong co (0.1 do) doc ngay luc dat dieu kien.
 * Gioi han quang duong max_x10 (goc khop, 0.1 do), kiem tra ket, bam phim de huy. */
static home_status_t seek_dir(uint8_t n, int8_t joint_dir, uint16_t dps_x10, uint8_t want,
                              uint32_t max_x10, int32_t *m_hit)
{
    joint_t *j = joint_get(n);
    int32_t m0, m, m_win;

    if (zdt_read_position(j->addr, &m0) != ZDT_OK) return HOME_ERR_DRIVER;

    int32_t  limit_m  = (int32_t)max_x10 * j->ratio;
    int32_t  expect_m = (int32_t)dps_x10 * j->ratio * STALL_WINDOW_MS / 1000 / 4;   /* 25% quang duong ky vong */
    uint32_t timeout  = (uint32_t)max_x10 * 1000u / dps_x10 * 2u + 3000u;
    int32_t  v        = rpm_x10_of(j, dps_x10) * j->dir * joint_dir;

    if (zdt_speed(j->addr, v, acc_of(j)) != ZDT_OK) return HOME_ERR_DRIVER;

    uint32_t t0 = HAL_GetTick(), t_pos = t0, t_win = t0;
    m_win = m0;

    for (;;) {
        home_sensor_update();
        if (stable[n - 1] == want) {
            zdt_status_t s = zdt_read_position(j->addr, &m);   /* doc truoc khi dung de lay dung diem cham */
            stop_joint(j);
            if (s != ZDT_OK) return HOME_ERR_DRIVER;
            *m_hit = m;
            return HOME_OK;
        }
        if (user_abort()) { stop_joint(j); return HOME_ERR_ABORT; }

        uint32_t now = HAL_GetTick();
        if (now - t0 > timeout) { stop_joint(j); return HOME_ERR_TIMEOUT; }

        if (now - t_pos >= 20) {
            t_pos = now;
            if (zdt_read_position(j->addr, &m) != ZDT_OK) { stop_joint(j); return HOME_ERR_DRIVER; }
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
}

/* Tim cam bien, dung ngay khi gap. Tra ve vi tri dong co luc gap va mep nao:
 *   *far = 0: gap mep gan (dung chieu dir, hoac luc dau dang dung tren kim loai roi nhich ra)
 *   *far = 1: gap mep xa (tim nguoc, tu phia ben kia toi) */
static home_status_t find_edge(uint8_t n, int32_t *m_hit, uint8_t *far)
{
    const home_cfg_t *c = &cfg[n - 1];
    home_status_t st;

    if (!wait_ms(DEBOUNCE_SAMPLES + 2)) return HOME_ERR_ABORT;   /* lam moi bo loc truoc khi xet */

    /* Dang dung tren kim loai: nhich CHAM nguoc chieu dir toi khi cam bien vua nha = mep gan */
    if (home_sensor(n)) {
        printf("  Dang tren cam bien, nhich ra mep...\r\n");
        uint32_t lim = c->band_x10 ? (uint32_t)c->band_x10 + 50u : BAND_MAX_X10;
        st = seek_dir(n, (int8_t)-c->dir, c->slow_dps_x10, 0, lim, m_hit);
        *far = 0;
        return st;
    }

    /* Tim theo chieu dir */
    printf("  Tim (toi da %lu do)...\r\n", (unsigned long)(c->travel_x10 / 10));
    st = seek_dir(n, c->dir, c->fast_dps_x10, 1, c->travel_x10, m_hit);
    *far = 0;
    if (st != HOME_ERR_NOT_FOUND || c->rev_x10 == 0) return st;

    /* Khong thay: tim nguoc. Tinh tu cuoi chieu dir: quay ve qua cho bat dau roi di them toi da rev */
    printf("  Khong thay, tim chieu nguoc (toi da %lu do tinh tu cho bat dau)...\r\n",
           (unsigned long)(c->rev_x10 / 10));
    st = seek_dir(n, (int8_t)-c->dir, c->fast_dps_x10, 1, (uint32_t)c->travel_x10 + c->rev_x10, m_hit);
    *far = 1;
    return st;
}

/* Chay khop toi goc khop joint_x10 (vi tri tuyet doi theo offset hien tai), KHONG kiem tra gioi han
 * trong joint.c (diem giua cam bien co the nam ngoai gioi han lam viec). Cho toi noi. */
static home_status_t move_to(uint8_t n, int32_t joint_x10, uint16_t dps_x10)
{
    joint_t *j = joint_get(n);
    int32_t mt = j->offset_x10 + (int32_t)j->dir * (int32_t)j->ratio * joint_x10;
    int32_t m0, m;
    if (zdt_read_position(j->addr, &m0) != ZDT_OK) return HOME_ERR_DRIVER;
    if (zdt_move(j->addr, mt, (uint16_t)rpm_x10_of(j, dps_x10), acc_of(j), acc_of(j),
                 ZDT_MOVE_ABSOLUTE) != ZDT_OK) return HOME_ERR_DRIVER;

    int32_t  tol     = (int32_t)j->ratio * 3;              /* 0.3 do tai khop */
    if (tol < 10) tol = 10;
    uint32_t dist    = (uint32_t)iabs(mt - m0) / j->ratio;  /* 0.1 do tai khop */
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

/* ------------------------------------------------------------------ */

home_status_t home_jog_to_sensor(uint8_t n, int8_t joint_dir, int32_t *joint_x10)
{
    joint_t *j = joint_get(n);
    if (!j || (joint_dir != 1 && joint_dir != -1)) return HOME_ERR_ARG;
    const home_cfg_t *c = &cfg[n - 1];
    int32_t m_hit;

    if (!wait_ms(DEBOUNCE_SAMPLES + 2)) return HOME_ERR_ABORT;
    if (home_sensor(n)) return HOME_ERR_STUCK_ON;            /* dang tac dong san: khong biet di dau */

    home_status_t st = seek_dir(n, joint_dir, c->fast_dps_x10, 1, c->travel_x10, &m_hit);
    if (st != HOME_OK) return st;
    *joint_x10 = joint_of(j, m_hit);
    j->target_x10 = *joint_x10;
    return HOME_OK;
}

/* Do mep gan (home_x10) va be rong vung kim loai (band_x10) theo goc 0 HIEN TAI. Khong doi offset.
 * Can khop dang o phia tu the goc cua cam bien (phia binh thuong). 3 doan chay ngan:
 *   1. di theo dir toi khi gap mep gan
 *   2. di tiep theo dir toi khi ra khoi vung kim loai
 *   3. quay nguoc toi khi gap mep xa (tu phia ben kia) */
home_status_t home_measure(uint8_t n, int8_t dir_override, int32_t *home_x10, int32_t *band_x10)
{
    joint_t *j = joint_get(n);
    if (!j) return HOME_ERR_ARG;
    home_cfg_t cc = cfg[n - 1];                   /* ban sao de dung chieu nhap tu lenh neu co */
    if (dir_override == 1 || dir_override == -1) cc.dir = dir_override;
    const home_cfg_t *c = &cc;
    int32_t m_near, m_out, m_far;
    home_status_t st;

    if (!wait_ms(DEBOUNCE_SAMPLES + 2)) return HOME_ERR_ABORT;
    if (home_sensor(n)) return HOME_ERR_STUCK_ON;   /* can bat dau o ngoai vung kim loai, phia binh thuong */

    printf("  1/3 Tim mep gan...\r\n");
    st = seek_dir(n, c->dir, c->fast_dps_x10, 1, c->travel_x10, &m_near);
    if (st != HOME_OK) return st;

    printf("  2/3 Di qua vung kim loai...\r\n");
    st = seek_dir(n, c->dir, c->slow_dps_x10, 0, BAND_MAX_X10, &m_out);
    if (st != HOME_OK) return st;
    if (!wait_ms(200)) return HOME_ERR_ABORT;

    printf("  3/3 Quay lai tim mep xa...\r\n");
    st = seek_dir(n, (int8_t)-c->dir, c->slow_dps_x10, 1, BAND_MAX_X10, &m_far);
    if (st != HOME_OK) return st;

    *home_x10 = joint_of(j, m_near);
    *band_x10 = iabs(joint_of(j, m_near) - joint_of(j, m_far));
    j->target_x10 = joint_of(j, m_far);
    return HOME_OK;
}

home_status_t home_joint(uint8_t n)
{
    joint_t *j = joint_get(n);
    if (!j) return HOME_ERR_ARG;
    const home_cfg_t *c = &cfg[n - 1];
    if (!c->enabled) return HOME_ERR_NOT_CONFIG;

    homed[n - 1] = 0;
    printf("Home khop %u:\r\n", n);

    int32_t m_hit;
    uint8_t far;
    home_status_t st = find_edge(n, &m_hit, &far);
    if (st != HOME_OK) return st;

    /* Goc khop tai diem gap:
     *   mep gan: home_x10
     *   mep xa : home_x10 + dir * band_x10 (vung kim loai nam phia dir so voi mep gan) */
    int32_t at = c->home_x10;
    if (far) {
        at += (int32_t)c->dir * c->band_x10;
        if (c->band_x10 == 0) printf("  CANH BAO: gap tu phia ben kia ma band_x10 = 0, goc co the lech. Chay hcal %u\r\n", n);
    }
    /* m_hit = offset + dir * ratio * at */
    j->offset_x10 = m_hit - (int32_t)j->dir * (int32_t)j->ratio * at;
    j->target_x10 = at;

    /* Dua khop vao GIUA vung kim loai: den opto sang on dinh, rung tu khop khac khong lam cam bien nha.
     * Vung kim loai nam phia dir so voi mep gan: giua = home_x10 + dir * band / 2 */
    if (c->band_x10 > 0) {
        int32_t mid = c->home_x10 + (int32_t)c->dir * (int32_t)(c->band_x10 / 2);
        printf("  Vao giua cam bien (");
        printf("%s%ld.%ld do)...\r\n", mid < 0 ? "-" : "", (long)(iabs(mid) / 10), (long)(iabs(mid) % 10));
        st = move_to(n, mid, c->slow_dps_x10);
        if (st != HOME_OK) return st;
        if (!wait_ms(DEBOUNCE_SAMPLES + 2)) return HOME_ERR_ABORT;
        if (!home_sensor(n)) printf("  CANH BAO: giua cam bien ma den khong sang, kiem tra band_x10 (chay lai hcal %u)\r\n", n);
    }
    homed[n - 1] = 1;
    return HOME_OK;
}

home_status_t home_all(void)
{
    for (uint8_t k = 0; k < JOINT_COUNT; k++) {
        uint8_t n = HOME_ORDER[k];
        if (!cfg[n - 1].enabled) { printf("Bo qua khop %u (chua bat home)\r\n", n); continue; }
        home_status_t st = home_joint(n);
        printf("Home khop %u: %s\r\n", n, home_status_str(st));
        if (st != HOME_OK) { joint_stop_all(); return st; }
    }
    return HOME_OK;
}
