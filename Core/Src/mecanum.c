#include "mecanum.h"
#include "wheel.h"
#include "zdt_x42s.h"
#include "ros_link.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEC_PI           3.14159265f
#define MEC_L_MM         (WHEEL_LX_MM + WHEEL_LY_MM)                     /* 229 mm */
#define MEC_CIRC_MM      (2.0f * MEC_PI * WHEEL_RADIUS_MM)                /* 235.6 mm / vong */
#define DEG_X10_PER_MM   (3600.0f / MEC_CIRC_MM)                         /* 0.1 do banh / mm vanh */
#define RPM_X10_PER_MMS  (600.0f / MEC_CIRC_MM)                          /* 0.1 RPM / (mm/s) */

/* Thu tu theo dong hoc: 0 FL, 1 FR, 2 RL, 3 RR */
typedef struct {
    uint8_t     addr;
    int8_t      sign;
    const char *name;
} mec_wheel_t;

static mec_wheel_t mw[4] = {
    { MEC_ADDR_FL, MEC_SIGN_FL, "FL truoc-trai" },
    { MEC_ADDR_FR, MEC_SIGN_FR, "FR truoc-phai" },
    { MEC_ADDR_RL, MEC_SIGN_RL, "RL sau-trai  " },
    { MEC_ADDR_RR, MEC_SIGN_RR, "RR sau-phai  " },
};

/* Trang thai chay */
static uint8_t  vel_timed   = 0;     /* che do van toc co hen gio */
static uint32_t vel_stop_at = 0;
static uint8_t  pos_moving  = 0;     /* che do vi tri dang chay, cho toi noi */
static uint8_t  pos_active  = 0;     /* bit i = banh i co chay */
static uint32_t pos_t0      = 0;
static uint32_t pos_timeout = 0;     /* ms */
static uint32_t pos_next    = 0;     /* lan doc co tiep theo */

/* ------------------------------------------------------------------ */

static float fabs_f(float x) { return (x < 0.0f) ? -x : x; }
static int32_t round_i(float x) { return (int32_t)(x + ((x < 0.0f) ? -0.5f : 0.5f)); }

static void px10(int32_t v)
{
    if (v < 0) { putchar('-'); v = -v; }
    printf("%ld.%ld", (long)(v / 10), (long)(v % 10));
}

static int in_ros(void) { return ros_link_active(); }

/* pt 2.20: dx, dy (mm hoac mm/s), dth (rad hoac rad/s) -> 4 gia tri tai vanh banh */
static void ik(float vx, float vy, float wz, float out[4])
{
    float lw = MEC_L_MM * wz;
    out[0] = vx - vy - lw;   /* FL */
    out[1] = vx + vy + lw;   /* FR */
    out[2] = vx + vy - lw;   /* RL */
    out[3] = vx - vy + lw;   /* RR */
}

void mecanum_stop(void)
{
    vel_timed  = 0;
    pos_moving = 0;
    for (int i = 0; i < 4; i++) {
        if (zdt_stop(mw[i].addr) != ZDT_OK) (void)zdt_stop(mw[i].addr);   /* thu lai 1 lan */
    }
}

/* ------------------------------------------------------------------ */
/* CHE DO VAN TOC */

zdt_status_t mecanum_set_velocity(float vx, float vy, float wz)
{
    float s[4], r[4], rmax = 0.0f;
    ik(vx, vy, wz, s);
    for (int i = 0; i < 4; i++) {
        r[i] = s[i] * RPM_X10_PER_MMS;
        if (fabs_f(r[i]) > rmax) rmax = fabs_f(r[i]);
    }
    /* Vuot gioi han: thu nho DEU ca 4 banh de giu dung huong chuyen dong */
    float k = (rmax > (float)WHEEL_MAX_RPM_X10) ? (float)WHEEL_MAX_RPM_X10 / rmax : 1.0f;

    zdt_status_t res = ZDT_OK;
    for (int i = 0; i < 4; i++) {
        int32_t cmd = round_i(r[i] * k) * mw[i].sign;
        zdt_status_t st = zdt_speed(mw[i].addr, cmd, WHEEL_ACC_DEFAULT);
        if (st != ZDT_OK) res = st;
    }
    if (res != ZDT_OK) mecanum_stop();     /* 1 banh khong nhan lenh: dung ca xe */
    return res;
}

static void cmd_vel(const char *svx, const char *svy, const char *swz, const char *ssec)
{
    if (!svx || !svy || !swz) { printf("Vd: xe v 80 0 0   (vx mm/s, vy mm/s, wz do/s, [giay])\r\n"); return; }
    float vx = strtof(svx, NULL), vy = strtof(svy, NULL);
    float wz = strtof(swz, NULL) * MEC_PI / 180.0f;
    uint32_t sec = ssec ? (uint32_t)atoi(ssec) : MEC_VEL_S_DEFAULT;

    wheel_cancel_timer();
    pos_moving = 0;
    zdt_status_t st = mecanum_set_velocity(vx, vy, wz);
    printf("Xe van toc vx %s mm/s, vy %s mm/s, wz %s do/s: %s\r\n", svx, svy, swz, zdt_status_str(st));
    if (st != ZDT_OK) { printf("Co banh khong tra loi, da dung ca xe. Go 'bx scan'\r\n"); return; }
    if (sec == 0u) { vel_timed = 0; printf("CHAY KHONG HEN GIO. Go 'xe x' de dung\r\n"); return; }
    vel_timed   = 1;
    vel_stop_at = HAL_GetTick() + sec * 1000u;
    printf("Tu dung sau %lu s\r\n", (unsigned long)sec);
}

/* ------------------------------------------------------------------ */
/* CHE DO VI TRI: dich chuyen dx, dy (mm), xoay dth (rad) trong he than xe.
 * Moi banh chay quang duong rieng, toc do va gia toc ty le voi quang duong
 * nen 4 banh bat dau cung luc (co sync + lenh FF 66) va toi noi cung luc. */

static void car_move(float dx, float dy, float dth, const char *what)
{
    float s[4], smax = 0.0f;
    ik(dx, dy, dth, s);
    for (int i = 0; i < 4; i++) if (fabs_f(s[i]) > smax) smax = fabs_f(s[i]);
    if (smax < 1.0f) { printf("Quang duong qua nho\r\n"); return; }

    wheel_cancel_timer();
    vel_timed  = 0;
    pos_active = 0;

    for (int i = 0; i < 4; i++) {
        if (fabs_f(s[i]) < 0.5f) continue;                 /* banh nay dung yen (vd: di cheo) */
        float    k   = fabs_f(s[i]) / smax;
        int32_t  deg = round_i(s[i] * DEG_X10_PER_MM) * mw[i].sign;
        int32_t  rpm = round_i((float)MEC_MOVE_RPM_X10 * k);
        int32_t  acc = round_i((float)MEC_MOVE_ACC * k);
        if (rpm < 1) rpm = 1;
        if (acc < 1) acc = 1;
        zdt_status_t st = zdt_move_ex(mw[i].addr, deg, (uint16_t)rpm, (uint16_t)acc, (uint16_t)acc,
                                      ZDT_MOVE_REL_CURRENT, 1);       /* sync = 1: cho lenh FF 66 */
        if (st != ZDT_OK) {
            printf("Banh %s (dia chi %u): %s. Huy lenh, dung ca xe\r\n", mw[i].name, mw[i].addr, zdt_status_str(st));
            mecanum_stop();                                  /* xoa cac lenh dang cho */
            return;
        }
        pos_active |= (uint8_t)(1u << i);
    }
    zdt_status_t st = zdt_sync_trigger();                    /* 00 FF 66 6B: 4 banh cung bat dau */

    /* Thoi gian du kien = quang duong / toc do + 1 lan thoi gian tang toc */
    float v_mms = (float)MEC_MOVE_RPM_X10 / RPM_X10_PER_MMS;
    float t_s   = smax / v_mms + ((float)MEC_MOVE_RPM_X10 / 10.0f) / (float)MEC_MOVE_ACC;
    printf("Xe %s: %s, du kien %ld.%ld s\r\n", what, zdt_status_str(st),
           (long)round_i(t_s * 10.0f) / 10, (long)round_i(t_s * 10.0f) % 10);
    for (int i = 0; i < 4; i++) {
        printf("  %s (%u): ", mw[i].name, mw[i].addr);
        if (pos_active & (1u << i)) { printf("%ld mm = ", (long)round_i(s[i])); px10(round_i(s[i] * DEG_X10_PER_MM)); printf(" do\r\n"); }
        else printf("dung yen\r\n");
    }
    pos_moving  = 1;
    pos_t0      = HAL_GetTick();
    pos_timeout = (uint32_t)(t_s * 2000.0f) + 3000u;
    pos_next    = pos_t0 + 300u;
}

/* ------------------------------------------------------------------ */

void mecanum_task(void)
{
    uint32_t now = HAL_GetTick();

    if (vel_timed && (int32_t)(now - vel_stop_at) >= 0) {
        vel_timed = 0;
        (void)mecanum_set_velocity(0.0f, 0.0f, 0.0f);       /* ve 0 theo doc giam toc */
        if (!in_ros()) printf("\r\n[Xe: het gio, da dung]\r\n> ");
    }

    if (pos_moving && (int32_t)(now - pos_next) >= 0) {
        pos_next = now + 100u;
        uint8_t done = 1;
        for (int i = 0; i < 4; i++) {
            if (!(pos_active & (1u << i))) continue;
            uint8_t f;
            if (zdt_read_flags(mw[i].addr, &f) != ZDT_OK || !(f & 0x02u)) { done = 0; break; }  /* bit1 = toi noi */
        }
        uint32_t el = now - pos_t0;
        if (done) {
            pos_moving = 0;
            if (!in_ros()) printf("\r\n[Xe: da toi noi sau %lu.%lu s]\r\n> ",
                                  (unsigned long)(el / 1000u), (unsigned long)((el % 1000u) / 100u));
        } else if (el > pos_timeout) {
            pos_moving = 0;
            if (!in_ros()) printf("\r\n[Xe: qua thoi gian ma chua toi noi. Go 'xe p' kiem tra, 'bx c' neu bi bao ve]\r\n> ");
        }
    }
}

static void show_table(void)
{
    printf("--- Xe mecanum: L = lx + ly = %ld mm, r = ", (long)round_i(MEC_L_MM)); px10(round_i(WHEEL_RADIUS_MM * 10.0f));
    printf(" mm ---\r\n");
    for (int i = 0; i < 4; i++) {
        uint16_t fw, hw;
        int ok = (zdt_read_version(mw[i].addr, &fw, &hw) == ZDT_OK);
        printf("  %s  dia chi %u  dau %+d  %s\r\n", mw[i].name, mw[i].addr, mw[i].sign, ok ? "OK" : "KHONG TRA LOI");
    }
    printf("  Toc do banh: chay vi tri "); px10(MEC_MOVE_RPM_X10); printf(" RPM, toi da "); px10(WHEEL_MAX_RPM_X10);
    printf(" RPM\r\n");
}

static void show_pos(void)
{
    for (int i = 0; i < 4; i++) {
        int32_t rpm = 0, pos = 0;
        zdt_status_t a = zdt_read_speed(mw[i].addr, &rpm);
        zdt_status_t b = zdt_read_position(mw[i].addr, &pos);
        printf("  %s: ", mw[i].name);
        if (a != ZDT_OK || b != ZDT_OK) { printf("khong doc duoc\r\n"); continue; }
        /* Doi ve quy uoc "duong = day xe di toi" */
        rpm *= mw[i].sign;
        pos *= mw[i].sign;
        printf("toc do "); px10(rpm); printf(" RPM | vi tri "); px10(pos);
        printf(" do = %ld mm\r\n", (long)round_i((float)pos / DEG_X10_PER_MM));
    }
}

static float arg_mm(const char *s)
{
    if (!s) return (float)MEC_DIST_DEFAULT;
    float d = strtof(s, NULL);
    if (d <= 0.0f || d > 3000.0f) { printf("Quang duong %s sai, dung %d mm\r\n", s, MEC_DIST_DEFAULT); return (float)MEC_DIST_DEFAULT; }
    return d;
}

void mecanum_print_help(void)
{
    printf("  --- XE 4 BANH (ke xe len khi test lan dau) ---\r\n");
    printf("  xe                 bang dia chi, dau, trang thai 4 banh\r\n");
    printf("  xe tien [mm]       tien (mac dinh %d mm)\r\n", MEC_DIST_DEFAULT);
    printf("  xe lui [mm]        lui\r\n");
    printf("  xe ngang <trai|phai> [mm]   di ngang\r\n");
    printf("  xe cheo <tt|tp|st|sp> [mm]  di cheo 45 do: truoc-trai, truoc-phai, sau-trai, sau-phai\r\n");
    printf("  xe xoay [do]       xoay tai cho (mac dinh 90). Duong = trai (nguoc kim), am = phai\r\n");
    printf("  xe v <vx> <vy> <wz> [giay]  van toc: mm/s, mm/s, do/s (0 giay = khong hen gio)\r\n");
    printf("  xe p               toc do va vi tri 4 banh\r\n");
    printf("  xe z               lay vi tri hien tai 4 banh lam 0\r\n");
    printf("  xe dao <dia chi>   dao dau 1 banh (tam thoi, tat nguon la mat)\r\n");
    printf("  xe x               DUNG CA XE\r\n");
}

void mecanum_cmd(int argc, char **argv)
{
    const char *sub = (argc > 1) ? argv[1] : NULL;
    const char *a   = (argc > 2) ? argv[2] : NULL;
    const char *b   = (argc > 3) ? argv[3] : NULL;
    const char *c   = (argc > 4) ? argv[4] : NULL;
    const char *d   = (argc > 5) ? argv[5] : NULL;

    if (!sub) { show_table(); return; }

    if (!strcmp(sub, "x")) {
        mecanum_stop();
        printf("Da dung ca xe\r\n");
    }
    else if (!strcmp(sub, "tien")) {
        car_move(arg_mm(a), 0.0f, 0.0f, "tien");
    }
    else if (!strcmp(sub, "lui")) {
        car_move(-arg_mm(a), 0.0f, 0.0f, "lui");
    }
    else if (!strcmp(sub, "ngang")) {
        if (a && !strcmp(a, "trai"))      car_move(0.0f,  arg_mm(b), 0.0f, "ngang trai");
        else if (a && !strcmp(a, "phai")) car_move(0.0f, -arg_mm(b), 0.0f, "ngang phai");
        else printf("Vd: xe ngang trai 200\r\n");
    }
    else if (!strcmp(sub, "cheo")) {
        /* 45 do: dx = dy = d / sqrt(2) */
        float h = arg_mm(b) * 0.70710678f;
        if      (a && !strcmp(a, "tt")) car_move( h,  h, 0.0f, "cheo truoc-trai");
        else if (a && !strcmp(a, "tp")) car_move( h, -h, 0.0f, "cheo truoc-phai");
        else if (a && !strcmp(a, "st")) car_move(-h,  h, 0.0f, "cheo sau-trai");
        else if (a && !strcmp(a, "sp")) car_move(-h, -h, 0.0f, "cheo sau-phai");
        else printf("Vd: xe cheo tp 300   (tt truoc-trai, tp truoc-phai, st sau-trai, sp sau-phai)\r\n");
    }
    else if (!strcmp(sub, "xoay")) {
        float deg = a ? strtof(a, NULL) : 90.0f;
        if (deg == 0.0f || deg > 720.0f || deg < -720.0f) { printf("Goc sai (-720..720)\r\n"); return; }
        car_move(0.0f, 0.0f, deg * MEC_PI / 180.0f * MEC_ROT_GAIN, (deg > 0) ? "xoay trai" : "xoay phai");
    }
    else if (!strcmp(sub, "v")) {
        cmd_vel(a, b, c, d);
    }
    else if (!strcmp(sub, "p")) {
        show_pos();
    }
    else if (!strcmp(sub, "z")) {
        for (int i = 0; i < 4; i++)
            printf("  %s: %s\r\n", mw[i].name, zdt_status_str(zdt_zero_position(mw[i].addr)));
    }
    else if (!strcmp(sub, "dao")) {
        int n = a ? atoi(a) : 0;
        for (int i = 0; i < 4; i++) {
            if (mw[i].addr == n) {
                mw[i].sign = (int8_t)-mw[i].sign;
                printf("%s (dia chi %u): dau = %+d. Nho sua MEC_SIGN trong mecanum.h cho lan sau\r\n",
                       mw[i].name, mw[i].addr, mw[i].sign);
                return;
            }
        }
        printf("Dia chi sai. Vd: xe dao 6\r\n");
    }
    else {
        printf("Khong hieu 'xe %s'. Go h de xem danh sach.\r\n", sub);
    }
}
