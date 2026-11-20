#include "mecanum.h"
#include "motor.h"
#include "ros_link.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEC_PI           3.14159265f
#define MEC_L_MM         (MEC_LX_MM + MEC_LY_MM)                   /* 229 mm */
#define MEC_CIRC_MM      (2.0f * MEC_PI * MEC_RADIUS_MM)           /* 251.3 mm / vong (banh 80 mm) */
#define DEG_PER_MM       (360.0f / MEC_CIRC_MM)                    /* do banh / mm vanh */
#define RPM_PER_MMS      (60.0f / MEC_CIRC_MM)                     /* RPM / (mm/s) */

/* Thu tu dong hoc 0 FL, 1 FR, 2 RL, 3 RR trung voi mot_id_t */

static uint8_t  vel_timed   = 0;
static uint32_t vel_stop_at = 0;
static uint8_t  pos_moving  = 0;
static uint32_t pos_t0      = 0;
static uint32_t pos_timeout = 0;

/* ------------------------------------------------------------------ */

static float fabs_f(float x) { return (x < 0.0f) ? -x : x; }
static int32_t round_i(float x) { return (int32_t)(x + ((x < 0.0f) ? -0.5f : 0.5f)); }

static void pf1(float v)
{
    int32_t x = round_i(v * 10.0f);
    if (x < 0) { putchar('-'); x = -x; }
    printf("%ld.%ld", (long)(x / 10), (long)(x % 10));
}

static int in_ros(void) { return ros_link_active(); }

/* pt 2.20: vx, vy (mm hoac mm/s), wz (rad hoac rad/s) -> 4 gia tri tai vanh banh */
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
    mot_stop_all();
}

/* ------------------------------------------------------------------ */
/* CHE DO VAN TOC */

void mecanum_set_velocity(float vx, float vy, float wz)
{
    float s[4], r[4], rmax = 0.0f;
    ik(vx, vy, wz, s);
    for (int i = 0; i < 4; i++) {
        r[i] = s[i] * RPM_PER_MMS;
        if (fabs_f(r[i]) > rmax) rmax = fabs_f(r[i]);
    }
    /* Vuot gioi han: thu nho DEU ca 4 banh de giu dung huong chuyen dong */
    float k = (rmax > MEC_VEL_RPM_MAX) ? MEC_VEL_RPM_MAX / rmax : 1.0f;
    for (int i = 0; i < 4; i++) mot_set_rpm((mot_id_t)i, r[i] * k);
}

static void cmd_vel(const char *svx, const char *svy, const char *swz, const char *ssec)
{
    if (!svx || !svy || !swz) { printf("Vd: xe v 80 0 0   (vx mm/s, vy mm/s, wz do/s, [giay])\r\n"); return; }
    float vx = strtof(svx, NULL), vy = strtof(svy, NULL);
    float wz = strtof(swz, NULL) * MEC_PI / 180.0f;
    uint32_t sec = ssec ? (uint32_t)atoi(ssec) : MEC_VEL_S_DEFAULT;

    pos_moving = 0;
    mecanum_set_velocity(vx, vy, wz);
    printf("Xe van toc vx %s mm/s, vy %s mm/s, wz %s do/s\r\n", svx, svy, swz);
    if (sec == 0u) { vel_timed = 0; printf("CHAY KHONG HEN GIO. Go 'xe x' de dung\r\n"); return; }
    vel_timed   = 1;
    vel_stop_at = HAL_GetTick() + sec * 1000u;
    printf("Tu dung sau %lu s\r\n", (unsigned long)sec);
}

/* ------------------------------------------------------------------ */
/* CHE DO VI TRI: dich chuyen dx, dy (mm), xoay dth (rad) trong he than xe.
 * Moi banh chay quang duong rieng, toc do va gia toc ty le voi quang duong,
 * 4 banh bat dau trong cung 1 chu ky PID va toi noi cung luc. */

static void car_move(float dx, float dy, float dth, const char *what)
{
    float s[4], smax = 0.0f;
    ik(dx, dy, dth, s);
    for (int i = 0; i < 4; i++) if (fabs_f(s[i]) > smax) smax = fabs_f(s[i]);
    if (smax < 1.0f) { printf("Quang duong qua nho\r\n"); return; }
    for (int i = 0; i < 4; i++)
        if (mot_fault((mot_id_t)i)) { printf("Banh %s dang loi, go 'm c' truoc\r\n", mot_name((mot_id_t)i)); return; }

    vel_timed = 0;
    mot_group_begin();
    for (int i = 0; i < 4; i++) {
        if (fabs_f(s[i]) < 0.5f) continue;                 /* banh nay dung yen (vd: di cheo) */
        float k = fabs_f(s[i]) / smax;
        mot_move_rel((mot_id_t)i, s[i] * DEG_PER_MM, MEC_MOVE_RPM * k, MEC_MOVE_ACC * k);
    }
    mot_group_go();

    /* Thoi gian du kien = quang duong / toc do + thoi gian tang toc */
    float v_mms = MEC_MOVE_RPM / RPM_PER_MMS;
    float t_s   = smax / v_mms + MEC_MOVE_RPM / MEC_MOVE_ACC;
    printf("Xe %s, du kien ", what); pf1(t_s); printf(" s\r\n");
    for (int i = 0; i < 4; i++) {
        printf("  %s: ", mot_name((mot_id_t)i));
        if (fabs_f(s[i]) >= 0.5f) { printf("%ld mm = ", (long)round_i(s[i])); pf1(s[i] * DEG_PER_MM); printf(" do\r\n"); }
        else printf("dung yen\r\n");
    }
    pos_moving  = 1;
    pos_t0      = HAL_GetTick();
    pos_timeout = (uint32_t)(t_s * 2000.0f) + 3000u;
}

/* ------------------------------------------------------------------ */

void mecanum_task(void)
{
    uint32_t now = HAL_GetTick();

    if (vel_timed && (int32_t)(now - vel_stop_at) >= 0) {
        vel_timed = 0;
        mecanum_set_velocity(0.0f, 0.0f, 0.0f);            /* PID dua ve 0 roi phanh */
        if (!in_ros()) printf("\r\n[Xe: het gio, da dung]\r\n> ");
    }

    if (pos_moving) {
        int done = 1, fault = 0;
        for (int i = 0; i < 4; i++) {
            if (!mot_move_done((mot_id_t)i)) done = 0;
            if (mot_fault((mot_id_t)i)) fault = 1;
        }
        uint32_t el = now - pos_t0;
        if (fault) {
            pos_moving = 0;
            mecanum_stop();
            if (!in_ros()) printf("\r\n[Xe: co banh bi loi, da dung ca xe. Go 'm' xem]\r\n> ");
        } else if (done) {
            pos_moving = 0;
            if (!in_ros()) printf("\r\n[Xe: da toi noi sau %lu.%lu s]\r\n> ",
                                  (unsigned long)(el / 1000u), (unsigned long)((el % 1000u) / 100u));
        } else if (el > pos_timeout) {
            pos_moving = 0;
            if (!in_ros()) printf("\r\n[Xe: qua thoi gian ma chua toi noi. Go 'm' kiem tra]\r\n> ");
        }
    }
}

static void show_pos(void)
{
    for (int i = 0; i < 4; i++) {
        float deg = mot_get_deg((mot_id_t)i);
        printf("  %s: toc do ", mot_name((mot_id_t)i)); pf1(mot_get_rpm((mot_id_t)i));
        printf(" RPM | vi tri "); pf1(deg); printf(" do = %ld mm\r\n", (long)round_i(deg / DEG_PER_MM));
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
    printf("  xe tien [mm]       tien (mac dinh %d mm)\r\n", MEC_DIST_DEFAULT);
    printf("  xe lui [mm]        lui\r\n");
    printf("  xe ngang <trai|phai> [mm]   di ngang\r\n");
    printf("  xe cheo <tt|tp|st|sp> [mm]  di cheo 45 do: truoc-trai, truoc-phai, sau-trai, sau-phai\r\n");
    printf("  xe xoay [do]       xoay tai cho (mac dinh 90). Duong = trai (nguoc kim), am = phai\r\n");
    printf("  xe v <vx> <vy> <wz> [giay]  van toc: mm/s, mm/s, do/s (0 giay = khong hen gio)\r\n");
    printf("  xe p               toc do va vi tri 4 banh\r\n");
    printf("  xe x               DUNG CA XE\r\n");
}

void mecanum_cmd(int argc, char **argv)
{
    const char *sub = (argc > 1) ? argv[1] : NULL;
    const char *a   = (argc > 2) ? argv[2] : NULL;
    const char *b   = (argc > 3) ? argv[3] : NULL;
    const char *c   = (argc > 4) ? argv[4] : NULL;
    const char *d   = (argc > 5) ? argv[5] : NULL;

    if (!sub || !strcmp(sub, "p")) { show_pos(); return; }

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
        float h = arg_mm(b) * 0.70710678f;                  /* 45 do: dx = dy = d / sqrt(2) */
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
    else {
        printf("Khong hieu 'xe %s'. Go h de xem danh sach.\r\n", sub);
    }
}
