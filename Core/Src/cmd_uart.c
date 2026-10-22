#include "cmd_uart.h"
#include "zdt_x42s.h"
#include "joint.h"
#include "main.h"

/* 0 = khong dung gripper, 1 = co gripper servo tren PA6 (TIM3 CH1) */
#ifndef USE_SERVO
#define USE_SERVO 1
#endif

#if USE_SERVO
#include "servo.h"
/* 3 diem hanh trinh gripper voi cach lap tay don hien tai (s cang nho cang mo).
 * Tu so do: s 500 = 53 mm, s 1000 = 23 mm, s 1300 = dong chat. Khoang 0,06 mm/us. */
#define GRIP_OPEN_US    500    /* mo toi da = gioi han servo, ~53 mm */
#define GRIP_HOLD_US    850    /* kep vat 35 mm: ra lenh ~32 mm de co luc kep */
#define GRIP_CLOSE_US  1270    /* dong het: 2 ngon vua cham nhau, khong ep them */
#define GRIP_STEP_US   10      /* moi 20 ms tien 10 us = 500 us/giay */
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;   /* do CubeMX tao trong main.c */

#define LINE_MAX 64
#define ARG_MAX  8
static char    line[LINE_MAX];
static uint8_t line_len = 0;

/* ------------------------------------------------------------------ */
/* Tien ich in / doc so co 1 chu so thap phan, khong can printf %f */

static void print_x10(int32_t v)
{
    if (v < 0) { putchar('-'); v = -v; }
    printf("%ld.%ld", (long)(v / 10), (long)(v % 10));
}

static int32_t parse_x10(const char *s)
{
    float f = strtof(s, NULL);
    return (int32_t)(f * 10.0f + ((f < 0) ? -0.5f : 0.5f));
}

static uint16_t u16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static int32_t sgn32(uint8_t sign, const uint8_t *p) { int32_t v = (int32_t)u32(p); return sign ? -v : v; }

static void report(const char *what, zdt_status_t s)
{
    printf("%s: %s\r\n", what, zdt_status_str(s));
}

static void jreport(const char *what, uint8_t n, joint_status_t s)
{
    printf("%s khop %u: %s\r\n", what, n, joint_status_str(s));
}

/* Doc so khop tu chuoi, tra ve 1..4 hoac 0 neu sai (co in loi) */
static uint8_t arg_joint(const char *s)
{
    if (s == NULL) { printf("Thieu so khop (1..%d)\r\n", JOINT_COUNT); return 0; }
    int n = atoi(s);
    if (n < 1 || n > JOINT_COUNT) { printf("So khop sai: %s (1..%d)\r\n", s, JOINT_COUNT); return 0; }
    return (uint8_t)n;
}

/* ------------------------------------------------------------------ */
/* Doc trang thai / cau hinh cua 1 driver (dung cho do loi) */

static void show_status(uint8_t addr)
{
    uint8_t b[36];
    zdt_status_t s = zdt_read_long(addr, 0x43, 0x7A, b, 36);
    if (s != ZDT_OK) { printf("Doc trang thai driver %u: %s\r\n", addr, zdt_status_str(s)); return; }
    printf("--- Driver %u ---\r\n", addr);
    printf("Dien ap nguon : %u mV\r\n", u16(&b[3]));
    printf("Dong nguon    : %u mA\r\n", u16(&b[5]));
    printf("Dong pha      : %u mA\r\n", u16(&b[7]));
    printf("Vi tri dich   : "); print_x10(sgn32(b[13], &b[14])); printf(" do (truc dong co)\r\n");
    printf("Toc do thuc   : "); print_x10(b[18] ? -(int32_t)u16(&b[19]) : (int32_t)u16(&b[19])); printf(" RPM\r\n");
    printf("Vi tri thuc   : "); print_x10(sgn32(b[21], &b[22])); printf(" do (truc dong co)\r\n");
    { int32_t e = sgn32(b[26], &b[27]);
      printf("Sai so vi tri : %s%ld.%02ld do\r\n", e < 0 ? "-" : "", (long)((e < 0 ? -e : e) / 100), (long)((e < 0 ? -e : e) % 100)); }
    printf("Nhiet do      : %s%u C\r\n", b[31] ? "-" : "", b[32]);
    printf("Co he thong 0x%02X, co dong co 0x%02X\r\n", b[33], b[34]);
}

static void show_config(uint8_t addr)
{
    static const char *pul[] = { "PUL_OFF", "PUL_ENA", "ESI_RCO", "pLR_ESI" };
    static const char *ser[] = { "OFF", "ESI_ALO", "UART", "CAN", "uLR_ESI" };
    static const char *en[]  = { "L", "H", "Hold" };
    static const char *rsp[] = { "None", "Receive", "Reached", "Both", "Other" };
    uint8_t b[36];
    zdt_status_t s = zdt_read_long(addr, 0x42, 0x6C, b, 36);
    if (s != ZDT_OK) { printf("Doc cau hinh driver %u: %s\r\n", addr, zdt_status_str(s)); return; }
    printf("--- Driver %u ---\r\n", addr);
    printf("Che do        : %s\r\n", b[4] ? "FOC vong kin" : "VONG HO");
    printf("P_Pul         : %s\r\n", b[5] < 4 ? pul[b[5]] : "?");
    printf("P_Serial      : %s\r\n", b[6] < 5 ? ser[b[6]] : "?");
    printf("En            : %s\r\n", b[7] < 3 ? en[b[7]] : "?");
    printf("Ma_Limit      : %u mA\r\n", u16(&b[15]));
    printf("Vm_Limit      : %u RPM\r\n", u16(&b[17]));
    printf("Response      : %s\r\n", b[24] < 5 ? rsp[b[24]] : "?");
    printf("Bao ve ket    : %u (toc do %u RPM, dong %u mA, %u ms)\r\n", b[26], u16(&b[27]), u16(&b[29]), u16(&b[31]));
    printf("Cua so toi noi: %u (x0.1 do)\r\n", u16(&b[33]));
}

/* ------------------------------------------------------------------ */
/* BANG THONG SO DA DO cho tung khop. STM32 tu ghi TAM vao driver moi lan khoi dong
 * (va khi go lenh "tune"). Khong luu vao flash driver, tat 24 V la driver ve mac dinh.
 * stiff = 0: khong ghi do cung.  vel_kp = 0: khong ghi PID. Kp vi tri giu mac dinh 126640.
 * Khop 1..3 co hop so nen de mac dinh driver. */
typedef struct {
    uint8_t  joint;
    uint32_t stiff;      /* he so cung */
    uint32_t vel_kp;     /* Kp vong toc do */
    uint32_t vel_ki;     /* Ki vong toc do */
} joint_tune_t;

static const joint_tune_t tune_table[] = {
    /* khop  do cung  Kp toc do  Ki toc do      (do ngay 2026-10-21) */
    {  4,    3000,    25000,     60 },
};

static void apply_tuning(void)
{
    for (unsigned i = 0; i < sizeof(tune_table) / sizeof(tune_table[0]); i++) {
        const joint_tune_t *t = &tune_table[i];
        joint_t *j = joint_get(t->joint);
        if (!j) continue;
        printf("Ghi thong so khop %u:", t->joint);
        if (t->stiff) {
            zdt_status_t s = zdt_write_stiffness(j->addr, t->stiff, 0);
            printf(" do cung %lu %s,", (unsigned long)t->stiff, s == ZDT_OK ? "OK" : zdt_status_str(s));
            HAL_Delay(20);
        }
        if (t->vel_kp) {
            zdt_pid_t p = { ZDT_PID_DEFAULT_POS_KP, ZDT_PID_DEFAULT_POS_KP, t->vel_kp, t->vel_ki };
            zdt_status_t s = zdt_write_pid(j->addr, &p, 0);
            printf(" PID toc do %lu/%lu %s", (unsigned long)t->vel_kp, (unsigned long)t->vel_ki,
                   s == ZDT_OK ? "OK" : zdt_status_str(s));
            HAL_Delay(20);
        }
        printf("\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* Chinh vong dieu khien driver (firmware X). Mac dinh KHONG luu: tat nguon driver la ve gia tri cu.
 * Them chu "luu" o cuoi lenh de luu vinh vien. */

static int is_store_word(const char *s) { return s && !strcmp(s, "luu"); }

static void show_pid(uint8_t n)
{
    zdt_pid_t p;
    zdt_status_t s = zdt_read_pid(joint_get(n)->addr, &p);
    if (s != ZDT_OK) { report("Doc PID", s); return; }
    printf("Khop %u PID: Kp vi tri (hinh thang) %lu, Kp vi tri (truc tiep) %lu, Kp toc do %lu, Ki toc do %lu\r\n",
           n, (unsigned long)p.pos_kp_trap, (unsigned long)p.pos_kp_direct,
           (unsigned long)p.vel_kp, (unsigned long)p.vel_ki);
}

/* Mac dinh he so cung phu thuoc firmware cua driver. Tra ve 0 neu khong doc duoc firmware */
static uint32_t stiff_default(uint8_t n)
{
    uint16_t fw = 0, hw = 0;
    if (zdt_read_version(joint_get(n)->addr, &fw, &hw) != ZDT_OK) return 0;
    return (fw >= 206) ? ZDT_STIFF_DEFAULT_FW206 : ZDT_STIFF_DEFAULT_FW204;
}

static void show_stiffness(uint8_t n)
{
    uint32_t v;
    zdt_status_t s = zdt_read_stiffness(joint_get(n)->addr, &v);
    if (s != ZDT_OK) { report("Doc he so cung", s); return; }
    printf("Khop %u he so cung: %lu (mac dinh theo firmware %lu)\r\n",
           n, (unsigned long)v, (unsigned long)stiff_default(n));
}

/* rig <n>  |  rig <n> <gia tri> [luu]  |  rig <n> md [luu] */
static void cmd_rig(int argc, char **argv)
{
    uint8_t n;
    if (!(n = arg_joint(argc > 1 ? argv[1] : NULL))) return;
    if (argc < 3) { show_stiffness(n); return; }

    uint32_t v;
    if (!strcmp(argv[2], "md")) {
        v = stiff_default(n);
        if (v == 0) { printf("Khong doc duoc firmware driver, khong biet mac dinh\r\n"); return; }
    } else {
        v = (uint32_t)strtoul(argv[2], NULL, 10);
    }
    if (v < 50 || v > 4000) { printf("He so cung phai trong 50..4000\r\n"); return; }
    uint8_t store = is_store_word(argc > 3 ? argv[3] : NULL);

    report(store ? "Ghi va LUU he so cung" : "Ghi tam he so cung", zdt_write_stiffness(joint_get(n)->addr, v, store));
    show_stiffness(n);
}

/* pid <n>
 * pid <n> <KpViTriHT> <KpViTriTT> <KpTocDo> <KiTocDo> [luu]
 * pid <n> v <KpTocDo> [KiTocDo] [luu]   chi doi vong toc do, giu nguyen vong vi tri
 * pid <n> md [luu]                      tra ve mac dinh */
static void cmd_pid(int argc, char **argv)
{
    uint8_t n;
    if (!(n = arg_joint(argc > 1 ? argv[1] : NULL))) return;
    if (argc < 3) { show_pid(n); return; }

    uint8_t   addr  = joint_get(n)->addr;
    uint8_t   store = is_store_word(argv[argc - 1]);
    int       nums  = argc - 2 - (store ? 1 : 0);      /* so tham so so sau <n> */
    zdt_pid_t p;

    if (!strcmp(argv[2], "md")) {
        p.pos_kp_trap = p.pos_kp_direct = ZDT_PID_DEFAULT_POS_KP;
        p.vel_kp = ZDT_PID_DEFAULT_VEL_KP;
        p.vel_ki = ZDT_PID_DEFAULT_VEL_KI;
    }
    else if (!strcmp(argv[2], "v")) {
        if (nums < 2) { printf("Vi du: pid 4 v 23000   hoac   pid 4 v 23000 26\r\n"); return; }
        zdt_status_t s = zdt_read_pid(addr, &p);          /* lay vong vi tri hien tai */
        if (s != ZDT_OK) { report("Doc PID", s); return; }
        p.vel_kp = (uint32_t)strtoul(argv[3], NULL, 10);
        if (nums >= 3) p.vel_ki = (uint32_t)strtoul(argv[4], NULL, 10);
    }
    else {
        if (nums < 4) { printf("Can du 4 so. Vi du: pid 4 126640 126640 15600 26\r\n"); return; }
        p.pos_kp_trap   = (uint32_t)strtoul(argv[2], NULL, 10);
        p.pos_kp_direct = (uint32_t)strtoul(argv[3], NULL, 10);
        p.vel_kp        = (uint32_t)strtoul(argv[4], NULL, 10);
        p.vel_ki        = (uint32_t)strtoul(argv[5], NULL, 10);
    }

    /* chan gia tri vo ly de tranh go nham (vd thieu/thua chu so) */
    if (p.pos_kp_trap < 1000 || p.pos_kp_trap > 500000 ||
        p.pos_kp_direct < 1000 || p.pos_kp_direct > 500000 ||
        p.vel_kp < 1000 || p.vel_kp > 100000 || p.vel_ki > 1000) {
        printf("Gia tri ngoai khoang cho phep: Kp vi tri 1000..500000, Kp toc do 1000..100000, Ki toc do 0..1000\r\n");
        return;
    }
    report(store ? "Ghi va LUU PID" : "Ghi tam PID", zdt_write_pid(addr, &p, store));
    show_pid(n);
}

/* ------------------------------------------------------------------ */

static void cmd_scan(void)
{
    printf("Quet bus CAN:\r\n");
    for (uint8_t n = 1; n <= JOINT_COUNT; n++) {
        uint16_t fw = 0, hw = 0;
        zdt_status_t s = zdt_read_version(joint_get(n)->addr, &fw, &hw);
        printf("  Khop %u (dia chi %u): ", n, joint_get(n)->addr);
        if (s == ZDT_OK) printf("firmware %u, phan cung 0x%04X\r\n", fw, hw);
        else             printf("%s\r\n", zdt_status_str(s));
    }
}

static void cmd_positions(void)
{
    for (uint8_t n = 1; n <= JOINT_COUNT; n++) {
        int32_t deg;
        joint_status_t s = joint_read(n, &deg);
        printf("  Khop %u: ", n);
        if (s == JOINT_OK) { print_x10(deg); printf(" do"); }
        else               printf("%s", joint_status_str(s));
        printf("   (dich "); print_x10(joint_get(n)->target_x10); printf(")\r\n");
    }
}

static void cmd_speed_table(void)
{
    printf("Khop | ti so | gioi han (do)  | toc do (do/s) | gia toc (do/s2)\r\n");
    for (uint8_t n = 1; n <= JOINT_COUNT; n++) {
        joint_t *j = joint_get(n);
        printf("  %u  | 1:%-3u | ", n, j->ratio);
        print_x10(j->min_x10); printf(" .. "); print_x10(j->max_x10);
        printf(" | "); print_x10(j->dps_x10);
        printf(" | "); print_x10(j->acc_x10); printf("\r\n");
    }
}

static void print_help(void)
{
    printf("\r\n--- LENH (goc tinh tai DAU RA khop, don vi do) ---\r\n");
    printf("  scan               quet 4 driver tren bus\r\n");
    printf("  en [n] / dis [n]   enable / disable khop n (bo trong = ca 4)\r\n");
    printf("                     CHU Y: dis khop 2 thi canh tay roi xuong theo trong luc\r\n");
    printf("  j <n> <do>         chay khop n toi goc tuyet doi (vd: j 2 45)\r\n");
    printf("  jr <n> <do>        chay khop n them mot goc (vd: jr 1 -10)\r\n");
    printf("  all <d1> <d2> <d3> <d4>  chay ca 4 khop cung bat dau, cung toi noi\r\n");
    printf("  ve0                ca 4 khop ve 0 cung luc\r\n");
    printf("  p                  doc goc thuc 4 khop\r\n");
    printf("  sp                 xem bang toc do / gioi han\r\n");
    printf("  sp <n> <do/s> [do/s2]  dat toc do (va gia toc) cho khop n\r\n");
    printf("  z <n>              lay vi tri hien tai cua khop n lam 0 do\r\n");
    printf("  x                  dung tat ca khop\r\n");
    printf("  c <n>              xoa bao ve ket truc / qua dong khop n\r\n");
    printf("  r <n>              xoa che do tu chay khi bat nguon cua driver n\r\n");
    printf("  t <n>              trang thai khop n\r\n");
    printf("  i <n> / f <n>      trang thai he thong / cau hinh driver n\r\n");
    printf("  k <n>              kiem tra encoder va hieu chuan driver n\r\n");
    printf("  b                  bat/tat in frame CAN\r\n");
    printf("  rig <n> [gt|md] [luu]        doc / ghi he so cung (md: fw204 384, fw206 1560)\r\n");
    printf("  pid <n>                      doc PID driver\r\n");
    printf("  pid <n> v <KpTD> [KiTD] [luu]  chi doi vong toc do\r\n");
    printf("  pid <n> <KpVT1> <KpVT2> <KpTD> <KiTD> [luu]   ghi ca 4 so\r\n");
    printf("  pid <n> md [luu]             tra PID ve mac dinh\r\n");
    printf("                     khong co 'luu' = ghi tam, tat nguon driver la mat\r\n");
    printf("  tune               ghi lai bang thong so da do (tu chay khi khoi dong)\r\n");
#if USE_SERVO
    printf("  o / n / u          gripper: mo / kep vat / dong het (chay cham)\r\n");
    printf("  s <us>             gripper: do rong xung 500..2500, s 0 = tha long\r\n");
#endif
    printf("  h                  hien danh sach lenh\r\n");
}

/* ------------------------------------------------------------------ */

static void execute(char *cmd)
{
    char *argv[ARG_MAX];
    int   argc = 0;
    for (char *tok = strtok(cmd, " "); tok && argc < ARG_MAX; tok = strtok(NULL, " ")) argv[argc++] = tok;
    if (argc == 0) return;

    const char *c  = argv[0];
    const char *a1 = (argc > 1) ? argv[1] : NULL;
    const char *a2 = (argc > 2) ? argv[2] : NULL;
    const char *a3 = (argc > 3) ? argv[3] : NULL;
    uint8_t n;

    if (!strcmp(c, "h")) {
        print_help();
    }
    else if (!strcmp(c, "scan")) {
        cmd_scan();
    }
    else if (!strcmp(c, "en") || !strcmp(c, "dis")) {
        int on = !strcmp(c, "en");
        n = 0;
        if (a1) { n = arg_joint(a1); if (!n) return; }
        joint_status_t s = joint_enable(n, on);
        if (n) jreport(on ? "Enable" : "Disable", n, s);
        else   printf("%s ca 4 khop: %s\r\n", on ? "Enable" : "Disable", joint_status_str(s));
    }
    else if (!strcmp(c, "j") || !strcmp(c, "jr")) {
        if (!(n = arg_joint(a1))) return;
        if (!a2) { printf("Thieu goc. Vi du: %s 2 45\r\n", c); return; }
        joint_t *j   = joint_get(n);
        int32_t  deg = parse_x10(a2);
        int32_t  goal = (c[1] == 'r') ? j->target_x10 + deg : deg;   /* goc dich tuyet doi */
        joint_status_t s = joint_move(n, goal);
        printf("Khop %u -> ", n); print_x10(goal);
        printf(" do: %s\r\n", joint_status_str(s));
        if (s == JOINT_ERR_LIMIT) {
            printf("  Gioi han khop %u: ", n); print_x10(j->min_x10); printf(" .. "); print_x10(j->max_x10); printf(" do\r\n");
        }
    }
    else if (!strcmp(c, "all") || !strcmp(c, "ve0")) {
        int32_t d[JOINT_COUNT] = { 0, 0, 0, 0 };
        if (!strcmp(c, "all")) {
            if (argc < 1 + JOINT_COUNT) { printf("Can du 4 goc. Vi du: all 0 45 30 0\r\n"); return; }
            for (int i = 0; i < JOINT_COUNT; i++) d[i] = parse_x10(argv[1 + i]);
        }
        joint_status_t s = joint_move_all(d);
        printf("Ca 4 khop -> ");
        for (int i = 0; i < JOINT_COUNT; i++) { print_x10(d[i]); printf(i < JOINT_COUNT - 1 ? " / " : " do: "); }
        printf("%s\r\n", joint_status_str(s));
    }
    else if (!strcmp(c, "p")) {
        cmd_positions();
    }
    else if (!strcmp(c, "sp")) {
        if (!a1) { cmd_speed_table(); return; }
        if (!(n = arg_joint(a1))) return;
        if (!a2) { printf("Thieu toc do. Vi du: sp 2 10 10\r\n"); return; }
        int32_t v = parse_x10(a2), a = a3 ? parse_x10(a3) : 0;
        if (v <= 0 || v > 65535 || a < 0 || a > 65535) { printf("Gia tri khong hop le\r\n"); return; }
        jreport("Dat toc do", n, joint_set_speed(n, (uint16_t)v, (uint16_t)a));
        cmd_speed_table();
    }
    else if (!strcmp(c, "z")) {
        if (!(n = arg_joint(a1))) return;
        jreport("Dat diem 0", n, joint_zero_here(n));
    }
    else if (!strcmp(c, "x")) {
        joint_stop_all();
        printf("Da gui lenh dung toi ca 4 khop\r\n");
    }
    else if (!strcmp(c, "c")) {
        if (!(n = arg_joint(a1))) return;
        report("Xoa bao ve", zdt_clear_protection(joint_get(n)->addr));
    }
    else if (!strcmp(c, "r")) {
        if (!(n = arg_joint(a1))) return;
        report("Xoa che do tu chay khi bat nguon", zdt_clear_autorun(joint_get(n)->addr));
    }
    else if (!strcmp(c, "t")) {
        if (!(n = arg_joint(a1))) return;
        uint8_t f;
        zdt_status_t s = zdt_read_flags(joint_get(n)->addr, &f);
        if (s == ZDT_OK) {
            printf("Khop %u, trang thai 0x%02X: %s, %s%s%s\r\n", n, f,
                   (f & ZDT_FLAG_ENABLED) ? "dang enable" : "chua enable",
                   (f & ZDT_FLAG_REACHED) ? "da toi noi" : "chua toi noi",
                   (f & ZDT_FLAG_STALL) ? ", DANG KET TRUC" : "",
                   (f & ZDT_FLAG_STALL_PROT) ? ", DA BAO VE KET TRUC (go c n)" : "");
        } else report("Doc trang thai", s);
    }
    else if (!strcmp(c, "i")) {
        if (!(n = arg_joint(a1))) return;
        show_status(joint_get(n)->addr);
    }
    else if (!strcmp(c, "f")) {
        if (!(n = arg_joint(a1))) return;
        show_config(joint_get(n)->addr);
    }
    else if (!strcmp(c, "k")) {
        if (!(n = arg_joint(a1))) return;
        uint8_t f;
        zdt_status_t s = zdt_read_home_flags(joint_get(n)->addr, &f);
        if (s == ZDT_OK) printf("Khop %u, he thong 0x%02X: encoder %s, hieu chuan %s\r\n", n, f,
                                (f & 0x01) ? "OK" : "LOI", (f & 0x02) ? "DA CO" : "CHUA CO");
        else report("Doc co he thong", s);
    }
    else if (!strcmp(c, "tune")) {
        apply_tuning();
    }
    else if (!strcmp(c, "rig")) {
        cmd_rig(argc, argv);
    }
    else if (!strcmp(c, "pid")) {
        cmd_pid(argc, argv);
    }
    else if (!strcmp(c, "b")) {
        zdt_debug = !zdt_debug;
        printf("In frame CAN: %s\r\n", zdt_debug ? "BAT" : "TAT");
    }
#if USE_SERVO
    else if (!strcmp(c, "o")) {
        servo_move_to(GRIP_OPEN_US, GRIP_STEP_US);
        printf("Gripper: dang mo, toi %u us\r\n", GRIP_OPEN_US);
    }
    else if (!strcmp(c, "n")) {
        servo_move_to(GRIP_HOLD_US, GRIP_STEP_US);
        printf("Gripper: dang kep vat, toi %u us\r\n", GRIP_HOLD_US);
    }
    else if (!strcmp(c, "u")) {
        servo_move_to(GRIP_CLOSE_US, GRIP_STEP_US);
        printf("Gripper: dang dong het, toi %u us\r\n", GRIP_CLOSE_US);
    }
    else if (!strcmp(c, "s")) {
        if (!a1) { printf("Servo dang o %u us\r\n", servo_get_us()); return; }
        servo_set_us((uint16_t)atoi(a1));
        printf("Servo: %u us\r\n", servo_get_us());
    }
#endif
    else {
        printf("Khong hieu lenh '%s'. Go h de xem danh sach.\r\n", c);
    }
}

void cmd_uart_init(void)
{
    line_len = 0;
    print_help();
    cmd_scan();
    apply_tuning();
    printf("Neu bao loi: bat 24 V roi go 'tune'. Tat bat lai 24 V ma STM32 van chay cung phai go 'tune'.\r\n");
    printf("> ");
}

void cmd_uart_poll(void)
{
#if USE_SERVO
    if (servo_update()) printf("\r\n[Gripper da toi %u us]\r\n> ", servo_get_us());
#endif

    uint8_t addr = zdt_poll_reached();
    if (addr) printf("\r\n[Khop %u da toi vi tri]\r\n> ", addr);

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) { __HAL_UART_CLEAR_OREFLAG(&huart1); }

    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        char ch = (char)(huart1.Instance->RDR & 0xFF);

        if (ch == '\r' || ch == '\n') {
            if (line_len > 0) {
                line[line_len] = '\0';
                printf("\r\n");
                execute(line);
                line_len = 0;
                printf("> ");
            }
        } else if ((ch == '\b' || ch == 0x7F) && line_len > 0) {
            line_len--;
            printf("\b \b");
        } else if (line_len < LINE_MAX - 1 && ch >= ' ' && ch <= '~') {
            line[line_len++] = ch;
            putchar(ch);
        }
    }
}
