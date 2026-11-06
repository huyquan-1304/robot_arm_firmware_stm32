#include "wheel.h"
#include "zdt_x42s.h"
#include "ros_link.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Chu vi banh / 3600: so mm di duoc ung voi 0.1 do quay cua banh */
#define MM_PER_DEG_X10   (2.0f * 3.14159265f * WHEEL_RADIUS_MM / 3600.0f)

static uint8_t  w_addr    = WHEEL_TEST_ADDR;
static uint16_t w_acc     = WHEEL_ACC_DEFAULT;
static uint8_t  w_timed   = 0;      /* 1 = dang chay co hen gio tu dung */
static uint32_t w_stop_at = 0;

/* ------------------------------------------------------------------ */

static void px10(int32_t v)
{
    if (v < 0) { putchar('-'); v = -v; }
    printf("%ld.%ld", (long)(v / 10), (long)(v % 10));
}

static int32_t parse_x10(const char *s)
{
    float f = strtof(s, NULL);
    return (int32_t)(f * 10.0f + ((f < 0) ? -0.5f : 0.5f));
}

static void rep(const char *what, zdt_status_t s)
{
    printf("Banh %u - %s: %s\r\n", w_addr, what, zdt_status_str(s));
}

/* mm/s tai mat san -> 0.1 RPM tai banh:  rpm = v / (2 pi r) * 60 */
static int32_t mm_s_to_rpm_x10(int32_t mm_s)
{
    float rpm = (float)mm_s * 60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_MM);
    return (int32_t)(rpm * 10.0f + ((rpm < 0) ? -0.5f : 0.5f));
}

zdt_status_t wheel_set_rpm_x10(uint8_t addr, int32_t rpm_x10)
{
    if (rpm_x10 >  WHEEL_MAX_RPM_X10) rpm_x10 =  WHEEL_MAX_RPM_X10;
    if (rpm_x10 < -WHEEL_MAX_RPM_X10) rpm_x10 = -WHEEL_MAX_RPM_X10;
    return zdt_speed(addr, rpm_x10, w_acc);      /* lenh F6, sync = 0, chay ngay */
}

/* Chay toc do, sec_arg = so giay tu dung (NULL = mac dinh, "0" = khong hen gio) */
static void run(int32_t rpm_x10, const char *sec_arg)
{
    if (rpm_x10 > WHEEL_MAX_RPM_X10 || rpm_x10 < -WHEEL_MAX_RPM_X10) {
        printf("Vuot gioi han +-"); px10(WHEEL_MAX_RPM_X10); printf(" RPM, da cat bot\r\n");
    }
    uint32_t sec = sec_arg ? (uint32_t)atoi(sec_arg) : WHEEL_RUN_S_DEFAULT;

    zdt_status_t s = wheel_set_rpm_x10(w_addr, rpm_x10);
    printf("Banh %u - toc do ", w_addr); px10(rpm_x10); printf(" RPM: %s\r\n", zdt_status_str(s));
    if (s != ZDT_OK) { w_timed = 0; return; }

    if (rpm_x10 == 0 || sec == 0u) {
        w_timed = 0;
        if (rpm_x10 != 0) printf("CHAY KHONG HEN GIO. Go 'bx x' de dung\r\n");
        return;
    }
    w_timed   = 1;
    w_stop_at = HAL_GetTick() + sec * 1000u;
    printf("Tu dung sau %lu s ('bx x' de dung ngay)\r\n", (unsigned long)sec);
}

void wheel_cancel_timer(void) { w_timed = 0; }

void wheel_task(void)
{
    if (!w_timed) return;
    if ((int32_t)(HAL_GetTick() - w_stop_at) < 0) return;

    zdt_status_t s = zdt_speed(w_addr, 0, w_acc);      /* ve 0 theo doc giam toc */
    if (s != ZDT_OK) s = zdt_stop(w_addr);             /* khong duoc thi dung ngay bang FE */
    if (s != ZDT_OK) { w_stop_at = HAL_GetTick() + 100u; return; }   /* thu lai sau 100 ms */

    w_timed = 0;
    if (!ros_link_active()) printf("\r\n[Banh %u: het gio, da dung]\r\n> ", w_addr);
}

/* ------------------------------------------------------------------ */

static void show_pos(void)
{
    int32_t rpm, pos;
    zdt_status_t s = zdt_read_speed(w_addr, &rpm);
    if (s != ZDT_OK) { rep("doc toc do", s); return; }
    s = zdt_read_position(w_addr, &pos);
    if (s != ZDT_OK) { rep("doc vi tri", s); return; }

    int32_t mm = (int32_t)((float)pos * MM_PER_DEG_X10);
    printf("Banh %u: toc do ", w_addr); px10(rpm);
    printf(" RPM | vi tri "); px10(pos);
    printf(" do = "); px10(pos / 360);                 /* 3600 (0.1 do) = 1 vong -> /360 ra 0.1 vong */
    printf(" vong = %ld mm lan tren san\r\n", (long)mm);
}

static void show_status(void)
{
    uint16_t fw, hw;
    printf("--- Banh, dia chi %u ---\r\n", w_addr);
    zdt_status_t s = zdt_read_version(w_addr, &fw, &hw);
    if (s != ZDT_OK) {
        printf("Khong doc duoc driver: %s\r\n", zdt_status_str(s));
        printf("Kiem tra: ID_Addr = %u, P_Serial = CAN1_MAP, CAN_Baud = 500000, day H/L, 120 ohm\r\n", w_addr);
        return;
    }
    printf("Firmware %u, phan cung 0x%04X\r\n", fw, hw);

    uint32_t gear;
    s = zdt_read_gear(w_addr, &gear);
    if (s == ZDT_OK) {
        printf("Ti so truyen: %lu.%02lu", (unsigned long)(gear / 100u), (unsigned long)(gear % 100u));
        if (gear != WHEEL_GEAR_X100) printf("   <- CHUA DUNG 131. Go: bx gear 131 luu");
        printf("\r\n");
    } else {
        rep("doc ti so truyen", s);
    }

    uint8_t f;
    s = zdt_read_flags(w_addr, &f);
    if (s == ZDT_OK) {
        /* Co 3A cua M370: bit0 Ens (enable), bit2 Cgi (qua dong), bit3 Cgp (bao ve ket truc) */
        printf("Enable: %u | bao ve ket truc: %u | bao ve qua dong: %u",
               (unsigned)(f & 0x01u), (unsigned)((f >> 3) & 1u), (unsigned)((f >> 2) & 1u));
        if (f & 0x0Cu) printf("   <- go 'bx c' de xoa bao ve");
        printf("\r\n");
    } else {
        rep("doc co trang thai", s);
    }
    uint8_t h;
    if (zdt_read_home_flags(w_addr, &h) == ZDT_OK)   /* 3B: bit0 Enc_Rdy, bit1 Cal_Rdy */
        printf("Encoder san sang: %u | da hieu chuan: %u%s\r\n", (unsigned)(h & 1u), (unsigned)((h >> 1) & 1u),
               (h & 0x02u) ? "" : "   <- go 'bx cal'");
    show_pos();
    printf("Gia toc lenh: %u RPM/s, gioi han toc do: ", w_acc); px10(WHEEL_MAX_RPM_X10); printf(" RPM\r\n");
}

/* Quet dia chi 1..32, in driver nao tra loi (1..4 la tay may) */
static void scan(void)
{
    uint8_t found = 0;
    printf("Quet dia chi 1..32 tren FDCAN1...\r\n");
    for (uint8_t a = 1; a <= 32; a++) {
        uint16_t fw, hw;
        if (zdt_read_version(a, &fw, &hw) == ZDT_OK) {
            printf("  Dia chi %2u: firmware %u, phan cung 0x%04X%s\r\n", a, fw, hw,
                   (a <= 4) ? "  (tay may)" : "");
            found++;
        }
    }
    printf("Tim thay %u driver\r\n", found);
}

/* ------------------------------------------------------------------ */

void wheel_print_help(void)
{
    printf("  --- BANH XE (driver M370, dang chon dia chi %u) ---\r\n", w_addr);
    printf("  bx                 trang thai banh: firmware, ti so truyen, co bao ve, toc do, vi tri\r\n");
    printf("  bx scan            quet dia chi 1..32, tim driver banh\r\n");
    printf("  bx a <dia chi>     chon banh de test (5 FR, 6 FL, 7 RL, 8 RR)\r\n");
    printf("  bx id <moi>        DOI DIA CHI driver dang chon va luu (vd: bx a 1 roi bx id 6)\r\n");
    printf("                     CHI cam 1 driver mang dia chi cu tren bus, rut tay may ra\r\n");
    printf("  bx cal             hieu chuan dong co (banh phai KHONG cham dat)\r\n");
    printf("  bx gear [ti so] [luu]  doc / ghi ti so truyen (vd: bx gear 131 luu)\r\n");
    printf("  bx en / bx dis     enable / tha banh\r\n");
    printf("  bx v <rpm> [giay]  chay toc do tai banh, tu dung sau %u s (0 = khong hen gio)\r\n", WHEEL_RUN_S_DEFAULT);
    printf("  bx mm <mm/s> [giay]  chay theo van toc dai tren san\r\n");
    printf("  bx q <vong>        quay dung so vong (kiem tra ti so truyen, vd: bx q 10)\r\n");
    printf("  bx p               doc toc do va vi tri banh\r\n");
    printf("  bx z               lay vi tri hien tai lam 0\r\n");
    printf("  bx acc <rpm/s>     dat gia toc lenh\r\n");
    printf("  bx x               dung banh ngay\r\n");
    printf("  bx c               xoa bao ve ket truc / qua dong\r\n");
}

void wheel_cmd(int argc, char **argv)
{
    const char *sub = (argc > 1) ? argv[1] : NULL;
    const char *a   = (argc > 2) ? argv[2] : NULL;
    const char *b   = (argc > 3) ? argv[3] : NULL;

    if (sub == NULL) {
        show_status();
    }
    else if (!strcmp(sub, "scan")) {
        scan();
    }
    else if (!strcmp(sub, "a")) {
        if (!a) { printf("Dang chon banh dia chi %u\r\n", w_addr); return; }
        int n = atoi(a);
        if (n < 1 || n > 32) { printf("Dia chi sai: %s (1..32)\r\n", a); return; }
        if (w_timed) { (void)zdt_stop(w_addr); w_timed = 0; }   /* dung banh cu truoc khi doi */
        w_addr = (uint8_t)n;
        printf("Chon banh dia chi %u%s\r\n", w_addr, (n <= 4) ? "  CHU Y: 1..4 la driver TAY MAY" : "");
    }
    else if (!strcmp(sub, "id")) {
        if (!a) { printf("Thieu dia chi moi. Vd: bx a 1 roi bx id 6\r\n"); return; }
        int n = atoi(a);
        if (n < 5 || n > 32) { printf("Dia chi moi sai: %s. Banh dung 5..32 (1..4 la tay may)\r\n", a); return; }
        uint16_t fw, hw;
        if (zdt_read_version((uint8_t)n, &fw, &hw) == ZDT_OK) {
            printf("Da co driver o dia chi %d tren bus, khong doi de tranh trung\r\n", n); return;
        }
        if (zdt_read_version(w_addr, &fw, &hw) != ZDT_OK) {
            printf("Khong thay driver o dia chi %u. Go 'bx scan' de tim dia chi hien tai\r\n", w_addr); return;
        }
        w_timed = 0;
        zdt_status_t s = zdt_write_id(w_addr, (uint8_t)n, 1);
        printf("Doi dia chi %u -> %d (luu): %s\r\n", w_addr, n, zdt_status_str(s));
        HAL_Delay(100);
        if (zdt_read_version((uint8_t)n, &fw, &hw) == ZDT_OK) {
            w_addr = (uint8_t)n;
            printf("Xac nhan: driver da tra loi o dia chi %u. Dang chon banh %u\r\n", w_addr, w_addr);
        } else {
            printf("Chua thay driver o dia chi %d. Tat bat lai nguon driver roi go 'bx scan'\r\n", n);
        }
    }
    else if (!strcmp(sub, "cal")) {
        w_timed = 0;
        rep("hieu chuan (banh se tu quay, cho vai giay)", zdt_calibrate(w_addr));
    }
    else if (!strcmp(sub, "gear")) {
        if (!a) {
            uint32_t g;
            zdt_status_t s = zdt_read_gear(w_addr, &g);
            if (s != ZDT_OK) { rep("doc ti so truyen", s); return; }
            printf("Banh %u: ti so truyen %lu.%02lu\r\n", w_addr, (unsigned long)(g / 100u), (unsigned long)(g % 100u));
            return;
        }
        float f = strtof(a, NULL);
        if (f < 1.0f || f > 1000.0f) { printf("Ti so truyen sai: %s\r\n", a); return; }
        uint32_t g = (uint32_t)(f * 100.0f + 0.5f);
        uint8_t store = (b && !strcmp(b, "luu")) ? 1u : 0u;
        rep(store ? "ghi va LUU ti so truyen" : "ghi tam ti so truyen (tat nguon la mat)",
            zdt_write_gear(w_addr, g, store));
    }
    else if (!strcmp(sub, "en") || !strcmp(sub, "dis")) {
        int on = !strcmp(sub, "en");
        if (!on) w_timed = 0;
        rep(on ? "enable" : "tha banh", zdt_enable(w_addr, on));
    }
    else if (!strcmp(sub, "v")) {
        if (!a) { printf("Thieu toc do. Vd: bx v 20   hoac   bx v -20 3\r\n"); return; }
        run(parse_x10(a), b);
    }
    else if (!strcmp(sub, "mm")) {
        if (!a) { printf("Thieu van toc. Vd: bx mm 100   (100 mm/s)\r\n"); return; }
        int32_t rpm_x10 = mm_s_to_rpm_x10(atoi(a));
        printf("%s mm/s = ", a); px10(rpm_x10); printf(" RPM\r\n");
        run(rpm_x10, b);
    }
    else if (!strcmp(sub, "q")) {
        if (!a) { printf("Thieu so vong. Vd: bx q 10\r\n"); return; }
        int32_t turns_x10 = parse_x10(a);
        if (turns_x10 == 0 || turns_x10 > WHEEL_TURN_MAX * 10 || turns_x10 < -WHEEL_TURN_MAX * 10) {
            printf("So vong sai (toi da +-%d)\r\n", WHEEL_TURN_MAX); return;
        }
        w_timed = 0;
        /* FD, tuong doi so voi vi tri THUC: 1 vong = 3600 (0.1 do) */
        zdt_status_t s = zdt_move(w_addr, turns_x10 * 360, WHEEL_TURN_RPM_X10, w_acc, w_acc,
                                  ZDT_MOVE_REL_CURRENT);
        printf("Banh %u - quay ", w_addr); px10(turns_x10); printf(" vong o "); px10(WHEEL_TURN_RPM_X10);
        printf(" RPM: %s\r\n", zdt_status_str(s));
        if (s == ZDT_OK) printf("Danh dau 1 diem tren banh de dem vong. Xong go 'bx p' xem vi tri\r\n");
    }
    else if (!strcmp(sub, "p")) {
        show_pos();
    }
    else if (!strcmp(sub, "z")) {
        rep("lay vi tri hien tai lam 0", zdt_zero_position(w_addr));
    }
    else if (!strcmp(sub, "acc")) {
        if (!a) { printf("Gia toc lenh: %u RPM/s\r\n", w_acc); return; }
        int v = atoi(a);
        if (v < 1 || v > 65535) { printf("Gia toc sai: %s (1..65535)\r\n", a); return; }
        w_acc = (uint16_t)v;
        printf("Gia toc lenh: %u RPM/s (ap dung tu lenh chay ke tiep)\r\n", w_acc);
    }
    else if (!strcmp(sub, "x")) {
        w_timed = 0;
        rep("dung ngay", zdt_stop(w_addr));
    }
    else if (!strcmp(sub, "c")) {
        rep("xoa bao ve", zdt_clear_protection(w_addr));
    }
    else {
        printf("Khong hieu 'bx %s'. Go h de xem danh sach.\r\n", sub);
    }
}
