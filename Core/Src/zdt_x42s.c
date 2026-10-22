#include "zdt_x42s.h"
#include "bsp_can.h"
#include "main.h"
#include <stdio.h>

#define ZDT_CHECKSUM      0x6B
#define ZDT_REPLY_TIMEOUT 50     /* ms */

uint8_t zdt_debug = 0;   /* 1 = in moi frame CAN gui/nhan ra terminal */

static void dbg_frame(const char *dir, uint32_t id, const uint8_t *d, uint8_t n)
{
    if (!zdt_debug) return;
    printf("  [%s 0x%03lX]", dir, (unsigned long)id);
    for (uint8_t i = 0; i < n; i++) printf(" %02X", d[i]);
    printf("\r\n");
}

/* Thong bao "da toi noi" (xx 9F 6B) co the den bat ky luc nao, ke ca luc dang cho tra loi
 * lenh khac. Ghi lai theo dia chi (bit n = driver n) de khong bi mat. */
static volatile uint32_t reached_mask = 0;

static void note_frame(uint32_t id, const uint8_t *d, uint8_t n)
{
    uint8_t addr = (uint8_t)((id >> 8) & 0xFF);
    if (n == 3 && d[1] == 0x9F && addr > 0 && addr < 32) reached_mask |= (1u << addr);
}

/* Doc het frame dang cho trong FIFO: ghi nhan thong bao toi noi, bo cac frame cu khac */
static void zdt_drain(void)
{
    uint32_t id;
    uint8_t  d[8], n;
    while (bsp_can_recv(&id, d, &n)) {
        dbg_frame("RX", id, d, n);
        note_frame(id, d, n);
    }
}

/* ------------------------------------------------------------------ */
/* Gui 1 lenh qua CAN theo quy tac cua ZDT (manual muc 4.2.1):
 *   - ID mo rong = (dia chi << 8) | so thu tu goi
 *   - Byte dia chi KHONG nam trong du lieu
 *   - Lenh <= 8 byte: 1 frame, goi 0
 *   - Lenh > 8 byte : tach goi, MOI goi bat dau bang ma lenh, toi da 7 byte tiep theo
 * cmd = [ma lenh, du lieu..., 0x6B], len = so byte cua cmd */
static zdt_status_t zdt_send(uint8_t addr, const uint8_t *cmd, uint8_t len)
{
    uint32_t base_id = ((uint32_t)addr) << 8;

    if (len <= 8) {
        dbg_frame("TX", base_id, cmd, len);
        return (bsp_can_send(base_id, cmd, len) == HAL_OK) ? ZDT_OK : ZDT_ERR_CAN;
    }

    uint8_t        code   = cmd[0];
    const uint8_t *rest   = &cmd[1];
    uint8_t        remain = (uint8_t)(len - 1);
    uint8_t        packet = 0;
    uint8_t        frame[8];

    while (remain > 0) {
        uint8_t n = (remain > 7) ? 7 : remain;
        frame[0] = code;
        for (uint8_t i = 0; i < n; i++) frame[1 + i] = rest[i];
        dbg_frame("TX", base_id | packet, frame, (uint8_t)(n + 1));
        if (bsp_can_send(base_id | packet, frame, (uint8_t)(n + 1)) != HAL_OK) return ZDT_ERR_CAN;
        rest   += n;
        remain  = (uint8_t)(remain - n);
        packet++;
    }
    return ZDT_OK;
}

/* Cho frame tra loi tu driver 'addr' co byte dau = 'code'.
 * Bo qua thong bao "da toi vi tri" (xx 9F 6B) de khong nham voi tra loi lenh. */
static zdt_status_t zdt_wait(uint8_t addr, uint8_t code, uint8_t *data, uint8_t *len)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t id;
    uint8_t  d[8];
    uint8_t  n;

    while (HAL_GetTick() - t0 < ZDT_REPLY_TIMEOUT) {
        if (!bsp_can_recv(&id, d, &n)) continue;
        dbg_frame("RX", id, d, n);
        note_frame(id, d, n);
        if (((id >> 8) & 0xFF) != addr) continue;
        if (n < 2 || d[0] != code)      continue;
        if (n == 3 && d[1] == 0x9F)     continue;   /* thong bao toi noi, khong phai tra loi */
        for (uint8_t i = 0; i < n; i++) data[i] = d[i];
        *len = n;
        return ZDT_OK;
    }
    return ZDT_ERR_TIMEOUT;
}

/* Lenh dieu khien: tra loi dang [code, 02/E2/EE, 6B] */
static zdt_status_t zdt_cmd_ack(uint8_t addr, const uint8_t *cmd, uint8_t len)
{
    zdt_drain();
    zdt_status_t s = zdt_send(addr, cmd, len);
    if (s != ZDT_OK) return s;

    uint8_t d[8], n;
    s = zdt_wait(addr, cmd[0], d, &n);
    if (s != ZDT_OK) return s;

    switch (d[1]) {
    case 0x02: return ZDT_OK;
    case 0xE2: return ZDT_ERR_PARAM;
    case 0xEE: return ZDT_ERR_FORMAT;
    default:   return ZDT_ERR_OTHER;
    }
}

/* Lenh doc: gui [code, 6B], nhan du lieu tra ve */
static zdt_status_t zdt_cmd_read(uint8_t addr, uint8_t code, uint8_t *data, uint8_t *len)
{
    uint8_t cmd[2] = { code, ZDT_CHECKSUM };
    zdt_drain();
    zdt_status_t s = zdt_send(addr, cmd, 2);
    if (s != ZDT_OK) return s;
    s = zdt_wait(addr, code, data, len);
    if (s == ZDT_OK && *len == 3 && data[1] == 0xEE) return ZDT_ERR_FORMAT;
    return s;
}

/* Lenh doc co tra loi dai (nhieu frame CAN, moi frame bat dau bang ma lenh).
 * Gui cmd (cmd[0] = ma lenh), ghep tra loi thanh buf = [code, du lieu...].
 * need = so byte can co (tinh ca code). */
static zdt_status_t read_long_raw(uint8_t addr, const uint8_t *cmd, uint8_t cmd_len,
                                  uint8_t *buf, uint8_t need)
{
    uint8_t code = cmd[0];
    zdt_drain();
    zdt_status_t s = zdt_send(addr, cmd, cmd_len);
    if (s != ZDT_OK) return s;

    /* Driver gui lien tiep nhieu frame, FIFO nhan cua G4 chi chua 3 frame.
     * Vi vay KHONG in debug trong luc nhan (in qua UART rat cham, lam tran FIFO),
     * luu lai roi in sau khi nhan xong. */
    uint8_t  idx = 0;
    uint32_t t0  = HAL_GetTick();
    uint32_t id;
    uint8_t  d[8], n;
    uint32_t log_id[8];
    uint8_t  log_d[8][8], log_n[8], logs = 0;
    zdt_status_t r = ZDT_ERR_TIMEOUT;

    buf[idx++] = code;
    while (idx < need && HAL_GetTick() - t0 < 200) {
        if (!bsp_can_recv(&id, d, &n)) continue;
        if (logs < 8) {
            log_id[logs] = id; log_n[logs] = n;
            for (uint8_t i = 0; i < n; i++) log_d[logs][i] = d[i];
            logs++;
        }
        note_frame(id, d, n);
        if (((id >> 8) & 0xFF) != addr || n < 2 || d[0] != code) continue;
        if (n == 3 && (d[1] == 0xEE || d[1] == 0xE2)) { r = ZDT_ERR_FORMAT; break; }
        for (uint8_t i = 1; i < n && idx < need; i++) buf[idx++] = d[i];
    }
    if (idx >= need) r = ZDT_OK;
    for (uint8_t k = 0; k < logs; k++) dbg_frame("RX", log_id[k], log_d[k], log_n[k]);
    return r;
}

zdt_status_t zdt_read_long(uint8_t addr, uint8_t code, uint8_t aux, uint8_t *buf, uint8_t need)
{
    uint8_t cmd[3] = { code, aux, ZDT_CHECKSUM };
    return read_long_raw(addr, cmd, 3, buf, need);
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* ------------------------------------------------------------------ */

const char *zdt_status_str(zdt_status_t s)
{
    switch (s) {
    case ZDT_OK:          return "OK";
    case ZDT_ERR_PARAM:   return "E2 (sai tham so / dang bao ve)";
    case ZDT_ERR_FORMAT:  return "EE (sai dinh dang lenh)";
    case ZDT_ERR_TIMEOUT: return "khong co tra loi";
    case ZDT_ERR_CAN:     return "loi gui CAN";
    default:              return "loi khac";
    }
}

zdt_status_t zdt_read_version(uint8_t addr, uint16_t *fw, uint16_t *hw)
{
    uint8_t d[8], n;
    zdt_status_t s = zdt_cmd_read(addr, 0x1F, d, &n);
    if (s != ZDT_OK) return s;
    if (n < 6) return ZDT_ERR_OTHER;
    *fw = (uint16_t)((d[1] << 8) | d[2]);
    *hw = (uint16_t)((d[3] << 8) | d[4]);
    return ZDT_OK;
}

zdt_status_t zdt_enable(uint8_t addr, int on)   /* manual 5.3.2: F3 AB en sync 6B */
{
    uint8_t cmd[5] = { 0xF3, 0xAB, (uint8_t)(on ? 1 : 0), 0x00, ZDT_CHECKSUM };
    return zdt_cmd_ack(addr, cmd, 5);
}

zdt_status_t zdt_stop(uint8_t addr)              /* manual 5.3.15: FE 98 sync 6B */
{
    uint8_t cmd[4] = { 0xFE, 0x98, 0x00, ZDT_CHECKSUM };
    return zdt_cmd_ack(addr, cmd, 4);
}

zdt_status_t zdt_zero_position(uint8_t addr)     /* manual 5.2.3: 0A 6D 6B */
{
    uint8_t cmd[3] = { 0x0A, 0x6D, ZDT_CHECKSUM };
    return zdt_cmd_ack(addr, cmd, 3);
}

zdt_status_t zdt_clear_protection(uint8_t addr)  /* manual 5.2.4: 0E 52 6B */
{
    uint8_t cmd[3] = { 0x0E, 0x52, ZDT_CHECKSUM };
    return zdt_cmd_ack(addr, cmd, 3);
}

/* manual 5.7.1 (firmware X): xoa bo toc do "tu chay khi bat nguon" da luu
 * F7 1C | 00 = xoa | huong | acc(2) | toc do(2) | En pin | 6B  -> 10 byte, 2 goi CAN */
zdt_status_t zdt_clear_autorun(uint8_t addr)
{
    uint8_t cmd[10] = { 0xF7, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ZDT_CHECKSUM };
    return zdt_cmd_ack(addr, cmd, 10);
}

/* manual 5.3.5 (firmware X): che do toc do, lenh ngan 1 frame
 * F6 | huong | acc(2) | toc do 0.1RPM (2) | sync | 6B */
zdt_status_t zdt_speed(uint8_t addr, int32_t rpm_x10, uint16_t acc)
{
    uint8_t  dir = (rpm_x10 < 0) ? 1 : 0;
    uint32_t v   = (uint32_t)((rpm_x10 < 0) ? -rpm_x10 : rpm_x10);
    if (v > 30000) v = 30000;
    uint8_t cmd[8] = { 0xF6, dir, (uint8_t)(acc >> 8), (uint8_t)acc,
                       (uint8_t)(v >> 8), (uint8_t)v, 0x00, ZDT_CHECKSUM };
    return zdt_cmd_ack(addr, cmd, 8);
}

/* manual 5.3.10 (firmware X):
 * FD | huong | acc(2) | dec(2) | toc do 0.1RPM (2) | goc 0.1 do (4) | kieu chay | sync | 6B
 * Tong 15 byte (khong tinh dia chi) -> tach thanh 2 goi CAN */
zdt_status_t zdt_move_ex(uint8_t addr, int32_t deg_x10, uint16_t rpm_x10,
                         uint16_t acc, uint16_t dec, zdt_move_mode_t mode, uint8_t sync)
{
    uint8_t  dir = (deg_x10 < 0) ? 0x01 : 0x00;           /* 00 = CW, 01 = CCW */
    uint32_t pos = (uint32_t)((deg_x10 < 0) ? -deg_x10 : deg_x10);
    if (rpm_x10 > 30000) rpm_x10 = 30000;

    uint8_t cmd[15] = {
        0xFD, dir,
        (uint8_t)(acc >> 8),     (uint8_t)acc,
        (uint8_t)(dec >> 8),     (uint8_t)dec,
        (uint8_t)(rpm_x10 >> 8), (uint8_t)rpm_x10,
        (uint8_t)(pos >> 24), (uint8_t)(pos >> 16), (uint8_t)(pos >> 8), (uint8_t)pos,
        (uint8_t)mode,
        (uint8_t)(sync ? 1 : 0),  /* 0 = chay ngay, 1 = cho lenh dong bo FF 66 */
        ZDT_CHECKSUM
    };
    return zdt_cmd_ack(addr, cmd, 15);
}

zdt_status_t zdt_move(uint8_t addr, int32_t deg_x10, uint16_t rpm_x10,
                      uint16_t acc, uint16_t dec, zdt_move_mode_t mode)
{
    return zdt_move_ex(addr, deg_x10, rpm_x10, acc, dec, mode, 0);
}

/* manual 5.3.16: gui "00 FF 66 6B" (dia chi quang ba 0) de cac driver da nhan lenh
 * co sync = 1 cung bat dau chay. Khong cho tra loi vi nhieu driver cung tra loi. */
zdt_status_t zdt_sync_trigger(void)
{
    uint8_t cmd[3] = { 0xFF, 0x66, ZDT_CHECKSUM };
    zdt_drain();
    zdt_status_t s = zdt_send(0, cmd, 3);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 20) zdt_drain();   /* nhan va bo cac tra loi "FF 02" */
    return s;
}

zdt_status_t zdt_read_position(uint8_t addr, int32_t *deg_x10)   /* manual 5.5.13 */
{
    uint8_t d[8], n;
    zdt_status_t s = zdt_cmd_read(addr, 0x36, d, &n);
    if (s != ZDT_OK) return s;
    if (n < 7) return ZDT_ERR_OTHER;
    int32_t v = (int32_t)be32(&d[2]);        /* firmware X: don vi 0.1 do */
    *deg_x10 = (d[1] == 0x01) ? -v : v;
    return ZDT_OK;
}

zdt_status_t zdt_read_speed(uint8_t addr, int32_t *rpm_x10)      /* manual 5.5.11 */
{
    uint8_t d[8], n;
    zdt_status_t s = zdt_cmd_read(addr, 0x35, d, &n);
    if (s != ZDT_OK) return s;
    if (n < 5) return ZDT_ERR_OTHER;
    int32_t v = (int32_t)((d[2] << 8) | d[3]);  /* firmware X: don vi 0.1 RPM */
    *rpm_x10 = (d[1] == 0x01) ? -v : v;
    return ZDT_OK;
}

zdt_status_t zdt_read_home_flags(uint8_t addr, uint8_t *flags)   /* manual 5.4.4 */
{
    uint8_t d[8], n;
    zdt_status_t s = zdt_cmd_read(addr, 0x3B, d, &n);
    if (s != ZDT_OK) return s;
    if (n < 2) return ZDT_ERR_OTHER;
    *flags = d[1];
    return ZDT_OK;
}

zdt_status_t zdt_read_flags(uint8_t addr, uint8_t *flags)        /* manual 5.5.15 */
{
    uint8_t d[8], n;
    zdt_status_t s = zdt_cmd_read(addr, 0x3A, d, &n);
    if (s != ZDT_OK) return s;
    if (n < 3) return ZDT_ERR_OTHER;
    *flags = d[1];
    return ZDT_OK;
}

/* ------------------------------------------------------------------ */
/* Tham so vong dieu khien (firmware X, X42S) */

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

/* manual 5.6.14: gui 21 6B, tra ve 21 | pTkp(4) | pBkp(4) | vkp(4) | vki(4) | 6B
 * = 18 byte -> 3 frame CAN */
zdt_status_t zdt_read_pid(uint8_t addr, zdt_pid_t *p)
{
    uint8_t cmd[2] = { 0x21, ZDT_CHECKSUM };
    uint8_t b[18];
    zdt_status_t s = read_long_raw(addr, cmd, 2, b, 18);
    if (s != ZDT_OK) return s;
    p->pos_kp_trap   = be32(&b[1]);
    p->pos_kp_direct = be32(&b[5]);
    p->vel_kp        = be32(&b[9]);
    p->vel_ki        = be32(&b[13]);
    return ZDT_OK;
}

/* manual 5.6.15: 4A C3 | luu | pTkp(4) | pBkp(4) | vkp(4) | vki(4) | 6B = 20 byte -> 3 frame CAN */
zdt_status_t zdt_write_pid(uint8_t addr, const zdt_pid_t *p, uint8_t store)
{
    uint8_t cmd[20];
    cmd[0] = 0x4A; cmd[1] = 0xC3; cmd[2] = store ? 1 : 0;
    put32(&cmd[3],  p->pos_kp_trap);
    put32(&cmd[7],  p->pos_kp_direct);
    put32(&cmd[11], p->vel_kp);
    put32(&cmd[15], p->vel_ki);
    cmd[19] = ZDT_CHECKSUM;
    return zdt_cmd_ack(addr, cmd, 20);
}

/* manual 5.6.26: gui 23 6B, tra ve 23 | he so cung(4) | 6B */
zdt_status_t zdt_read_stiffness(uint8_t addr, uint32_t *v)
{
    uint8_t d[8], n;
    zdt_status_t s = zdt_cmd_read(addr, 0x23, d, &n);
    if (s != ZDT_OK) return s;
    if (n < 6) return ZDT_ERR_OTHER;
    *v = be32(&d[1]);
    return ZDT_OK;
}

/* manual 5.6.27: 4B 57 | luu | he so cung(4) | 6B = 8 byte, 1 frame */
zdt_status_t zdt_write_stiffness(uint8_t addr, uint32_t v, uint8_t store)
{
    uint8_t cmd[8];
    cmd[0] = 0x4B; cmd[1] = 0x57; cmd[2] = store ? 1 : 0;
    put32(&cmd[3], v);
    cmd[7] = ZDT_CHECKSUM;
    return zdt_cmd_ack(addr, cmd, 8);
}

uint8_t zdt_poll_reached(void)
{
    zdt_drain();
    for (uint8_t a = 1; a < 32; a++) {
        if (reached_mask & (1u << a)) {
            reached_mask &= ~(1u << a);
            return a;
        }
    }
    return 0;
}
