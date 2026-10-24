// NBMON: a monitor on COM1 (3F8h, 115200 8N1, polled) to drive a board without keyboard, disk or OS. ASCII lines
// in (CR or LF), hex numbers, no echo; each command answers data lines, then "OK" or "ERR <reason>".
// Host side: retro-firmware tools/nbmon.py.

#include "config.h" // CONFIG_NEWBIOS_NBMON
#include "malloc.h" // malloc_tmphigh_free
#include "stacks.h" // yield
#include "string.h" // strcmp
#include "util.h" // timer_calc
#include "x86.h" // inb

#define COM1 0x3f8
#define CDECL __attribute__((regparm(0)))
#define LINE_MAX 300

u32 NbmonLo VARLOW, NbmonHi VARLOW;   // the RAM a payload may use

struct nbmon_api {   // tools/nbmon/nbmon_api.h
    void CDECL (*putc)(int c);
    int CDECL (*getc)(u32 timeout_ms);
    void CDECL (*mdelay)(u32 ms);
    u32 CDECL (*ticks)(void);
};

static void CDECL
com_putc(int c)
{
    int n;
    for (n = 0; n < 100000 && !(inb(COM1 + 5) & 0x20); n++)
        ;
    outb(c, COM1);
}

static int CDECL
com_getc(u32 timeout_ms)
{
    u32 end = timeout_ms == (u32)-1 ? 0 : timer_calc(timeout_ms);
    while (!(inb(COM1 + 5) & 0x01))
        if (end && timer_check(end))
            return -1;
    return inb(COM1);
}

static void CDECL
api_mdelay(u32 ms)
{
    mdelay(ms);
}

static u32 CDECL
api_ticks(void)
{
    return timer_read() / GET_GLOBAL(TimerKHz);
}

static void
put_str(const char *s)
{
    while (*s)
        com_putc(*s++);
}

static void
put_hex(u32 v, int digits)
{
    while (digits--)
        com_putc("0123456789abcdef"[(v >> (4 * digits)) & 15]);
}

static void
put_line(const char *s)
{
    put_str(s);
    put_str("\r\n");
}

// A line without its end; "nbmon" only within timeout_ms (-1: none); -1 on timeout.
static int
get_line(char *line, u32 timeout_ms)
{
    int len = 0;
    for (;;) {
        int c = com_getc(timeout_ms);
        if (c < 0)
            return -1;
        if (c == '\r' || c == '\n') {
            if (len)
                break;
        } else if (len < LINE_MAX - 1)
            line[len++] = c;
    }
    line[len] = 0;
    return len;
}

static int
hex_digit(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    c |= 0x20;
    return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

// The next hex number of *p into *v; 0 if none.
static int
get_hex(char **p, u32 *v)
{
    while (**p == ' ')
        (*p)++;
    if (hex_digit(**p) < 0)
        return 0;
    for (*v = 0; hex_digit(**p) >= 0; (*p)++)
        *v = *v << 4 | hex_digit(**p);
    return 1;
}

static u32
mem_read(u32 addr, int size)
{
    return size == 1 ? *(volatile u8 *)addr : size == 2 ? *(volatile u16 *)addr : *(volatile u32 *)addr;
}

static void
mem_write(u32 addr, int size, u32 v)
{
    if (size == 1)
        *(volatile u8 *)addr = v;
    else if (size == 2)
        *(volatile u16 *)addr = v;
    else
        *(volatile u32 *)addr = v;
}

static u32
crc32(u8 *p, u32 len)
{
    u32 crc = ~0;
    while (len--) {
        crc ^= *p++;
        int i;
        for (i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
    }
    return ~crc;
}

// One command: NULL, else the reason of its error; *leave = 1: back to the POST (boot).
static const char *
command(char *line, int *leave)
{
    char *p = line;
    u32 a, v;
    while (*p && *p != ' ')
        p++;
    int len = p - line;
    char op = line[0], kind = len == 2 ? line[1] : 0;
    int size = kind == 'b' ? 1 : kind == 'w' ? 2 : kind == 'l' ? 4 : 0;
    if (len == 2 && size && (op == 'r' || op == 'w' || op == 'i' || op == 'o')) {
        if (!get_hex(&p, &a))
            return "address";
        if (op == 'r') {
            u32 count = 1, i;
            get_hex(&p, &count);
            for (i = 0; i < count; i++, a += size) {
                if (!(i % (16 / size))) {
                    if (i)
                        put_str("\r\n");
                    put_hex(a, 8);
                    com_putc(':');
                }
                com_putc(' ');
                put_hex(mem_read(a, size), 2 * size);
            }
            put_str("\r\n");
        } else if (op == 'w') {
            if (!get_hex(&p, &v))
                return "value";
            do
                mem_write(a, size, v), a += size;
            while (get_hex(&p, &v));
        } else if (op == 'i') {
            put_hex(size == 1 ? inb(a) : size == 2 ? inw(a) : inl(a), 2 * size);
            put_str("\r\n");
        } else {
            if (!get_hex(&p, &v))
                return "value";
            if (size == 1)
                outb(v, a);
            else if (size == 2)
                outw(v, a);
            else
                outl(v, a);
        }
    } else if (!strcmp(line, "boot")) {
        *leave = 1;
    } else if (!strcmp(line, "reset")) {
        put_line("OK");
        outb(0x8f, 0x70);   // CMOS 0Fh = 0: a full boot
        outb(0, 0x71);
        reset();
    } else if (len == 2 && op == 'l' && kind == 'd') {
        if (!get_hex(&p, &a))
            return "address";
        while (*p == ' ')
            p++;
        int n;
        for (n = 0; hex_digit(p[0]) >= 0 && hex_digit(p[1]) >= 0; n++, p += 2)
            if (n < 128)
                *(u8 *)(a + n) = hex_digit(p[0]) << 4 | hex_digit(p[1]);
        if (*p || n > 128)
            return "data";
    } else if (len == 3 && !memcmp(line, "crc", 3)) {
        if (!get_hex(&p, &a) || !get_hex(&p, &v))
            return "address length";
        put_hex(crc32((u8 *)a, v), 8);
        put_str("\r\n");
    } else if (len == 4 && !memcmp(line, "call", 4)) {
        static const struct nbmon_api api = { com_putc, com_getc, api_mdelay, api_ticks };
        u32 arg = 0;
        if (!get_hex(&p, &a))
            return "address";
        get_hex(&p, &arg);
        u32 CDECL (*f)(u32, const struct nbmon_api *) = (void *)a;
        put_hex(f(arg, &api), 8);
        put_str("\r\n");
    } else
        return "command";
    return NULL;
}

static void
monitor(void)
{
    char line[LINE_MAX];
    put_line("OK");
    for (;;) {
        get_line(line, -1);
        int leave = 0;
        const char *err = command(line, &leave);
        if (err) {
            put_str("ERR ");
            put_line(err);
        } else
            put_line("OK");
        if (leave)
            return;
    }
}

// POST start: the UART (FIFO on), the start line.
void
nbmon_start(void)
{
    if (!CONFIG_NEWBIOS_NBMON)
        return;
    outb(0x83, COM1 + 3);
    outb(1, COM1);
    outb(0, COM1 + 1);
    outb(0x03, COM1 + 3);
    outb(0x00, COM1 + 1);
    outb(0x07, COM1 + 2);
    outb(0x03, COM1 + 4);
    put_line("NBMON start");
}

// "NBMON ready <lo>-<hi>" (the RAM a payload may use), then 2 s for the line "nbmon": the monitor until "boot".
void
nbmon_window(void)
{
    if (!CONFIG_NEWBIOS_NBMON)
        return;
    put_str("NBMON ready ");
    put_hex(GET_LOW(NbmonLo), 8);
    com_putc('-');
    put_hex(GET_LOW(NbmonHi), 8);
    put_str("\r\n");
    char line[8];
    int len = 0;
    u32 end = timer_calc(2000);
    while (!timer_check(end)) {
        if (!(inb(COM1 + 5) & 0x01)) {
            yield();
            continue;
        }
        char c = inb(COM1);
        if (c != '\r' && c != '\n') {
            if (len < sizeof(line) - 1)
                line[len++] = c;
            continue;
        }
        line[len] = 0;
        if (!strcmp(line, "nbmon")) {
            monitor();
            return;
        }
        len = 0;
    }
}

// The POST's window: the largest free range of its temporary high zone, kept for the boot failure retries.
void
nbmon_post(void)
{
    if (!CONFIG_NEWBIOS_NBMON)
        return;
    u32 s, e, lo = 0, hi = 0;
    int n;
    for (n = 0; malloc_tmphigh_free(n, &s, &e); n++)
        if (e - s > hi - lo)
            lo = s, hi = e;
    SET_LOW(NbmonLo, lo);
    SET_LOW(NbmonHi, hi);
    nbmon_window();
}
