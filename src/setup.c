// Award-style CMOS setup utility (DEL during POST), the settings it keeps in CMOS (newbios_cmos.h)
// and the POST options SeaBIOS applies from them.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_BDA
#include "block.h" // struct drive_s
#include "e820map.h" // e820_list
#include "malloc.h" // malloc_tmphigh
#include "bregs.h" // struct bregs
#include "hw/pci.h" // pci_config_readb
#include "hw/pci_regs.h" // PCI_INTERRUPT_LINE
#include "hw/pcidevice.h" // foreachpci
#include "hw/ps2port.h" // ps2_kbd_command
#include "hw/rtc.h" // rtc_read
#include "newbios_cmos.h" // NB_*
#include "output.h" // snprintf
#include "stacks.h" // call16_int
#include "string.h" // memset
#include "util.h" // setup_menu
#include "x86.h" // cpuid

u8 Setting[NB_COUNT];
static int SetupDefaulted, ChipsetFallback;   // POST warnings, see setup_post_warnings()
u8 SetupL1Off VARFSEG;                          // "CPU Internal Cache" disabled, for handle_resume()
static u16 Password;   // hash, 0 = none

static const u8 Defaults[NB_COUNT] = {
    [NB_L1_CACHE] = 1, [NB_L2_CACHE] = 1,
    [NB_BOOT1] = 1, [NB_BOOT2] = 2, [NB_BOOT3] = 3, [NB_BOOT4] = 4,
    [NB_BOOT_DELAY] = 1, [NB_NUMLOCK] = 1, [NB_QUICK_POST] = 1, [NB_CHIPSET_AUTO] = 1, [NB_APM] = 1,
    [NB_IDE_32BIT] = 1, [NB_IDE_PRIMARY] = 1, [NB_IDE_SECONDARY] = 1, [NB_FDC] = 1,
    [NB_COM1] = 1, [NB_COM2] = 1, [NB_LPT] = 1,
    // manual chipset values: those of the original BIOS with auto configuration off
    [NB_DRAM_READ_WS] = 2, [NB_DRAM_WRITE_WS] = 2, [NB_L2_WS] = 1, [NB_SYSBIOS_CACHE] = 1, [NB_VIDBIOS_CACHE] = 1, [NB_HOST_PCI_POST_WS] = 1,
    [NB_HOST_PCI_BURST] = 1, [NB_PCI_BUS_PARK] = 1, [NB_PCI_POSTED_WRITE] = 1,
    [NB_SLOW_REFRESH] = 1,
};

#define SIO_INDEX 0x108
#define SIO_DATA  0x109
#define HB_BDF    pci_to_bdf(0, 0x10, 0)
#define IBC_BDF   pci_to_bdf(0, 0x12, 0)
#define IDE_BDF   pci_to_bdf(0, 0x12, 1)


/****************************************************************
 * CMOS
 ****************************************************************/

// AT standard checksum area 10h-2Dh, sum at 2Eh-2Fh.
#define CMOS_AT_SUM 0x2e

static u16
cmos_sum(u8 first, u8 last)
{
    u16 sum = 0;
    int i;
    for (i = first; i <= last; i++)
        sum += rtc_read(i);
    return sum;
}

static void
cmos_put_sum(u8 first, u8 last, u8 at)
{
    u16 sum = cmos_sum(first, last);
    rtc_write(at, sum >> 8);
    rtc_write(at + 1, sum);
}

static int
cmos_valid(void)
{
    u16 sum = rtc_read(NB_CMOS_SUM) << 8 | rtc_read(NB_CMOS_SUM + 1);
    return rtc_read(NB_CMOS_SIG) == NB_SIG_VALUE
        && sum == cmos_sum(NB_CMOS_BASE, NB_CMOS_SIG);
}

// Memory above 1 MiB: up to the end of the last RAM range, with what coreboot keeps there (CBMEM).
static u32
ext_mem_kb(void)
{
    u32 top = 0x100000;
    int i;
    for (i = 0; i < e820_count; i++) {
        u64 end = e820_list[i].start + e820_list[i].size;
        if (e820_list[i].start >= 0x100000 && e820_list[i].start < 0xf0000000 && end > top)
            top = end;
    }
    return (top - 0x100000) / 1024;
}

// Write the settings and the AT standard bytes that mirror them (floppy types, equipment, memory).
static void
cmos_write(const u8 *val, u16 password)
{
    int i;
    for (i = 0; i < NB_COUNT; i += 2)
        rtc_write(NB_CMOS_BASE + i / 2, val[i] | (i + 1 < NB_COUNT ? val[i + 1] << 4 : 0));
    rtc_write(NB_CMOS_PASSWORD, password);
    rtc_write(NB_CMOS_PASSWORD + 1, password >> 8);
    rtc_write(NB_CMOS_SIG, NB_SIG_VALUE);
    cmos_put_sum(NB_CMOS_BASE, NB_CMOS_SIG, NB_CMOS_SUM);

    int floppies = !!val[NB_FLOPPY_A] + !!val[NB_FLOPPY_B];
    u32 extkb = ext_mem_kb() > 0xffff ? 0xffff : ext_mem_kb();
    rtc_write(CMOS_FLOPPY_DRIVE_TYPE, val[NB_FLOPPY_A] << 4 | val[NB_FLOPPY_B]);
    rtc_write(CMOS_DISK_DATA, 0);
    rtc_write(CMOS_EQUIPMENT_INFO, (floppies ? (floppies - 1) << 6 | 1 : 0) | 0x02);
    rtc_write(0x15, 640 & 0xff);
    rtc_write(0x16, 640 >> 8);
    rtc_write(0x17, extkb);
    rtc_write(0x18, extkb >> 8);
    rtc_write(CMOS_MEM_EXTMEM_LOW, extkb);
    rtc_write(CMOS_MEM_EXTMEM_HIGH, extkb >> 8);
    cmos_put_sum(0x10, 0x2d, CMOS_AT_SUM);
}


/****************************************************************
 * Settings applied by SeaBIOS
 ****************************************************************/

static u8
sio_read(u8 reg)
{
    outb(reg, SIO_INDEX);
    return inb(SIO_DATA);
}

// UM8663BF super I/O: C0h enables FDC/UART1/UART2/LPT (bits 0-3), C1h selects their addresses and
// the parallel port mode (same encoding as the original BIOS).
static void
superio_apply(void)
{
    static const u8 lpt_mode[] = { 0x80, 0x40, 0xc0 };   // SPP, EPP, ECP
    u8 c0 = 0, c1 = lpt_mode[Setting[NB_LPT_MODE]];
    if (Setting[NB_FDC])
        c0 |= 0x01, c1 |= 0x01;
    if (Setting[NB_COM1])
        c0 |= 0x02, c1 |= Setting[NB_COM1] == 1 ? 0x02 : 0;
    if (Setting[NB_COM2])
        c0 |= 0x04, c1 |= Setting[NB_COM2] == 1 ? 0x04 : 0;
    if (Setting[NB_LPT])
        c0 |= 0x08, c1 |= Setting[NB_LPT] == 1 ? 0x08 : 0;
    outb(0xaa, SIO_INDEX);
    c0 |= sio_read(0xc0) & 0xf0;
    c1 |= sio_read(0xc1) & 0x30;
    outb(0xc0, SIO_INDEX);
    outb(c0, SIO_DATA);
    outb(0xc1, SIO_INDEX);
    outb(c1, SIO_DATA);
    outb(0x55, SIO_INDEX);
}

static int
onboard_ide(void)
{
    return pci_config_readw(IDE_BDF, PCI_VENDOR_ID) == 0x1060;
}

// UM8886BF IDE function: register 41h bit 7 primary, bit 6 secondary channel.
static void
ide_channels_apply(void)
{
    if (!onboard_ide())
        return;
    u8 v = pci_config_readb(IDE_BDF, 0x41) & ~0xc0;
    v |= (Setting[NB_IDE_PRIMARY] ? 0x80 : 0) | (Setting[NB_IDE_SECONDARY] ? 0x40 : 0);
    pci_config_writeb(IDE_BDF, 0x41, v);
}

// Drive 0-3 (primary master ... secondary slave) of the onboard controller, or -1.
int
setup_ide_index(int pci_bdf, int ataid, int slave)
{
    return (pci_bdf == IDE_BDF && ataid < 2) ? ataid * 2 + slave : -1;
}

static u8
ide_pio_max(u16 *identify)
{
    if ((identify[53] & 0x02) && (identify[64] & 0x02))
        return 4;
    if ((identify[53] & 0x02) && (identify[64] & 0x01))
        return 3;
    u8 mode = identify[51] >> 8;
    return mode > 2 ? 2 : mode;
}

// PIO mode for a drive: the setting, else the best one IDENTIFY reports.
u8
setup_ide_pio(int idx, u16 *identify)
{
    return Setting[NB_IDE_PIO0 + idx] ? Setting[NB_IDE_PIO0 + idx] - 1 : ide_pio_max(identify);
}

// UM8886BF timings for one drive (the values of the coreboot_486 IDE driver for PIO 0-4).
void
setup_ide_timing(int idx, u8 mode)
{
    static const u8 ta[] = { 3, 2, 1, 0, 0 }, tb[] = { 6, 5, 4, 3, 2 }, tc[] = { 0xb, 8, 5, 1, 0 };
    if (!onboard_ide() || mode > 4)
        return;
    int shift = 6 - 2 * idx, nib = (idx & 1) ? 0 : 4, chan = idx >> 1;
    u8 a = pci_config_readb(IDE_BDF, 0x43) & ~(3 << shift);
    pci_config_writeb(IDE_BDF, 0x43, a | ta[mode] << shift);
    u8 b = pci_config_readb(IDE_BDF, 0x44 + chan) & ~(0xf << nib);
    pci_config_writeb(IDE_BDF, 0x44 + chan, b | tb[mode] << nib);
    u8 c = pci_config_readb(IDE_BDF, 0x46 + chan) & ~(0xf << nib);
    pci_config_writeb(IDE_BDF, 0x46 + chan, c | tc[mode] << nib);
}

// What the ATA driver found, for the setup screens.
static struct {
    char model[41];
    u32 sectors;
    u16 cyl, heads, spt;
    u8 present, atapi, pio_max, pio;
    struct drive_s *drive;
} IdeInfo[4];

void
setup_ide_report(int idx, struct drive_s *drive, u16 *identify, int atapi, u8 pio)
{
    if (idx < 0 || idx > 3)
        return;
    IdeInfo[idx].present = 1;
    IdeInfo[idx].atapi = atapi;
    IdeInfo[idx].drive = drive;
    IdeInfo[idx].sectors = drive->sectors;
    IdeInfo[idx].cyl = identify[1];
    IdeInfo[idx].heads = identify[3];
    IdeInfo[idx].spt = identify[6];
    IdeInfo[idx].pio = pio;
    IdeInfo[idx].pio_max = ide_pio_max(identify);
    int i;
    for (i = 0; i < 20; i++) {
        IdeInfo[idx].model[2 * i] = identify[27 + i] >> 8;
        IdeInfo[idx].model[2 * i + 1] = identify[27 + i];
    }
    for (i = 40; i > 0 && IdeInfo[idx].model[i - 1] == ' '; i--)
        IdeInfo[idx].model[i - 1] = 0;
}

static void settings_check(u8 *val);

void
post_code(u8 code)
{
    outb(code, 0x80);
}

// Lines above the setup prompt (coreboot has no console in the flash ROM).
void
setup_post_warnings(void)
{
    if (SetupDefaulted)
        printf("CMOS settings invalid - defaults loaded.\n");
    if (ChipsetFallback)
        printf("Chipset setup values not used after %d unfinished POSTs.\n", ChipsetFallback);
}

// Validate the CMOS settings (load defaults if needed), apply the early ones; returns the boot order.
u16
setup_init(void)
{
    if (cmos_valid()) {
        int i;
        for (i = 0; i < NB_COUNT; i++)
            Setting[i] = NB_CMOS_GET(rtc_read, i);
        Password = rtc_read(NB_CMOS_PASSWORD) | rtc_read(NB_CMOS_PASSWORD + 1) << 8;
        settings_check(Setting);
        int fails = rtc_read(NB_CMOS_BOOTFAIL) - 1;   // coreboot counted this POST already
        if (!Setting[NB_CHIPSET_AUTO] && fails >= NB_BOOTFAIL_MAX)
            ChipsetFallback = fails;                   // coreboot used the chipset defaults
    } else {
        dprintf(1, "CMOS settings invalid - defaults loaded\n");
        SetupDefaulted = 1;
        memcpy(Setting, Defaults, sizeof(Setting));
    }
    cmos_write(Setting, Password);
    superio_apply();
    ide_channels_apply();
    return Setting[NB_BOOT1] | Setting[NB_BOOT2] << 4 | Setting[NB_BOOT3] << 8
        | Setting[NB_BOOT4] << 12;
}

int
setup_option(int index)
{
    return Setting[index];
}

// Translation chosen in setup for an onboard IDE drive: 0 auto, 1 normal, 2 LBA, 3 large.
int
setup_ide_translation(struct drive_s *drive)
{
    int i;
    for (i = 0; i < 4; i++)
        if (IdeInfo[i].present && IdeInfo[i].drive == drive)
            return Setting[NB_IDE_MODE0 + i];
    return 0;
}

u32
setup_boot_delay(void)
{
    static const u16 ms[] = { 1000, 2000, 3000, 5000 };
    return ms[Setting[NB_BOOT_DELAY] & 3];
}

int
setup_isa_shadow(u32 addr)
{
    return addr >= 0xc8000 && addr < 0xe0000 && Setting[NB_SHADOW_C8 + (addr - 0xc8000) / 0x4000];
}

// End of POST: keyboard, caches, and the POST is complete for coreboot's failure counter.
void
setup_finish(void)
{
    static const u8 rate[] = { 0x12, 0x0f, 0x0c, 0x0a, 0x08, 0x04, 0x02, 0x00 };   // 6..30 chars/s
    u16 flags = GET_BDA(kbd_flag0) & ~KF0_NUMACTIVE;
    SET_BDA(kbd_flag0, flags | (Setting[NB_NUMLOCK] ? KF0_NUMACTIVE : 0));
    if (Setting[NB_TYPEMATIC]) {
        u8 param = (Setting[NB_TYPEMATIC_DELAY] & 3) << 5 | rate[Setting[NB_TYPEMATIC_RATE] & 7];
        ps2_kbd_command(0x10f3, &param);
    }
    SetupL1Off = !Setting[NB_L1_CACHE];
    if (SetupL1Off) {
        wbinvd();
        cr0_mask(0, CR0_CD);
        wbinvd();
    }
    ApmEnabled = Setting[NB_APM];
    rtc_write(NB_CMOS_BOOTFAIL, 0);
}


/****************************************************************
 * Screen
 ****************************************************************/

static u16 *Vram;
static u8 AttrText, AttrTitle, AttrValue, AttrSelect, AttrDim;

#define ROW_HELP 21

static void
int10(u16 ax, u16 cx)
{
    struct bregs br;
    memset(&br, 0, sizeof(br));
    br.flags = F_IF;
    br.ax = ax;
    br.cx = cx;
    call16_int(0x10, &br);
}

static void
scr_puts(int row, int col, u8 attr, const char *s)
{
    for (; *s && col < 80; s++, col++)
        Vram[row * 80 + col] = attr << 8 | (u8)*s;
}

static void
scr_fill(int row, int col, int len, u8 attr, char c)
{
    for (; len > 0 && col < 80; len--, col++)
        Vram[row * 80 + col] = attr << 8 | (u8)c;
}

static void
scr_center(int row, u8 attr, const char *s)
{
    scr_puts(row, (80 - strlen(s)) / 2, attr, s);
}

static void
scr_open(void)
{
    int10(0x0003, 0);                       // text mode 3, clears the screen
    int mono = GET_BDA(video_mode) == 7;
    Vram = (void*)(mono ? 0xb0000 : 0xb8000);
    AttrText = mono ? 0x07 : 0x1f;
    AttrTitle = mono ? 0x0f : 0x1e;
    AttrValue = mono ? 0x0f : 0x1e;
    AttrSelect = 0x70;
    AttrDim = mono ? 0x07 : 0x17;
    int10(0x0100, 0x2000);                  // hide the cursor
}

static void
scr_close(void)
{
    int10(0x0003, 0);
}

static void
scr_header(const char *title)
{
    int r;
    for (r = 0; r < 25; r++)
        scr_fill(r, 0, 80, AttrText, ' ');
    scr_center(0, AttrTitle, "ROM PCI/ISA BIOS (MB-8433UUD)");
    scr_center(1, AttrTitle, title);
    scr_center(2, AttrTitle, "newBIOS - coreboot + SeaBIOS");
}

// Centered one-line dialog; returns the key pressed (scan << 8 | ascii).
static int
scr_dialog(const char *msg)
{
    int len = strlen(msg) + 4, col = (80 - len) / 2;
    scr_fill(11, col, len, AttrSelect, ' ');
    scr_fill(12, col, len, AttrSelect, ' ');
    scr_fill(13, col, len, AttrSelect, ' ');
    scr_puts(12, col + 2, AttrSelect, msg);
    for (;;) {
        int key = get_keystroke_full(1000);
        if (key >= 0)
            return key;
    }
}

static int
scr_confirm(const char *msg)
{
    int key = scr_dialog(msg);
    return (key & 0xff) == 'y' || (key & 0xff) == 'Y';
}

static void
hex_byte(u8 b, char *out)
{
    static const char digits[] = "0123456789ABCDEF";
    out[0] = digits[b >> 4];
    out[1] = digits[b & 15];
    out[2] = 0;
}


/****************************************************************
 * Setup values
 ****************************************************************/

// Settings being edited, followed by the RTC fields.
enum { V_MONTH = NB_COUNT, V_DAY, V_YEAR, V_HOUR, V_MINUTE, V_SECOND, V_COUNT };
static u8 Val[V_COUNT], OrigVal[V_COUNT];
static u16 NewPassword;

static int
rtc_get(u8 reg)
{
    u8 v = rtc_read(reg);
    return (rtc_read(CMOS_STATUS_B) & RTC_B_BIN) ? v : (v >> 4) * 10 + (v & 0x0f);
}

static void
rtc_put(u8 reg, int v)
{
    rtc_write(reg, (rtc_read(CMOS_STATUS_B) & RTC_B_BIN) ? v : (v / 10) << 4 | v % 10);
}

static void rtc_fields_check(void);

static void
values_load(void)
{
    memcpy(Val, Setting, NB_COUNT);
    NewPassword = Password;
    rtc_updating();
    int year = rtc_get(CMOS_RTC_YEAR), century = rtc_get(CMOS_CENTURY);
    if (century != 19 && century != 20)
        century = year < 80 ? 20 : 19;
    Val[V_MONTH] = rtc_get(CMOS_RTC_MONTH);
    Val[V_DAY] = rtc_get(CMOS_RTC_DAY_MONTH);
    Val[V_YEAR] = (century - 19) * 100 + year;
    Val[V_HOUR] = rtc_get(CMOS_RTC_HOURS);
    Val[V_MINUTE] = rtc_get(CMOS_RTC_MINUTES);
    Val[V_SECOND] = rtc_get(CMOS_RTC_SECONDS);
    rtc_fields_check();
    memcpy(OrigVal, Val, sizeof(Val));
}

static void
values_save(void)
{
    if (memcmp(&Val[V_MONTH], &OrigVal[V_MONTH], V_COUNT - V_MONTH) != 0) {
        rtc_write(CMOS_STATUS_B, rtc_read(CMOS_STATUS_B) | RTC_B_SET);
        rtc_put(CMOS_CENTURY, 19 + Val[V_YEAR] / 100);
        rtc_put(CMOS_RTC_YEAR, Val[V_YEAR] % 100);
        rtc_put(CMOS_RTC_MONTH, Val[V_MONTH]);
        rtc_put(CMOS_RTC_DAY_MONTH, Val[V_DAY]);
        rtc_put(CMOS_RTC_HOURS, Val[V_HOUR]);
        rtc_put(CMOS_RTC_MINUTES, Val[V_MINUTE]);
        rtc_put(CMOS_RTC_SECONDS, Val[V_SECOND]);
        rtc_write(CMOS_STATUS_B, rtc_read(CMOS_STATUS_B) & ~RTC_B_SET);
    }
    cmos_write(Val, NewPassword);
    rtc_write(NB_CMOS_BOOTFAIL, 0);
}


/****************************************************************
 * Setup pages
 ****************************************************************/

static const char * const OffOn[] = { "Disabled", "Enabled" };
static const char * const OnOff[] = { "Off", "On" };
static const char * const Floppy[] = {
    "None", "360K, 5.25 in.", "1.2M, 5.25 in.", "720K, 3.5 in.", "1.44M, 3.5 in.", "2.88M, 3.5 in."
};
static const char * const IdeType[] = { "Auto", "None" };
static const char * const IdeMode[] = { "AUTO", "NORMAL", "LBA", "LARGE" };
static const char * const BootDev[] = { "", "Floppy", "Hard Disk", "CD-ROM", "Network" };
static const char * const Delay[] = { "1 Sec", "2 Sec", "3 Sec", "5 Sec" };
static const char * const Rate[] = { "6", "8", "10", "12", "15", "20", "24", "30" };
static const char * const RepDelay[] = { "250", "500", "750", "1000" };
static const char * const Security[] = { "Setup", "System" };
static const char * const DramWs[] = { "3 WS", "2 WS", "1 WS", "0 WS" };
static const char * const Ws[] = { "1 WS", "0 WS" };
static const char * const L2Ws[] = { "3-2-2-2", "3-1-1-1", "2-2-2-2", "2-1-1-1" };
static const char * const KbcClock[] = { "7.16MHz", "PCICLKI/4", "PCICLKI/3", "PCICLKI/2" };
static const char * const PciClock[] = { "1 : 1", "1 : 2/3", "1 : 1/2" };
static const char * const Devsel[] = { "Slow", "Medium", "Fast" };
static const char * const Standby[] = {
    "Disabled", "1 Min", "2 Min", "3 Min", "4 Min", "5 Min", "6 Min", "7 Min", "8 Min",
    "9 Min", "10 Min", "11 Min", "12 Min", "13 Min", "14 Min", "15 Min"
};
static const char * const Irq[] = {
    "Auto", "", "", "3", "4", "5", "", "7", "", "9", "10", "11", "12", "", "14", "15"
};
static const char * const Pio[] = { "Auto", "Mode 0", "Mode 1", "Mode 2", "Mode 3", "Mode 4" };
static const char * const Com1[] = { "Disabled", "COM1/3F8", "COM3/3E8" };
static const char * const Com2[] = { "Disabled", "COM2/2F8", "COM4/2E8" };
static const char * const Lpt[] = { "Disabled", "378H", "278H" };
static const char * const LptMode[] = { "SPP", "EPP", "ECP" };

#define F_MANUAL 1   // editable only when chipset auto configuration is off

struct item {
    u8 row, col, vcol, var, lo, hi, flags;
    const char *label;
    const char * const *names;   // value names, or NULL for numbers
};

struct page {
    const char *title;
    const struct item *items;
    int count;
    void (*draw)(void);
};

#define ENUM(r, c, v, var, label, names, flags) \
    { r, c, v, var, 0, ARRAY_SIZE(names) - 1, flags, label, names }
#define NUM(r, c, v, var, lo, hi, label) { r, c, v, var, lo, hi, 0, label, NULL }

static void draw_std(void);

static const struct item StdItems[] = {
    NUM(4, 5, 23, V_MONTH, 1, 12, "Date (mm:dd:yy) :"),
    NUM(4, 0, 26, V_DAY, 1, 31, NULL),
    NUM(4, 0, 29, V_YEAR, 80, 199, NULL),
    NUM(5, 5, 23, V_HOUR, 0, 23, "Time (hh:mm:ss) :"),
    NUM(5, 0, 26, V_MINUTE, 0, 59, NULL),
    NUM(5, 0, 29, V_SECOND, 0, 59, NULL),
    ENUM(8, 5, 24, NB_IDE_TYPE0, "Primary Master   :", IdeType, 0),
    ENUM(8, 0, 70, NB_IDE_MODE0, NULL, IdeMode, 0),
    ENUM(9, 5, 24, NB_IDE_TYPE1, "Primary Slave    :", IdeType, 0),
    ENUM(9, 0, 70, NB_IDE_MODE1, NULL, IdeMode, 0),
    ENUM(10, 5, 24, NB_IDE_TYPE2, "Secondary Master :", IdeType, 0),
    ENUM(10, 0, 70, NB_IDE_MODE2, NULL, IdeMode, 0),
    ENUM(11, 5, 24, NB_IDE_TYPE3, "Secondary Slave  :", IdeType, 0),
    ENUM(11, 0, 70, NB_IDE_MODE3, NULL, IdeMode, 0),
    ENUM(13, 5, 15, NB_FLOPPY_A, "Drive A :", Floppy, 0),
    ENUM(14, 5, 15, NB_FLOPPY_B, "Drive B :", Floppy, 0),
};

// RTC fields out of range (garbage after a battery loss) start at their minimum.
static void
rtc_fields_check(void)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(StdItems); i++) {
        const struct item *it = &StdItems[i];
        if (it->var >= V_MONTH && (Val[it->var] < it->lo || Val[it->var] > it->hi))
            Val[it->var] = it->lo;
    }
}

static const struct item BiosItems[] = {
    ENUM(4, 2, 31, NB_L1_CACHE, "CPU Internal Cache         :", OffOn, 0),
    ENUM(5, 2, 31, NB_L2_CACHE, "External Cache             :", OffOn, 0),
    ENUM(6, 2, 31, NB_BOOT1, "1st Boot Device            :", BootDev, 0),
    ENUM(7, 2, 31, NB_BOOT2, "2nd Boot Device            :", BootDev, 0),
    ENUM(8, 2, 31, NB_BOOT3, "3rd Boot Device            :", BootDev, 0),
    ENUM(9, 2, 31, NB_BOOT4, "4th Boot Device            :", BootDev, 0),
    ENUM(10, 2, 31, NB_BOOT_DELAY, "Setup Prompt Delay         :", Delay, 0),
    ENUM(11, 2, 31, NB_SWAP_FLOPPY, "Swap Floppy Drive          :", OffOn, 0),
    ENUM(12, 2, 31, NB_NUMLOCK, "Boot Up NumLock Status     :", OnOff, 0),
    ENUM(13, 2, 31, NB_TYPEMATIC, "Typematic Rate Setting     :", OffOn, 0),
    ENUM(14, 2, 31, NB_TYPEMATIC_RATE, "Typematic Rate (Chars/Sec) :", Rate, 0),
    ENUM(15, 2, 31, NB_TYPEMATIC_DELAY, "Typematic Delay (Msec)     :", RepDelay, 0),
    ENUM(16, 2, 31, NB_SECURITY, "Security Option            :", Security, 0),
    ENUM(17, 2, 31, NB_QUICK_POST, "Quick Power On Self Test   :", OffOn, 0),
    ENUM(4, 45, 68, NB_SHADOW_C8, "C8000-CBFFF Shadow   :", OffOn, 0),
    ENUM(5, 45, 68, NB_SHADOW_CC, "CC000-CFFFF Shadow   :", OffOn, 0),
    ENUM(6, 45, 68, NB_SHADOW_D0, "D0000-D3FFF Shadow   :", OffOn, 0),
    ENUM(7, 45, 68, NB_SHADOW_D4, "D4000-D7FFF Shadow   :", OffOn, 0),
    ENUM(8, 45, 68, NB_SHADOW_D8, "D8000-DBFFF Shadow   :", OffOn, 0),
    ENUM(9, 45, 68, NB_SHADOW_DC, "DC000-DFFFF Shadow   :", OffOn, 0),
};

static const struct item ChipItems[] = {
    ENUM(4, 2, 29, NB_CHIPSET_AUTO, "Auto Configuration       :", OffOn, 0),
    ENUM(6, 2, 29, NB_DRAM_READ_WS, "DRAM Read Wait States    :", DramWs, F_MANUAL),
    ENUM(7, 2, 29, NB_DRAM_WRITE_WS, "DRAM Write Wait States   :", DramWs, F_MANUAL),
    ENUM(8, 2, 29, NB_L2_WS, "L2 Cache Wait States     :", L2Ws, F_MANUAL),
    ENUM(9, 2, 29, NB_KBC_CLOCK, "Keyboard Controller Clock:", KbcClock, F_MANUAL),
    ENUM(10, 2, 29, NB_EDO, "EDO DRAM install option  :", OffOn, F_MANUAL),
    ENUM(11, 2, 29, NB_EARLY_CACHE_WRITE, "Early Cache Write Mode   :", OffOn, F_MANUAL),
    ENUM(12, 2, 29, NB_SYSBIOS_CACHE, "System BIOS Cacheable    :", OffOn, 0),
    ENUM(13, 2, 29, NB_VIDBIOS_CACHE, "Video  BIOS Cacheable    :", OffOn, 0),
    ENUM(14, 2, 29, NB_HOST_PCI_POST_WS, "HOST-to-PCI Post Write   :", Ws, F_MANUAL),
    ENUM(15, 2, 29, NB_HOST_PCI_BURST, "HOST-to-PCI Burst Write  :", OffOn, F_MANUAL),
    ENUM(16, 2, 29, NB_PCI_BUS_PARK, "PCI Bus Park Option      :", OffOn, F_MANUAL),
    ENUM(17, 2, 29, NB_PCI_POSTED_WRITE, "PCI Posted Memory Write  :", OffOn, F_MANUAL),
    ENUM(18, 2, 29, NB_SLOW_REFRESH, "Slow Refresh (1/4 Freq)  :", OffOn, F_MANUAL),
    ENUM(4, 42, 68, NB_HOST_PCI_CLOCK, "HOST Clock / PCI Clock  :", PciClock, F_MANUAL),
    ENUM(5, 42, 68, NB_BURST_COPYBACK, "Burst Copy-Back Option  :", OffOn, F_MANUAL),
    ENUM(6, 42, 68, NB_PREEMPT_PCI, "Preempt PCI Master      :", OffOn, F_MANUAL),
    ENUM(7, 42, 68, NB_DEVSEL, "IBC DEVSEL# Decoding    :", Devsel, F_MANUAL),
};

static const struct item PmItems[] = {
    ENUM(4, 2, 26, NB_APM, "APM BIOS              :", OffOn, 0),
    ENUM(5, 2, 26, NB_HDD_STANDBY, "HDD Standby Timer     :", Standby, 0),
};

static const struct item PciItems[] = {
    ENUM(4, 2, 30, NB_PCI_IRQ_A, "PCI INTA# IRQ             :", Irq, 0),
    ENUM(5, 2, 30, NB_PCI_IRQ_B, "PCI INTB# IRQ             :", Irq, 0),
    ENUM(6, 2, 30, NB_PCI_IRQ_C, "PCI INTC# IRQ             :", Irq, 0),
    ENUM(7, 2, 30, NB_PCI_IRQ_D, "PCI INTD# IRQ             :", Irq, 0),
    ENUM(9, 2, 30, NB_IDE_PIO0, "IDE Primary Master PIO    :", Pio, 0),
    ENUM(10, 2, 30, NB_IDE_PIO1, "IDE Primary Slave  PIO    :", Pio, 0),
    ENUM(11, 2, 30, NB_IDE_PIO2, "IDE Secondary Master PIO  :", Pio, 0),
    ENUM(12, 2, 30, NB_IDE_PIO3, "IDE Secondary Slave  PIO  :", Pio, 0),
    ENUM(13, 2, 30, NB_IDE_32BIT, "IDE 32-bit Transfer       :", OffOn, 0),
};

static const struct item IoItems[] = {
    ENUM(4, 2, 30, NB_IDE_PRIMARY, "Onboard IDE Primary       :", OffOn, 0),
    ENUM(5, 2, 30, NB_IDE_SECONDARY, "Onboard IDE Secondary     :", OffOn, 0),
    ENUM(6, 2, 30, NB_FDC, "Onboard FDD Controller    :", OffOn, 0),
    ENUM(7, 2, 30, NB_COM1, "Onboard Serial Port 1     :", Com1, 0),
    ENUM(8, 2, 30, NB_COM2, "Onboard Serial Port 2     :", Com2, 0),
    ENUM(9, 2, 30, NB_LPT, "Onboard Parallel Port     :", Lpt, 0),
    ENUM(10, 2, 30, NB_LPT_MODE, "Onboard Parallel Mode     :", LptMode, 0),
};

static const struct page Pages[] = {
    { "STANDARD CMOS SETUP", StdItems, ARRAY_SIZE(StdItems), draw_std },
    { "BIOS FEATURES SETUP", BiosItems, ARRAY_SIZE(BiosItems), NULL },
    { "CHIPSET FEATURES SETUP", ChipItems, ARRAY_SIZE(ChipItems), NULL },
    { "POWER MANAGEMENT SETUP", PmItems, ARRAY_SIZE(PmItems), NULL },
    { "PNP/PCI CONFIGURATION", PciItems, ARRAY_SIZE(PciItems), NULL },
    { "INTEGRATED PERIPHERALS", IoItems, ARRAY_SIZE(IoItems), NULL },
};

static int
item_valid_value(const struct item *it, u8 v)
{
    if (v < it->lo || v > it->hi)
        return 0;
    return !it->names || it->names[v][0];
}

// Out of range settings (CMOS from another layout) fall back to their default.
static void
settings_check(u8 *val)
{
    int p, i;
    for (p = 0; p < ARRAY_SIZE(Pages); p++)
        for (i = 0; i < Pages[p].count; i++) {
            const struct item *it = &Pages[p].items[i];
            if (it->var < NB_COUNT && !item_valid_value(it, val[it->var]))
                val[it->var] = Defaults[it->var];
        }
    // Each boot device class exactly once.
    int seen = 0;
    for (i = NB_BOOT1; i <= NB_BOOT4; i++)
        seen |= 1 << val[i];
    if (seen != 0x1e)
        for (i = NB_BOOT1; i <= NB_BOOT4; i++)
            val[i] = Defaults[i];
}

static int
item_editable(const struct item *it)
{
    return !(it->flags & F_MANUAL) || !Val[NB_CHIPSET_AUTO];
}

static void
item_draw(const struct item *it, int selected)
{
    char num[8] = { '0' + Val[it->var] / 10 % 10, '0' + Val[it->var] % 10, 0 };
    const char *s = num;
    u8 v = Val[it->var];
    int width = 0, i;
    if (it->names) {
        s = it->names[v];
        for (i = it->lo; i <= it->hi; i++)
            if (strlen(it->names[i]) > width)
                width = strlen(it->names[i]);
    } else if (it->var == V_YEAR) {
        snprintf(num, sizeof(num), "%d", 1900 + v);
    }
    if (it->label)
        scr_puts(it->row, it->col, AttrText, it->label);
    if (width < strlen(s))
        width = strlen(s);
    u8 attr = selected ? AttrSelect : item_editable(it) ? AttrValue : AttrDim;
    scr_fill(it->row, it->vcol, width, AttrText, ' ');
    scr_puts(it->row, it->vcol, attr, s);
}

// Step a value, skipping holes; boot devices stay a permutation by swapping.
static void
item_step(const struct item *it, int delta)
{
    u8 v = Val[it->var];
    do {
        v = v + delta < it->lo ? it->hi : v + delta > it->hi ? it->lo : v + delta;
    } while (!item_valid_value(it, v));
    if (it->var >= NB_BOOT1 && it->var <= NB_BOOT4) {
        int i;
        for (i = NB_BOOT1; i <= NB_BOOT4; i++)
            if (Val[i] == v)
                Val[i] = Val[it->var];
    }
    Val[it->var] = v;
}

static void
draw_help(int page)
{
    scr_puts(ROW_HELP, 2, AttrText,
             "ESC : Quit                \x18\x19\x1a\x1b : Select Item     PU/PD/+/- : Modify");
    if (page)
        scr_puts(ROW_HELP + 1, 2, AttrText,
                 "F5  : Old Values          F7 : Load Setup Defaults  F10 : Save & Exit");
    else
        scr_puts(ROW_HELP + 1, 2, AttrText, "F10 : Save & Exit Setup");
}

static void
draw_std(void)
{
    char line[64];
    int i;
    scr_puts(4, 25, AttrText, ":  :");
    scr_puts(5, 25, AttrText, ":  :");
    scr_puts(7, 5, AttrTitle,
             "HARD DISKS          TYPE       SIZE   CYLS HEAD SECTOR     MODE");
    for (i = 0; i < 4; i++) {
        if (!IdeInfo[i].present)
            continue;
        if (IdeInfo[i].atapi) {
            scr_puts(8 + i, 35, AttrText, "CD-ROM");
            continue;
        }
        u32 mb = IdeInfo[i].sectors / 2048;
        snprintf(line, sizeof(line), "%d%s", mb >= 10000 ? mb / 1024 : mb, mb >= 10000 ? "G" : "M");
        scr_puts(8 + i, 42 - strlen(line), AttrText, line);
        snprintf(line, sizeof(line), "%d", IdeInfo[i].cyl);
        scr_puts(8 + i, 49 - strlen(line), AttrText, line);
        snprintf(line, sizeof(line), "%d", IdeInfo[i].heads);
        scr_puts(8 + i, 54 - strlen(line), AttrText, line);
        snprintf(line, sizeof(line), "%d", IdeInfo[i].spt);
        scr_puts(8 + i, 61 - strlen(line), AttrText, line);
    }
    u32 extkb = ext_mem_kb();
    snprintf(line, sizeof(line), "    Base Memory: %6dK", GET_BDA(mem_size_kb));
    scr_puts(13, 48, AttrText, line);
    snprintf(line, sizeof(line), "Extended Memory: %6dK", extkb);
    scr_puts(14, 48, AttrText, line);
    snprintf(line, sizeof(line), "   Other Memory: %6dK", 1024 - GET_BDA(mem_size_kb));
    scr_puts(15, 48, AttrText, line);
    snprintf(line, sizeof(line), "   Total Memory: %6dK", extkb + 1024);
    scr_puts(17, 48, AttrText, line);
}

static void
values_default(void)
{
    memcpy(Val, Defaults, NB_COUNT);
}

static void save_and_reboot(void);

// Nearest editable item in the column left (dir -1) or right (dir 1) of item sel.
static int
item_beside(const struct page *pg, int sel, int dir)
{
    const struct item *it = &pg->items[sel];
    int best = sel, dist = 99, i;
    for (i = 0; i < pg->count; i++) {
        const struct item *o = &pg->items[i];
        int d = o->row > it->row ? o->row - it->row : it->row - o->row;
        int side = dir > 0 ? o->vcol > it->vcol : o->vcol < it->vcol;
        if (side && item_editable(o) && d < dist) {
            best = i;
            dist = d;
        }
    }
    return best;
}

// Run one settings page; returns when the user leaves it with ESC.
static void
page_run(const struct page *pg)
{
    int sel = 0, i;
    for (;;) {
        scr_header(pg->title);
        draw_help(1);
        if (pg->draw)
            pg->draw();
        for (i = 0; i < pg->count; i++)
            item_draw(&pg->items[i], i == sel);
        int key = get_keystroke_full(1000);
        if (key < 0)
            continue;
        int scan = key >> 8, ascii = key & 0xff, step = 0;
        switch (scan) {
        case 0x01: return;                                       // ESC
        case 0x48:                                               // up
            do sel = (sel + pg->count - 1) % pg->count; while (!item_editable(&pg->items[sel]));
            break;
        case 0x50:                                               // down
            do sel = (sel + 1) % pg->count; while (!item_editable(&pg->items[sel]));
            break;
        case 0x4b: sel = item_beside(pg, sel, -1); break;        // left
        case 0x4d: sel = item_beside(pg, sel, 1); break;         // right
        case 0x49: step = -1; break;                             // PgUp
        case 0x51: step = 1; break;                              // PgDn
        case 0x3f: memcpy(Val, OrigVal, NB_COUNT); break;        // F5
        case 0x41:                                               // F7
            if (scr_confirm("Load Setup Defaults (Y/N)? "))
                values_default();
            break;
        case 0x44: save_and_reboot(); break;                     // F10
        }
        if (ascii == '-')
            step = -1;
        if (ascii == '+')
            step = 1;
        if (step && item_editable(&pg->items[sel]))
            item_step(&pg->items[sel], step);
    }
}


/****************************************************************
 * Hardware information
 ****************************************************************/

static void
info_line(int row, const char *s)
{
    scr_puts(row, 3, AttrText, s);
}

static const char *
cpu_name(const char *vendor, u32 fms, int *mhz_valid)
{
    int fam = (fms >> 8) & 0xf, model = (fms >> 4) & 0xf;
    *mhz_valid = fam == 4;
    if (!strcmp(vendor, "GenuineIntel")) {
        static const char * const n[] = { "486DX", "486DX", "486SX", "486DX2", "486SL", "486SX2",
                                          "486", "486DX2-WB", "486DX4", "486DX4-WB" };
        if (fam == 4)
            return model < ARRAY_SIZE(n) ? n[model] : "486";
        if (fam == 5)
            return "Pentium OverDrive";
    }
    if (!strcmp(vendor, "AuthenticAMD") && fam == 4) {
        switch (model) {
        case 3: return "Am486DX2";
        case 7: return "Am486DX2-WB";
        case 8: return "Am486DX4";
        case 9: return "Am486DX4-WB";
        case 14: return "Am5x86";
        case 15: return "Am5x86-WB";
        }
        return "Am486";
    }
    if (!strcmp(vendor, "CyrixInstead"))
        return fam == 4 ? "Cyrix 5x86" : "Cyrix";
    if (!strcmp(vendor, "UMC UMC UMC "))
        return model == 2 ? "UMC U5S" : "UMC U5D";
    *mhz_valid = 0;
    return "unknown";
}

static int
cpu_has_cpuid(void)
{
    u32 f = save_flags();
    restore_flags(f ^ 0x200000);
    u32 g = save_flags();
    restore_flags(f);
    return ((f ^ g) & 0x200000) != 0;
}

// Cyrix CPUs keep the flags in DIV (and have CPUID off by default); DIR0 names the model.
static int
cpu_cyrix_dir0(void)
{
    u16 ax;
    asm volatile("xorw %%ax, %%ax; sahf; movw $5, %%ax; movb $2, %%bl; divb %%bl; lahf"
                 : "=a"(ax) : : "ebx", "cc");
    if ((ax >> 8) != 0x02)
        return -1;
    outb(0xfe, 0x22);
    return inb(0x23);
}

// Core clock of 486-class CPUs: dec + taken jnz is 4 clocks, timed with PIT channel 2.
static u32
cpu_mhz(void)
{
    u32 n = 200000, flags = save_flags();
    irq_disable();
    outb((inb(0x61) & ~0x02) | 0x01, 0x61);
    outb(0xb0, 0x43);
    outb(0xff, 0x42);
    outb(0xff, 0x42);
    asm volatile("1: decl %0; jnz 1b" : "+r"(n));
    outb(0x80, 0x43);
    u32 left = inb(0x42);
    left |= inb(0x42) << 8;
    restore_flags(flags);
    u32 ticks = 0xffff - left;
    return ticks ? (954400000 / ticks + 500) / 1000 : 0;   // 800000 clocks * 1.193 MHz
}

static void
info_cpu(void)
{
    char line[80], vendor[13] = "";
    u32 eax = 0, ebx, ecx, edx = 0, fms = 0;
    int mhz_valid = 0, row = 4;
    const char *name = "486 (no CPUID)";
    if (cpu_has_cpuid()) {
        cpuid(0, &eax, &ebx, &ecx, &edx);
        memcpy(vendor, &ebx, 4);
        memcpy(vendor + 4, &edx, 4);
        memcpy(vendor + 8, &ecx, 4);
        vendor[12] = 0;
        if (eax >= 1)
            cpuid(1, &fms, &ebx, &ecx, &edx);
        name = cpu_name(vendor, fms, &mhz_valid);
        snprintf(line, sizeof(line), "CPU            : %s (%s)", name, vendor);
        info_line(row++, line);
        snprintf(line, sizeof(line), "CPUID          : family %d, model %d, stepping %d (%x)",
                 (fms >> 8) & 0xf, (fms >> 4) & 0xf, fms & 0xf, fms);
        info_line(row++, line);
        snprintf(line, sizeof(line), "Features       :%s%s%s%s%s%s", edx & 1 ? " FPU" : "",
                 edx & 2 ? " VME" : "", edx & 8 ? " PSE" : "", edx & 0x10 ? " TSC" : "",
                 edx & 0x20 ? " MSR" : "", edx & 0x100 ? " CX8" : "");
        info_line(row++, line);
    } else {
        int dir0 = cpu_cyrix_dir0();
        if (dir0 >= 0x28 && dir0 <= 0x2f)
            name = "Cyrix 5x86 (CPUID disabled)";
        else if (dir0 >= 0x1a && dir0 <= 0x1f)
            name = "Cyrix 486DX/DX2/DX4 (no CPUID)";
        snprintf(line, sizeof(line), "CPU            : %s", name);
        info_line(row++, line);
        row += 2;
    }
    if (mhz_valid) {
        snprintf(line, sizeof(line), "Core clock     : %d MHz (measured)", cpu_mhz());
        info_line(row, line);
    }
    row += 2;
    snprintf(line, sizeof(line), "L1 cache       : %s",
             (cr0_read() & CR0_CD) ? "disabled" : "enabled");
    info_line(row++, line);
    u8 l2 = pci_config_readb(HB_BDF, 0x50);
    snprintf(line, sizeof(line), "L2 cache       : %d KB, %s, %s, %d bank(s)",
             (l2 & 7) ? 32 << (l2 & 7) : 0, (l2 & 0x40) ? "write-back" : "write-through",
             (l2 & 0x80) ? "enabled" : "disabled", (l2 & 8) ? 2 : 1);
    info_line(row++, line);
    row++;
    u32 extkb = ext_mem_kb();
    snprintf(line, sizeof(line), "DRAM           : %d KB (base %d KB, extended %d KB)",
             extkb + 1024, GET_BDA(mem_size_kb), extkb);
    info_line(row++, line);
    snprintf(line, sizeof(line), "Chipset        : UMC UM8881F rev %x, UM8886BF rev %x, UM8663BF",
             pci_config_readb(HB_BDF, PCI_REVISION_ID), pci_config_readb(IBC_BDF, PCI_REVISION_ID));
    info_line(row++, line);
}

static const char *
pci_vendor(u16 v)
{
    static const struct { u16 id; const char *name; } t[] = {
        { 0x1060, "UMC" }, { 0x10ec, "Realtek" }, { 0x1023, "Trident" }, { 0x5333, "S3" },
        { 0x1013, "Cirrus" }, { 0x1002, "ATI" }, { 0x102b, "Matrox" }, { 0x10de, "NVIDIA" },
        { 0x8086, "Intel" }, { 0x1022, "AMD" }, { 0x1000, "NCR/LSI" }, { 0x9004, "Adaptec" },
        { 0x10b7, "3Com" }, { 0x1011, "DEC" }, { 0x121a, "3dfx" }, { 0x1274, "Ensoniq" },
        { 0x10b9, "ALi" }, { 0x1039, "SiS" }, { 0x1106, "VIA" }, { 0x1095, "CMD" },
        { 0x105a, "Promise" }, { 0x1102, "Creative" }, { 0x104c, "TI" }, { 0x100b, "NatSemi" },
    };
    int i;
    for (i = 0; i < ARRAY_SIZE(t); i++)
        if (t[i].id == v)
            return t[i].name;
    return "";
}

static const char *
pci_class_name(u16 c)
{
    static const struct { u16 id; const char *name; } t[] = {
        { 0x0001, "VGA (pre-2.0)" }, { 0x0100, "SCSI" }, { 0x0101, "IDE" }, { 0x0180, "Storage" },
        { 0x0200, "Ethernet" }, { 0x0280, "Network" }, { 0x0300, "VGA" }, { 0x0301, "XGA (VGA)" },
        { 0x0380, "Display" }, { 0x0400, "Video" }, { 0x0401, "Audio" }, { 0x0480, "Multimedia" },
        { 0x0600, "Host bridge" }, { 0x0601, "ISA bridge" }, { 0x0604, "PCI bridge" },
        { 0x0680, "Bridge" }, { 0x0700, "Serial" }, { 0x0701, "Parallel" }, { 0x0c03, "USB" },
    };
    int i;
    for (i = 0; i < ARRAY_SIZE(t); i++)
        if (t[i].id == c)
            return t[i].name;
    return "Other";
}

static void
info_pci(void)
{
    char line[80];
    int row = 5;
    scr_puts(4, 3, AttrTitle, "Bus:Dev.F   ID          Vendor    Class          INT  IRQ");
    struct pci_device *pci;
    foreachpci(pci) {
        if (row > 19)
            break;
        u8 pin = pci_config_readb(pci->bdf, PCI_INTERRUPT_PIN);
        u8 irq = pci_config_readb(pci->bdf, PCI_INTERRUPT_LINE);
        u8 devfn = pci_bdf_to_devfn(pci->bdf);
        snprintf(line, sizeof(line), "%02x:%02x.%x   %04x:%04x", pci_bdf_to_bus(pci->bdf)
                 , devfn >> 3, devfn & 7, pci->vendor, pci->device);
        info_line(row, line);
        scr_puts(row, 27, AttrText, pci_vendor(pci->vendor));
        scr_puts(row, 37, AttrText, pci_class_name(pci->class));
        if (pin) {
            snprintf(line, sizeof(line), "%c    %d", 'A' + pin - 1, irq);
            scr_puts(row, 53, AttrText, (irq && irq < 16) ? line : "-    -");
        }
        row++;
    }
}

static void
info_isa(void)
{
    static const char * const fixed[16] = {
        "System timer", "Keyboard", "Cascade (IRQ 9)", "", "", "", "", "", "Real-time clock",
        "", "", "", "", "Math coprocessor", "", ""
    };
    const char *owner[16];
    char line[80];
    int i, row = 4;
    memcpy(owner, fixed, sizeof(owner));
    for (i = 0; i < 4; i++) {
        u16 port = GET_BDA(port_com[i]);
        if (port)
            owner[(port == 0x3f8 || port == 0x3e8) ? 4 : 3] = "Serial port";
    }
    if (GET_BDA(port_lpt[0]))
        owner[GET_BDA(port_lpt[0]) == 0x278 ? 5 : 7] = "Parallel port";
    if (Setting[NB_FDC])
        owner[6] = "Floppy controller";
    if (Setting[NB_IDE_PRIMARY])
        owner[14] = "IDE primary";
    if (Setting[NB_IDE_SECONDARY])
        owner[15] = "IDE secondary";
    if (GET_BDA(equipment_list_flags) & 0x04)
        owner[12] = "PS/2 mouse";
    struct pci_device *pci;
    foreachpci(pci) {
        u8 irq = pci_config_readb(pci->bdf, PCI_INTERRUPT_LINE);
        if (pci_config_readb(pci->bdf, PCI_INTERRUPT_PIN) && irq && irq < 16)
            owner[irq] = pci_class_name(pci->class);
    }
    snprintf(line, sizeof(line), "Serial ports   : %x %x %x %x", GET_BDA(port_com[0]),
             GET_BDA(port_com[1]), GET_BDA(port_com[2]), GET_BDA(port_com[3]));
    info_line(row++, line);
    snprintf(line, sizeof(line), "Parallel ports : %x %x %x", GET_BDA(port_lpt[0]),
             GET_BDA(port_lpt[1]), GET_BDA(port_lpt[2]));
    info_line(row++, line);
    u32 a;
    char roms[64] = "", *p = roms;
    for (a = 0xc8000; a < 0xe0000; a += 2048) {
        u8 *rom = (void*)a;
        if (rom[0] == 0x55 && rom[1] == 0xaa && rom[2] && p < roms + sizeof(roms) - 12) {
            p += snprintf(p, roms + sizeof(roms) - p, " %x (%dK)", a, rom[2] / 2);
            a += (rom[2] * 512 - 1) & ~2047;
        }
    }
    snprintf(line, sizeof(line), "Option ROMs    :%s", roms[0] ? roms : " none at C8000-DFFFF");
    info_line(row++, line);
    row++;
    for (i = 0; i < 16; i++) {
        snprintf(line, sizeof(line), "IRQ %d : %s", i, owner[i][0] ? owner[i] : "free");
        scr_puts(row + (i & 7), 3 + (i >> 3) * 38, AttrText, line);
    }
}

static void
info_ide(void)
{
    static const char * const unit[] = { "Primary Master  ", "Primary Slave   ",
                                         "Secondary Master", "Secondary Slave " };
    static const char * const trans[] = { "NORMAL", "LBA", "LARGE", "RECHS", "HOST" };
    char line[80];
    int i, row = 4;
    for (i = 0; i < 4; i++, row += 3) {
        if (!IdeInfo[i].present) {
            snprintf(line, sizeof(line), "%s : None", unit[i]);
            info_line(row, line);
            continue;
        }
        snprintf(line, sizeof(line), "%s : %s", unit[i], IdeInfo[i].model);
        info_line(row, line);
        if (IdeInfo[i].atapi)
            snprintf(line, sizeof(line), "    ATAPI CD-ROM, PIO mode %d (drive max %d)",
                     IdeInfo[i].pio, IdeInfo[i].pio_max);
        else
            snprintf(line, sizeof(line),
                     "    %dMB, CHS %d/%d/%d, %s, PIO mode %d (drive max %d)",
                     IdeInfo[i].sectors / 2048, IdeInfo[i].cyl, IdeInfo[i].heads, IdeInfo[i].spt,
                     trans[get_translation(IdeInfo[i].drive) % 5], IdeInfo[i].pio,
                     IdeInfo[i].pio_max);
        info_line(row + 1, line);
    }
}

static void
info_regs(const char *name, u16 bdf, int first, int rows, int row)
{
    char line[80], *p;
    int r, i;
    for (r = 0; r < rows; r++) {
        p = line + snprintf(line, sizeof(line), "%s %x:", name, first + 16 * r);
        for (i = 0; i < 16; i++, p += 3) {
            *p = ' ';
            hex_byte(pci_config_readb(bdf, first + 16 * r + i), p + 1);
        }
        info_line(row + r, line);
    }
}

static void
info_chipset(void)
{
    char line[80], c0[3], c1[3];
    info_regs("UM8881F  00:10.0", HB_BDF, 0x40, 3, 4);
    info_regs("UM8886BF 00:12.0", IBC_BDF, 0x40, 7, 8);
    info_regs("IDE      00:12.1", IDE_BDF, 0x40, 1, 16);
    outb(0xaa, SIO_INDEX);
    hex_byte(sio_read(0xc0), c0);
    hex_byte(sio_read(0xc1), c1);
    outb(0x55, SIO_INDEX);
    snprintf(line, sizeof(line), "UM8663BF super I/O   C0: %s  C1: %s", c0, c1);
    info_line(18, line);
}

static void info_speed(void);
static void chart_end(void);
static void mt_run(void);

static const struct {
    const char *title;
    void (*draw)(void);
} InfoPages[] = {
    { "CPU, CACHE AND MEMORY", info_cpu },
    { "PCI DEVICES", info_pci },
    { "ISA RESOURCES AND IRQ MAP", info_isa },
    { "IDE DRIVES", info_ide },
    { "CHIPSET REGISTERS", info_chipset },
    { "MEMORY READ SPEED", info_speed },
};

static void
info_run(int first, int last)
{
    int p = first;
    for (;;) {
        chart_end();
        scr_header(InfoPages[p].title);
        InfoPages[p].draw();
        scr_puts(ROW_HELP, 2, AttrText, first == last ? "ESC : Back"
                 : "ESC : Back                PU/PD : Previous/Next Page");
        int key = get_keystroke(1000);
        if (key == 0x01)
            break;
        if (key == 0x49 && p > first)
            p--;
        if (key == 0x51 && p < last)
            p++;
    }
    chart_end();
}


/****************************************************************
 * Password
 ****************************************************************/

static u16
password_hash(const char *s)
{
    u16 h = 0x4e42;
    for (; *s; s++)
        h = h * 31 + (u8)*s;
    return h ? h : 1;
}

// Masked input of up to 8 characters; returns its hash, 0 for an empty entry.
static u16
password_read(const char *prompt)
{
    char buf[9];
    int len = 0, col = 22 + strlen(prompt);
    scr_fill(11, 20, 40, AttrSelect, ' ');
    scr_fill(12, 20, 40, AttrSelect, ' ');
    scr_fill(13, 20, 40, AttrSelect, ' ');
    scr_puts(12, 22, AttrSelect, prompt);
    for (;;) {
        int key = get_keystroke_full(1000);
        if (key < 0)
            continue;
        int ascii = key & 0xff;
        if (ascii == '\r')
            break;
        if (ascii == '\b' && len) {
            scr_puts(12, col + --len, AttrSelect, " ");
        } else if (ascii >= ' ' && ascii < 0x7f && len < 8) {
            buf[len] = ascii;
            scr_puts(12, col + len++, AttrSelect, "*");
        }
    }
    buf[len] = 0;
    return len ? password_hash(buf) : 0;
}

static void
password_set(void)
{
    u16 h = password_read("Enter Password: ");
    if (h && password_read("Confirm Password: ") != h) {
        scr_dialog("Password does not match - press a key");
        return;
    }
    NewPassword = h;
    scr_dialog(h ? "Password set (saved with F10) - press a key"
               : "Password disabled (saved with F10) - press a key");
}

// Ask for the password (3 tries); returns 1 when correct or none is set.
static int
password_check(void)
{
    u16 h = Password;
    int i;
    if (!h)
        return 1;
    for (i = 0; i < 3; i++)
        if (password_read("Enter Password: ") == h)
            return 1;
    return 0;
}

// Security option "System": password before every boot.
void
setup_boot_password(void)
{
    if (!Setting[NB_SECURITY] || !Password)
        return;
    scr_open();
    scr_header("");
    int ok = password_check();
    scr_close();
    if (ok)
        return;
    printf("System halted - wrong password.\n");
    for (;;)
        asm volatile("cli; hlt");
}


/****************************************************************
 * Main menu
 ****************************************************************/

enum { M_PAGE, M_INFO, M_PASSWORD, M_IDE, M_MEMTEST, M_DEFAULTS, M_SAVE, M_QUIT };

static const struct {
    u8 kind, arg;
    const char *name, *help;
} Menu[] = {
    { M_PAGE, 0, "STANDARD CMOS SETUP", "Time, Date, Hard Disk Type, Floppy..." },
    { M_PAGE, 1, "BIOS FEATURES SETUP", "Caches, Boot Sequence, Keyboard, Shadow..." },
    { M_PAGE, 2, "CHIPSET FEATURES SETUP", "DRAM and Cache Timing (Auto Configuration)" },
    { M_PAGE, 3, "POWER MANAGEMENT SETUP", "APM BIOS, HDD Standby Timer" },
    { M_PAGE, 4, "PNP/PCI CONFIGURATION", "PCI IRQ Routing, IDE PIO Modes" },
    { M_PAGE, 5, "INTEGRATED PERIPHERALS", "Onboard IDE, FDC, Serial and Parallel Ports" },
    { M_INFO, 0, "HARDWARE INFORMATION", "CPU, Memory, PCI, ISA/IRQ, IDE, Chipset" },
    { M_PASSWORD, 0, "SUPERVISOR PASSWORD", "Change, Set, or Disable Password" },
    { M_IDE, 0, "IDE HDD AUTO DETECTION", "Drives found at this boot" },
    { M_MEMTEST, 0, "MEMORY TEST", "memtest86+ 4.20 tests on the free RAM" },
    { M_DEFAULTS, 0, "LOAD SETUP DEFAULTS", "Load Setup Defaults" },
    { M_SAVE, 0, "SAVE & EXIT SETUP", "Save Data to CMOS & Exit SETUP" },
    { M_QUIT, 0, "EXIT WITHOUT SAVING", "Abandon all Data & Exit SETUP" },
};

#define MENU_ROWS 7

static void
save_and_reboot(void)
{
    if (!scr_confirm("SAVE to CMOS and EXIT (Y/N)? "))
        return;
    values_save();
    scr_close();
    printf("Settings saved - rebooting\n");
    reset();
}

static void
menu_draw(int sel)
{
    int i;
    scr_header("CMOS SETUP UTILITY");
    for (i = 0; i < ARRAY_SIZE(Menu); i++) {
        int row = 4 + 2 * (i % MENU_ROWS), col = i < MENU_ROWS ? 6 : 44;
        scr_puts(row, col, i == sel ? AttrSelect : AttrText, Menu[i].name);
    }
    draw_help(0);
    scr_center(23, AttrTitle, Menu[sel].help);
}

void
setup_menu(void)
{
    int sel = 0;
    post_code(POST_SETUP);
    scr_open();
    if (!password_check()) {
        scr_close();
        return;
    }
    values_load();
    for (;;) {
        menu_draw(sel);
        int key = get_keystroke(1000), n = ARRAY_SIZE(Menu);
        switch (key) {
        case 0x48: sel = (sel + n - 1) % n; break;
        case 0x50: sel = (sel + 1) % n; break;
        case 0x4b: case 0x4d: sel = (sel + MENU_ROWS) % n; break;
        case 0x44: save_and_reboot(); break;
        case 0x01:
            if (scr_confirm("Quit Without Saving (Y/N)? ")) {
                scr_close();
                return;
            }
            break;
        case 0x1c:
            switch (Menu[sel].kind) {
            case M_PAGE: page_run(&Pages[Menu[sel].arg]); break;
            case M_INFO: info_run(0, ARRAY_SIZE(InfoPages) - 1); break;
            case M_IDE: info_run(3, 3); break;
            case M_MEMTEST: mt_run(); break;
            case M_PASSWORD: password_set(); break;
            case M_DEFAULTS:
                if (scr_confirm("Load Setup Defaults (Y/N)? "))
                    values_default();
                break;
            case M_SAVE: save_and_reboot(); break;
            case M_QUIT:
                if (scr_confirm("Quit Without Saving (Y/N)? ")) {
                    scr_close();
                    return;
                }
                break;
            }
        }
    }
}


static void
cursor_row(int row)
{
    struct bregs br;
    memset(&br, 0, sizeof(br));
    br.flags = F_IF;
    br.ah = 0x02;
    br.dh = row;
    call16_int(0x10, &br);
}

/****************************************************************
 * POST screen
 ****************************************************************/

int SetupRequested;

#define ROW_CPU    4
#define ROW_MEMORY 5
#define ROW_CACHE  6

static const char *
cpu_short_name(int *mhz)
{
    static char vendor[13];
    u32 eax, ebx, ecx, edx, fms = 0;
    int valid = 0;
    const char *name = "486";
    if (cpu_has_cpuid()) {
        cpuid(0, &eax, &ebx, &ecx, &edx);
        memcpy(vendor, &ebx, 4);
        memcpy(vendor + 4, &edx, 4);
        memcpy(vendor + 8, &ecx, 4);
        if (eax >= 1)
            cpuid(1, &fms, &ebx, &ecx, &edx);
        name = cpu_name(vendor, fms, &valid);
    }
    *mhz = valid ? cpu_mhz() : 0;
    return name;
}

static u32
l2_size_kb(void)
{
    u8 l2 = pci_config_readb(HB_BDF, 0x50);
    return (l2 & 0x80) && (l2 & 7) ? 32 << (l2 & 7) : 0;
}

// Award-style POST screen in the BIOS text mode: CPU, cache; then the memory test and the
// read speed chart fill it, and the teletype output continues below.
void
setup_post_banner(void)
{
    char line[80];
    int mhz;
    const char *name = cpu_short_name(&mhz);
    int10(0x0003, 0);
    Vram = (void*)0xb8000;
    AttrText = 0x07;
    if (GET_BDA(video_mode) != 3)
        return;
    scr_puts(0, 0, AttrText, "newBIOS for Biostar MB-8433UUD-A");
    scr_puts(1, 0, AttrText, "coreboot + SeaBIOS");
    scr_puts(24, 0, AttrText, VERSION);
    if (mhz)
        snprintf(line, sizeof(line), "%s CPU at %dMHz", name, mhz);
    else
        snprintf(line, sizeof(line), "%s CPU", name);
    scr_puts(ROW_CPU, 0, AttrText, line);
    u32 l2 = l2_size_kb();
    snprintf(line, sizeof(line), l2 ? "External Cache : %dK installed" : "External Cache : none", l2);
    scr_puts(ROW_CACHE, 0, AttrText, line);
    cursor_row(8);                          // teletype output continues below
}

// Keys during the memory test: ESC skips it, DEL requests setup at the end of POST.
static int
memtest_key(void)
{
    int scan = get_keystroke(0);
    if (scan == 0x53)
        SetupRequested = 1;
    return scan == 0x01;
}

// Test one 4 KiB block without losing its contents (POST data lives in this RAM): two patterns,
// each written, flushed from the cache and read back.
static int
memtest_block(u32 *block, u32 *save, int step)
{
    int i, bad = 0, n = 1024;
    memcpy(save, block, 4096);
    for (i = 0; i < n; i += step)
        block[i] = (u32)&block[i] ^ 0x5aa5a55a;
    wbinvd();
    for (i = 0; i < n; i += step)
        bad |= block[i] != ((u32)&block[i] ^ 0x5aa5a55a);
    for (i = 0; i < n; i += step)
        block[i] = ~((u32)&block[i] ^ 0x5aa5a55a);
    wbinvd();
    for (i = 0; i < n; i += step)
        bad |= block[i] != ~((u32)&block[i] ^ 0x5aa5a55a);
    memcpy(block, save, 4096);
    return bad;
}

static void
memtest_show(u32 kb, const char *result)
{
    char line[80];
    snprintf(line, sizeof(line), "Memory Test : %dK %s", kb, result);
    scr_fill(ROW_MEMORY, 0, 50, AttrText, ' ');
    scr_puts(ROW_MEMORY, 0, AttrText, line);
}

// Award-style count of the RAM with a read/write test of every block nothing uses yet (the code and
// data of this POST live in the rest).
static u32 MemtestBad;

static void
memtest(void)
{
    u32 *save = malloc_tmphigh(4096), start, end, a, bad = 0;
    int step = Setting[NB_QUICK_POST] ? 256 : 1, n, skipped = 0;
    u32 kb = 1024, total = 1024 + ext_mem_kb();
    char fail[40];
    if (!save)
        return;
    scr_puts(23, 0, AttrText, "Press DEL to enter SETUP, ESC to skip memory test");   // keyboard ready
    for (n = 0; !skipped && malloc_tmphigh_free(n, &start, &end); n++) {
        for (a = ALIGN(start, 4096); a + 4096 <= end && !skipped; a += 4096, kb += 4) {
            u32 flags = save_flags();
            irq_disable();
            if (memtest_block((void*)a, save, step) && !bad)
                bad = a;
            restore_flags(flags);
            if (!(kb & 1023)) {
                memtest_show(kb, "");
                skipped = memtest_key();
            }
        }
    }
    free(save);
    snprintf(fail, sizeof(fail), "FAIL at %x", bad);
    memtest_show(total, bad ? fail : skipped ? "Skipped" : "OK");
    MemtestBad = bad;
}

// Memory read speed by block size, 4K to 2M in quarter octaves: L1, L2 and DRAM show up as steps.
#define SPEED_POINTS 37
static u16 SpeedMBs[SPEED_POINTS];

// Block size of point i: 4 KiB * 2^(i/4), a multiple of 64 bytes.
static u32
speed_block(int i)
{
    static const u16 quarter[4] = { 1000, 1189, 1414, 1682 };   // 1000 * 2^(n/4)
    return (64u << (i / 4)) * quarter[i % 4] / 1000 * 64;
}

static void
read_block(u32 addr, u32 bytes)
{
    u32 n = bytes / 64, dummy;
    asm volatile("1: movl (%1), %0; movl 4(%1), %0; movl 8(%1), %0; movl 12(%1), %0\n"
                 "movl 16(%1), %0; movl 20(%1), %0; movl 24(%1), %0; movl 28(%1), %0\n"
                 "movl 32(%1), %0; movl 36(%1), %0; movl 40(%1), %0; movl 44(%1), %0\n"
                 "movl 48(%1), %0; movl 52(%1), %0; movl 56(%1), %0; movl 60(%1), %0\n"
                 "addl $64, %1; decl %2; jnz 1b"
                 : "=&r"(dummy), "+r"(addr), "+r"(n) : : "memory");
}

// MB/s reading 'block' bytes from 1 MiB over and over for about 20 ms (after one warm-up pass).
static u16
read_speed(u32 block)
{
    u32 chunk = block < 0x10000 ? block : 0x10000, done = 0, ticks = 0, off;
    u32 flags = save_flags();
    irq_disable();
    read_block(0x100000, block);
    u32 t0 = timer_read();
    do {
        for (off = 0; off < block; off += chunk) {
            u32 n = block - off < chunk ? block - off : chunk;
            read_block(0x100000 + off, n);
            done += n;
            ticks = timer_read() - t0;          // read often: the timer wraps
        }
    } while (ticks < TimerKHz * 20);
    restore_flags(flags);
    return (done >> 10) * TimerKHz / ticks * 1024 / 1000;
}

// Measured once, the first time setup shows the speeds (about 1 s).
static void
read_speed_measure(void)
{
    int i;
    if (SpeedMBs[0])
        return;
    for (i = 0; i < SPEED_POINTS; i++)
        SpeedMBs[i] = speed_block(i) <= ext_mem_kb() * 1024 ? read_speed(speed_block(i)) : 0;
}

void
setup_memtest(void)
{
    if (GET_BDA(video_mode) == 3 && Vram)
        memtest();
}

// Line chart of SpeedMBs (SysSpeed style): a 512x160-dot bitmap shown as redefined characters in
// 8-dot cells, so that they join without the 9th-dot gap.
#define CHART_COLS   64
#define CHART_ROWS   10
#define CHART_W      (CHART_COLS * 8)
#define CHART_H      (CHART_ROWS * 16)
#define CHART_GLYPH0 0x80                   // characters 0x80-0xff are redefined
#define CHART_ROW    4
#define CHART_COL    8
static int ChartMode;                       // 8-dot cells and redefined characters: chart_end() restores

// 25.175 MHz dot clock and 8-dot cells: the same 31.5 kHz / 70 Hz signal as the 9-dot text mode.
static void
text_8dot(void)
{
    outb(0x00, 0x3c4);
    outb(0x01, 0x3c5);                      // sequencer synchronous reset
    outb(inb(0x3cc) & ~0x0c, 0x3c2);        // 25.175 MHz
    outb(0x01, 0x3c4);
    outb(inb(0x3c5) | 0x01, 0x3c5);         // 8 dots per character
    outb(0x00, 0x3c4);
    outb(0x03, 0x3c5);
    inb(0x3da);
    outb(0x33, 0x3c0);                      // horizontal panning 0 for 8-dot cells
    outb(0x00, 0x3c0);
}

static void
chart_end(void)
{
    if (ChartMode)
        scr_open();                         // mode set: 9-dot cells, ROM font
    ChartMode = 0;
}

static void
chart_dot(u8 *bmp, int x, int y)
{
    bmp[y * CHART_COLS + x / 8] |= 0x80 >> (x & 7);
}

// Bresenham line, x0 <= x1.
static void
chart_line(u8 *bmp, int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0, dy = y1 > y0 ? y1 - y0 : y0 - y1, sy = y1 > y0 ? 1 : -1, err = dx - dy;
    for (;;) {
        chart_dot(bmp, x0, y0);
        if (x0 == x1 && y0 == y1)
            return;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0++;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Text centered on column col.
static void
chart_label(int row, int col, const char *s)
{
    scr_puts(row, col - strlen(s) / 2, AttrText, s);
}

static void
info_speed(void)
{
    static const char * const size[] = {
        "4K", "8K", "16K", "32K", "64K", "128K", "256K", "512K", "1M", "2M"
    };
    u8 *bmp = malloc_tmphigh(CHART_COLS * CHART_H), *font = malloc_tmplow(128 * 16);
    char num[8];
    int i, r, c, max = 1, glyphs = 0, x0 = -1, y0 = 0;
    if (!bmp || !font)
        goto done;
    read_speed_measure();
    for (i = 0; i < SPEED_POINTS; i++)
        if (SpeedMBs[i] > max)
            max = SpeedMBs[i];
    memset(bmp, 0, CHART_COLS * CHART_H);
    for (i = 0; i < CHART_H; i++)
        chart_dot(bmp, 0, i);
    for (i = 0; i < CHART_W; i++)
        chart_dot(bmp, i, CHART_H - 1);
    for (i = 0; i < SPEED_POINTS && SpeedMBs[i]; i++) {
        int x = i * (CHART_W - 1) / (SPEED_POINTS - 1);
        int y = CHART_H - 1 - SpeedMBs[i] * (CHART_H - 9) / max;
        chart_line(bmp, x0 < 0 ? x : x0, x0 < 0 ? y : y0, x, y);
        x0 = x;
        y0 = y;
    }
    for (r = 0; r < CHART_ROWS; r++)        // cells -> characters, identical ones shared
        for (c = 0; c < CHART_COLS; c++) {
            u8 cell[16];
            int line, g, used = 0;
            for (line = 0; line < 16; line++)
                used |= cell[line] = bmp[(r * 16 + line) * CHART_COLS + c];
            for (g = 0; used && g < glyphs && memcmp(font + g * 16, cell, 16); g++)
                ;
            if (used && g == glyphs && glyphs < 128)
                memcpy(font + glyphs++ * 16, cell, 16);
            scr_fill(CHART_ROW + r, CHART_COL + c, 1, AttrValue,
                     used && g < glyphs ? CHART_GLYPH0 + g : ' ');
        }
    if (GET_BDA(char_height) == 16) {
        struct bregs br;
        memset(&br, 0, sizeof(br));
        br.flags = F_IF;
        br.ax = 0x1110;                     // load user font: 16 scanlines per character
        br.bx = 0x1000;
        br.cx = glyphs;
        br.dx = CHART_GLYPH0;
        br.es = FLATPTR_TO_SEG(font);
        br.bp = FLATPTR_TO_OFFSET(font);
        if (GET_BDA(video_mode) == 3)
            text_8dot();
        call16_int(0x10, &br);
        ChartMode = 1;
    }
    scr_puts(CHART_ROW - 1, 2, AttrText, "Memory read speed (MB/s) by block size");
    snprintf(num, sizeof(num), "%d", max);
    scr_puts(CHART_ROW, CHART_COL - 1 - strlen(num), AttrText, num);
    snprintf(num, sizeof(num), "%d", max / 2);
    scr_puts(CHART_ROW + (CHART_H - 1 - (CHART_H - 9) / 2) / 16, CHART_COL - 1 - strlen(num), AttrText, num);
    scr_puts(CHART_ROW + CHART_ROWS - 1, CHART_COL - 2, AttrText, "0");
    for (i = 0; i < ARRAY_SIZE(size); i++) {   // block sizes and speeds at the octaves
        int col = CHART_COL + i * 4 * (CHART_W - 1) / (SPEED_POINTS - 1) / 8;
        chart_label(CHART_ROW + CHART_ROWS, col, size[i]);
        snprintf(num, sizeof(num), "%d", SpeedMBs[i * 4]);
        chart_label(CHART_ROW + CHART_ROWS + 1, col, SpeedMBs[i * 4] ? num : "-");
    }
done:
    free(bmp);
    free(font);
}

static void
summary_line(const char *left, const char *right)
{
    char line[81];
    int n = snprintf(line, sizeof(line), " %s", left);
    while (n < 40)
        line[n++] = ' ';
    snprintf(line + n, sizeof(line) - n, "%s", right);
    printf("%s\n", line);
}

static void
ports_str(char *out, int size, int first_lpt)
{
    char *p = out;
    int i;
    *p = 0;
    for (i = 0; i < (first_lpt ? 3 : 4); i++) {
        u16 port = first_lpt ? GET_BDA(port_lpt[i]) : GET_BDA(port_com[i]);
        if (port)
            p += snprintf(p, out + size - p, "%X ", port);
    }
    if (!out[0])
        snprintf(out, size, "None");
}

// Award-style system configuration box at the end of POST.
void
setup_post_summary(void)
{
    static const char * const unit[] = { "Pri. Master", "Pri. Slave ",
                                         "Sec. Master", "Sec. Slave " };
    char rule[79], left[48], right[48], ports[32];
    int mhz, i;
    const char *name = cpu_short_name(&mhz);
    u8 l2 = pci_config_readb(HB_BDF, 0x50);
    u32 fpu = 0, eax, ebx, ecx;
    if (cpu_has_cpuid())
        cpuid(1, &eax, &ebx, &ecx, &fpu);
    else
        fpu = (cr0_read() & 0x10) ? 1 : 0;
    memset(rule, 0xc4, sizeof(rule) - 1);
    rule[sizeof(rule) - 1] = 0;
    int10(0x0003, 0);
    printf("%s\n", rule);
    snprintf(left, sizeof(left), "CPU Type     : %s", name);
    snprintf(right, sizeof(right), "Base Memory      : %dK", GET_BDA(mem_size_kb));
    summary_line(left, right);
    snprintf(left, sizeof(left), "Co-Processor : %s", (fpu & 1) ? "Installed" : "None");
    snprintf(right, sizeof(right), "Extended Memory  : %dK", ext_mem_kb());
    summary_line(left, right);
    snprintf(left, sizeof(left), "CPU Clock    : %dMHz", mhz);
    snprintf(right, sizeof(right), "Cache Memory     : %dK",
             (l2 & 0x80) && (l2 & 7) ? 32 << (l2 & 7) : 0);
    summary_line(left, right);
    printf("%s\n", rule);
    snprintf(left, sizeof(left), "Diskette Drive A : %s", Floppy[Setting[NB_FLOPPY_A]]);
    ports_str(ports, sizeof(ports), 0);
    snprintf(right, sizeof(right), "Serial Port(s)   : %s", ports);
    summary_line(left, right);
    snprintf(left, sizeof(left), "Diskette Drive B : %s", Floppy[Setting[NB_FLOPPY_B]]);
    ports_str(ports, sizeof(ports), 1);
    snprintf(right, sizeof(right), "Parallel Port(s) : %s", ports);
    summary_line(left, right);
    for (i = 0; i < 4; i++) {
        u32 mb = IdeInfo[i].sectors / 2048;
        if (!IdeInfo[i].present)
            printf(" %s Disk : None\n", unit[i]);
        else if (IdeInfo[i].atapi)
            printf(" %s Disk : CD-ROM, PIO %d  %s\n", unit[i], IdeInfo[i].pio, IdeInfo[i].model);
        else
            printf(" %s Disk : %d%s, PIO %d  %s\n", unit[i], mb >= 10000 ? mb / 1024 : mb,
                   mb >= 10000 ? "GB" : "MB", IdeInfo[i].pio, IdeInfo[i].model);
    }
    printf("%s\n", rule);
    if (MemtestBad)
        printf("Memory Test FAIL at %x\n", MemtestBad);
}


/****************************************************************
 * Memory test: the test sequence of memtest86+ 4.20 (tests 0-8, its default pass) on the RAM
 * above 1 MiB that POST does not use. Reimplemented around the POST code: CPU, cache, speeds.
 ****************************************************************/

#define MT_SEGS     8
#define MT_CHUNK    0x40000                 // bytes between screen updates and key checks
#define MT_ERR_ROWS 5
#define MT_ROW_ERR  16
#define MT_TESTS    9

typedef void (*mt_fn)(volatile u32 *p, volatile u32 *e);

static struct { u32 start, end; } MtSeg[MT_SEGS];
static int MtSegs, MtTest, MtPass, MtAbort, MtErrRows, MtDry;
static u32 MtP1, MtInv, MtRot, MtOff, MtIter, MtRand, MtErrors, MtErrBits, MtTime;
static u32 MtKB, MtTestKB[MT_TESTS], MtTestStart, MtPassKB;   // progress: KiB swept (MtDry: counted only)
static struct { u32 addr, good, bad, count; int test; } MtErr[MT_ERR_ROWS];

static const char * const MtName[MT_TESTS] = {
    "Address test, walking ones", "Address test, own address", "Moving inversions, ones & zeros",
    "Moving inversions, 8 bit pattern", "Moving inversions, random pattern", "Block move, 80 moves",
    "Moving inversions, 32 bit pattern", "Random number sequence", "Modulo 20, random pattern",
};
static const u8 MtIters[MT_TESTS] = { 1, 1, 4, 2, 50, 80, 2, 30, 6 };   // halved in the first pass

static inline u32
rol32(u32 v, u32 n)
{
    return (v << (n & 31)) | (v >> (-n & 31));
}

static u32
mt_rand(void)
{
    MtRand ^= MtRand << 13;
    MtRand ^= MtRand >> 17;
    MtRand ^= MtRand << 5;
    return MtRand;
}

static u32
mt_seconds(void)
{
    return rtc_get(CMOS_RTC_HOURS) * 3600 + rtc_get(CMOS_RTC_MINUTES) * 60 + rtc_get(CMOS_RTC_SECONDS);
}

static void
mt_error(u32 addr, u32 good, u32 bad)
{
    int i;
    MtErrors++;
    MtErrBits |= good ^ bad;
    for (i = 0; i < MtErrRows && MtErr[i].addr != addr; i++)
        ;
    if (i == MT_ERR_ROWS)                   // table full: only counted
        return;
    if (i == MtErrRows) {                   // new address: its first error is shown
        MtErrRows++;
        MtErr[i].addr = addr;
        MtErr[i].good = good;
        MtErr[i].bad = bad;
        MtErr[i].test = MtTest;
        MtErr[i].count = 0;
    }
    MtErr[i].count++;
}

static inline void
mt_check(volatile u32 *p, u32 good)
{
    u32 bad = *p;
    if (bad != good)
        mt_error((u32)p, good, bad);
}

static int
mt_tested(u32 addr)
{
    int i;
    for (i = 0; i < MtSegs; i++)
        if (addr >= MtSeg[i].start && addr < MtSeg[i].end)
            return 1;
    return 0;
}

// "Name     : nnn% " and a 40-character bar.
static void
mt_bar(int row, const char *name, u32 done, u32 total)
{
    char line[80];
    int pct = total >= 100 ? done / (total / 100) : 0, i;
    if (pct > 100)
        pct = 100;
    snprintf(line, sizeof(line), "%s : %s%s%d%%", name, pct < 100 ? " " : "", pct < 10 ? " " : "", pct);
    scr_puts(row, 3, AttrText, line);
    for (i = 0; i < 40; i++)
        scr_fill(row, 20 + i, 1, AttrText, i < pct * 40 / 100 ? 0xdb : 0xb0);
}

// Screen update and ESC check, between chunks.
static void
mt_poll(u32 addr)
{
    char line[80];
    int i, t = (mt_seconds() + 86400 - MtTime) % 86400;
    mt_bar(11, "Pass    ", MtKB, MtPassKB);
    if (MtTest < MT_TESTS)                  // MT_TESTS: the pass is complete
        mt_bar(12, "Test    ", MtKB - MtTestStart, MtTestKB[MtTest]);
    snprintf(line, sizeof(line), "Pattern  : %08x   Address : %dK      ", MtP1, addr >> 10);
    scr_puts(13, 3, AttrText, line);
    snprintf(line, sizeof(line), "Pass     : %d", MtPass);
    scr_puts(4, 42, AttrText, line);
    snprintf(line, sizeof(line), "Time     : %d:%d%d:%d%d", t / 3600, t / 600 % 6, t / 60 % 10,
             t % 60 / 10, t % 10);           // (%d has no width)
    scr_puts(5, 42, AttrText, line);
    snprintf(line, sizeof(line), "Errors   : %d", MtErrors);
    scr_puts(6, 42, MtErrors ? AttrValue : AttrText, line);
    snprintf(line, sizeof(line), "Err-Bits : %08x", MtErrBits);
    scr_puts(7, 42, AttrText, line);
    for (i = 0; i < MtErrRows; i++) {
        snprintf(line, sizeof(line), " %d   %08x  %08x  %08x  %08x  %d", MtErr[i].test, MtErr[i].addr,
                 MtErr[i].good, MtErr[i].bad, MtErr[i].good ^ MtErr[i].bad, MtErr[i].count);
        scr_puts(MT_ROW_ERR + i, 3, AttrValue, line);
    }
    if (get_keystroke(0) == 0x01)
        MtAbort = 1;
}

// fn on all tested RAM, MT_CHUNK bytes at a time, upwards or downwards (fn walks its chunk the same way).
static void
mt_sweep(mt_fn fn, int down)
{
    int i;
    for (i = 0; i < MtSegs && MtDry; i++)
        MtKB += (MtSeg[i].end - MtSeg[i].start) >> 10;
    for (i = 0; i < MtSegs && !MtAbort && !MtDry; i++) {
        u32 start = MtSeg[down ? MtSegs - 1 - i : i].start, end = MtSeg[down ? MtSegs - 1 - i : i].end;
        u32 a, n;
        if (down)
            for (a = end; a > start && !MtAbort; a -= n) {
                n = a - start < MT_CHUNK ? a - start : MT_CHUNK;
                fn((void*)(a - n), (void*)a);
                MtKB += n >> 10;
                mt_poll(a - n);
            }
        else
            for (a = start; a < end && !MtAbort; a += n) {
                n = end - a < MT_CHUNK ? end - a : MT_CHUNK;
                fn((void*)a, (void*)(a + n));
                MtKB += n >> 10;
                mt_poll(a);
            }
    }
}

// Writing ~v at p | each address bit (in tested RAM below end) must leave v at p.
static void
mt_addr_bits(u32 addr, u32 end, u32 v, int reps)
{
    volatile u32 *p = (void*)addr;
    u32 bit, got;
    *p = v;
    while (reps--)
        for (bit = 4; bit && (addr | bit) < end; bit <<= 1)
            if (!(addr & bit) && mt_tested(addr | bit)) {
                *(volatile u32*)(addr | bit) = ~v;
                if ((got = *p) != v) {
                    mt_error(addr, v, got);
                    return;
                }
            }
}

static void
mt_addr_walk(void)
{
    u32 first = ALIGN(MtSeg[0].start, 0x20000), end = MtSeg[MtSegs - 1].end, a;
    u32 bank = ext_mem_kb() > 7 * 1024 ? 0x100000 : 0x40000;
    int k, i;
    if (MtDry)
        return;
    for (k = 0; k < 2 && mt_tested(first); k++)
        mt_addr_bits(first, end, k ? ~0 : 0, 100);
    for (k = 0; k < 2; k++)
        for (i = 0; i < MtSegs; i++)
            for (a = ALIGN(MtSeg[i].start, bank); a < MtSeg[i].end && !MtAbort; a += bank) {
                mt_addr_bits(a, MtSeg[i].end, k ? ~0 : 0, 200);
                mt_poll(a);
            }
}

static void
mt_own_fill(volatile u32 *p, volatile u32 *e)
{
    for (; p < e; p++)
        *p = (u32)p;
}

static void
mt_own_check(volatile u32 *p, volatile u32 *e)
{
    for (; p < e; p++)
        mt_check(p, (u32)p);
}

// Moving inversions: the pattern of a dword is MtP1 rotated by MtRot * (its index + MtOff).
static u32
mt_pattern(volatile u32 *p)
{
    return MtRot ? rol32(MtP1, ((u32)p >> 2) + MtOff) : MtP1;
}

static void
mt_inv_fill(volatile u32 *p, volatile u32 *e)
{
    u32 pat = mt_pattern(p), rot = MtRot;
    for (; p < e; p++, pat = rol32(pat, rot))
        *p = pat;
}

static void
mt_inv_up(volatile u32 *p, volatile u32 *e)
{
    u32 pat = mt_pattern(p), rot = MtRot;
    for (; p < e; p++, pat = rol32(pat, rot)) {
        mt_check(p, pat);
        *p = ~pat;
    }
}

static void
mt_inv_down(volatile u32 *p, volatile u32 *e)
{
    u32 pat = mt_pattern(e - 1), rot = MtRot;
    while (e-- > p) {
        mt_check(e, ~pat);
        *e = pat;
        pat = rol32(pat, -rot);
    }
}

static void
mt_movinv(int iter, u32 p1, int rot, int off)
{
    MtP1 = p1;
    MtRot = rot;
    MtOff = off;
    mt_sweep(mt_inv_fill, 0);
    while (iter--) {
        mt_sweep(mt_inv_up, 0);
        mt_sweep(mt_inv_down, 1);
    }
}

static void
mt_rand_fill(volatile u32 *p, volatile u32 *e)
{
    for (; p < e; p++)
        *p = mt_rand();
}

static void
mt_rand_check(volatile u32 *p, volatile u32 *e)
{
    u32 inv = MtInv;
    for (; p < e; p++) {
        u32 num = mt_rand() ^ inv;
        mt_check(p, num);
        *p = ~num;
    }
}

// Random sequence written, then checked and complemented twice from the same seed.
static void
mt_movinvr(void)
{
    u32 seed = mt_rand();
    int i;
    MtP1 = MtRand = seed;
    mt_sweep(mt_rand_fill, 0);
    for (i = 0; i < 2; i++) {
        MtRand = seed;
        MtInv = i ? ~0 : 0;
        mt_sweep(mt_rand_check, 0);
    }
}

// Block move: 64-byte blocks of a rotating pattern in pairs of equal dwords, each chunk's first half
// copied to its second half and back 32 bytes further MtIter times; the pairs must still match.
static void
mt_block_fill(volatile u32 *p, volatile u32 *e)
{
    u32 a = rol32(1, (u32)p >> 6), i;
    for (; p < e; p += 16, a = rol32(a, 1))
        for (i = 0; i < 16; i++)
            p[i] = (0xcc30 >> i) & 1 ? ~a : a;
}

static void
mt_block_moves(volatile u32 *p, volatile u32 *e)
{
    u32 half = (e - p) / 2, i, *a = (u32*)p, *b = a + half;
    for (i = 0; i < MtIter; i++) {
        memcpy(b, a, half * 4);
        memcpy(a + 8, b, (half - 8) * 4);
        memcpy(a, b + half - 8, 32);
    }
}

static void
mt_block_check(volatile u32 *p, volatile u32 *e)
{
    for (; p < e; p += 2)
        mt_check(p + 1, p[0]);
}

// Modulo 20: MtP1 in every 20th dword (index MtOff), its complement written around it twice.
static volatile u32 *
mt_mod_first(volatile u32 *p)
{
    return p + (MtOff + 20 - ((u32)p >> 2) % 20) % 20;
}

static void
mt_mod_set(volatile u32 *p, volatile u32 *e)
{
    for (p = mt_mod_first(p); p < e; p += 20)
        *p = MtP1;
}

static void
mt_mod_others(volatile u32 *p, volatile u32 *e)
{
    u32 k = ((u32)p >> 2) % 20, off = MtOff, p2 = ~MtP1;
    for (; p < e; p++, k = k == 19 ? 0 : k + 1)
        if (k != off)
            *p = p2;
}

static void
mt_mod_check(volatile u32 *p, volatile u32 *e)
{
    for (p = mt_mod_first(p); p < e; p += 20)
        mt_check(p, MtP1);
}

static void
mt_modtst(int off, u32 p1)
{
    MtOff = off;
    MtP1 = p1;
    mt_sweep(mt_mod_set, 0);
    mt_sweep(mt_mod_others, 0);
    mt_sweep(mt_mod_others, 0);
    mt_sweep(mt_mod_check, 0);
}

static void
mt_test(int test, int iter)
{
    u32 p;
    int i, j;
    MtP1 = 0;
    switch (test) {
    case 0: mt_addr_walk(); break;
    case 1: mt_sweep(mt_own_fill, 0); mt_sweep(mt_own_check, 0); break;
    case 2: mt_movinv(iter, 0, 0, 0); mt_movinv(iter, ~0, 0, 0); break;
    case 3:
        for (p = 0x80808080; p; p = (p >> 1) & 0x7f7f7f7f) {
            mt_movinv(iter, p, 0, 0);
            mt_movinv(iter, ~p, 0, 0);
        }
        break;
    case 4:
        for (i = 0; i < iter; i++)
            mt_movinv(2, mt_rand(), 0, 0);
        break;
    case 5:
        MtIter = iter;
        mt_sweep(mt_block_fill, 0);
        mt_sweep(mt_block_moves, 0);
        mt_sweep(mt_block_check, 0);
        break;
    case 6:
        for (i = 0; i < 32; i++) {
            mt_movinv(iter, 1, 1, i);
            mt_movinv(iter, ~1, 1, i);
        }
        break;
    case 7:
        for (i = 0; i < iter; i++)
            mt_movinvr();
        break;
    case 8:
        for (j = 0; j < iter; j++) {
            p = mt_rand();
            for (i = 0; i < 20; i++) {
                mt_modtst(i, p);
                mt_modtst(i, ~p);
            }
        }
        break;
    }
}

// Read speed measured at POST for a block of 'kb' KiB or the largest one below.
static u16
mt_speed(u32 kb)
{
    int i = 0;
    while (i < SPEED_POINTS - 1 && SpeedMBs[i + 1] && speed_block(i + 1) <= (u64)kb * 1024)
        i++;
    return SpeedMBs[i];
}

static void
mt_run(void)
{
    char line[80];
    u32 start, end, l2 = l2_size_kb(), total = 0;
    int mhz, n, i;
    const char *name = cpu_short_name(&mhz);
    post_code(POST_SETUP_MEMTEST);
    read_speed_measure();
    MtSegs = 0;
    for (n = 0; MtSegs < MT_SEGS && malloc_tmphigh_free(n, &start, &end); n++) {
        start = ALIGN(start, 4096);
        end = ALIGN_DOWN(end, 4096);
        if (end <= start)
            continue;
        for (i = MtSegs++; i > 0 && MtSeg[i - 1].start > start; i--)
            MtSeg[i] = MtSeg[i - 1];
        MtSeg[i].start = start;
        MtSeg[i].end = end;
        total += end - start;
    }
    scr_header("MEMORY TEST");
    snprintf(line, sizeof(line), mhz ? "CPU      : %s %dMHz" : "CPU      : %s", name, mhz);
    info_line(4, line);
    snprintf(line, sizeof(line), "L1 Cache : %d MB/s", mt_speed(4));
    info_line(5, line);
    if (l2)
        snprintf(line, sizeof(line), "L2 Cache : %dK, %d MB/s", l2, mt_speed(l2 / 2));
    else
        snprintf(line, sizeof(line), "L2 Cache : none");
    info_line(6, line);
    snprintf(line, sizeof(line), "Memory   : %dK, %d MB/s", 1024 + ext_mem_kb(), mt_speed(-1));
    info_line(7, line);
    if (MtSegs)
        snprintf(line, sizeof(line), "Testing  : %dK - %dK, %dK free", MtSeg[0].start >> 10,
                 MtSeg[MtSegs - 1].end >> 10, total >> 10);
    info_line(8, MtSegs ? line : "Testing  : no free RAM");
    info_line(MT_ROW_ERR - 1, "Tst  Address   Good      Bad       Err-Bits  Count");
    scr_puts(ROW_HELP, 2, AttrText, "ESC : Back");
    MtPass = MtAbort = MtErrRows = MtErrors = MtErrBits = 0;
    MtRand = 0x2545f491;
    MtTime = mt_seconds();
    while (MtSegs && !MtAbort) {
        u32 rand = MtRand;
        MtDry = 1;                          // sizes of this pass's tests, for the progress bars
        for (MtPassKB = MtTest = 0; MtTest < MT_TESTS; MtPassKB += MtTestKB[MtTest++]) {
            MtKB = 0;
            mt_test(MtTest, MtPass ? MtIters[MtTest] : MtIters[MtTest] / 2);
            MtTestKB[MtTest] = MtKB;
        }
        MtDry = MtKB = 0;
        MtRand = rand;
        for (MtTest = 0; MtTest < MT_TESTS && !MtAbort; MtTest++) {
            snprintf(line, sizeof(line), "Test %d [%s]", MtTest, MtName[MtTest]);
            scr_fill(10, 3, 60, AttrText, ' ');
            scr_puts(10, 3, AttrTitle, line);
            MtTestStart = MtKB;
            mt_test(MtTest, MtPass ? MtIters[MtTest] : MtIters[MtTest] / 2);
        }
        if (MtAbort)
            break;
        MtPass++;
        mt_poll(0);
        if (!MtErrors)
            scr_center(22, AttrTitle, "Pass complete, no errors, press Esc to exit");
    }
    while (!MtSegs && get_keystroke(1000) != 0x01)
        ;
    post_code(POST_SETUP);
}
