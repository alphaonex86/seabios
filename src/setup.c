// Award-style CMOS setup utility (DEL during POST), the settings it keeps in CMOS (newbios_cmos.h)
// and the POST options SeaBIOS applies from them.
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_BDA
#include "block.h" // struct drive_s
#include "e820map.h" // e820_list
#include "malloc.h" // malloc_tmphigh
#include "bregs.h" // struct bregs
#include "hw/ata.h" // ata_process_op
#include "hw/pci.h" // pci_config_readb
#include "hw/usb.h" // UsbListed
#include "hw/pci_regs.h" // PCI_INTERRUPT_LINE
#include "hw/pcidevice.h" // foreachpci
#include "hw/ps2port.h" // ps2_kbd_command
#include "hw/usb-hid.h" // usb_kbd_active
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
    [NB_FLOPPY_A] = 4, [NB_HALT_ON] = CONFIG_NEWBIOS_HALT_ON, [NB_L1_CACHE] = 1, [NB_L2_CACHE] = 1,
    [NB_BOOT1] = 1, [NB_BOOT2] = 2, [NB_BOOT3] = 3, [NB_BOOT4] = 4,
    [NB_BOOT_DELAY] = 1, [NB_NUMLOCK] = 1, [NB_QUICK_POST] = 1, [NB_IDE_BLOCK] = 1, [NB_CHIPSET_AUTO] = 1, [NB_APM] = 1,
    [NB_IDE_32BIT] = 1, [NB_ISAPNP] = 1, [NB_IDE_PRIMARY] = 1, [NB_IDE_SECONDARY] = 1, [NB_FDC] = 1,
    [NB_COM1] = 1, [NB_COM2] = 2, [NB_LPT] = 1,
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
// Hard disk types of the original BIOS (the AT table); type 15 is reserved.
static const struct { u16 cyl; u8 heads, spt; } HddTypes[46] = {
    { 306, 4, 17 }, { 615, 4, 17 }, { 615, 6, 17 }, { 940, 8, 17 }, { 940, 6, 17 }, { 615, 4, 17 },
    { 462, 8, 17 }, { 733, 5, 17 }, { 900, 15, 17 }, { 820, 3, 17 }, { 855, 5, 17 }, { 855, 7, 17 },
    { 306, 8, 17 }, { 733, 7, 17 }, { 0, 0, 0 }, { 612, 4, 17 }, { 977, 5, 17 }, { 977, 7, 17 },
    { 1024, 7, 17 }, { 733, 5, 17 }, { 733, 7, 17 }, { 733, 5, 17 }, { 306, 4, 17 }, { 977, 5, 17 },
    { 1024, 9, 17 }, { 1224, 7, 17 }, { 1224, 11, 17 }, { 1224, 15, 17 }, { 1024, 8, 17 },
    { 1024, 11, 17 }, { 918, 11, 17 }, { 925, 9, 17 }, { 1024, 10, 17 }, { 1024, 12, 17 },
    { 1024, 13, 17 }, { 1024, 14, 17 }, { 1024, 2, 17 }, { 1024, 16, 17 }, { 918, 15, 17 },
    { 820, 6, 17 }, { 1024, 5, 17 }, { 1024, 5, 26 }, { 809, 6, 17 }, { 809, 6, 26 }, { 776, 8, 33 },
    { 684, 16, 38 },
};

// IDE drive types (NB_HDD_*) with the User geometry, in CMOS at NB_CMOS_HDD.
struct hdd {
    u8 type, heads, spt;
    u16 cyl;
};
static struct hdd Hdd[4];
_Static_assert(NB_CMOS_BASE + (NB_COUNT + 1) / 2 <= NB_CMOS_HDD, "settings overlap the drive types");
_Static_assert(NB_COUNT == NB_LAYOUT_COUNT, "CMOS layout changed: bump NB_SIG_VALUE, then NB_LAYOUT_COUNT");
_Static_assert(NB_CMOS_HDD + 4 * 5 <= NB_CMOS_PASSWORD, "drive types overlap the password");

// Geometry of a fixed drive type (1-46 or User); 0 for Auto, None or an incomplete User entry.
static int
hdd_chs(const struct hdd *h, u16 *cyl, u8 *heads, u8 *spt)
{
    if (h->type == NB_HDD_USER) {
        *cyl = h->cyl;
        *heads = h->heads;
        *spt = h->spt;
    } else if (h->type > NB_HDD_NONE) {
        *cyl = HddTypes[h->type - 2].cyl;
        *heads = HddTypes[h->type - 2].heads;
        *spt = HddTypes[h->type - 2].spt;
    } else {
        return 0;
    }
    return *cyl && *heads && *spt;
}

// AT CMOS type number: 0 none or auto, 1-46, 47 user.
static u8
hdd_at_type(const struct hdd *h)
{
    return h->type > NB_HDD_NONE ? h->type - 1 : 0;
}

static void
hdd_load(void)
{
    int i;
    for (i = 0; i < 4; i++) {
        u8 at = NB_CMOS_HDD + 5 * i;
        Hdd[i].type = rtc_read(at) <= NB_HDD_USER ? rtc_read(at) : 0;
        Hdd[i].cyl = rtc_read(at + 1) | rtc_read(at + 2) << 8;
        Hdd[i].heads = rtc_read(at + 3);
        Hdd[i].spt = rtc_read(at + 4);
    }
}

int
setup_ide_type(int idx)
{
    return Hdd[idx].type;
}

// A fixed drive type sets the BIOS (INT 13h) geometry, as the original BIOS does.
void
setup_ide_geometry(int idx, struct chs_s *lchs)
{
    u16 cyl;
    u8 heads, spt;
    if (!hdd_chs(&Hdd[idx], &cyl, &heads, &spt))
        return;
    lchs->cylinder = cyl;
    lchs->head = heads;
    lchs->sector = spt;
}

static void
cmos_write(const u8 *val, const struct hdd *hdd, u16 password)
{
    int i;
    for (i = 0; i < NB_COUNT; i += 2)
        rtc_write(NB_CMOS_BASE + i / 2, val[i] | (i + 1 < NB_COUNT ? val[i + 1] << 4 : 0));
    for (i = 0; i < 4; i++) {
        u8 at = NB_CMOS_HDD + 5 * i;
        rtc_write(at, hdd[i].type);
        rtc_write(at + 1, hdd[i].cyl);
        rtc_write(at + 2, hdd[i].cyl >> 8);
        rtc_write(at + 3, hdd[i].heads);
        rtc_write(at + 4, hdd[i].spt);
    }
    rtc_write(NB_CMOS_PASSWORD, password);
    rtc_write(NB_CMOS_PASSWORD + 1, password >> 8);
    rtc_write(NB_CMOS_SIG, NB_SIG_VALUE);
    cmos_put_sum(NB_CMOS_BASE, NB_CMOS_SIG, NB_CMOS_SUM);

    int floppies = !!val[NB_FLOPPY_A] + !!val[NB_FLOPPY_B];
    u32 extkb = ext_mem_kb() > 0xffff ? 0xffff : ext_mem_kb();
    rtc_write(CMOS_FLOPPY_DRIVE_TYPE, val[NB_FLOPPY_A] << 4 | val[NB_FLOPPY_B]);
    u8 c = hdd_at_type(&hdd[0]), d = hdd_at_type(&hdd[1]);   // AT types of drives C and D
    rtc_write(CMOS_DISK_DATA, (c < 15 ? c : 15) << 4 | (d < 15 ? d : 15));
    rtc_write(0x19, c < 15 ? 0 : c);
    rtc_write(0x1a, d < 15 ? 0 : d);
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
    u8 present, atapi, pio_max, pio, multiple;
    struct drive_s *drive;
} IdeInfo[4];

void
setup_ide_report(int idx, struct drive_s *drive, u16 *identify, int atapi, u8 pio, u8 multiple)
{
    if (idx < 0 || idx > 3)
        return;
    IdeInfo[idx].present = 1;
    IdeInfo[idx].atapi = atapi;
    IdeInfo[idx].drive = drive;
    IdeInfo[idx].sectors = drive->sectors > 0xffffffff ? 0xffffffff : drive->sectors;
    IdeInfo[idx].cyl = identify[1];
    IdeInfo[idx].heads = identify[3];
    IdeInfo[idx].spt = identify[6];
    IdeInfo[idx].pio = pio;
    IdeInfo[idx].multiple = multiple;
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

static u32 MemtestBad;

// The original BIOS's POST errors above the setup prompt, then "Halt On" decides whether to wait for
// F1 (continue) or DEL (setup). Also the coreboot fallback warning (no console in the flash ROM).
enum { ERR_KEYBOARD = 1, ERR_DISKETTE = 2, ERR_DISK = 4, ERR_OTHER = 8 };

static int
post_error(int kind, const char *msg)
{
    printf("%s\n", msg);
    return kind;
}

void
setup_post_warnings(void)
{
    static const u8 halts[] = {             // per Halt On value
        ERR_KEYBOARD | ERR_DISKETTE | ERR_DISK | ERR_OTHER, 0, ERR_DISKETTE | ERR_DISK | ERR_OTHER,
        ERR_KEYBOARD | ERR_DISK | ERR_OTHER, ERR_DISKETTE | ERR_OTHER
    };
    char msg[40];
    int errors = 0, i;
    for (i = 0; i < 2; i++)
        if (Setting[NB_FLOPPY_A + i] && floppy_drive_check(i)) {
            errors |= post_error(ERR_DISKETTE, "Floppy disk(s) fail (80)");
            break;
        }
    for (i = 0; i < 4; i++)
        if (Hdd[i].type > NB_HDD_NONE && !IdeInfo[i].present
            && Setting[i < 2 ? NB_IDE_PRIMARY : NB_IDE_SECONDARY]) {
            errors |= post_error(ERR_DISK, "Hard disk(s) fail (80)");
            break;
        }
    if (!PS2KeyboardOK && !usb_kbd_active()) {
        char why[48], line[80];
        ps2_kbd_failure(why, sizeof(why));
        snprintf(line, sizeof(line), "Keyboard error or no keyboard present%s", why);
        errors |= post_error(ERR_KEYBOARD, line);
    }
    if (SetupDefaulted)
        errors |= post_error(ERR_OTHER, "CMOS checksum error - Defaults loaded");
    if (!(rtc_read(CMOS_STATUS_D) & 0x80))  // VRT: RAM and time valid
        errors |= post_error(ERR_OTHER, "CMOS battery failed");
    if (MemtestBad) {
        snprintf(msg, sizeof(msg), "Memory test fail at %x", MemtestBad);
        errors |= post_error(ERR_OTHER, msg);
    }
    if (ChipsetFallback)
        printf("Chipset setup values not used after %d unfinished POSTs.\n", ChipsetFallback);
    if (!(errors & halts[Setting[NB_HALT_ON]])) {
        if (errors) {
            printf("WARNING: POST errors above, boot continues (Halt On)\n");
            msleep(3000);
        }
        return;
    }
    printf("Press F1 to continue, DEL to enter SETUP\n");
    for (;;) {
        int key = get_keystroke(1000);
        if (key == 0x3b)
            return;
        if (key == 0x53) {
            SetupRequested = 1;
            return;
        }
    }
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
        hdd_load();
        int fails = rtc_read(NB_CMOS_BOOTFAIL) - 1;   // coreboot counted this POST already
        if (!Setting[NB_CHIPSET_AUTO] && fails >= NB_BOOTFAIL_MAX)
            ChipsetFallback = fails;                   // coreboot used the chipset defaults
    } else {
        dprintf(1, "CMOS settings invalid - defaults loaded\n");
        SetupDefaulted = 1;
        memcpy(Setting, Defaults, sizeof(Setting));
    }
    cmos_write(Setting, Hdd, Password);
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

// Settings being edited, then the RTC fields, the drive types and User heads/sectors (Cyl: cylinders).
enum { V_MONTH = NB_COUNT, V_DAY, V_YEAR, V_HOUR, V_MINUTE, V_SECOND,
       V_TYPE0, V_HEAD0 = V_TYPE0 + 4, V_SPT0 = V_HEAD0 + 4, V_COUNT = V_SPT0 + 4 };
static u8 Val[V_COUNT], OrigVal[V_COUNT];
static u16 Cyl[4], OrigCyl[4];
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
    int i;
    for (i = 0; i < 4; i++) {
        Val[V_TYPE0 + i] = Hdd[i].type;
        Val[V_HEAD0 + i] = Hdd[i].heads;
        Val[V_SPT0 + i] = Hdd[i].spt;
        Cyl[i] = Hdd[i].cyl;
    }
    memcpy(OrigVal, Val, sizeof(Val));
    memcpy(OrigCyl, Cyl, sizeof(Cyl));
}

static void
values_save(void)
{
    if (memcmp(&Val[V_MONTH], &OrigVal[V_MONTH], V_SECOND + 1 - V_MONTH) != 0) {
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
    struct hdd hdd[4];
    int i;
    for (i = 0; i < 4; i++) {
        hdd[i].type = Val[V_TYPE0 + i];
        hdd[i].heads = Val[V_HEAD0 + i];
        hdd[i].spt = Val[V_SPT0 + i];
        hdd[i].cyl = Cyl[i];
    }
    cmos_write(Val, hdd, NewPassword);
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
static const char * const HaltOn[] = {
    "All Errors", "No Errors", "All, But Keyboard", "All, But Diskette", "All, But Disk/Key"
};
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
#define F_TYPE   2   // drive type: Auto, None, 1-46, User
#define F_CYL    4   // User cylinders, Cyl[aux], 1-65535
#define F_USER   8   // User geometry of drive aux: shown and editable with that type only

struct item {
    u8 row, col, vcol, var, lo, hi, flags;
    const char *label;
    const char * const *names;   // value names, or NULL for numbers
    u8 aux;                      // drive of F_CYL/F_USER items
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

// Drive n: type, then User cylinders/heads/sectors under CYLS/HEAD/SECTOR, then translation mode.
#define HDD(n, label) \
    { 8 + n, 5, 24, V_TYPE0 + n, 0, NB_HDD_USER, F_TYPE, label, NULL, 0 }, \
    { 8 + n, 0, 44, 0, 1, 255, F_CYL | F_USER, NULL, NULL, n }, \
    { 8 + n, 0, 51, V_HEAD0 + n, 1, 16, F_USER, NULL, NULL, n }, \
    { 8 + n, 0, 58, V_SPT0 + n, 1, 63, F_USER, NULL, NULL, n }, \
    ENUM(8 + n, 0, 70, NB_IDE_MODE0 + n, NULL, IdeMode, 0)

static const struct item StdItems[] = {
    NUM(4, 5, 23, V_MONTH, 1, 12, "Date (mm:dd:yy) :"),
    NUM(4, 0, 26, V_DAY, 1, 31, NULL),
    NUM(4, 0, 29, V_YEAR, 80, 199, NULL),
    NUM(5, 5, 23, V_HOUR, 0, 23, "Time (hh:mm:ss) :"),
    NUM(5, 0, 26, V_MINUTE, 0, 59, NULL),
    NUM(5, 0, 29, V_SECOND, 0, 59, NULL),
    HDD(0, "Primary Master   :"),
    HDD(1, "Primary Slave    :"),
    HDD(2, "Secondary Master :"),
    HDD(3, "Secondary Slave  :"),
    ENUM(13, 5, 15, NB_FLOPPY_A, "Drive A :", Floppy, 0),
    ENUM(14, 5, 15, NB_FLOPPY_B, "Drive B :", Floppy, 0),
    ENUM(16, 5, 15, NB_HALT_ON, "Halt On :", HaltOn, 0),
};

// RTC fields out of range (garbage after a battery loss) start at their minimum.
static void
rtc_fields_check(void)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(StdItems); i++) {
        const struct item *it = &StdItems[i];
        if (it->var >= V_MONTH && it->var <= V_SECOND && (Val[it->var] < it->lo || Val[it->var] > it->hi))
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
    ENUM(18, 2, 31, NB_IDE_BLOCK, "IDE HDD Block Mode         :", OffOn, 0),
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
    ENUM(15, 2, 30, NB_ISAPNP, "PnP BIOS Auto-Config      :", OffOn, 0),
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
    if (v < it->lo || v > it->hi || ((it->flags & F_TYPE) && v == 16))   // type 15: reserved
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
    if (it->flags & F_USER)
        return Val[V_TYPE0 + it->aux] == NB_HDD_USER;
    return !(it->flags & F_MANUAL) || !Val[NB_CHIPSET_AUTO];
}

static void
item_draw(const struct item *it, int selected)
{
    char num[8] = { '0' + Val[it->var] / 10 % 10, '0' + Val[it->var] % 10, 0 };
    const char *s = num;
    u8 v = Val[it->var];
    int width = 0, i;
    if (it->flags & F_USER) {                       // right-aligned under CYLS, HEAD, SECTOR
        if (!item_editable(it))
            return;
        width = it->flags & F_CYL ? 5 : 3;
        snprintf(num, sizeof(num), "%d", it->flags & F_CYL ? Cyl[it->aux] : v);
        scr_fill(it->row, it->vcol, width, AttrText, ' ');
        scr_puts(it->row, it->vcol + width - strlen(num), selected ? AttrSelect : AttrValue, num);
        return;
    }
    if (it->flags & F_TYPE) {
        static const char * const fixed[] = { "Auto", "None" };
        snprintf(num, sizeof(num), "%d", v - 1);
        s = v <= NB_HDD_NONE ? fixed[v] : v == NB_HDD_USER ? "User" : num;
        width = 4;
    } else if (it->names) {
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
// Typed digits append to a User geometry number; a digit past its maximum starts a new one.
static void
item_digit(const struct item *it, int digit)
{
    u32 max = it->flags & F_CYL ? 0xffff : it->hi;
    u32 v = (it->flags & F_CYL ? Cyl[it->aux] : Val[it->var]) * 10 + digit;
    if (v > max)
        v = digit;
    if (it->flags & F_CYL)
        Cyl[it->aux] = v;
    else
        Val[it->var] = v;
}

static void
item_step(const struct item *it, int delta)
{
    if (it->flags & F_CYL) {
        int c = Cyl[it->aux] + delta;
        Cyl[it->aux] = c < 1 ? 0xffff : c > 0xffff ? 1 : c;
        return;
    }
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
    if (page) {
        scr_puts(ROW_HELP, 2, AttrText,
                 "ESC : Quit       \x18\x19\x1a\x1b : Select Item   PU/PD/+/- : Modify   F10 : Save & Exit");
        scr_puts(ROW_HELP + 1, 2, AttrText,
                 "F5  : Old Values          F6 : Load BIOS Defaults   F7 : Load Setup Defaults");
        return;
    }
    scr_puts(ROW_HELP, 2, AttrText,
             "ESC : Quit                \x18\x19\x1a\x1b : Select Item     PU/PD/+/- : Modify");
    scr_puts(ROW_HELP + 1, 2, AttrText, "F10 : Save & Exit Setup   F8 : Boot Menu");
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
    for (i = 0; i < 4; i++) {                       // size and geometry: of the type, else detected
        struct hdd h = { Val[V_TYPE0 + i], Val[V_HEAD0 + i], Val[V_SPT0 + i], Cyl[i] };
        u16 cyl = IdeInfo[i].cyl;
        u8 heads = IdeInfo[i].heads, spt = IdeInfo[i].spt;
        u32 mb = IdeInfo[i].sectors / 2048;
        if (hdd_chs(&h, &cyl, &heads, &spt)) {
            mb = (u32)cyl * heads * spt / 2048;
        } else if (h.type || !IdeInfo[i].present) {
            continue;
        } else if (IdeInfo[i].atapi) {
            scr_puts(8 + i, 35, AttrText, "CD-ROM");
            continue;
        }
        snprintf(line, sizeof(line), "%d%s", mb >= 10000 ? mb / 1024 : mb, mb >= 10000 ? "G" : "M");
        scr_puts(8 + i, 42 - strlen(line), AttrText, line);
        if (h.type == NB_HDD_USER)
            continue;                               // the User items show its geometry
        snprintf(line, sizeof(line), "%d", cyl);
        scr_puts(8 + i, 49 - strlen(line), AttrText, line);
        snprintf(line, sizeof(line), "%d", heads);
        scr_puts(8 + i, 54 - strlen(line), AttrText, line);
        snprintf(line, sizeof(line), "%d", spt);
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

// LOAD BIOS DEFAULTS: the original BIOS's fail-safe values where they differ from the Setup ones.
static const struct { u8 var, value; } FailSafe[] = {
    { NB_QUICK_POST, 0 }, { NB_SYSBIOS_CACHE, 0 }, { NB_VIDBIOS_CACHE, 0 }, { NB_SLOW_REFRESH, 0 },
    { NB_PCI_POSTED_WRITE, 0 }, { NB_COM2, 1 }, { NB_LPT_MODE, 2 },
};

// Defaults for everything but the Standard CMOS page, as the original BIOS does.
static void
values_default(int failsafe)
{
    u8 keep[NB_COUNT];
    int i;
    memcpy(keep, Val, NB_COUNT);
    memcpy(Val, Defaults, NB_COUNT);
    for (i = 0; i < ARRAY_SIZE(StdItems); i++)
        if (StdItems[i].var < NB_COUNT)
            Val[StdItems[i].var] = keep[StdItems[i].var];
    for (i = 0; failsafe && i < ARRAY_SIZE(FailSafe); i++)
        Val[FailSafe[i].var] = FailSafe[i].value;
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
        case 0x3f:                                               // F5
            memcpy(Val, OrigVal, sizeof(Val));
            memcpy(Cyl, OrigCyl, sizeof(Cyl));
            break;
        case 0x40:                                               // F6
            if (scr_confirm("Load BIOS Defaults (Y/N)? "))
                values_default(1);
            break;
        case 0x41:                                               // F7
            if (scr_confirm("Load SETUP Defaults (Y/N)? "))
                values_default(0);
            break;
        case 0x44: save_and_reboot(); break;                     // F10
        }
        if (ascii >= '0' && ascii <= '9' && (pg->items[sel].flags & F_USER))
            item_digit(&pg->items[sel], ascii - '0');
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

// CPUID vendor, family/model/stepping and feature flags; 0 for a CPU without CPUID.
static int
cpu_ident(char vendor[13], u32 *fms, u32 *features)
{
    u32 max, ebx, ecx, edx;
    *fms = *features = 0;
    vendor[0] = 0;
    if (!cpu_has_cpuid())
        return 0;
    cpuid(0, &max, &ebx, &ecx, &edx);
    memcpy(vendor, &ebx, 4);
    memcpy(vendor + 4, &edx, 4);
    memcpy(vendor + 8, &ecx, 4);
    vendor[12] = 0;
    if (max >= 1)
        cpuid(1, fms, &ebx, &ecx, features);
    return 1;
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

// Core clock of 486-class CPUs: dec + taken jnz is 4 clocks, timed with PIT channel 2; 16-byte aligned
// (unaligned it took 4-5 clocks: 107-121 MHz shown on the 133 MHz board).
static u32
cpu_mhz(void)
{
    u32 n = 200000, flags = save_flags();
    irq_disable();
    outb((inb(0x61) & ~0x02) | 0x01, 0x61);
    outb(0xb0, 0x43);
    outb(0xff, 0x42);
    outb(0xff, 0x42);
    asm volatile(".align 16\n1: decl %0; jnz 1b" : "+r"(n));
    outb(0x80, 0x43);
    u32 left = inb(0x42);
    left |= inb(0x42) << 8;
    restore_flags(flags);
    u32 ticks = 0xffff - left;
    return ticks ? (954400000 / ticks + 500) / 1000 : 0;   // 800000 clocks * 1.193 MHz
}

// What setup_l2_cache found, for the CPU page (the reason when the L2 stays off).
enum { L2_NOT_RUN, L2_NONE, L2_SETUP_OFF, L2_RAM_SMALL, L2_HUNG, L2_TAG0, L2_TAGMIX, L2_TAGMAX, L2_NOCACHE, L2_OK };
static const char * const L2Names[] = {
    "not checked", "none found", "off in setup", "RAM too small to init", "hang in the previous L2 init",
    "check failed: tag 0", "check failed: mixed tag", "check failed: tag all ones", "check failed: no caching",
    "checked"
};
static int L2Status;

// Steps of the L2 init, kept in CMOS 3Eh bits 6-4 while they run: after a hang or a reset the next POST
// shows the step (CPU page, CACHE TEST). In the CACHE TEST log each step also shows for 1 s.
enum { L2S_NONE, L2S_CACHES_OFF, L2S_FILL_WT, L2S_CHECKS, L2S_FILL_WB, L2S_RESTORE, L2S_SPEED, L2S_DISABLE };
static const char * const L2StepNames[] = {
    "", "L1 off and WBINVD", "write-through fill", "tag checks", "write-back fill", "caches back on",
    "speed test", "L2 off (evict pass)"
};
static u8 L2HungStep;   // the step the previous POST's L2 init stopped at

static void
info_cpu(void)
{
    char line[80], vendor[13];
    u32 fms, edx;
    int mhz_valid = 0, row = 4;
    const char *name = "486 (no CPUID)";
    if (cpu_ident(vendor, &fms, &edx)) {
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
    u8 l2 = pci_config_readb(HB_BDF, 0x50), found = rtc_read(NB_CMOS_L2);
    if (L2Status == L2_OK || L2Status == L2_NOT_RUN)
        snprintf(line, sizeof(line), "L2 cache       : %d KB, %s, %s, %d bank(s)",
                 (l2 & 7) ? 32 << (l2 & 7) : 0, (l2 & 0x40) ? "write-back" : "write-through",
                 (l2 & 0x80) ? "enabled" : "disabled", (l2 & 8) ? 2 : 1);
    else if (L2Status == L2_NONE)
        snprintf(line, sizeof(line), "L2 cache       : none");
    else if (L2Status == L2_HUNG && L2HungStep)
        snprintf(line, sizeof(line), "L2 cache       : %d KB, OFF: last init stopped at %s",
                 (found & 7) ? 32 << (found & 7) : 0, L2StepNames[L2HungStep]);
    else    // found but left off: why
        snprintf(line, sizeof(line), "L2 cache       : %d KB, %d bank(s), OFF: %s",
                 (found & 7) ? 32 << (found & 7) : 0, (found & 8) ? 2 : 1, L2Names[L2Status]);
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

static const char *PnpIrq[16];               // IRQs given to ISA PnP cards
static char PnpCard[4][48];                  // what the POST configured, per card

// Who uses each IRQ ("" if nobody known).
static void
irq_owners(const char *owner[16])
{
    static const char * const fixed[16] = {
        "System timer", "Keyboard", "Cascade (IRQ 9)", "", "", "", "", "", "Real-time clock",
        "", "", "", "", "Math coprocessor", "", ""
    };
    int i;
    memcpy(owner, fixed, sizeof(fixed));
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
    for (i = 0; i < 4; i++) {                  // PIRQ links, even without a device (level mode)
        u8 irq = pci_config_readb(IBC_BDF, 0x43 + i / 2) >> (i & 1 ? 0 : 4) & 0x0f;
        if (irq && !owner[irq][0])
            owner[irq] = "PCI interrupt";
    }
    for (i = 0; i < 16; i++)
        if (PnpIrq[i])
            owner[i] = PnpIrq[i];
}

// USB tree: each level one more space of indentation.
static int
info_usb_level(int parent, int depth, int row)
{
    static const char * const speed[] = { "full", "low", "high", "super" };
    static const char * const type[] = { "", "UHCI", "OHCI", "EHCI", "xHCI" };
    char line[80], what[24];
    int i, ind = depth < 20 ? depth : 20;
    memset(line, ' ', ind);                 // one space per level (SeaBIOS snprintf has no '*' width)
    for (i = 0; i < UsbListedCount && row <= 19; i++) {
        struct usb_listed *l = &UsbListed[i];
        if (l->parent != parent)
            continue;
        if (parent < 0)
            snprintf(line + ind, sizeof(line) - ind, "%s %02x:%02x.%x, %d ports", type[l->type & 7],
                     pci_bdf_to_bus(l->bdf), pci_bdf_to_dev(l->bdf), pci_bdf_to_fn(l->bdf), l->port);
        else {
            if (l->cls == USB_CLASS_HUB)
                snprintf(what, sizeof(what), "hub");
            else if (l->cls == USB_CLASS_MASS_STORAGE)
                snprintf(what, sizeof(what), "storage");
            else if (l->cls == USB_CLASS_HID)
                snprintf(what, sizeof(what), l->proto == 1 ? "keyboard" : l->proto == 2 ? "mouse" : "HID");
            else
                snprintf(what, sizeof(what), "class %02x", l->cls);
            snprintf(line + ind, sizeof(line) - ind, "port %d: %04x:%04x %s, %s speed%s", l->port,
                     l->vid, l->pid, what, speed[l->speed & 3], l->ok ? "" : ", no driver");
        }
        info_line(row++, line);
        row = info_usb_level(i, depth + 1, row);
    }
    return row;
}

static void
info_usb(void)
{
    scr_puts(4, 3, AttrTitle, "Controllers, ports and devices");
    if (info_usb_level(-1, 0, 5) == 5)
        info_line(5, "none");
}

static void
info_isa(void)
{
    const char *owner[16];
    char line[80];
    int i, row = 4;
    irq_owners(owner);
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
    for (i = 0; i < ARRAY_SIZE(PnpCard) && PnpCard[i][0]; i++)
        info_line(row++, PnpCard[i]);
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
    static const char * const trans[] = { "NORMAL", "LBA", "LARGE", "RECHS", "TYPE" };
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
                     "    %dMB, CHS %d/%d/%d, %s, PIO mode %d (drive max %d), block %d",
                     IdeInfo[i].sectors / 2048, IdeInfo[i].cyl, IdeInfo[i].heads, IdeInfo[i].spt,
                     trans[get_translation(IdeInfo[i].drive) % 5], IdeInfo[i].pio,
                     IdeInfo[i].pio_max, IdeInfo[i].multiple ?: 1);
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

static void info_bench_memory(void);
static void info_bench_cpu(void);
static void info_bench_disk(void);
static void info_bench_disk_bars(void);
static void mt_run(void);

/****************************************************************
 * ISA Plug and Play cards: isolation, resources and activation at POST, like the original BIOS
 ****************************************************************/

#define PNP_ADDRESS 0x279
#define PNP_WRITE   0xa79
#define PNP_READ    0x20b                       // read data port given to the cards
#define PNP_ITEMS   24

static void
pnp_write(u8 reg, u8 val)
{
    outb(reg, PNP_ADDRESS);
    outb(val, PNP_WRITE);
}

static u8
pnp_read(u8 reg)
{
    outb(reg, PNP_ADDRESS);
    return inb(PNP_READ);
}

// Initiation key: the cards leave "Wait for Key".
static void
pnp_key(void)
{
    u8 v = 0x6a;
    int i;
    outb(0, PNP_ADDRESS);
    outb(0, PNP_ADDRESS);
    for (i = 0; i < 32; i++) {
        outb(v, PNP_ADDRESS);
        v = v >> 1 | ((v ^ v >> 1) & 1) << 7;
    }
}

// Serial isolation: the 72-bit identifier of the card that wins; 1 if its checksum is right.
static int
pnp_isolate(u8 *id)
{
    u8 sum = 0x6a;
    int i, any = 0;
    memset(id, 0, 9);
    outb(0x01, PNP_ADDRESS);
    msleep(1);
    for (i = 0; i < 72; i++) {
        u8 a = inb(PNP_READ), b = inb(PNP_READ);
        int bit = a == 0x55 && b == 0xaa;
        any |= bit;
        if (i < 64)
            sum = (((sum ^ sum >> 1) & 1) ^ bit) << 7 | sum >> 1;
        id[i / 8] |= bit << (i % 8);
        usleep(250);
    }
    return any && sum == id[8];
}

static int PnpLeft;                            // resource bytes the current card may still send
static int IsaCsns;                            // cards isolated and configured

static u8
pnp_data(void)
{
    int i;
    for (i = 0; i < 1000 && !(pnp_read(0x05) & 1); i++)
        udelay(10);
    PnpLeft = i == 1000 ? 0 : PnpLeft - 1;
    return pnp_read(0x04);
}

// A resource descriptor of a logical device; alt: 0 always, n: dependent function n.
struct pnp_item {
    u8 kind, alt, align, len;                   // kind: 'I' I/O range, 'F' fixed I/O, 'Q' IRQ, 'D' DMA
    u16 min, max;                               // I/O range, or the IRQ / DMA mask in min
};

// Used ISA I/O: known users, then the bus must float (0xff) over the range.
static int
pnp_io_free(u16 base, u8 len, const u16 (*used)[2], int count)
{
    int i;
    if (base < 0x100 || base + len > 0x400)
        return 0;
    for (i = 0; i < count; i++)
        if (base < used[i][0] + used[i][1] && used[i][0] < base + len)
            return 0;
    for (i = 0; i < len; i++)
        if (inb(base + i) != 0xff)
            return 0;
    return 1;
}

// Gives one dependent function of a logical device free resources; 0 if they are not all free.
static int
pnp_assign(struct pnp_item *items, int count, int alt, u16 *irqs, u8 *dmas, u16 (*used)[2], int *nused,
           u16 *io, u8 *iolen, u8 *irq, u8 *dma)
{
    int i, nio = 0, nirq = 0, ndma = 0;
    for (i = 0; i < count; i++) {
        struct pnp_item *it = &items[i];
        int v;
        if (it->alt && it->alt != alt)
            continue;
        if ((it->kind == 'Q' || it->kind == 'D') && !it->min)
            continue;                           // empty mask: none needed
        if (it->kind == 'Q' || it->kind == 'D') {
            u16 mask = it->min & ~(it->kind == 'Q' ? *irqs : *dmas);
            for (v = 0; v < 16 && !(mask >> v & 1); v++)
                ;
            if (v == 16 || (it->kind == 'Q' ? nirq : ndma) == 2)
                return 0;
            if (it->kind == 'Q')
                *irqs |= 1 << v, irq[nirq++] = v;
            else
                *dmas |= 1 << v, dma[ndma++] = v;
            continue;
        }
        if (nio == 8 || *nused == PNP_ITEMS)
            return 0;
        for (v = it->min; v <= it->max && !pnp_io_free(v, it->len, used, *nused); v += it->align ?: 1)
            ;
        if (v > it->max)
            return 0;
        used[*nused][0] = v;
        used[(*nused)++][1] = it->len;
        iolen[nio] = it->len;
        io[nio++] = v;
    }
    return 1;
}

// Reads the resources of the selected card and configures its logical devices.
static void
pnp_card(int csn, u8 *id, u16 *irqs, u8 *dmas, u16 (*used)[2], int *nused)
{
    struct pnp_item items[PNP_ITEMS];
    char name[21] = "";
    int count = 0, alt = 0, ldn = -1, nalt = 0, tags, n, i, mem = 0, done = 0;
    char *line = PnpCard[csn - 1];
    snprintf(line, sizeof(PnpCard[0]), "ISA PnP %c%c%c%02x%02x:", '@' + (id[0] >> 2 & 0x1f),
             '@' + ((id[0] & 3) << 3 | id[1] >> 5), '@' + (id[1] & 0x1f), id[2], id[3]);
    PnpLeft = 4096;
    for (i = 0; i < 9; i++)
        pnp_data();                             // the serial identifier again
    for (tags = 0; tags < 200 && !done && PnpLeft > 0; tags++) {
        u8 tag = pnp_data();
        u8 d[24] = { 0 };                       // short items: missing bytes read as 0
        n = tag & 7;
        if (tag & 0x80) {                       // large item: 16-bit length
            n = pnp_data();
            n |= pnp_data() << 8;
        }
        for (i = 0; i < n && PnpLeft > 0; i++) {
            u8 b = pnp_data();
            if (i < sizeof(d))
                d[i] = b;
        }
        if (tag == 0x82 && !name[0])            // ANSI name of the card
            memcpy(name, d, n < sizeof(name) ? n : sizeof(name) - 1), name[n < sizeof(name) ? n : sizeof(name) - 1] = 0;
        done = tag == 0x79 || tag == 0x78;      // end tag
        if (done || (tag & 0xf8) == 0x10) {     // end of the current logical device
            if (ldn >= 0 && !mem) {
                u16 io[8] = { 0 };
                u8 iolen[8] = { 0 }, irq[2] = { 0, 0 }, dma[2] = { 4, 4 };
                u16 saved_irqs = *irqs, saved_used = *nused;
                u8 saved_dmas = *dmas;
                for (alt = nalt ? 1 : 0; alt <= nalt; alt++) {
                    if (pnp_assign(items, count, alt, irqs, dmas, used, nused, io, iolen, irq, dma))
                        break;
                    *irqs = saved_irqs, *dmas = saved_dmas, *nused = saved_used;
                }
                pnp_write(0x07, ldn);
                if (alt <= nalt) {
                    for (i = 0; i < 8; i++)
                        pnp_write(0x60 + 2 * i, io[i] >> 8), pnp_write(0x61 + 2 * i, io[i]);
                    for (i = 0; i < 2; i++) {
                        pnp_write(0x70 + 2 * i, irq[i]);
                        pnp_write(0x71 + 2 * i, 0x02);  // edge, high
                        pnp_write(0x74 + i, dma[i]);
                        if (irq[i])
                            PnpIrq[irq[i]] = "ISA PnP card";
                    }
                    pnp_write(0x30, 1);
                    pnp_isa_ldev(csn, id, io, iolen, irq, dma);
                    n = strlen(line);
                    if (io[0] && n < 40)
                        snprintf(line + n, sizeof(PnpCard[0]) - n, " %x", io[0]);
                    n = strlen(line);
                    if (irq[0] && n < 40)
                        snprintf(line + n, sizeof(PnpCard[0]) - n, "/%d", irq[0]);
                }
            }
            ldn++;
            count = nalt = mem = 0;
            alt = 0;
            continue;
        }
        if ((tag & 0xf8) == 0x30)               // start / end of a dependent function
            alt = ++nalt;
        else if ((tag & 0xf8) == 0x38)
            alt = 0;
        else if (tag & 0x80)
            mem |= tag == 0x81 || tag == 0x85 || tag == 0x86;
        else if (count < PNP_ITEMS) {
            struct pnp_item *it = &items[count];
            memset(it, 0, sizeof(*it));
            it->alt = alt;
            if ((tag & 0xf8) == 0x20)
                it->kind = 'Q', it->min = d[0] | d[1] << 8;
            else if ((tag & 0xf8) == 0x28)
                it->kind = 'D', it->min = d[0];
            else if ((tag & 0xf8) == 0x40)
                it->kind = 'I', it->min = d[1] | d[2] << 8, it->max = d[3] | d[4] << 8,
                    it->align = d[5], it->len = d[6];
            else if ((tag & 0xf8) == 0x48)
                it->kind = 'F', it->min = it->max = (d[0] | d[1] << 8) & 0x3ff, it->len = d[2];
            if (it->kind)
                count++;
        }
    }
    n = strlen(line);
    snprintf(line + n, sizeof(PnpCard[0]) - n, " %s", name);
    printf("Card-%d%d: %s\n", csn / 10, csn % 10, line + 8);
}

// POST: isolates the ISA PnP cards and gives their logical devices free I/O, IRQ and DMA.
void
setup_isapnp(void)
{
    static const u16 board[][2] = {             // I/O of the board besides 000-0FF
        { 0x170, 8 }, { 0x1f0, 8 }, { 0x376, 1 }, { 0x3f6, 1 }, { 0x3f0, 8 }, { 0x3b0, 0x30 },
        { PNP_ADDRESS, 1 }, { PNP_READ, 1 },
    };
    const char *owner[16];
    u16 used[PNP_ITEMS][2], irqs = 0;
    u8 id[9], dmas = 0x14;                      // DMA 2: floppy, 4: cascade
    int csn, nused = 0, i;
    if (!Setting[NB_ISAPNP]) {                  // left to the OS: power-up state, as after a cold boot
        pnp_key();
        pnp_write(0x02, 0x07);                  // reset, wait for key, reset the card select numbers
        return;
    }
    for (i = 0; i < ARRAY_SIZE(board); i++, nused++)
        used[nused][0] = board[i][0], used[nused][1] = board[i][1];
    for (i = 0; i < 4; i++)
        if (GET_BDA(port_com[i]))
            used[nused][0] = GET_BDA(port_com[i]), used[nused++][1] = 8;
    for (i = 0; i < 3; i++)
        if (GET_BDA(port_lpt[i]))
            used[nused][0] = GET_BDA(port_lpt[i]), used[nused++][1] = 8;
    irq_owners(owner);
    for (i = 0; i < 16; i++)
        if (owner[i][0])
            irqs |= 1 << i;
    if (Setting[NB_LPT] && Setting[NB_LPT_MODE] == 2)
        dmas |= 0x08;                           // ECP parallel port: DMA 3
    pnp_key();
    pnp_write(0x02, 0x04);                      // reset the card select numbers
    msleep(2);
    pnp_key();
    pnp_write(0x03, 0);                         // Wake[0]: isolation
    pnp_write(0x00, PNP_READ >> 2);
    for (csn = 1; csn <= ARRAY_SIZE(PnpCard) && pnp_isolate(id); csn++) {
        pnp_write(0x06, csn);
        pnp_write(0x03, csn);                   // Wake[csn]: configuration
        if (csn == 1)
            printf("Initializing Plug and Play Cards...\n");
        pnp_card(csn, id, &irqs, &dmas, used, &nused);
        pnp_write(0x03, 0);
    }
    IsaCsns = csn - 1;
    pnp_write(0x02, 0x02);                      // all cards back to Wait for Key
}

// POST, at the end: the PnP BIOS system device nodes and the ESCD of what the POST configured.
void
setup_pnp(void)
{
    pnp_finish(IsaCsns, PNP_READ, Setting[NB_FDC] | Setting[NB_IDE_PRIMARY] << 1 | Setting[NB_IDE_SECONDARY] << 2
               | (Setting[NB_LPT] ? Setting[NB_LPT_MODE] : 0) << 3);
}

static const struct {
    const char *title;
    void (*draw)(void);
    int own;                                    // graphics benchmark: draws itself, until ESC
} InfoPages[] = {
    { "CPU, CACHE AND MEMORY", info_cpu },
    { "PCI DEVICES", info_pci },
    { "USB DEVICES", info_usb },
    { "ISA RESOURCES AND IRQ MAP", info_isa },
    { "IDE DRIVES", info_ide },
    { "CHIPSET REGISTERS", info_chipset },
    { "MEMORY SPEED", info_bench_memory, 1 },
    { "CPU BENCHMARK", info_bench_cpu, 1 },
    { "HARD DISK BENCHMARK", info_bench_disk, 1 },
    { "HARD DISK SPEED", info_bench_disk_bars, 1 },
};

// One page until ESC.
static void
info_page(int p)
{
    if (InfoPages[p].own) {
        InfoPages[p].draw();
        return;
    }
    scr_header(InfoPages[p].title);
    InfoPages[p].draw();
    scr_puts(ROW_HELP, 2, AttrText, "ESC : Back");
    while (get_keystroke(1000) != 0x01)
        ;
}

// HARDWARE INFORMATION: a menu of the pages.
static void
info_menu(void)
{
    int sel = 0, n = ARRAY_SIZE(InfoPages);
    for (;;) {
        scr_header("HARDWARE INFORMATION");
        for (int i = 0; i < n; i++)
            scr_puts(4 + i, 26, i == sel ? AttrSelect : AttrText, InfoPages[i].title);
        scr_puts(ROW_HELP, 2, AttrText, "ESC : Back      \x18\x19 : Select Page      Enter : Show");
        switch (get_keystroke(1000)) {
        case 0x01: return;
        case 0x48: sel = (sel + n - 1) % n; break;
        case 0x50: sel = (sel + 1) % n; break;
        case 0x1c: info_page(sel); break;
        }
    }
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

enum { M_PAGE, M_INFO, M_PASSWORD, M_IDE, M_MEMTEST, M_CACHETEST, M_BIOS_DEFAULTS, M_DEFAULTS, M_SAVE, M_QUIT };

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
    { M_INFO, 0, "HARDWARE INFORMATION", "CPU, Memory, PCI, ISA/IRQ, IDE, Chipset, Memory Speed Chart" },
    { M_PASSWORD, 0, "SUPERVISOR PASSWORD", "Change, Set, or Disable Password" },
    { M_IDE, 0, "IDE HDD AUTO DETECTION", "Drives found at this boot" },
    { M_MEMTEST, 0, "MEMORY TEST", "memtest86+ 4.20 tests on the free RAM" },
    { M_CACHETEST, 0, "CACHE TEST", "L2 cache: each write policy, timing and tag width" },
    { M_BIOS_DEFAULTS, 0, "LOAD BIOS DEFAULTS", "Load BIOS Defaults except Standard CMOS SETUP" },
    { M_DEFAULTS, 0, "LOAD SETUP DEFAULTS", "Load SETUP Defaults except Standard CMOS SETUP" },
    { M_SAVE, 0, "SAVE & EXIT SETUP", "Save Data to CMOS & Exit SETUP" },
    { M_QUIT, 0, "EXIT WITHOUT SAVING", "Abandon all Data & Exit SETUP" },
};

#define MENU_ROWS 8

static void l2_test_run(void);

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
        case 0x42:                                               // F8: leave without saving, boot menu
            SetupBootMenu = 1;
            scr_close();
            return;
        case 0x01:
            if (scr_confirm("Quit Without Saving (Y/N)? ")) {
                scr_close();
                return;
            }
            break;
        case 0x1c:
            switch (Menu[sel].kind) {
            case M_PAGE: page_run(&Pages[Menu[sel].arg]); break;
            case M_INFO: info_menu(); break;
            case M_IDE: info_page(4); break;
            case M_MEMTEST: mt_run(); break;
            case M_CACHETEST: l2_test_run(); break;
            case M_PASSWORD: password_set(); break;
            case M_BIOS_DEFAULTS:
                if (scr_confirm("Load BIOS Defaults (Y/N)? "))
                    values_default(1);
                break;
            case M_DEFAULTS:
                if (scr_confirm("Load SETUP Defaults (Y/N)? "))
                    values_default(0);
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

int SetupRequested, SetupBootMenu;

#define ROW_CPU    4
#define ROW_MEMORY 5
#define ROW_CACHE  6
#define ROW_IDE    8                        // 4 drive detection lines
#define ROW_TTY    13

static const char *
cpu_short_name(int *mhz)
{
    char vendor[13];
    u32 fms, features;
    int valid = 0;
    const char *name = "486";
    if (cpu_ident(vendor, &fms, &features)) {
        name = cpu_name(vendor, fms, &valid);
    } else if (cpu_cyrix_dir0() >= 0) {
        name = "Cyrix 486/5x86";
    } else {                                // pre-CPUID 486: family/model from DX at reset
        u8 sig = rtc_read(NB_CMOS_CPUSIG);
        if (sig >> 4 == 4)
            name = cpu_name("GenuineIntel", sig << 4, &valid);
        valid = 1;
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
    scr_puts(0, 0, AttrText, "Biostar MB-8433UUD-A");
    scr_puts(1, 0, AttrText, RELEASE_DATE);
    scr_puts(24, 0, AttrText, VERSION);
    if (mhz)
        snprintf(line, sizeof(line), "%s CPU at %dMHz", name, mhz);
    else
        snprintf(line, sizeof(line), "%s CPU", name);
    scr_puts(ROW_CPU, 0, AttrText, line);
    setup_l2_post_line();
    cursor_row(ROW_TTY);                    // teletype output continues below
}

// The POST screen's L2 line: what the L2 init did (it runs after the banner), or why the L2 is off.
void
setup_l2_post_line(void)
{
    char line[80];
    u8 found = rtc_read(NB_CMOS_L2) & 0x0f, hb50 = pci_config_readb(HB_BDF, 0x50);
    u32 kb = (found & 7) && (found & 7) <= 5 ? 32 << (found & 7) : 0;
    if (L2Status == L2_OK)
        snprintf(line, sizeof(line), "External Cache : %dK, %s", kb, hb50 & 0x40 ? "write-back" : "write-through");
    else if (L2Status == L2_NOT_RUN)
        snprintf(line, sizeof(line), kb ? "External Cache : %dK found" : "External Cache : none", kb);
    else if (L2Status == L2_HUNG && L2HungStep)
        snprintf(line, sizeof(line), "External Cache : %dK OFF, last init stopped at %s", kb, L2StepNames[L2HungStep]);
    else
        snprintf(line, sizeof(line), kb ? "External Cache : %dK OFF, %s" : "External Cache : none", kb,
                 L2Names[L2Status]);
    scr_fill(ROW_CACHE, 0, 80, AttrText, ' ');
    scr_puts(ROW_CACHE, 0, AttrText, line);
}

// Award-style "Detecting HDD" line of an onboard IDE position at POST: shown when its detection
// starts, completed with the drive model or None when it ends.
void
setup_ide_detect(int idx, int done)
{
    static const char * const unit[] = { "Primary Master  ", "Primary Slave   ",
                                         "Secondary Master", "Secondary Slave " };
    char line[80];
    if (idx < 0 || !Vram)
        return;
    snprintf(line, sizeof(line), "Detecting HDD %s ... %s", unit[idx],
             !done ? "" : IdeInfo[idx].present ? IdeInfo[idx].model : "None");
    scr_puts(ROW_IDE + idx, 0, AttrText, line);
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

// Test the dwords 'step' apart in one 4 KiB block without losing them (POST data lives in this RAM):
// two patterns, each written, flushed from the internal cache and read back. This runs before the L2 is
// enabled (setup_l2_cache), as by the original BIOS: on this chipset a flush also empties the L2.
static int
memtest_block(u32 *block, u32 *save, int step)
{
    int i, bad = 0, n = 1024;
    for (i = 0; i < n; i += step)
        save[i] = block[i];
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
    for (i = 0; i < n; i += step)
        block[i] = save[i];
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

// External cache enabled like the original BIOS at POST 52h: coreboot only sized it and left it off.
// The tag RAM has no valid bits, so with L1 and the BIOS segment out of the caches, reading every
// dword of 8 L2 sizes of DRAM fills every line; this runs from the F segment and uses no memory
// while enabling L2 and filling.
void VISIBLE32FLAT
l2_fill(u32 base, u32 dwords, u8 hb50)
{
    asm volatile("movl $0x80008050, %%eax\n"
                 "movw $0xcf8, %%dx\n"
                 "outl %%eax, %%dx\n"
                 "movw $0xcfc, %%dx\n"
                 "movb %%bl, %%al\n"
                 "outb %%al, %%dx\n"
                 "cld\n"
                 "rep lodsl\n"
                 : "+S"(base), "+c"(dwords) : "b"(hb50) : "eax", "edx", "memory");
}

// The L2 turned fully off (50h = hb50 & 30h) after reading DRAM over the same range as l2_fill: in write-back
// every dirty line is evicted to DRAM first, with no memory write in between that could be left in the L2.
void VISIBLE32FLAT
l2_fill_off(u32 base, u32 dwords, u8 hb50)
{
    asm volatile("cld\n"
                 "rep lodsl\n"
                 "movl $0x80008050, %%eax\n"
                 "movw $0xcf8, %%dx\n"
                 "outl %%eax, %%dx\n"
                 "movw $0xcfc, %%dx\n"
                 "movb %%bl, %%al\n"
                 "andb $0x30, %%al\n"
                 "outb %%al, %%dx\n"
                 : "+S"(base), "+c"(dwords) : "b"(hb50) : "eax", "edx", "memory");
}

// The checks toggle 50h bit 7 with no stack or memory access in between other than the tested lines:
// the L2 does not follow writes made while it is off, so a stack slot written then would be read back
// stale once it is on again (a wrong return address). Port CF8h/CFCh are written directly.
#define L2_HB50(on) "movl $0x80008050, %%eax\n movw $0xcf8, %%dx\n outl %%eax, %%dx\n movw $0xcfc, %%dx\n" \
                    "movb %%bl, %%al\n" on "outb %%al, %%dx\n"

u32 L2Saved VARFSEG;   // the tested dword: in the F segment, which is not cached during the checks
u32 L2Got[2] VARFSEG;  // what the checks read with the L2 on, for the CACHE TEST log

// With L2 off a DRAM dword is written, with it on it must read back twice: a stale tag hits falsely, a
// line not filled from DRAM fails the second read; the dword is restored and its line evicted.
u32 VISIBLE32FLAT
l2_line_works(u32 addr, u32 size, u8 hb50)
{
    u32 works;
    asm volatile(L2_HB50("andb $0x7f, %%al\n")
                 "movl (%%esi), %%eax\n movl %%eax, %4\n"
                 "movl %%esi, %%edi\n xorl $0x5aa5c33c, %%edi\n movl %%edi, (%%esi)\n"
                 L2_HB50("")
                 "movl %%esi, %%eax\n xorl %%ecx, %%eax\n movl (%%eax), %%eax\n"
                 "movl (%%esi), %%eax\n movl %%eax, %5\n cmpl %%eax, %%edi\n jne 1f\n"
                 "movl (%%esi), %%eax\n movl %%eax, %6\n cmpl %%eax, %%edi\n jne 1f\n movl $1, %%edi\n jmp 2f\n"
                 "1: xorl %%edi, %%edi\n 2:\n"
                 L2_HB50("andb $0x7f, %%al\n")
                 "movl %4, %%eax\n movl %%eax, (%%esi)\n"
                 L2_HB50("")
                 "movl %%esi, %%eax\n xorl %%ecx, %%eax\n movl (%%eax), %%eax\n"
                 : "=D"(works), "+S"(addr), "+c"(size), "+b"(hb50), "=m"(L2Saved), "=m"(L2Got[0]), "=m"(L2Got[1])
                 : : "eax", "edx", "memory");
    return works;
}

// With L2 on a line is read, its DRAM changed with L2 off: reading it again must give the cached value,
// else the tags are not stored (fake or missing tag RAM). DRAM is restored and the line evicted.
u32 VISIBLE32FLAT
l2_caches(u32 addr, u32 size, u8 hb50)
{
    u32 cached;
    asm volatile("movl (%%esi), %%eax\n movl %%eax, %4\n movl (%%esi), %%eax\n"
                 L2_HB50("andb $0x7f, %%al\n")
                 "movl %4, %%edi\n notl %%edi\n movl %%edi, (%%esi)\n"
                 L2_HB50("")
                 "movl (%%esi), %%edx\n movl %%edx, %5\n"
                 "movl %4, %%eax\n xorl %%edi, %%edi\n cmpl %%edx, %%eax\n jne 1f\n incl %%edi\n 1:\n"
                 L2_HB50("andb $0x7f, %%al\n")
                 "movl %4, %%eax\n movl %%eax, (%%esi)\n"
                 L2_HB50("")
                 "movl %%esi, %%eax\n xorl %%ecx, %%eax\n movl (%%eax), %%eax\n"
                 : "=D"(cached), "+S"(addr), "+c"(size), "+b"(hb50), "+m"(L2Saved), "=m"(L2Got[0])
                 : : "eax", "edx", "memory");
    return cached;
}


static u32
l2_size(u8 hb50)
{
    return (hb50 & 7) && (hb50 & 7) <= 5 ? 0x8000 << (hb50 & 7) : 0;
}

// Scratch DRAM that l2_fill reads for an L2 of 'size': [base, end).
static u32
l2_scratch(u32 size, u32 *end)
{
    u32 base = size >= 256*1024 ? 1024*1024 : 0;
    *end = base + 8*size < LegacyRamSize ? base + 8*size : LegacyRamSize;
    return base;
}

static int L2Log;       // CACHE TEST step log on: its next row

#define L2_LOG_TOP 3
#define L2_LOG_END (ROW_HELP - 1)

static void
l2_log(const char *text)
{
    if (!L2Log)
        return;
    if (L2Log > L2_LOG_END) {                   // full: start again at the top
        for (int r = L2_LOG_TOP; r <= L2_LOG_END; r++)
            scr_fill(r, 0, 80, AttrText, ' ');
        L2Log = L2_LOG_TOP;
    }
    info_line(L2Log++, text);
    msleep(1000);
}

static void
l2_step(int step, const char *text)
{
    rtc_write(NB_CMOS_L2, (rtc_read(NB_CMOS_L2) & 0x8f) | step << 4);
    if (L2Log) {
        char line[80];
        snprintf(line, sizeof(line), "Step %d %s: %s", step, L2StepNames[step], text);
        l2_log(line);
    }
}

// Host bridge 50h-5Fh, CR0, CMOS 3Eh in the log.
static void
l2_log_regs(const char *when)
{
    char line[80];
    int n = snprintf(line, sizeof(line), "%s 50h:", when);
    for (int r = 0x50; r < 0x60; r++)
        n += snprintf(line + n, sizeof(line) - n, " %02x", pci_config_readb(HB_BDF, r));
    snprintf(line + n, sizeof(line) - n, " CR0 %x 3Eh %02x", cr0_read(), rtc_read(NB_CMOS_L2));
    l2_log(line);
}

// One fill pass (l2_fill), in L2-size blocks with the progress logged.
static void
l2_fill_logged(u32 base, u32 end, u32 size, u8 hb50)
{
    char line[80];
    if (!L2Log) {
        l2_fill(base, (end - base) / 4, hb50);
        return;
    }
    for (u32 a = base; a < end; a += size) {
        u32 n = end - a < size ? end - a : size;
        snprintf(line, sizeof(line), "  50h=%02x reading %08x-%08x (%d/%d)", hb50, a, a + n - 1,
                 (a - base) / size + 1, (end - base + size - 1) / size);
        l2_log(line);
        l2_fill(a, n / 4, hb50);
    }
}

#define L2_SAMPLES 16

// DRAM dwords across the scratch range: read with the L2 off, then through it; the differences are logged.
static void
l2_samples(u32 base, u32 end, u32 *dram, int through_l2)
{
    u32 step = (end - base) / L2_SAMPLES, bad = 0, first = 0, got = 0, want = 0;
    char line[80];
    if (!L2Log)
        return;
    for (int i = 0; i < L2_SAMPLES; i++) {
        u32 addr = base + step * i + 0x40, val = readl((void *)addr);
        if (!through_l2)
            dram[i] = val;
        else if (val != dram[i] && !bad++) {
            first = addr;
            got = val;
            want = dram[i];
        }
    }
    if (!through_l2)
        return;
    if (!bad)
        snprintf(line, sizeof(line), "  %d DRAM samples read through the L2: all equal", L2_SAMPLES);
    else
        snprintf(line, sizeof(line), "  through the L2 %d of %d samples differ, first %08x: %08x, DRAM %08x",
                 bad, L2_SAMPLES, first, got, want);
    l2_log(line);
}

// One tag check, logged with what it read.
static int
l2_check(int caches, u32 addr, u32 size, u8 hb50)
{
    char line[80];
    L2Got[0] = L2Got[1] = 0xdeadbeef;
    int ok = caches ? l2_caches(addr, size, hb50) : l2_line_works(addr, size, hb50);
    if (caches)
        snprintf(line, sizeof(line), "  cached at %08x: read %08x, want %08x: %s", addr, L2Got[0],
                 L2Saved, ok ? "ok" : "FAIL");
    else
        snprintf(line, sizeof(line), "  line at %08x: read %08x %08x, want %08x: %s", addr, L2Got[0], L2Got[1],
                 addr ^ 0x5aa5c33c, ok ? "ok" : "FAIL");
    l2_log(line);
    return ok;
}

// The L2 enabled as hb50 says (size, banks, timing, write policy) with 51h bit 0 = tag8 (8 tag bits, else 7):
// an L2 status; the L2 is left on when L2_OK, else fully off.
static int
l2_enable(u8 hb50, int tag8)
{
    u8 hb55 = pci_config_readb(HB_BDF, 0x55), hb51 = pci_config_readb(HB_BDF, 0x51);
    u32 size = l2_size(hb50), end, base = l2_scratch(size, &end);
    u32 cacheable = size << (tag8 ? 8 : 7);
    u32 dram_top = ALIGN(LegacyRamSize, 1024*1024);   // above the reserved CBMEM
    if (cacheable > dram_top)
        cacheable = dram_top;
    u32 tag_max = cacheable / size - 1, samples[L2_SAMPLES];
    char line[80];
    int status;
    if (L2Log) {
        snprintf(line, sizeof(line), "L2 %dK, scratch %08x-%08x, tags 0-%d, code %08x, stack %08x", size >> 10,
                 base, end - 1, tag_max, (u32)l2_enable, getesp());
        l2_log(line);
        l2_log_regs("before");
    }
    pci_config_writeb(HB_BDF, 0x51, (hb51 & ~1) | !!tag8);
    u32 cr0 = cr0_read();
    snprintf(line, sizeof(line), "55h %02x->%02x, CR0 %x->%x", hb55, hb55 & ~0x21, cr0, cr0 | CR0_CD | CR0_NW);
    l2_step(L2S_CACHES_OFF, line);
    pci_config_writeb(HB_BDF, 0x55, hb55 & ~0x21);
    cr0_write(cr0 | CR0_CD | CR0_NW);
    wbinvd();
    l2_samples(base, end, samples, 0);
    // Two passes over the same scratch range, as the original BIOS: write-through first gives every line a
    // tag in it; write-back then clears the dirty bits, and a line still dirty from power-on is written
    // back into that same range instead of over live memory.
    u8 wt = (hb50 & ~0x40) | 0x80;
    l2_step(L2S_FILL_WT, "every line given a tag");
    l2_fill_logged(base, end, size, wt);
    l2_samples(base, end, samples, 1);
    // checked while still write-through: a broken tag RAM would write garbage back once write-back is on
    l2_step(L2S_CHECKS, "tag 0, mixed, max; caching");
    status = !l2_check(0, 0x100, size, wt) ? L2_TAG0
        : !l2_check(0, (tag_max & 0x55) * size + 0x100, size, wt) ? L2_TAGMIX
        : !l2_check(0, tag_max * size + 0x100, size, wt) ? L2_TAGMAX
        : !l2_check(1, 0x100, size, wt) ? L2_NOCACHE : L2_OK;
    if (status == L2_OK) {
        l2_step(L2S_FILL_WB, hb50 & 0x40 ? "write-back on" : "write-through kept");
        l2_fill_logged(base, end, size, hb50 | 0x80);
    } else
        pci_config_writeb(HB_BDF, 0x50, hb50 & 0x30);   // fully off (no size): the state the board booted in
    l2_step(L2S_RESTORE, status == L2_OK ? "L2 on" : L2Names[status]);
    pci_config_writeb(HB_BDF, 0x55, hb55);
    wbinvd();
    cr0_write(cr0);
    if (L2Log)
        l2_log_regs("after ");
    return status;
}

// The L2 off again after l2_enable, its dirty lines written back first.
static void
l2_disable(u8 hb50)
{
    u32 end, base = l2_scratch(l2_size(hb50), &end);
    u8 hb55 = pci_config_readb(HB_BDF, 0x55);
    u32 cr0 = cr0_read();
    l2_step(L2S_DISABLE, "read the scratch range, then 50h off");
    pci_config_writeb(HB_BDF, 0x55, hb55 & ~0x21);
    cr0_write(cr0 | CR0_CD | CR0_NW);
    wbinvd();
    l2_fill_off(base, (end - base) / 4, pci_config_readb(HB_BDF, 0x50));
    pci_config_writeb(HB_BDF, 0x55, hb55);
    wbinvd();
    cr0_write(cr0);
    if (L2Log)
        l2_log_regs("off   ");
}

static u8 L2PostHb50;   // what the POST enabled (0: none), for the CACHE TEST page to restore

void
setup_l2_cache(void)
{
    u8 found = rtc_read(NB_CMOS_L2);
    u8 hb50 = (pci_config_readb(HB_BDF, 0x50) & 0xf0) | (found & 0x0f);
    u32 size = l2_size(hb50), end, base = l2_scratch(size, &end);
    L2Status = !size ? L2_NONE : !Setting[NB_L2_CACHE] ? L2_SETUP_OFF : end < base + 2*size ? L2_RAM_SMALL
        : (!SetupDefaulted && (found & 0x80)) ? L2_HUNG : L2_NOT_RUN;
    if (found & 0x80)
        L2HungStep = (found >> 4) & 7;
    rtc_write(NB_CMOS_L2, found & 0x0f);
    if (L2Status != L2_NOT_RUN)
        return;
    rtc_write(NB_CMOS_L2, found | 0x80);    // cleared below; still set at the next POST if this hangs
    L2Status = l2_enable(hb50 | 0x40, pci_config_readb(HB_BDF, 0x51) & 1);   // write-back, as the original
    L2PostHb50 = L2Status == L2_OK ? hb50 | 0x40 : 0;
    rtc_write(NB_CMOS_L2, found & 0x0f);
}

// CACHE TEST: the POST's L2 init (l2_enable) with each write policy, timing (50h bits 5-4) and tag width on the
// size the POST found. Enter runs the selected one step by step, each step and its values on screen for 1 s
// (a photo of the last screen shows where it hangs); A runs all of them quickly. For each that works, the read
// speed inside the L2 and from DRAM. If one hangs or resets the board, the next POST leaves the L2 off once and
// shows the step.
static void
put_at(char *line, int col, const char *text)
{
    memcpy(line + col, text, strlen(text));
}

static u8
l2_variant(u8 found, int v)
{
    return found | (v & 6) << 3 | (v & 8 ? 0x40 : 0);   // v: bit 0 tag8, bits 2-1 timing, bit 3 write-back
}

// One variant: result and speeds into its row text.
static void
l2_variant_run(u8 found, int v, char *line)
{
    u32 size = l2_size(found);
    char field[24], text[80];
    rtc_write(NB_CMOS_L2, found | 0x80);
    int status = l2_enable(l2_variant(found, v), v & 1);
    memset(line + 21, ' ', 55);
    put_at(line, 21, status == L2_OK ? "works" : L2Names[status]);
    if (status == L2_OK) {
        l2_step(L2S_SPEED, "inside the L2, then from DRAM");
        u16 in_l2 = read_speed(size / 2), dram = read_speed(size * 4);
        snprintf(text, sizeof(text), "  %d MB/s in the L2, %d MB/s from DRAM", in_l2, dram);
        l2_log(text);
        l2_disable(l2_variant(found, v));
        snprintf(field, sizeof(field), "%d", in_l2);
        put_at(line, 53, field);
        snprintf(field, sizeof(field), "%d", dram);
        put_at(line, 62, field);
    }
    rtc_write(NB_CMOS_L2, found);
}

static void
l2_test_run(void)
{
    u8 found = rtc_read(NB_CMOS_L2) & 0x0f, hb51 = pci_config_readb(HB_BDF, 0x51);
    u32 size = l2_size(found);
    char rows[16][80], line[80];
    int sel = (pci_config_readb(HB_BDF, 0x50) & 0x30) >> 3 | 8 | (hb51 & 1);   // the POST's: write-back
    if (!size) {
        scr_header("CACHE TEST");
        scr_puts(ROW_HELP, 2, AttrText, "ESC : Back");
        info_line(3, "No L2 cache found at POST");
        while (get_keystroke(1000) != 0x01)
            ;
        return;
    }
    for (int v = 0; v < 16; v++) {
        memset(rows[v], ' ', 76);
        rows[v][76] = 0;
        put_at(rows[v], 0, v & 8 ? "WB" : "WT");
        snprintf(line, sizeof(line), "%d", (v >> 1) & 3);
        put_at(rows[v], 8, line);
        put_at(rows[v], 16, v & 1 ? "8" : "7+1");
        put_at(rows[v], 21, "-");
    }
    if (L2PostHb50) {
        rtc_write(NB_CMOS_L2, found | 0x80);
        l2_disable(L2PostHb50);
        rtc_write(NB_CMOS_L2, found);
    }
    for (;;) {
        scr_header("CACHE TEST");
        if (L2HungStep)
            snprintf(line, sizeof(line), "L2 %dK, %d bank(s); the last L2 init stopped at step %d: %s", size >> 10,
                     found & 8 ? 2 : 1, L2HungStep, L2StepNames[L2HungStep]);
        else
            snprintf(line, sizeof(line), "L2 found at POST: %dK, %d bank(s); POST setting 50h=%x", size >> 10,
                     found & 8 ? 2 : 1, L2PostHb50);
        info_line(3, line);
        info_line(4, "Policy  Timing  Tag  Result                          L2 MB/s  DRAM MB/s");
        for (int v = 0; v < 16; v++)
            scr_puts(5 + v, 3, v == sel ? AttrSelect : AttrText, rows[v]);
        scr_puts(ROW_HELP, 2, AttrText, "ESC : Back   \x18\x19 : Select   Enter : Run step by step   A : Run all");
        int key = get_keystroke(1000);
        if (key == 0x01)
            break;
        if (key == 0x48)
            sel = (sel + 15) % 16;
        if (key == 0x50)
            sel = (sel + 1) % 16;
        if (key == 0x1c) {
            snprintf(line, sizeof(line), "CACHE TEST: %s, timing %d, tag %s", sel & 8 ? "WB" : "WT", (sel >> 1) & 3,
                     sel & 1 ? "8" : "7+1");
            scr_header(line);
            L2Log = L2_LOG_TOP;
            l2_variant_run(found, sel, rows[sel]);
            l2_log(rows[sel] + 21);
            L2Log = 0;
            scr_puts(ROW_HELP, 2, AttrText, "Done, press a key");
            while (get_keystroke(1000) < 0)
                ;
        }
        if (key == 0x1e) {
            for (int v = 0; v < 16; v++) {
                put_at(rows[v], 21, "testing...");
                scr_puts(5 + v, 3, AttrText, rows[v]);
                l2_variant_run(found, v, rows[v]);
                scr_puts(5 + v, 3, AttrText, rows[v]);
            }
        }
    }
    pci_config_writeb(HB_BDF, 0x51, hb51);
    if (L2PostHb50) {
        rtc_write(NB_CMOS_L2, found | 0x80);
        l2_enable(L2PostHb50, hb51 & 1);
        rtc_write(NB_CMOS_L2, found);
    }
}

void
setup_memtest(void)
{
    if (GET_BDA(video_mode) == 3 && Vram)
        memtest();
}

/****************************************************************
 * Benchmarks (SpeedSys style), VGA mode 12h: 640x480, 16 colors, 80x30 text cells
 ****************************************************************/

enum { C_BLACK, C_BLUE, C_GREEN, C_CYAN, C_RED, C_MAGENTA, C_BROWN, C_GRAY, C_DGRAY, C_LBLUE, C_LGREEN, C_LCYAN,
       C_LRED, C_LMAGENTA, C_YELLOW, C_WHITE };
#define GFX ((volatile u8 *)0xa0000)

static void
gfx_reg(u8 index, u8 val)
{
    outb(index, 0x3ce);
    outb(val, 0x3cf);
}

// Mode 12h with writes through set/reset (each write paints the bit mask's pixels in the color): 0 without VGA.
static int
gfx_open(void)
{
    int10(0x0012, 0);
    return GET_BDA(video_mode) == 0x12;
}

static void
gfx_span(int x0, int x1, int y, u8 color)       // pixels x0..x1 of row y
{
    outb(0x02, 0x3c4);                          // the video BIOS's text output changes these: set again
    outb(0x0f, 0x3c5);                          // all planes
    gfx_reg(1, 0x0f);                           // set/reset on all planes: the color comes from register 0
    gfx_reg(3, 0x00);
    gfx_reg(5, 0x00);                           // write mode 0
    gfx_reg(0, color);
    for (int x = x0; x <= x1; ) {
        int bits = 8 - (x & 7), n = x1 - x + 1 < bits ? x1 - x + 1 : bits;
        volatile u8 *p = GFX + y * 80 + x / 8;
        gfx_reg(8, (0xff >> (x & 7)) & ~(0xff >> ((x & 7) + n)));
        (void)*p;                               // load the latches
        *p = 0xff;
        x += n;
    }
    gfx_reg(8, 0xff);
}

static void
gfx_rect(int x0, int y0, int x1, int y1, u8 color)
{
    for (int y = y0; y <= y1; y++)
        gfx_span(x0, x1, y, color);
}

static void
gfx_frame(int x0, int y0, int x1, int y1, u8 color)
{
    gfx_span(x0, x1, y0, color);
    gfx_span(x0, x1, y1, color);
    gfx_rect(x0, y0, x0, y1, color);
    gfx_rect(x1, y0, x1, y1, color);
}

static void
gfx_line(int x0, int y0, int x1, int y1, u8 color)
{
    int dx = x1 > x0 ? x1 - x0 : x0 - x1, sx = x1 > x0 ? 1 : -1;
    int dy = y1 > y0 ? y1 - y0 : y0 - y1, sy = y1 > y0 ? 1 : -1, err = dx - dy;
    for (;;) {
        gfx_span(x0, x0, y0, color);
        if (x0 == x1 && y0 == y1)
            return;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Text in the 8x16 cell grid, by the video BIOS (it draws the ROM font in graphics modes).
static void
gfx_text(int row, int col, u8 color, const char *s)
{
    for (; *s && col < 80; s++, col++) {
        struct bregs br;
        memset(&br, 0, sizeof(br));
        br.flags = F_IF;
        br.ah = 0x02;
        br.dh = row;
        br.dl = col;
        call16_int(0x10, &br);
        memset(&br, 0, sizeof(br));
        br.flags = F_IF;
        br.ax = 0x0900 | (u8)*s;
        br.bl = color;
        br.cx = 1;
        call16_int(0x10, &br);
    }
}

static void
gfx_title(const char *title)
{
    gfx_rect(0, 0, 639, 479, C_BLUE);
    gfx_text(0, 40 - strlen(title) / 2, C_WHITE, title);
    gfx_text(29, 1, C_LCYAN, "ESC : Back");
}

/* Memory: read, write and move speed by block size */

static u16 MemMBs[3][SPEED_POINTS];             // read, write, move

static void
write_block(u32 addr, u32 bytes)
{
    u32 n = bytes / 64;
    asm volatile("1: movl %%eax, (%0); movl %%eax, 4(%0); movl %%eax, 8(%0); movl %%eax, 12(%0)\n"
                 "movl %%eax, 16(%0); movl %%eax, 20(%0); movl %%eax, 24(%0); movl %%eax, 28(%0)\n"
                 "movl %%eax, 32(%0); movl %%eax, 36(%0); movl %%eax, 40(%0); movl %%eax, 44(%0)\n"
                 "movl %%eax, 48(%0); movl %%eax, 52(%0); movl %%eax, 56(%0); movl %%eax, 60(%0)\n"
                 "addl $64, %0; decl %1; jnz 1b" : "+r"(addr), "+r"(n) : "a"(0x5aa5c33c) : "memory");
}

static void
move_block(u32 src, u32 dst, u32 bytes)
{
    u32 n = bytes / 4;
    asm volatile("cld; rep movsl" : "+S"(src), "+D"(dst), "+c"(n) : : "memory");
}

// MB/s of one kind on 'block' bytes at src (and dst) for about 20 ms, after one warm-up pass; the timer is read
// once per 64 KB or more (reading the PIT costs microseconds).
static u16
mem_speed(int kind, u32 src, u32 dst, u32 block)
{
    u32 done = 0, ticks = 0, reps = block < 0x10000 ? 0x10000 / block : 1;
    u32 flags = save_flags();
    irq_disable();
    for (int pass = 0; pass < 2; pass++) {
        u32 t0 = timer_read();
        do {
            for (u32 r = 0; r < reps; r++) {
                if (kind == 0)
                    read_block(src, block);
                else if (kind == 1)
                    write_block(dst, block);
                else
                    move_block(src, dst, block);
            }
            done += pass ? reps * block : 0;
            ticks = timer_read() - t0;
        } while (pass && ticks < TimerKHz * 20);
    }
    restore_flags(flags);
    return ticks ? (done >> 10) * TimerKHz / ticks * 1024 / 1000 : 0;
}

static void
mem_measure(void)
{
    u32 top = speed_block(SPEED_POINTS - 1);
    u8 *src = malloc_tmphigh(top), *dst = malloc_tmphigh(top);
    if (!src || !dst)
        goto done;
    memset(src, 0x5a, top);
    for (int i = 0; i < SPEED_POINTS; i++)
        for (int kind = 0; kind < 3; kind++)
            MemMBs[kind][i] = mem_speed(kind, (u32)src, (u32)dst, speed_block(i));
done:
    free(src);
    free(dst);
}

#define MCH_X0 64
#define MCH_X1 616
#define MCH_Y0 64
#define MCH_Y1 384

static int
bench_memory(void)
{
    static const u8 colors[3] = { C_YELLOW, C_LRED, C_LGREEN };
    static const char * const names[3] = { "Reading", "Writing", "Moving" };
    static const char * const size[] = { "4K", "8K", "16K", "32K", "64K", "128K", "256K", "512K", "1M", "2M" };
    char text[48];
    int max = 1;
    gfx_title("MEMORY SPEED");
    gfx_text(2, 2, C_WHITE, "Measuring...");
    if (!MemMBs[0][0])
        mem_measure();
    gfx_rect(16, 32, 300, 47, C_BLUE);
    for (int k = 0; k < 3; k++)
        for (int i = 0; i < SPEED_POINTS; i++)
            if (MemMBs[k][i] > max)
                max = MemMBs[k][i];
    max = (max + 49) / 50 * 50;
    gfx_text(2, 2, C_WHITE, "Memory speed (MB/s) by block size");
    gfx_rect(MCH_X0, MCH_Y0, MCH_X1, MCH_Y1, C_BLACK);
    for (int g = 0; g <= 4; g++) {              // grid and MB/s scale
        int y = MCH_Y1 - g * (MCH_Y1 - MCH_Y0) / 4;
        gfx_span(MCH_X0, MCH_X1, y, C_DGRAY);
        snprintf(text, sizeof(text), "%d", max * g / 4);
        gfx_text(y / 16, MCH_X0 / 8 - 1 - strlen(text), C_WHITE, text);
    }
    for (int i = 0; i < ARRAY_SIZE(size); i++) {
        int x = MCH_X0 + i * 4 * (MCH_X1 - MCH_X0) / (SPEED_POINTS - 1);
        gfx_rect(x, MCH_Y0, x, MCH_Y1, C_DGRAY);
        gfx_text(MCH_Y1 / 16 + 1, x / 8 - strlen(size[i]) / 2, C_WHITE, size[i]);
    }
    for (int k = 0; k < 3; k++) {
        int x0 = -1, y0 = 0;
        for (int i = 0; i < SPEED_POINTS && MemMBs[k][i]; i++) {
            int x = MCH_X0 + i * (MCH_X1 - MCH_X0) / (SPEED_POINTS - 1);
            int y = MCH_Y1 - MemMBs[k][i] * (MCH_Y1 - MCH_Y0) / max;
            if (x0 >= 0)
                gfx_line(x0, y0, x, y, colors[k]);
            x0 = x;
            y0 = y;
        }
        snprintf(text, sizeof(text), "%s: %d MB/s at 4K, %d MB/s at 2M", names[k], MemMBs[k][0],
                 MemMBs[k][SPEED_POINTS - 1]);
        gfx_text(26 + k, 8, colors[k], text);
    }
    return 0;
}

/* CPU: integer (ALU) and floating point (FPU) speed against 486-class CPUs */

// Microseconds in 'ticks' timer ticks, without 64-bit arithmetic.
static u32
ticks_us(u32 ticks)
{
    return ticks / TimerKHz * 1000 + ticks % TimerKHz * 1000 / TimerKHz;
}

// Million instructions/s of 16 one-clock ALU instructions + dec/jnz (20 clocks on a 486 core) for about 30 ms.
static u32
cpu_alu_mips(void)
{
    u32 flags = save_flags(), loops = 0, ticks, t0;
    irq_disable();
    t0 = timer_read();
    do {
        u32 n = 4000;
        asm volatile(".align 16\n1: addl %%ebx, %%eax; xorl %%ecx, %%edx; subl %%eax, %%edi; andl %%edx, %%ebx\n"
                     "orl %%edi, %%ecx; addl %%ebx, %%eax; xorl %%ecx, %%edx; subl %%eax, %%edi\n"
                     "andl %%edx, %%ebx; orl %%edi, %%ecx; addl %%ebx, %%eax; xorl %%ecx, %%edx\n"
                     "subl %%eax, %%edi; andl %%edx, %%ebx; orl %%edi, %%ecx; addl %%ebx, %%eax\n"
                     "decl %%esi; jnz 1b"
                     : "+S"(n) : "a"(1), "b"(2), "c"(3), "d"(4), "D"(5) : "cc");
        loops += 4000;
        ticks = timer_read() - t0;
    } while (ticks < TimerKHz * 30);
    restore_flags(flags);
    return loops * 18 / ticks_us(ticks);                // 18 instructions per loop: per us = million per s
}

// Million FPU operations/s of 4 FADD + 4 FMUL per loop for about 30 ms; 0 without FPU.
static u32
cpu_fpu_mflops(void)
{
    if (!(GET_BDA(equipment_list_flags) & 2))
        return 0;
    u32 flags = save_flags(), loops = 0, ticks, t0;
    irq_disable();
    asm volatile("fninit; fld1; fld1; fld1");
    t0 = timer_read();
    do {
        u32 n = 2000;
        asm volatile(".align 16\n1: fadd %%st(1), %%st; fmul %%st(2), %%st; fadd %%st(1), %%st; fmul %%st(2), %%st\n"
                     "fadd %%st(1), %%st; fmul %%st(2), %%st; fadd %%st(1), %%st; fmul %%st(2), %%st\n"
                     "decl %0; jnz 1b" : "+r"(n));
        loops += 2000;
        ticks = timer_read() - t0;
    } while (ticks < TimerKHz * 30);
    asm volatile("fninit");
    restore_flags(flags);
    return loops * 8 * 1000 / ticks_us(ticks);          // operations per ms = thousands per s
}

// A SpeedSys bar: this machine's value against references, labels above and below the scale.
static void
gfx_bar(int y, const char *title, u32 value, const char *unit, const char * const *ref_name, const u32 *ref_value,
        int refs, int decimals)
{
    char text[40];
    u32 max = value;
    for (int r = 0; r < refs; r++)
        if (ref_value[r] > max)
            max = ref_value[r];
    max += max / 10 + 1;
    gfx_frame(16, y, 623, y + 95, C_LCYAN);
    gfx_text(y / 16, 4, C_LCYAN, title);
    gfx_rect(40, y + 40, 600, y + 55, C_BLACK);
    gfx_rect(40, y + 40, 40 + value * 560 / max, y + 55, C_LBLUE);
    for (int r = 0; r < refs; r++) {            // references alternately above and below the scale
        int x = 40 + ref_value[r] * 560 / max, up = r & 1;
        gfx_rect(x, up ? y + 30 : y + 56, x, up ? y + 39 : y + 64, C_WHITE);
        int col = x / 8 - strlen(ref_name[r]) / 2;
        gfx_text((up ? y + 16 : y + 66) / 16, col < 3 ? 3 : col, C_WHITE, ref_name[r]);
    }
    if (decimals)
        snprintf(text, sizeof(text), " %d.%d %s ", value / 1000, value % 1000 / 100, unit);
    else
        snprintf(text, sizeof(text), " %d %s ", value, unit);
    gfx_text((y + 40) / 16, 6, C_YELLOW, text);
}

// The 486-class CPUs (Intel and AMD cores: the same clocks per instruction, so speed scales with the core clock).
static const char * const RefCpu[] = {
    "486SX-25", "486DX-33", "486DX2-50", "486DX2-66", "486DX4-100", "5x86-133", "5x86-160"
};
static const u16 RefMhz[] = { 25, 33, 50, 66, 100, 133, 160 };

static int
bench_cpu(void)
{
    char vendor[13], text[80];
    u32 fms = 0, features, refs = 0, alu, fpu, ref_alu[7], ref_fpu[7];
    int mhz, row = 2;
    const char *name = cpu_short_name(&mhz);
    int known = cpu_ident(vendor, &fms, &features) && (fms >> 8 & 15) == 4
        && (!strcmp(vendor, "GenuineIntel") || !strcmp(vendor, "AuthenticAMD"));
    gfx_title("CPU BENCHMARK");
    gfx_text(row, 2, C_WHITE, "Measuring...");
    alu = cpu_alu_mips();
    fpu = cpu_fpu_mflops();
    gfx_rect(16, 32, 300, 47, C_BLUE);
    snprintf(text, sizeof(text), "Processor   : %s, %d MHz (measured)", name, mhz);
    gfx_text(row++, 2, C_WHITE, text);
    snprintf(text, sizeof(text), "CPUID       : %s family %d model %d stepping %d", vendor, fms >> 8 & 15, fms >> 4 & 15,
             fms & 15);
    gfx_text(row++, 2, C_WHITE, fms ? text : "CPUID       : none (pre-CPUID 486)");
    snprintf(text, sizeof(text), "FPU         : %s", fpu ? "integrated" : "none");
    gfx_text(row++, 2, C_WHITE, text);
    u8 l2 = pci_config_readb(HB_BDF, 0x50);
    snprintf(text, sizeof(text), "L2 cache    : %s", L2Status == L2_OK ? (l2 & 0x40 ? "on, write-back" : "on, write-through")
             : L2Names[L2Status]);
    gfx_text(row++, 2, C_WHITE, text);
    snprintf(text, sizeof(text), "Memory      : %d KB", 1024 + ext_mem_kb());
    gfx_text(row++, 2, C_WHITE, text);
    gfx_text(row++, 2, C_WHITE, "Chipset     : UMC UM8881F + UM8886BF");
    snprintf(text, sizeof(text), "BIOS        : newBIOS %s", RELEASE_DATE);
    gfx_text(row++, 2, C_WHITE, text);
    if (known && mhz) {                         // references: this CPU's own speed per MHz
        for (refs = 0; refs < ARRAY_SIZE(RefMhz); refs++) {
            ref_alu[refs] = alu * RefMhz[refs] / mhz;
            ref_fpu[refs] = refs ? fpu * RefMhz[refs] / mhz : 0;   // the 486SX has no FPU
        }
    }
    gfx_bar(176, "Integer (ALU) speed, million instructions/s", alu, "MIPS", RefCpu, ref_alu, refs, 0);
    if (fpu)
        gfx_bar(288, "Floating point (FPU) speed, million operations/s", fpu, "MFLOPS", RefCpu + 1, ref_fpu + 1,
                refs ? refs - 1 : 0, 1);
    else
        gfx_text(19, 4, C_LCYAN, "No FPU");
    gfx_text(26, 2, C_GRAY, known ? "References: Intel and AMD 486-class cores run these loops in the same clocks:"
             : "No references: this CPU's clocks per instruction differ from the 486's");
    if (known)
        gfx_text(27, 2, C_GRAY, "their speed is this CPU's per MHz times their clock.");
    return 0;
}

/* Hard disks: seek times, linear read speed across the disk, interface speed */

#define HD_MAX    8
#define HD_POINTS 64
static struct {
    struct drive_s *drive;
    char name[48];
    u32 sectors;
    int tested, error;                          // error: one of HdErrors
    u32 err_lba;
    u32 max_seek_us, avg_seek_us, t2t_us, buffered_kbs, avg_kbs;
    u16 kbs[HD_POINTS], access_ms10[HD_POINTS];
} Hd[HD_MAX];
static int HdCount;
enum { HD_READ_ERROR = 1, HD_SLOW, HD_ABORTED, HD_SMALL };
static const char * const HdErrors[] = {
    "", "Read error at LBA %d", "Too slow: a read at LBA %d took over 2 s", "Stopped (ESC)",
    "Disk under 256 KB, or no memory"
};

static void
hd_list(void)
{
    struct drive_s *drives[HD_MAX];
    const char *descs[HD_MAX];
    int n = boot_hd_list(drives, descs, HD_MAX);
    for (HdCount = 0; HdCount < n; HdCount++) {
        struct drive_s *drive = drives[HdCount];
        memset(&Hd[HdCount], 0, sizeof(Hd[0]));
        Hd[HdCount].drive = drive;
        Hd[HdCount].sectors = drive->sectors > 0xffffffff ? 0xffffffff : drive->sectors;
        snprintf(Hd[HdCount].name, sizeof(Hd[0].name), "%s", descs[HdCount] ? descs[HdCount] : "disk");
        for (int k = 0; k < 4; k++)
            if (IdeInfo[k].present && IdeInfo[k].drive == drive)
                snprintf(Hd[HdCount].name, sizeof(Hd[0].name), "%s %s", k & 2 ? "Secondary" : "Primary",
                         IdeInfo[k].model);
    }
}

// Microseconds to read count sectors at lba into buf; -1 on an error.
static int
hd_read(struct drive_s *drive, u32 lba, u16 count, void *buf)
{
    struct disk_op_s op;
    memset(&op, 0, sizeof(op));
    op.drive_fl = drive;
    op.command = CMD_READ;
    op.lba = lba;
    op.count = count;
    op.buf_fl = buf;
    u32 t0 = timer_read();
    // process_op() runs ATA only in 16-bit mode; the driver itself works in 32-bit mode too
    int ret = drive->type == DTYPE_ATA ? ata_process_op(&op) : process_op(&op);
    u32 ticks = timer_read() - t0;
    return ret ? -1 : (int)ticks_us(ticks);
}

static u32
hd_rand(u32 *seed)
{
    *seed = *seed * 1103515245 + 12345;
    return *seed >> 8;
}

// The whole test of disk h: 0, or -1 with Hd[h].error set at the first read error.
static int
hd_test(int h, void (*progress)(int percent))
{
    struct drive_s *drive = Hd[h].drive;
    u32 n = Hd[h].sectors, seed = 0x1234567, sum, t;
    u8 *buf = malloc_tmphigh(64 * 1024);
    int us;
    if (!buf || n < 512) {
        free(buf);
        Hd[h].error = HD_SMALL;
        return -1;
    }
#define HD_TRY(expr, at) do {                                               \
        us = (expr);                                                        \
        if (us < 0 || us > 2000000) {                                       \
            Hd[h].error = us < 0 ? HD_READ_ERROR : HD_SLOW;                 \
            Hd[h].err_lba = (at);                                           \
            goto out;                                                       \
        }                                                                   \
    } while (0)
    HD_TRY(hd_read(drive, 0, 64, buf), 0);      // buffered: the same 32 KB again, from the drive's cache
    for (sum = 0, t = 0; t < 8; t++) {
        HD_TRY(hd_read(drive, 0, 64, buf), 0);
        sum += us;
    }
    Hd[h].buffered_kbs = sum ? 8 * 32 * 1000000 / sum : 0;
    u32 track = drive->lchs.head && drive->lchs.sector ? drive->lchs.head * drive->lchs.sector : 63;
    if (track > n / 32)                         // a small disk: 16 steps stay inside it
        track = n / 32;
    for (sum = 0, t = 0; t < 16; t++) {          // track to track
        HD_TRY(hd_read(drive, (n / 2) + t * track, 1, buf), (n / 2) + t * track);
        sum += t ? us : 0;
    }
    Hd[h].t2t_us = sum / 15;
    for (sum = 0, t = 0; t < 8; t++) {           // full stroke
        HD_TRY(hd_read(drive, t & 1 ? n - 1 : 0, 1, buf), t & 1 ? n - 1 : 0);
        sum += t ? us : 0;
    }
    Hd[h].max_seek_us = sum / 7;
    for (sum = 0, t = 0; t < 32; t++) {          // random
        u32 lba = hd_rand(&seed) % n;
        HD_TRY(hd_read(drive, lba, 1, buf), lba);
        sum += us;
    }
    Hd[h].avg_seek_us = sum / 32;
    for (sum = 0, t = 0; t < HD_POINTS; t++) {   // across the disk: access time, then 64 KB read on
        u32 lba = (n - 256) / (HD_POINTS - 1) * t + (n - 256) % (HD_POINTS - 1) * t / (HD_POINTS - 1);
        HD_TRY(hd_read(drive, lba, 1, buf), lba);
        Hd[h].access_ms10[t] = us / 100;
        HD_TRY(hd_read(drive, lba + 1, 128, buf), lba + 1);
        Hd[h].kbs[t] = us ? 64 * 1000000 / us : 0;
        sum += Hd[h].kbs[t];
        if (progress) {
            progress(t * 100 / HD_POINTS);
            if (get_keystroke(0) == 0x01) {
                Hd[h].error = HD_ABORTED;
                goto out;
            }
        }
    }
    Hd[h].avg_kbs = sum / HD_POINTS;
    Hd[h].tested = 1;
out:
    free(buf);
    return Hd[h].error ? -1 : 0;
}

static void
hd_progress(int percent)
{
    char text[16];
    snprintf(text, sizeof(text), "%d%%   ", percent);
    gfx_text(10, 60, C_WHITE, text);
}

#define HCH_X0 368
#define HCH_X1 624
#define HCH_Y0 192
#define HCH_Y1 400

static void
hd_show(int h)
{
    char text[80];
    u32 max = 1;
    gfx_rect(8, 112, 631, 463, C_BLUE);
    if (Hd[h].error) {
        snprintf(text, sizeof(text), HdErrors[Hd[h].error], Hd[h].err_lba);
        gfx_text(8, 2, C_LRED, text);
        Hd[h].error = Hd[h].error == HD_ABORTED ? 0 : Hd[h].error;   // stopped: may run again
        return;
    }
    snprintf(text, sizeof(text), "Maximal seek time     : %d.%02d ms", Hd[h].max_seek_us / 1000,
             Hd[h].max_seek_us % 1000 / 10);
    gfx_text(8, 2, C_LRED, text);
    snprintf(text, sizeof(text), "Average (random) seek : %d.%02d ms", Hd[h].avg_seek_us / 1000,
             Hd[h].avg_seek_us % 1000 / 10);
    gfx_text(9, 2, C_LMAGENTA, text);
    snprintf(text, sizeof(text), "Track-to-track seek   : %d.%02d ms", Hd[h].t2t_us / 1000, Hd[h].t2t_us % 1000 / 10);
    gfx_text(10, 2, C_LGREEN, text);
    snprintf(text, sizeof(text), "Buffered read speed   : %d KB/s", Hd[h].buffered_kbs);
    gfx_text(11, 2, C_LCYAN, text);
    snprintf(text, sizeof(text), "Linear read speed     : %d KB/s", Hd[h].avg_kbs);
    gfx_text(12, 2, C_YELLOW, text);
    for (int t = 0; t < HD_POINTS; t++)
        if (Hd[h].kbs[t] > max)
            max = Hd[h].kbs[t];
    max = (max + 499) / 500 * 500;
    gfx_rect(HCH_X0, HCH_Y0, HCH_X1, HCH_Y1, C_BLACK);
    for (int g = 0; g <= 4; g++) {
        int y = HCH_Y1 - g * (HCH_Y1 - HCH_Y0) / 4;
        gfx_span(HCH_X0, HCH_X1, y, C_DGRAY);
        snprintf(text, sizeof(text), "%d", max * g / 4);
        gfx_text(y / 16, HCH_X0 / 8 - 1 - strlen(text), C_YELLOW, text);
    }
    int x0 = -1, y0 = 0;
    for (int t = 0; t < HD_POINTS; t++) {
        int x = HCH_X0 + t * (HCH_X1 - HCH_X0) / (HD_POINTS - 1);
        int y = HCH_Y1 - Hd[h].kbs[t] * (HCH_Y1 - HCH_Y0) / max;
        int ya = HCH_Y1 - (Hd[h].access_ms10[t] > 400 ? 400 : Hd[h].access_ms10[t]) * (HCH_Y1 - HCH_Y0) / 400;
        gfx_rect(x, ya, x + 1, ya + 1, C_LGREEN);      // access time, 0-40 ms
        if (x0 >= 0)
            gfx_line(x0, y0, x, y, C_YELLOW);
        x0 = x;
        y0 = y;
    }
    gfx_text(HCH_Y1 / 16 + 1, HCH_X0 / 8, C_YELLOW, "read KB/s");
    gfx_text(HCH_Y1 / 16 + 1, HCH_X0 / 8 + 13, C_LGREEN, "access 0-40 ms");
    gfx_text(HCH_Y1 / 16 + 2, HCH_X0 / 8, C_WHITE, "start of the disk          end");
}

static int
bench_disk(void)
{
    char text[80];
    int sel = 0;
    hd_list();
    gfx_title("HARD DISK BENCHMARK");
    if (!HdCount) {
        gfx_text(3, 2, C_WHITE, "No hard disk");
        return 0;
    }
    for (;;) {
        for (int h = 0; h < HdCount; h++) {
            snprintf(text, sizeof(text), "%c %s, %d MB", h == sel ? '>' : ' ', Hd[h].name, Hd[h].sectors / 2048);
            gfx_rect(8, (2 + h) * 16, 631, (3 + h) * 16 - 1, C_BLUE);
            gfx_text(2 + h, 2, h == sel ? C_YELLOW : C_WHITE, text);
        }
        gfx_text(29, 20, C_LCYAN, "\x18\x19 : Select disk   Enter : Test it");
        int key = get_keystroke(1000);
        if (key == 0x01)
            return 1;                           // ESC taken
        if (key == 0x48 && sel > 0)
            sel--;
        if (key == 0x50 && sel < HdCount - 1)
            sel++;
        if (key == 0x1c) {
            gfx_rect(8, 112, 631, 463, C_BLUE);
            gfx_text(10, 2, C_WHITE, "Testing (seek, then reading across the disk), ESC stops...");
            hd_test(sel, hd_progress);
            hd_show(sel);
        }
    }
}

// The PIO mode limits of the ATA interface (MB/s x 10): the reference scale of the disk bars.
static const char * const RefPio[] = { "PIO 0", "PIO 1", "PIO 2", "PIO 3", "PIO 4" };
static const u32 RefPioKbs[] = { 3300, 5200, 8300, 11100, 16600 };

static int
bench_disk_bars(void)
{
    int shown = 0;
    gfx_title("HARD DISK SPEED");
    if (!HdCount)
        hd_list();
    gfx_text(2, 2, C_WHITE, "Testing the disks not tested yet...");
    for (int h = 0; h < HdCount && h < 3; h++)
        if (!Hd[h].tested && !Hd[h].error)
            hd_test(h, NULL);
    gfx_rect(8, 32, 631, 47, C_BLUE);
    for (int h = 0; h < HdCount && shown < 3; h++) {
        if (Hd[h].error)
            continue;
        gfx_bar(32 + shown++ * 112, Hd[h].name, Hd[h].avg_kbs, "KB/s linear read", RefPio, RefPioKbs, 5, 0);
    }
    if (!shown)
        gfx_text(3, 2, C_WHITE, "No hard disk could be read");
    gfx_text(27, 2, C_GRAY, "References: the transfer limit of each ATA PIO mode.");
    return 0;
}

// The benchmark pages: graphics until ESC, then back to the text mode of the setup.
static void
bench_run(int (*page)(void))
{
    if (!gfx_open()) {
        scr_open();
        info_line(3, "These pages need a VGA (mode 12h)");
        while (get_keystroke(1000) != 0x01)
            ;
        return;
    }
    if (!page())                                // a page with its own keys took ESC already
        while (get_keystroke(1000) != 0x01)
            ;
    scr_open();
}

static void info_bench_memory(void) { bench_run(bench_memory); }
static void info_bench_cpu(void) { bench_run(bench_cpu); }
static void info_bench_disk(void) { bench_run(bench_disk); }
static void info_bench_disk_bars(void) { bench_run(bench_disk_bars); }


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
    char vendor[13];
    u32 fms, fpu;
    if (!cpu_ident(vendor, &fms, &fpu))
        fpu = (cr0_read() & 0x10) ? 1 : 0;   // CR0.ET: coprocessor present
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
