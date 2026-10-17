// PnP BIOS calls (PnP BIOS 1.0A): system device nodes, ISA PnP configuration and the ESCD (ESCD 1.02A,
// version 2.1), kept in a 4 KiB parameter block of the flash as by the board's original Award BIOS.
//
// Copyright (C) 2008  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_GLOBAL
#include "config.h" // BUILD_BIOS_ADDR
#include "farptr.h" // SET_FARVAR
#include "hw/pci.h" // pci_config_readb
#include "malloc.h" // malloc_tmp
#include "output.h" // dprintf
#include "stacks.h" // call32_params
#include "std/pnpbios.h" // PNP_SIGNATURE
#include "string.h" // checksum
#include "util.h" // pnp_init
#include "x86.h" // readb

extern struct pnpheader PNPHEADER;
extern char pnp_string[];

#if CONFIG_PNPBIOS
struct pnpheader PNPHEADER __aligned(16) VARFSEG = {
    .signature = PNP_SIGNATURE,
    .version = 0x10,
    .length = sizeof(PNPHEADER),
    .real_cs = SEG_BIOS,
    .prot_base = BUILD_BIOS_ADDR,
    .real_ds = SEG_BIOS,
    .prot_database = BUILD_BIOS_ADDR,
};
#else
// We need a copy of this string in the 0xf000 segment, but we are not
// actually a PnP BIOS, so make sure it is *not* aligned, so OSes will
// not see it if they scan.
char pnp_string[] __aligned(2) VARFSEG = " $PnP";
#endif

#define PNP_UNKNOWN_FUNCTION 0x81
#define PNP_INVALID_HANDLE   0x83
#define PNP_BAD_PARAMETER    0x84
#define PNP_USE_ESCD_SUPPORT 0x8d
#define PNP_HARDWARE_ERROR   0x8f

// Flash 0x10000-0x1ffff at its 4 GiB alias (also the base of the ESCD selector of protected mode callers).
#ifndef PNP_FLASH_BASE
#define PNP_FLASH_BASE 0xffff0000
#endif
#define ESCD_OFF  0xc000                        // flash 0x1c000: a 4 KiB parameter block
#define ESCD_SIZE 0x1000
#define ESCD_SIG  0x47464341u                   // "ACFG"
#define FL_HB     0x80                         // UM8881F 00:10.0
#define FL_IBC    0x90                         // UM8886BF 00:12.0

enum { CHIP_NONE, CHIP_INTEL, CHIP_W29 };       // Intel 28F001BX-T, Winbond W29C010 (86Box)

u8 PnpNodes[768] VARFSEG;
u8 PnpNodeCount VARFSEG, PnpCsns VARFSEG, FlashChip VARFSEG;
u16 PnpNodeMax VARFSEG, PnpReadPort VARFSEG;


/****************************************************************
 * Flash ESCD block
 ****************************************************************/

// Offset 'off' of flash 0x10000-0x1ffff: through the caller's ESCD selector in 16-bit protected mode,
// else at the 4 GiB alias, which reaches the chip whatever the shadow RAM holds.
static u8
fl_rd(u16 sel, u16 off)
{
    if (MODESEGMENT)
        return GET_FARVAR(sel, *(u8 *)(u32)off);
    return readb((void *)(PNP_FLASH_BASE + off));
}

static void
fl_wr(u16 sel, u16 off, u8 val)
{
    if (MODESEGMENT)
        SET_FARVAR(sel, *(u8 *)(u32)off, val);
    else
        writeb((void *)(PNP_FLASH_BASE + off), val);
}

static void
fl_jedec(u16 sel, u8 cmd)
{
    fl_wr(sel, 0x5555, 0xaa);
    fl_wr(sel, 0x2aaa, 0x55);
    fl_wr(sel, 0x5555, cmd);
}

// Reads 'off' until (value & mask) == want or after 'ticks' refresh periods of 15 us (port 61h bit 4;
// also bounded in reads, of at least about 1 us on the ISA bus, if it never toggles); the last value read.
static u8
fl_poll(u16 sel, u16 off, u8 mask, u8 want, u32 ticks)
{
    u32 reads = ticks * 16;
    u8 val, last = 0;
    while (((val = fl_rd(sel, off)) & mask) != want && ticks && reads--) {
        u8 refresh = inb(0x61) & 0x10;
        ticks -= refresh != last;
        last = refresh;
    }
    return val;
}

// Flash writes enabled as by the original BIOS: IBC 47h bit 6 off, 57h bits 5 and 2 on, HB 55h bit 6 on.
static const u8 FlashRegs[3][4] = {
    { FL_IBC, 0x47, 0x40, 0x00 }, { FL_IBC, 0x57, 0x00, 0x24 }, { FL_HB, 0x55, 0x00, 0x40 }
};

// Enables (saved NULL) or restores the registers.
static void
fl_enable(u8 *saved, int restore)
{
    int i;
    for (i = 0; i < 3; i++) {
        u16 bdf = FlashRegs[i][0], reg = FlashRegs[i][1];
        if (restore)
            pci_config_writeb(bdf, reg, saved[i]);
        else {
            saved[i] = pci_config_readb(bdf, reg);
            pci_config_writeb(bdf, reg, (saved[i] & ~FlashRegs[i][2]) | FlashRegs[i][3]);
        }
    }
}

// The chip, by its identifier; one that does not leave the array mode is unknown.
static u8
fl_detect(void)
{
    u8 saved[3];
    fl_enable(saved, 0);
    u16 array = fl_rd(0, 0) | fl_rd(0, 1) << 8, id;
    u8 chip = CHIP_NONE;
    fl_wr(0, ESCD_OFF, 0x90);
    id = fl_rd(0, 0) | fl_rd(0, 1) << 8;
    fl_wr(0, ESCD_OFF, 0xff);
    if (id == 0x9489 && id != array)
        chip = CHIP_INTEL;
    else {
        fl_jedec(0, 0x90);
        id = fl_rd(0, 0) | fl_rd(0, 1) << 8;
        fl_jedec(0, 0xf0);
        if (id == 0xc1da && id != array)
            chip = CHIP_W29;
    }
    fl_wr(0, ESCD_OFF, 0xff);                   // read array, also for a JEDEC ID mode left on
    fl_enable(saved, 1);
    return chip;
}

static u8 *Escd;                                // POST: the ESCD being built

// Byte i of an ESCD in memory at seg:off (segment 0: the POST's).
static u8
escd_src(u16 seg, u16 off, u16 i)
{
    if (!MODESEGMENT && !seg)
        return Escd[i];
    return GET_FARVAR(seg, *(u8 *)(u32)(u16)(off + i));
}

static u16
escd_sum(u16 seg, u16 off, u16 size)
{
    u16 sum = 0, i;
    for (i = 0; i < size - 2; i++)
        sum += escd_src(seg, off, i);
    return -sum;
}

// Size of the valid ESCD in the flash block, else 0.
static u16
escd_check(u16 sel)
{
    u16 size = fl_rd(sel, ESCD_OFF) | fl_rd(sel, ESCD_OFF + 1) << 8, sum = 0, i;
    u32 sig = 0;
    if (size < 14 || size > ESCD_SIZE)
        return 0;
    for (i = 0; i < size - 2; i++) {
        u8 val = fl_rd(sel, ESCD_OFF + i);
        sum += val;
        if (i >= 2 && i < 6)
            sig |= (u32)val << (8 * i - 16);
    }
    sum += fl_rd(sel, ESCD_OFF + size - 2) | fl_rd(sel, ESCD_OFF + size - 1) << 8;
    return sig == ESCD_SIG && !sum ? size : 0;
}

// Waits until the flash holds bytes from..to-1 of the ESCD at seg:off: at most 15 ms per byte (a W29C010
// page being written); 1 if it does.
static int
escd_same(u16 sel, u16 seg, u16 off, u16 from, u16 to)
{
    for (; from < to; from++) {
        u8 val = escd_src(seg, off, from);
        if (fl_poll(sel, ESCD_OFF + from, 0xff, val, 1000) != val)
            return 0;
    }
    return 1;
}

// Writes the ESCD at seg:off to the flash block and verifies it; a PnP return code.
static u16
escd_write(u16 sel, u16 seg, u16 off, u16 size)
{
    u8 chip = GET_GLOBAL(FlashChip), status;
    u16 i, page;
    int ok = 1;
    if (chip == CHIP_NONE)
        return FUNCTION_NOT_SUPPORTED;
    u8 saved[3];
    fl_enable(saved, 0);
    if (chip == CHIP_INTEL) {
        fl_wr(sel, ESCD_OFF, 0x50);             // clear status
        fl_wr(sel, ESCD_OFF, 0x20);             // block erase, confirm; then the chip reads its status
        fl_wr(sel, ESCD_OFF, 0xd0);
        status = fl_poll(sel, ESCD_OFF, 0x80, 0x80, 700000);   // 10 s
        ok = (status & 0xb8) == 0x80;           // ready, no erase, program or Vpp error
        for (i = 0; ok && i < size; i++) {
            u8 val = escd_src(seg, off, i);
            if (val == 0xff)
                continue;
            fl_wr(sel, ESCD_OFF + i, 0x40);     // byte program
            fl_wr(sel, ESCD_OFF + i, val);
            status = fl_poll(sel, ESCD_OFF, 0x80, 0x80, 256);
            ok = (status & 0xb8) == 0x80;
        }
        fl_wr(sel, ESCD_OFF, 0x50);
        fl_wr(sel, ESCD_OFF, 0xff);             // read array
    } else {
        for (page = 0; ok && page < size; page += 128) {   // 128-byte pages, erased by the chip
            fl_jedec(sel, 0xa0);
            for (i = page; i < page + 128 && i < size; i++)
                fl_wr(sel, ESCD_OFF + i, escd_src(seg, off, i));
            ok = escd_same(sel, seg, off, page, i);
        }
    }
    fl_enable(saved, 1);
    return ok && escd_same(sel, seg, off, 0, size) ? 0 : PNP_HARDWARE_ERROR;
}

// Functions 42h (read) and 43h (write) with the caller's buffer at seg:off.
static u16
escd_rw(int write, u16 sel, u16 seg, u16 off)
{
    u16 size, i;
    if (write) {
        size = GET_FARVAR(seg, *(u16 *)(u32)off);
        u32 sig = GET_FARVAR(seg, *(u32 *)(u32)(u16)(off + 2));
        if (size < 14 || size > ESCD_SIZE || sig != ESCD_SIG
            || escd_sum(seg, off, size) != GET_FARVAR(seg, *(u16 *)(u32)(u16)(off + size - 2)))
            return PNP_BAD_PARAMETER;               // also its checksum: the last word
        return escd_write(sel, seg, off, size);
    }
    size = escd_check(sel);
    if (!size)
        return PNP_HARDWARE_ERROR;
    for (i = 0; i < size; i++)
        SET_FARVAR(seg, *(u8 *)(u32)(u16)(off + i), fl_rd(sel, ESCD_OFF + i));
    return 0;
}

u32 VISIBLE32FLAT
escd_rw32(int write, u32 segoff)
{
    return escd_rw(write, 0, segoff >> 16, segoff);
}


/****************************************************************
 * POST: device nodes and the ESCD
 ****************************************************************/

static u16 NodeLen, NodeStart, EscdLen, BoardPos, FuncPos, LastEntry;
static u8 LastKind, CardCsn, CardFuncs, CardId[8];

static void
nput(u8 val)
{
    if (NodeLen < sizeof(PnpNodes))
        PnpNodes[NodeLen] = val;
    NodeLen++;
}

static void
nput16(u16 val)
{
    nput(val);
    nput(val >> 8);
}

static void
nput32(u32 val)
{
    nput16(val);
    nput16(val >> 16);
}

static void
put(u8 val)
{
    Escd[EscdLen++] = val;
}

static void
put16(u16 val)
{
    put(val);
    put(val >> 8);
}

static void
put32(u32 val)
{
    put16(val);
    put16(val >> 16);
}

static void
board_begin(u8 slot, const u8 *id)
{
    BoardPos = EscdLen;
    put16(0);
    put(slot);
    put(0);
    put32(id ? *(u32 *)id : 0);                 // compressed EISA ID
    put16(0x4040);                              // ID not readable, no CFG file
    put16(0);                                   // CFG revision
}

static void
board_end(void)
{
    u16 size = EscdLen + 4 - BoardPos, sum = 0, i;
    put16(0);                                   // no more functions
    *(u16 *)&Escd[BoardPos] = size;
    for (i = BoardPos; i < EscdLen; i++)
        sum += Escd[i];
    put16(-sum);
    Escd[8]++;                                  // board count
}

// An EISA function (one selection) whose entries follow; info 80h: the BIOS may reconfigure it.
static void
func_begin(u8 info)
{
    FuncPos = EscdLen;
    put16(0);
    put16(1);                                   // selection 0 of 1
    put(info);
    LastKind = 0;
}

static void
func_end(void)
{
    Escd[FuncPos] = EscdLen - FuncPos - 2;
}

// Entry of a kind (function info bit): the previous one of the same kind gets its "more" bit.
static void
entry(u8 kind, u8 first)
{
    if (LastKind == kind)
        Escd[LastEntry] |= 0x80;
    Escd[FuncPos + 4] |= kind;
    LastKind = kind;
    LastEntry = EscdLen;
    put(first);
}

static void
escd_io(u16 base, u16 len)
{
    while (len) {
        u16 n = len > 32 ? 32 : len;
        entry(0x10, n - 1);
        put16(base);
        base += n;
        len -= n;
    }
}

// System device node 'id' (PNPxxxx; static: it cannot be disabled or configured) with its function in
// the ESCD system board record; its resources follow.
static void
node_begin(u16 id, u32 type)
{
    NodeStart = NodeLen;
    nput16(0);
    nput(PnpNodeCount);
    nput32(0xd041 | (id >> 8) << 16 | (u32)(id & 0xff) << 24);
    nput16(type >> 8);
    nput(type);
    nput16(0x0003);
    func_begin(0);
}

static void
node_end(void)
{
    u16 size = NodeLen - NodeStart + 6;
    nput16(0x0079);                             // ends allocated, possible resources, compatible IDs
    nput16(0x0079);
    nput16(0x0079);
    func_end();
    if (NodeLen > sizeof(PnpNodes)) {
        NodeLen = NodeStart;
        return;
    }
    *(u16 *)&PnpNodes[NodeStart] = size;
    if (size > PnpNodeMax)
        PnpNodeMax = size;
    PnpNodeCount++;
}

static void
res_irq(u8 irq)
{
    nput(0x22);
    nput16(1 << irq);
    entry(0x04, irq);
    put(0);
}

static void
res_dma(u8 dma)
{
    nput(0x2a);
    nput16(1 << dma);
    entry(0x08, dma);
    put(0);
}

static void
res_io(u16 base, u8 len)
{
    nput(0x47);                                 // 16-bit decode, fixed base
    nput(1);
    nput16(base);
    nput16(base);
    nput(1);
    nput(len);
    escd_io(base, len);
}

// Memory range; ROM if 'rom'. The ESCD only describes the ISA memory space.
static void
res_mem(u32 base, u32 len, int rom)
{
    nput(0x86);
    nput16(9);
    nput(rom ? 0x40 : 0x01);
    nput32(base);
    nput32(len);
    if (base + len > 0x1000000)
        return;
    entry(0x02, !rom);
    put(0x08);                                  // 32-bit decode
    put16(base >> 8);
    put(base >> 24);
    put16(len >> 10);
}

// The board's devices: PNPxxxx (2 bytes), type code (3), IRQ, DMA (0xff: none), present if bit 'when' of
// pnp_finish's 'board' is set (0xff: always), I/O ranges (count, then base and length each).
#define IO(base, len) (base) & 0xff, (base) >> 8, (len)
static const u8 BoardDevs[] = {
    0x00, 0x00, 8, 0, 1, 2, 0xff, 0xff, 2, IO(0x20, 2), IO(0xa0, 2),                     // PIC
    0x02, 0x00, 8, 1, 1, 0xff, 4, 0xff, 6, IO(0x00, 16), IO(0x81, 3), IO(0x87, 1), IO(0x89, 3),
                                           IO(0x8f, 1), IO(0xc0, 32),                     // DMA
    0x01, 0x00, 8, 2, 1, 0, 0xff, 0xff, 1, IO(0x40, 4),                                   // timer
    0x0b, 0x00, 8, 3, 1, 8, 0xff, 0xff, 1, IO(0x70, 2),                                   // RTC
    0x03, 0x03, 9, 0, 0, 1, 0xff, 0xff, 2, IO(0x60, 1), IO(0x64, 1),                      // keyboard controller
    0x0c, 0x04, 11, 0x80, 0, 13, 0xff, 0xff, 1, IO(0xf0, 16),                             // math coprocessor
    0x08, 0x00, 4, 1, 0, 0xff, 0xff, 0xff, 1, IO(0x61, 1),                                // speaker
    0x0a, 0x03, 6, 0, 0, 0xff, 0xff, 0xff, 1, IO(0xcf8, 8),                               // PCI bus
    0x0c, 0x02, 8, 0x80, 0, 0xff, 0xff, 0xff, 4, IO(0x80, 1), IO(0x92, 1), IO(0x108, 2), IO(0x4d0, 2),
                                                // motherboard: POST code, port 92h, super I/O index, ELCR
    0x07, 0x00, 1, 2, 0, 6, 2, 0, 2, IO(0x3f2, 4), IO(0x3f7, 1),                          // floppy controller
    0x06, 0x00, 1, 1, 0, 14, 0xff, 1, 2, IO(0x1f0, 8), IO(0x3f6, 1),                      // IDE
    0x06, 0x00, 1, 1, 0, 15, 0xff, 2, 2, IO(0x170, 8), IO(0x376, 1),
};

// The system board record: a node and a function per device present ('board': bits 0-2 floppy, IDE
// primary and secondary controllers on, bits 3-4 parallel port mode SPP, EPP, ECP).
static void
escd_board(u8 board)
{
    const u8 *d;
    int i, mode = board >> 3;
    board_begin(0, NULL);
    for (d = BoardDevs; d < BoardDevs + sizeof(BoardDevs); d += 9 + d[8] * 3) {
        if (d[7] != 0xff && !(board & 1 << d[7]))
            continue;
        node_begin(d[0] << 8 | d[1], d[2] << 16 | d[3] << 8 | d[4]);
        if (d[5] != 0xff)
            res_irq(d[5]);
        if (d[6] != 0xff)
            res_dma(d[6]);
        for (i = 0; i < d[8]; i++)
            res_io(d[9 + i * 3] | d[10 + i * 3] << 8, d[11 + i * 3]);
        node_end();
    }
    if (GET_BDA(equipment_list_flags) & 0x04) {
        node_begin(0x0f13, 0x090200);           // PS/2 mouse
        res_irq(12);
        node_end();
    }
    for (i = 0; i < 4; i++) {
        u16 port = GET_BDA(port_com[i]);
        if (!port)
            continue;
        node_begin(0x0501, 0x070002);           // 16550 serial port
        res_irq(port & 0x100 ? 4 : 3);
        res_io(port, 8);
        node_end();
    }
    u16 lpt = GET_BDA(port_lpt[0]);
    if (lpt) {
        node_begin(mode == 2 ? 0x0401 : 0x0400, 0x070100 | mode);   // parallel port
        res_irq(lpt == 0x278 ? 5 : 7);
        if (mode == 2)
            res_dma(3);
        res_io(lpt, mode ? 8 : 4);
        if (mode == 2)
            res_io(lpt + 0x400, 3);
        node_end();
    }
    node_begin(0x0c01, 0x050000);               // system board memory
    res_mem(0, 0xa0000, 0);
    res_mem(0xe0000, 0x20000, 1);
    res_mem(0x100000, LegacyRamSize - 0x100000, 0);
    res_mem(0xfffe0000, 0x20000, 1);
    node_end();
    board_end();
}

// The ESCD being built, with its header; 0 if there is no memory for it.
static int
escd_start(void)
{
    if (Escd)
        return 1;
    Escd = malloc_tmp(ESCD_SIZE);
    if (!Escd) {
        warn_noalloc();
        return 0;
    }
    EscdLen = 0;
    put32(ESCD_SIG << 16);
    put32(ESCD_SIG >> 16 | 0x0201 << 16);       // version 2.1
    put32(0);                                   // boards
    return 1;
}

// Ends the board record of the current ISA PnP card: the PnP ISA freeform function, with the functions
// the BIOS may reconfigure (all), the vendor ID and serial number.
static void
card_end(void)
{
    int i;
    if (!CardCsn)
        return;
    func_begin(0xc0);
    put(24);
    put32(ESCD_SIG);
    put32(0x10 << 16 | 0x0201);
    put32((1 << CardFuncs) - 1);
    put32(0);
    for (i = 0; i < 8; i++)
        put(CardId[i]);
    func_end();
    board_end();
    CardCsn = CardFuncs = 0;
}

// A logical device of ISA PnP card csn (id: its vendor ID and serial number) that the POST activated: a
// function in the card's board record (slot = CSN).
void
pnp_isa_ldev(int csn, const u8 *id, const u16 *io, const u8 *iolen, const u8 *irq, const u8 *dma)
{
    int i;
    if (!CONFIG_PNPBIOS || !escd_start() || EscdLen > ESCD_SIZE / 2 || CardFuncs == 16)
        return;
    if (csn != CardCsn) {
        card_end();
        board_begin(csn, id);
        memcpy(CardId, id, sizeof(CardId));
        CardCsn = csn;
    }
    func_begin(0x80);
    for (i = 0; i < 2; i++)
        if (irq[i]) {
            entry(0x04, irq[i]);
            put(0);
        }
    for (i = 0; i < 2; i++)
        if (dma[i] != 4) {
            entry(0x08, dma[i]);
            put(0);
        }
    for (i = 0; i < 8; i++)
        if (io[i])
            escd_io(io[i], iolen[i] ?: 1);
    func_end();
    CardFuncs++;
}

// POST, after the devices: the nodes and the ESCD (ISA PnP cards, system board), written to the flash if
// it differs from the one there.
void
pnp_finish(int csns, u16 readport, u8 board)
{
    u16 size;
    if (!CONFIG_PNPBIOS)
        return;
    PnpCsns = csns;
    PnpReadPort = csns ? readport : 0;
    FlashChip = fl_detect();
    if (!escd_start())
        return;
    card_end();
    escd_board(board);
    size = escd_check(0);
    *(u16 *)Escd = EscdLen + 2;
    put16(escd_sum(0, 0, EscdLen + 2));
    if (size != EscdLen || !escd_same(0, 0, 0, 0, size)) {
        u16 ret = escd_write(0, 0, 0, EscdLen);
        dprintf(1, "ESCD written: %x (flash chip %d)\n", ret, FlashChip);
    }
    free(Escd);
    Escd = NULL;
}


/****************************************************************
 * Runtime entry
 ****************************************************************/

static void
far_wr16(u16 seg, u16 off, u16 val)
{
    SET_FARVAR(seg, *(u16 *)(u32)off, val);
}

// Functions 01h (get) and 02h (set) of a system device node.
static u16
handle_pnp_node(u16 *args, int set)
{
    u16 nseg = args[2], noff = args[1], control = args[set ? 4 : 5], pos = 0, i;
    u8 node = set ? args[1] : GET_FARVAR(nseg, *(u8 *)(u32)noff), count = GET_GLOBAL(PnpNodeCount);
    if (node >= count)
        return PNP_INVALID_HANDLE;
    if (control != 1 && control != 2)           // now or for the next boot, one of them
        return PNP_BAD_PARAMETER;
    for (i = 0; i < node; i++)
        pos += GET_GLOBAL(PnpNodes[pos]);
    if (set)                                    // only nodes that can be configured
        return GET_GLOBAL(PnpNodes[pos + 10]) & 0x02 ? PNP_BAD_PARAMETER : 0;
    u16 bseg = args[4], boff = args[3], size = GET_GLOBAL(PnpNodes[pos]);
    for (i = 0; i < size; i++)
        SET_FARVAR(bseg, *(u8 *)(u32)(u16)(boff + i), GET_GLOBAL(PnpNodes[pos + i]));
    SET_FARVAR(nseg, *(u8 *)(u32)noff, node + 1 < count ? node + 1 : 0xff);
    return 0;
}

// Arguments on the caller's stack (words; far pointers: offset, segment or selector); 'prot': called
// through the 16-bit protected mode entry.
u16 VISIBLE16
handle_pnp(u16 *args, int prot)
{
    if (! CONFIG_PNPBIOS)
        return FUNCTION_NOT_SUPPORTED;

    u16 arg1 = args[0];
    dprintf(DEBUG_HDL_pnp, "pnp call arg1=%x\n", arg1);

    switch (arg1) {
    case 0x00:                                  // number of system device nodes, size of the largest
        SET_FARVAR(args[2], *(u8 *)(u32)args[1], GET_GLOBAL(PnpNodeCount));
        far_wr16(args[4], args[3], GET_GLOBAL(PnpNodeMax));
        return 0;
    case 0x01 ... 0x02:
        return handle_pnp_node(args, arg1 == 2);
    case 0x03 ... 0x08:                         // events, docking, statically allocated devices
        return FUNCTION_NOT_SUPPORTED;
    case 0x09 ... 0x0a:
        return PNP_USE_ESCD_SUPPORT;
    case 0x0b:                                  // APM ID table: empty
        far_wr16(args[2], args[1], 0);
        return 0;
    case 0x40:                                  // ISA PnP configuration: revision, CSNs, read port
        far_wr16(args[2], args[1], 1 | GET_GLOBAL(PnpCsns) << 8);
        far_wr16(args[2], args[1] + 2, GET_GLOBAL(PnpReadPort));
        far_wr16(args[2], args[1] + 4, 0);
        return 0;
    case 0x41:                                  // ESCD: minimum write size, size, NV storage base
        far_wr16(args[2], args[1], ESCD_SIZE);
        far_wr16(args[4], args[3], ESCD_SIZE);
        far_wr16(args[6], args[5], (u16)PNP_FLASH_BASE);
        far_wr16(args[6], args[5] + 2, PNP_FLASH_BASE >> 16);
        return 0;
    case 0x42 ... 0x43:                         // ESCD read, write
        if (!args[2])
            return PNP_BAD_PARAMETER;
        if (prot)
            return arg1 & 1 ? FUNCTION_NOT_SUPPORTED : escd_rw(0, args[3], args[2], args[1]);
        return call32_params(escd_rw32, arg1 & 1, args[2] << 16 | args[1], 0, PNP_HARDWARE_ERROR);
    case 0x60:                                  // BBS version 1.01
        far_wr16(args[2], args[1], 0x0101);
        return 0;
    default:
        return PNP_UNKNOWN_FUNCTION;
    }
}

u16
get_pnp_offset(void)
{
    if (! CONFIG_PNPBIOS)
        return (u32)pnp_string + 1 - BUILD_BIOS_ADDR;
    return (u32)&PNPHEADER - BUILD_BIOS_ADDR;
}

// romlayout.S
extern void entry_pnp_real(void);
extern void entry_pnp_prot(void);

void
pnp_init(void)
{
    if (! CONFIG_PNPBIOS)
        return;

    dprintf(3, "init PNPBIOS table\n");

    PNPHEADER.real_ip = (u32)entry_pnp_real - BUILD_BIOS_ADDR;
    PNPHEADER.prot_ip = (u32)entry_pnp_prot - BUILD_BIOS_ADDR;
    PNPHEADER.checksum -= checksum(&PNPHEADER, sizeof(PNPHEADER));
}
