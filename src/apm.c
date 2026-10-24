// Basic support for apmbios callbacks.
//
// Copyright (C) 2008  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2005 Struan Bartlett
// Copyright (C) 2004 Fabrice Bellard
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_GLOBAL
#include "bregs.h" // struct bregs
#include "config.h" // CONFIG_*
#include "output.h" // dprintf
#include "stacks.h" // yield_toirq
#include "util.h" // apm_shutdown
#include "x86.h" // outb

u8 ApmEnabled VARFSEG = 1;

// APM installation check
static void
handle_155300(struct bregs *regs)
{
    regs->ah = 1; // APM major version
    regs->al = 2; // APM minor version
    regs->bh = 'P';
    regs->bl = 'M';
    // bit 0 : 16 bit interface supported
    // bit 1 : 32 bit interface supported
    regs->cx = 0x03;
    set_success(regs);
}

// APM real mode interface connect
static void
handle_155301(struct bregs *regs)
{
    set_success(regs);
}

// APM 16 bit protected mode interface connect
static void
handle_155302(struct bregs *regs)
{
    extern void entry_apm16(void);
    regs->bx = (u32)entry_apm16;
    regs->ax = SEG_BIOS; // 16 bit code segment base
    regs->si = 0xfff0;   // 16 bit code segment size
    regs->cx = SEG_BIOS; // data segment address
    regs->di = 0xfff0;   // data segment length
    set_success(regs);
}

// APM 32 bit protected mode interface connect
static void
handle_155303(struct bregs *regs)
{
    extern void entry_apm32(void);
    regs->ax = SEG_BIOS; // 32 bit code segment base
    regs->ebx = (u32)entry_apm32;
    regs->cx = SEG_BIOS; // 16 bit code segment base
    // 32 bit code segment size (low 16 bits)
    // 16 bit code segment size (high 16 bits)
    regs->esi = 0xfff0fff0;
    regs->dx = SEG_BIOS; // data segment address
    regs->di = 0xfff0; // data segment length
    set_success(regs);
}

// APM interface disconnect
static void
handle_155304(struct bregs *regs)
{
    set_success(regs);
}

// APM cpu idle
static void
handle_155305(struct bregs *regs)
{
    yield_toirq();
    set_success(regs);
}

// APM cpu busy
static void
handle_155306(struct bregs *regs)
{
    set_success(regs);
}

#if CONFIG_NEWBIOS_BOARD_APTIVA2168
// The Aptiva's power microcontroller at ECh (data) and EDh (bit 0: ready, it toggles), chip select from the FDC37C93x
// GP14 decoder; protocol as the IBM BIOS (runtime F000:5FB8-62BB, soft off 5405), command meanings inferred from it.
#define PMC_DATA   0xec
#define PMC_STATUS 0xed

// Refresh toggles of port 61h bit 4, 15 us each (IBM F000:5B0A); each wait given up after 100 reads
static void
refresh_wait(u32 toggles)
{
    while (toggles--) {
        u8 last = inb(0x61) & 0x10;
        int i;
        for (i = 0; i < 100 && (inb(0x61) & 0x10) == last; i++)
            ;
    }
}

static int
pmc_ready(void)
{
    u32 i;
    for (i = 0; i < 0x10000; i++)
        if (inb(PMC_STATUS) & 1)
            return 1;
    return 0;
}

// Command BL after the prefix BH if its bit 7 is set, each byte as nibbles with bit 4 set; the reply if BH bit 6
// (IBM F000:617F); -1 if the controller is not ready.
static int
pmc_send(u16 cmd)
{
    u8 bh = cmd >> 8, bl = cmd;
    if (bh & 0x80) {
        if (!pmc_ready())
            return -1;
        outb(bh | 0x10, PMC_DATA);
    }
    if (!pmc_ready())
        return -1;
    outb((bl >> 4) | 0x10, PMC_DATA);
    if (!pmc_ready())
        return -1;
    outb(bl | 0x10, PMC_DATA);
    if (!(bh & 0x40))
        return 0;
    return pmc_ready() ? inb(PMC_DATA) : -1;
}

// Ready, then a 0 among 8 reads (IBM F000:6110)
static int
pmc_idle(void)
{
    int i;
    for (i = 0; i < 1000 && !(inb(PMC_STATUS) & 1); i++)
        ;
    if (i == 1000)
        return 0;
    for (i = 0; i < 1000; i++) {
        u8 all = 1, j;
        for (j = 0; j < 8; j++)
            all &= inb(PMC_STATUS);
        if (!(all & 1))
            return 1;
    }
    return 0;
}

// IBM F000:5FB8: the handshake, retried 5 times, then after a reset pulse; 0Ah and 4Ah clear and set status 8Ah bit 0
static int
pmc_present(void)
{
    int i, ok = pmc_idle();
    for (i = 0; i < 5 && !ok; i++) {
        refresh_wait(0x8000);
        ok = pmc_idle();
    }
    if (!ok) {
        outb(1, PMC_STATUS);
        refresh_wait(0xea60);
        outb(0, PMC_STATUS);
        refresh_wait(0x7530);
        pmc_send(0x30);
        ok = pmc_idle();
    }
    return ok && pmc_send(0x0a) == 0 && pmc_send(0x408a) == 0 && pmc_send(0x4a) == 0 && pmc_send(0x408a) == 1;
}

// The IBM BIOS's init at POST (5FEA-602A, 6030-603A, 60E9-60FB, without its CMOS power history), then its soft off
// (5424-5436): no wake on ring or alarm (61D1: 15h, 04h), 0Eh, 4Dh, 4057h.
u16 PmcOff[] VARFSEG = {
    0x8d2e, 0x8e02, 0x6d, 0x8d13, 0, 0x15, 0x04, 0x40b0, 0x30, 0x4e, 0x8d02, 0x8eac, 0x15, 0x04, 0x0e, 0x4d, 0x4057
};

static void
pmc_off(void)
{
    if (!pmc_present())
        return;
    outb(0x80, 0x70);
    u8 sec = inb(0x71);
    u32 i;
    for (i = 0; i < ARRAY_SIZE(PmcOff); i++) {
        u16 cmd = GET_GLOBAL(PmcOff[i]);
        pmc_send(cmd ? cmd : 0x8e00 | (60 - (sec >> 4) * 10 - (sec & 0x0f)));   // seconds to the next minute
    }
    refresh_wait(2 * 0xfa00);
}
#endif

// Power off: the Aptiva's controller (if it answers), ACPI; then as the IBM BIOS (F000:5447-5461): CMOS 0Fh = 0,
// 8042 reset pulse.
void
apm_shutdown(void)
{
    irq_disable();
#if CONFIG_NEWBIOS_BOARD_APTIVA2168
    pmc_off();
    outb(0x8f, 0x70);
    outb(0, 0x71);
    outb(0xfe, 0x64);
#endif
    u16 pm1a_cnt = GET_GLOBAL(acpi_pm1a_cnt);
    if (pm1a_cnt)
        outw(0x2000, pm1a_cnt);

    for (;;)
        hlt();
}

// APM Set Power State
static void
handle_155307(struct bregs *regs)
{
    if (regs->bx != 1) {
        set_success(regs);
        return;
    }
    switch (regs->cx) {
    case 1:
        dprintf(1, "APM standby request\n");
        break;
    case 2:
        dprintf(1, "APM suspend request\n");
        break;
    case 3:
        apm_shutdown();
        break;
    }
    set_success(regs);
}

static void
handle_155308(struct bregs *regs)
{
    set_success(regs);
}

// Get Power Status
static void
handle_15530a(struct bregs *regs)
{
    regs->bh = 0x01; // on line
    regs->bl = 0xff; // unknown battery status
    regs->ch = 0x80; // no system battery
    regs->cl = 0xff; // unknown remaining time
    regs->dx = 0xffff; // unknown remaining time
    regs->si = 0x00; // zero battery
    set_success(regs);
}

#define RET_ENOEVENT 0x80

// Get PM Event
static void
handle_15530b(struct bregs *regs)
{
    set_code_invalid_silent(regs, RET_ENOEVENT);
}

// APM Driver Version
static void
handle_15530e(struct bregs *regs)
{
    regs->ah = 1;
    regs->al = 2;
    set_success(regs);
}

// APM Engage / Disengage
static void
handle_15530f(struct bregs *regs)
{
    set_success(regs);
}

// APM Get Capabilities
static void
handle_155310(struct bregs *regs)
{
    regs->bl = 0;
    regs->cx = 0;
    set_success(regs);
}

static void
handle_1553XX(struct bregs *regs)
{
    set_unimplemented(regs);
}

void
handle_1553(struct bregs *regs)
{
    if (! CONFIG_APMBIOS || !GET_GLOBAL(ApmEnabled)) {
        set_code_invalid(regs, RET_EUNSUPPORTED);
        return;
    }

    //debug_stub(regs);
    switch (regs->al) {
    case 0x00: handle_155300(regs); break;
    case 0x01: handle_155301(regs); break;
    case 0x02: handle_155302(regs); break;
    case 0x03: handle_155303(regs); break;
    case 0x04: handle_155304(regs); break;
    case 0x05: handle_155305(regs); break;
    case 0x06: handle_155306(regs); break;
    case 0x07: handle_155307(regs); break;
    case 0x08: handle_155308(regs); break;
    case 0x0a: handle_15530a(regs); break;
    case 0x0b: handle_15530b(regs); break;
    case 0x0e: handle_15530e(regs); break;
    case 0x0f: handle_15530f(regs); break;
    case 0x10: handle_155310(regs); break;
    default:   handle_1553XX(regs); break;
    }
}

void VISIBLE16 VISIBLE32SEG
handle_apm(struct bregs *regs)
{
    debug_enter(regs, DEBUG_HDL_apm);
    handle_1553(regs);
}
