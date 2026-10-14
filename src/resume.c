// Code for handling calls to "post" that are resume related.
//
// Copyright (C) 2008,2009  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "bregs.h" // struct bregs
#include "config.h" // CONFIG_*
#include "farptr.h" // FLATPTR_TO_SEGOFF
#include "hw/pci.h" // pci_reboot
#include "hw/pci_ids.h" // PCI_BASE_CLASS_BRIDGE
#include "hw/pci_regs.h" // PCI_COMMAND
#include "hw/pic.h" // pic_eoi2
#include "hw/ps2port.h" // i8042_reboot
#include "hw/rtc.h" // rtc_read
#include "output.h" // dprintf
#include "stacks.h" // farcall16big
#include "std/bda.h" // struct bios_data_area_s
#include "string.h" // memset
#include "util.h" // dma_setup
#include "x86.h" // cr0_read
#include "tcgbios.h" // tpm_s3_resume
#include "fw/romfile_loader.h" // romfile_fw_cfg_resume

// Handler for post calls that look like a resume.
void VISIBLE16
handle_resume(void)
{
    ASSERT16();
    int status = rtc_read(CMOS_RESET_CODE);
    rtc_write(CMOS_RESET_CODE, 0);
    dprintf(1, "In resume (status=%d)\n", status);
    if (CONFIG_SETUP_MENU)      // the CPU reset set CR0.CD and NW: L1 cache as in setup
        cr0_write((cr0_read() & ~(CR0_CD | CR0_NW)) | (GET_GLOBAL(SetupL1Off) ? CR0_CD : 0));

    dma_setup();

    switch (status) {
    case 0x01 ... 0x04:
    case 0x06 ... 0x09:
        panic("Unimplemented shutdown status: %02x\n", status);

    case 0x05:
        // flush keyboard (issue EOI) and jump via 40h:0067h
        pic_eoi2();
        // NO BREAK
    case 0x0a:
#define BDA_JUMP (((struct bios_data_area_s *)0)->jump)
        // resume execution by jump via 40h:0067h
        asm volatile(
            "movw %w1, %%ds\n"
            "ljmpw *%0\n"
            : : "m"(BDA_JUMP), "r"(SEG_BDA)
            );
        break;

    case 0x0b:
        // resume execution via IRET via 40h:0067h
        asm volatile(
            "movw %w1, %%ds\n"
            "lssw %0, %%sp\n"
            "iretw\n"
            : : "m"(BDA_JUMP), "r"(SEG_BDA)
            );
        break;

    case 0x0c:
        // resume execution via RETF via 40h:0067h
        asm volatile(
            "movw %w1, %%ds\n"
            "lssw %0, %%sp\n"
            "lretw\n"
            : : "m"(BDA_JUMP), "r"(SEG_BDA)
            );
        break;

    default:
        break;
    }

    // Not a 16bit resume - do remaining checks in 32bit mode
    asm volatile(
        "movw %w1, %%ss\n"
        "movl %0, %%esp\n"
        "movl $_cfunc32flat_handle_resume32, %%edx\n"
        "jmp transition32\n"
        : : "i"(BUILD_S3RESUME_STACK_ADDR), "r"(0), "a"(status)
        );
}

// Handle an S3 resume event
static void
s3_resume(void)
{
    if (!CONFIG_S3_RESUME)
        return;

    u32 s3_resume_vector = find_resume_vector();
    if (!s3_resume_vector) {
        dprintf(1, "No resume vector set!\n");
        return;
    }

    pic_setup();
    smm_setup();
    smp_resume();

    pci_resume();

    /* resume TPM before we may measure option roms */
    tpm_s3_resume();
    s3_resume_vga();

    /* Replay any fw_cfg entries that go back to the host */
    romfile_fw_cfg_resume();

    make_bios_readonly();

    // Invoke the resume vector.
    struct bregs br;
    memset(&br, 0, sizeof(br));
    dprintf(1, "Jump to resume vector (%x)\n", s3_resume_vector);
    br.code = FLATPTR_TO_SEGOFF((void*)s3_resume_vector);
    farcall16big(&br);
}

// Bus masters off (USB schedules, network receive rings): a warm reset does not reset the cards, and they
// would keep writing to memory while the next POST sets up the DRAM and runs from the L1 cache (cache-as-RAM).
static void
pci_bus_masters_off(void)
{
    for (int bdf = 0; bdf < 0x100; bdf++)
        if (pci_config_readw(bdf, PCI_VENDOR_ID) != 0xffff
            && pci_config_readw(bdf, PCI_CLASS_DEVICE) >> 8 != PCI_BASE_CLASS_BRIDGE)
            pci_config_maskw(bdf, PCI_COMMAND, PCI_COMMAND_MASTER, 0);
}

// Attempt to invoke a hard-reboot.
static void
tryReboot(void)
{
    dprintf(1, "Attempting a hard reboot\n");
    pci_bus_masters_off();

    // Use a QEMU specific reboot on QEMU
    qemu_reboot();

    // Reboot using ACPI RESET_REG
    acpi_reboot();

    // Keyboard controller reset pulse, as the original BIOS (on the MB-8433UUD-A port 92h does not reset:
    // Ctrl+Alt+Del hung with it alone).
    i8042_reboot();

    // Try PCI 0xcf9 reboot
    pci_reboot();

    // PORT 92h reset: bit 0 resets on a 0 -> 1 transition
    u8 port92 = inb(0x92) & ~1;
    outb(port92, 0x92);
    outb(port92 | 1, 0x92);
    udelay(100);

    // Triple fault: an interrupt in 32-bit mode with the dummy IDT shuts the CPU down, which resets it
    asm volatile("int3");

    panic("Could not reboot");
}

void VISIBLE32FLAT
handle_resume32(int status)
{
    ASSERT32FLAT();
    dprintf(1, "In 32bit resume\n");

    if (status == 0xfe)
        s3_resume();

    // Must be a soft reboot - invoke a hard reboot.
    tryReboot();
}
