/*
 * newBIOS CMOS layout, shared by SeaBIOS (setup utility, POST options) and coreboot (chipset options).
 * build.sh copies this file into both trees. One nibble per setting: index i is in byte
 * NB_CMOS_BASE + i / 2, low nibble for even i.
 */
#ifndef NEWBIOS_CMOS_H
#define NEWBIOS_CMOS_H

#define NB_CMOS_BASE      0x40
#define NB_CMOS_HDD       0x62   /* 4 x 5 bytes per IDE drive: type, user cylinders (16-bit), heads, sectors */
#define NB_CMOS_PASSWORD  0x78   /* 16-bit hash, 0 = no password */
#define NB_CMOS_SIG       0x7b   /* NB_SIG_VALUE when the settings below are valid */
#define NB_CMOS_SUM       0x7c   /* big-endian 16-bit sum of NB_CMOS_BASE..NB_CMOS_SIG */
#define NB_CMOS_BOOTFAIL  0x7e   /* POSTs started (coreboot) since the last completed one (SeaBIOS) */
#define NB_CMOS_CPUSIG    0x7f   /* CPU family << 4 | model from DX at reset (bootblock), for CPUs without CPUID */
#define NB_SIG_VALUE      0xa6   /* layout version: change when indexes or meanings change */
#define NB_LAYOUT_COUNT   65     /* NB_COUNT of this version (setup.c checks it) */
#define NB_BOOTFAIL_MAX   2      /* coreboot falls back to chipset defaults after this many failures */
#define NB_HDD_NONE       1      /* drive types: 0 auto, 1 none, 2-47 type 1-46 (16: reserved 15), 48 user */
#define NB_HDD_USER       48

enum nb_setting {
	/* Standard CMOS */
	NB_FLOPPY_A, NB_FLOPPY_B,               /* 0 none, 1 360K, 2 1.2M, 3 720K, 4 1.44M, 5 2.88M */
	NB_IDE_MODE0, NB_IDE_MODE1, NB_IDE_MODE2, NB_IDE_MODE3,   /* PM, PS, SM, SS: 0 auto, 1 normal, 2 LBA, 3 large */
	NB_HALT_ON,                             /* 0 all errors, 1 none, 2 all but keyboard, 3 diskette, 4 disk/key */
	/* BIOS features */
	NB_L1_CACHE, NB_L2_CACHE,               /* 0 disabled, 1 enabled */
	NB_BOOT1, NB_BOOT2, NB_BOOT3, NB_BOOT4, /* 1 floppy, 2 hard disk, 3 CD-ROM, 4 network */
	NB_BOOT_DELAY,                          /* setup prompt: 0 1 s, 1 2 s, 2 3 s, 3 5 s */
	NB_SWAP_FLOPPY, NB_NUMLOCK,
	NB_TYPEMATIC, NB_TYPEMATIC_RATE, NB_TYPEMATIC_DELAY,  /* rate 6..30 chars/s, delay 250..1000 ms */
	NB_SECURITY,                            /* 0 setup, 1 system */
	NB_QUICK_POST,                          /* 1: memory test reads one dword per KiB */
	NB_IDE_BLOCK,                           /* 1: READ/WRITE MULTIPLE (IDE HDD Block Mode) */
	NB_SHADOW_C8, NB_SHADOW_CC, NB_SHADOW_D0, NB_SHADOW_D4, NB_SHADOW_D8, NB_SHADOW_DC,
	/* Chipset features (coreboot, only when NB_CHIPSET_AUTO is 0) */
	NB_CHIPSET_AUTO,
	NB_DRAM_READ_WS, NB_DRAM_WRITE_WS,      /* 3, 2, 1, 0 WS */
	NB_L2_WS,                               /* 3-2-2-2, 3-1-1-1, 2-2-2-2, 2-1-1-1 */
	NB_KBC_CLOCK, NB_EDO, NB_EARLY_CACHE_WRITE, NB_SYSBIOS_CACHE, NB_VIDBIOS_CACHE,
	NB_HOST_PCI_POST_WS,                    /* 0: 1 WS, 1: 0 WS */
	NB_HOST_PCI_BURST, NB_PCI_BUS_PARK, NB_PCI_POSTED_WRITE, NB_SLOW_REFRESH,
	NB_HOST_PCI_CLOCK, NB_BURST_COPYBACK, NB_PREEMPT_PCI, NB_DEVSEL,
	/* Power management */
	NB_APM, NB_HDD_STANDBY,                 /* standby: 0 disabled, n minutes */
	/* PnP/PCI */
	NB_PCI_IRQ_A, NB_PCI_IRQ_B, NB_PCI_IRQ_C, NB_PCI_IRQ_D,   /* 0 auto, else IRQ number */
	NB_IDE_PIO0, NB_IDE_PIO1, NB_IDE_PIO2, NB_IDE_PIO3,       /* 0 auto, n = PIO mode n-1 */
	NB_IDE_32BIT,
	NB_ISAPNP,                              /* 1: configure ISA PnP cards at POST (PnP BIOS Auto-Config) */
	/* Integrated peripherals */
	NB_IDE_PRIMARY, NB_IDE_SECONDARY, NB_FDC,
	NB_COM1,                                /* 0 disabled, 1 3F8/IRQ4, 2 3E8/IRQ4 */
	NB_COM2,                                /* 0 disabled, 1 2F8/IRQ3, 2 2E8/IRQ3 */
	NB_LPT,                                 /* 0 disabled, 1 378/IRQ7, 2 278/IRQ5 */
	NB_LPT_MODE,                            /* 0 SPP, 1 EPP, 2 ECP */
	NB_COUNT
};

#define NB_CMOS_GET(read, i)  (((read)(NB_CMOS_BASE + (i) / 2) >> ((i) % 2 * 4)) & 0x0f)

#endif
