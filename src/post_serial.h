/*
 * Each POST code also goes out on COM1 (3F8h, 115200 8N1) as "XX\r\n": a PC on a null-modem cable timestamps the
 * lines (tools/post_time.py), as the Am5x86 has no clock to time the POST itself. Shared by coreboot and SeaBIOS
 * (build.sh copies it into both trees); outb/inb (value, port) come from the includer.
 */
#ifndef NEWBIOS_POST_SERIAL_H
#define NEWBIOS_POST_SERIAL_H

#define POST_COM1 0x3f8

static inline void post_serial_wait(unsigned char lsr_bit)
{
	for (int n = 0; n < 10000 && !(inb(POST_COM1 + 5) & lsr_bit); n++)   /* no UART: bounded */
		;
}

static inline void post_serial(unsigned char code)
{
	static const char hex[] = "0123456789ABCDEF";
	const char line[4] = { hex[code >> 4], hex[code & 15], '\r', '\n' };

	post_serial_wait(0x40);                      /* all sent: the divisor can change */
	outb(0x83, POST_COM1 + 3);                   /* 8N1, divisor latch */
	outb(1, POST_COM1);                          /* 115200 */
	outb(0, POST_COM1 + 1);
	outb(0x03, POST_COM1 + 3);
	outb(0x03, POST_COM1 + 4);                   /* DTR, RTS */
	for (int i = 0; i < 4; i++) {
		post_serial_wait(0x20);
		outb(line[i], POST_COM1);
	}
}

#endif
