#ifndef __OUTPUT_H
#define __OUTPUT_H

#include <stdarg.h> // va_list
#include "config.h" // CONFIG_DEBUG_LEVEL
#include "types.h" // u32

// output.c
void debug_banner(void);
void panic(const char *fmt, ...)
    __attribute__ ((format (printf, 1, 2))) __noreturn;
void printf(const char *fmt, ...)
    __attribute__ ((format (printf, 1, 2)));
int vsnprintf(char *str, size_t size, const char *fmt, va_list args);
int snprintf(char *str, size_t size, const char *fmt, ...)
    __attribute__ ((format (printf, 3, 4)));
char * znprintf(size_t size, const char *fmt, ...)
    __attribute__ ((format (printf, 2, 3)));
void __dprintf(const char *fmt, ...)
    __attribute__ ((format (printf, 1, 2)));
struct bregs;
void __debug_enter(struct bregs *regs, const char *fname);
void __debug_isr(const char *fname);
// Warnings name their place by the caller's address and the source line, not the function's name: no strings in the
// debug ROM (out-debug/rom.o's symbols give the function; init code is moved by the "Relocating init" offset).
void __debug_stub(struct bregs *regs, int lineno);
void __warn_invalid(struct bregs *regs, int lineno);
void __warn_unimplemented(struct bregs *regs, int lineno);
void __warn_internalerror(int lineno);
void __warn_noalloc(int lineno);
void __warn_timeout(int lineno);
void __set_invalid(struct bregs *regs, int lineno);
void __set_unimplemented(struct bregs *regs, int lineno);
void __set_code_invalid(struct bregs *regs, u32 linecode);
void __set_code_unimplemented(struct bregs *regs, u32 linecode);
void hexdump(const void *d, int len);

#define dprintf(lvl, fmt, args...) do {                         \
        if (CONFIG_DEBUG_LEVEL && (lvl) <= CONFIG_DEBUG_LEVEL)  \
            __dprintf((fmt) , ##args );                         \
    } while (0)
#define debug_enter(regs, lvl) do {                     \
        if ((lvl) && (lvl) <= CONFIG_DEBUG_LEVEL)       \
            __debug_enter((regs), __func__);            \
    } while (0)
#define debug_isr(lvl) do {                             \
        if ((lvl) && (lvl) <= CONFIG_DEBUG_LEVEL)       \
            __debug_isr(__func__);                      \
    } while (0)
#define debug_stub(regs)                        \
    __debug_stub((regs), __LINE__)
#define warn_invalid(regs)                      \
    __warn_invalid((regs), __LINE__)
#define warn_unimplemented(regs)                        \
    __warn_unimplemented((regs), __LINE__)
#define warn_internalerror()                    \
    __warn_internalerror(__LINE__)
#define warn_noalloc()                          \
    __warn_noalloc(__LINE__)
#define warn_timeout()                          \
    __warn_timeout(__LINE__)
#define set_invalid(regs)                       \
    __set_invalid((regs), __LINE__)
#define set_code_invalid(regs, code)                                    \
    __set_code_invalid((regs), (code) | (__LINE__ << 8))
#define set_unimplemented(regs)                         \
    __set_unimplemented((regs), __LINE__)
#define set_code_unimplemented(regs, code)                              \
    __set_code_unimplemented((regs), (code) | (__LINE__ << 8))

#endif // output.h
