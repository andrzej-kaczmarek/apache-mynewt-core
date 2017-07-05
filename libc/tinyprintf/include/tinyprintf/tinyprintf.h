#include <stdio.h>
#include <inttypes.h>

#ifndef _TINYPRINTF_H
#define _TINYPRINTF_H

int tfp_vfprintf(FILE *f, const char *fmt, va_list va);

int tfp_fprintf(FILE *f, const char *fmt, ...);

int tfp_printf(const char *fmt, ...);

int tfp_vsnprintf(char *str, size_t size, const char *fmt, va_list va);

int tfp_snprintf(char *str, size_t size, const char *fmt, ...);

#endif
