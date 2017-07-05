/*
File: tinyprintf.c

Copyright (C) 2004  Kustaa Nyholm
Copyright (C) 2010  CJlano
Copyright (C) 2011  Petteri Aimonen

This file is dual-licensed. You can use either of these licenses:

1) GNU LGPL
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

2) BSD
Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Kustaa Nyholm or SpareTimeLabs nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/* This is a smaller implementation of printf-family of functions,
 * based on tinyprintf code by Kustaa Nyholm.
 * The formats supported by this implementation are:
 *     'd' 'u' 'c' 's' 'x' 'X' 'p'.
 * Zero padding and field width are also supported.
 * If the library is compiled with 'PRINTF_SUPPORT_LONG' defined then the
 * long specifier is also supported.
 * Otherwise it is ignored, so on 32 bit platforms there is no point to use
 * PRINTF_SUPPORT_LONG because int == long.
 */

#include <stdio.h>
#include "tinyprintf/tinyprintf.h"

/* NOTE: core tinyprintf code is moved to libc/tinyprintf */

int vfprintf(FILE *f, const char *fmt, va_list va)
{
    return tfp_vfprintf(f, fmt, va);
}

int fprintf(FILE *f, const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int rv = vfprintf(f, fmt, va);
    va_end(va);
    return rv;
}

int printf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int rv = vfprintf(stdout, fmt, va);
    va_end(va);
    return rv;
}

int vsnprintf(char *str, size_t size, const char *fmt, va_list va)
{
    return tfp_vsnprintf(str, size, fmt, va);
}

int snprintf(char *str, size_t size, const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int rv = vsnprintf(str, size, fmt, va);
    va_end(va);
    return rv;
}
