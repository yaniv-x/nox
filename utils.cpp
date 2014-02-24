/*
    Copyright (c) 2013-2014 Yaniv Kamay,
    All rights reserved.

    Source code is provided for evaluation purposes only. Modification or use in
    source code for any other purpose is prohibited.

    Binary code (i.e. the binary form of source code form) is allowed to use for
    evaluation purposes only. Modification or use in binary code for any other
    purpose is prohibited.

    Redistribution, in source form or in binary form, with or without modification,
    are not permitted.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTOR BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <map>

#include "utils.h"


static void svprintf(std::string& str, const char* format, va_list ap)
{
    int buf_size = 256;

    for (;;) {
        AutoArray<char> buf(new char[buf_size]);
        va_list args;

        va_copy(args, ap);

        int r = vsnprintf(buf.get(), buf_size, format, args);

        va_end(args);

        if (r == -1) {
            THROW("vsnprintf failed");
        }

        if (r + 1 > buf_size) {
            buf_size *= 2;
            continue;
        }

        str = buf.get();
        return;
    }
}


static void wsvprintf(std::wstring& str, const wchar_t* format, va_list ap)
{
    int buf_size = 256;

    for (;;) {
        AutoArray<wchar_t> buf(new wchar_t[buf_size]);
        va_list args;

        va_copy(args, ap);

        int r = vswprintf(buf.get(), buf_size, format, args);

        va_end(args);

        if (r != -1) {
            ASSERT(r < buf_size);
            str = buf.get();
            return;
        }

        buf_size *= 2;
    }
}


void sprintf(std::string& str, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    svprintf(str, format, ap);
    va_end(ap);
}


void wsprintf(std::wstring& str, const wchar_t* format, ...)
{
    va_list ap;
    va_start(ap, format);
    wsvprintf(str, format, ap);
    va_end(ap);
}


bool str_to_ulong(const char* str, unsigned long& num, int base)
{
    char* end;

    errno = 0;
    num = strtoul(str, &end, base);

    if (errno || end != str + strlen(str) || end == str) {
        return false;
    }

    return true;
}


bool str_to_uint(const char *str, uint& num, int base)
{
    unsigned long tmp;

    if (!str_to_ulong(str, tmp, base) || tmp > UINT_MAX) {
        return false;
    }

    num = tmp;
    return true;
}


bool str_to_long(const char* str, long& num, int base)
{
    char* end;

    errno = 0;
    num = strtol(str, &end, base);

    if (errno || end != str + strlen(str) || end == str) {
        return false;
    }

    return true;
}



bool str_to_int(const char *str, int& num, int base)
{
    long tmp;

    if (!str_to_long(str, tmp, base) || tmp > INT_MAX || tmp < INT_MIN) {
        return false;
    }

    num = tmp;
    return true;
}


void read_all(int fd, off_t from, void* in_dest, size_t size)
{
    uint8_t* dest = (uint8_t*)in_dest;

    if (lseek(fd, from, SEEK_SET) != from) {
        THROW("seek failed");
    }

    while (size) {
        ssize_t n = read(fd, dest, size);
        if (n <= 0) {
             if (n == 0 || errno != EINTR) {
                 THROW("read failed");
             }
             continue;
        }

        size -= n;
        dest += n;
    }
}


void write_all(int fd, off_t to, const void* in_src, uint size)
{
    const uint8_t* src = (uint8_t*)in_src;

    if (lseek(fd, to, SEEK_SET) != to) {
        THROW("seek failed");
    }

    while (size) {
        ssize_t n = write(fd, src, size);
        if (n == -1) {
             if (errno != EINTR) {
                 THROW("write failed");
             }
             continue;
        }

        size -= n;
        src += n;
    }
}


static void append_four_bytes(std::string& str, uint32_t four_bytes)
{
    uint64_t tmp = four_bytes;
    char* tmp_str = (char*)&tmp;
    tmp_str[4] = 0;

    str += tmp_str;
}


bool is_amd_proccesor()
{
    uint32_t eax;
    uint32_t ebx;
    uint32_t ecx;
    uint32_t edx;

    cpuid(0, eax, ebx, ecx, edx);

    std::string vendor;
    append_four_bytes(vendor, ebx);
    append_four_bytes(vendor, edx);
    append_four_bytes(vendor, ecx);

    return vendor == "AuthenticAMD";
}


int8_t checksum8(const void *start, uint size)
{
    uint8_t res = 0;
    uint8_t *now = (uint8_t*)start;
    uint8_t *end = now + size;

    for (; now < end; now++) {
        res += *now;
    }

    return -res;
}

