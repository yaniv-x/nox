/*
    Copyright (c) 2013 Yaniv Kamay,
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
        int r = vsnprintf(buf.get(), buf_size, format, ap);
        if (r != -1) {
            str = buf.get();
            return;
        }
        buf_size *= 2;
    }
}


static void wsvprintf(std::wstring& str, const wchar_t* format, va_list ap)
{
    int buf_size = 256;
    for (;;) {
        AutoArray<wchar_t> buf(new wchar_t[buf_size]);
        int r = vswprintf(buf.get(), buf_size, format, ap);
        if (r != -1) {
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


bool str_to_uint32(const char *str, uint32_t& num)
{
    char *end;

    ASSERT(sizeof(long) == 4);
    num = strtol(str, &end, 0);

    if (errno || end != str + strlen(str)) {
        return false;
    }

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

