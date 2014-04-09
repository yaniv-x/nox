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

#ifndef _H_UTILS
#define _H_UTILS

#include <exception>
#include <string>

#include "common.h"

#define ALIGN(a, b) (((a) + ((b) - 1)) & ~((b) - 1))

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define OFFSETOF(type, member) ((unsigned long)&((type *)0)->member)
#define CONTAINEROF(ptr, type, member) ((type *)((uint8_t *)(ptr) - OFFSETOF(type, member)))

class AutoFD {
public:
    AutoFD() : _fd (-1) {}
    AutoFD(int fd) : _fd (fd) {}
    ~AutoFD() { close();}

    int get() { return _fd;}
    void reset(int fd) { close(); _fd = fd; }
    bool is_valid() { return _fd != -1;}
    int release() { int fd = _fd; _fd = -1; return fd; }

private:
    void close() { if (_fd != -1) ::close(_fd);}

private:
    int _fd;
};

template<class T>
class AutoArray {
public:
    AutoArray() : _array (NULL) {}
    AutoArray(T* array) : _array (array) {}
    ~AutoArray() { delete[] _array;}

    void set(T* array) { delete[] _array; _array = array;}
    T* get() {return _array;}
    T* release() {T* tmp = _array; _array = NULL; return tmp; }
    T& operator [] (int i) {return _array[i];}

private:
    T* _array;
};

template<class T>
class AutoRef {
public:
    AutoRef() : _obj (NULL) {}
    AutoRef(T* obj) : _obj (obj) {}
    ~AutoRef() {  if (_obj) _obj->unref();}

    T* get() {return _obj;}
    T* operator -> () {return _obj;}
    void reset(T* obj) { if (_obj) _obj->unref(); _obj = obj;}
    T* release() {T* tmp = _obj; _obj = NULL; return tmp;}

private:
    T* _obj;
};


void sprintf(std::string& str, const char* format, ...);
void wsprintf(std::wstring& str, const wchar_t* format, ...);
bool str_to_ulong(const char *str, unsigned long& num, int base = 0);
bool str_to_long(const char *str, long& num, int base = 0);
bool str_to_uint(const char *str, uint& num, int base = 0);
bool str_to_int(const char *str, int& num, int base = 0);

typedef uint64_t nox_time_t;

inline nox_time_t get_monolitic_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC , &ts);
    return nox_time_t(ts.tv_nsec) + nox_time_t(ts.tv_sec) * 1000 * 1000 * 1000;
}

struct OOMException {};

static inline uint to_bcd(uint val)
{
    uint ret = 0;
    uint shift = 0;

    for (; val; shift += 4) {
        ret |= (val % 10) << shift;
        val = val / 10;
    }

    return ret;
}

static inline uint from_bcd(uint val)
{
    uint ret = 0;
    uint factor = 1;

    for (; val; factor *= 10) {
        ret += (val & 0x0f) * factor;
        val = val >> 4;
    }

    return ret;
}

static inline void cpuid(uint32_t function, uint32_t& eax, uint32_t& ebx, uint32_t& ecx,
                         uint32_t& edx)
{
    asm (
        "cpuid;"
        : "=a" (eax), "=b" (ebx), "=c" (ecx), "=d" (edx)
        : "a" (function)
    );
}

static inline uint64_t rdtsc()
{
    uint32_t vec[2];
    uint64_t* ret = (uint64_t*)vec;

    asm (
        "rdtsc;"
        : "=a" (vec[0]), "=d" (vec[1])
    );

    return *ret;
}


//todo: use bit scan revers
template<class T>
static inline int find_msb(T val)
{
    int i;

    for (i = sizeof(T) * 8 - 1; i >=0; i--) {
        if (val & (T(1) << i)) {
            return i;
        }
    }

    return i;
}

inline char* copy_cstr(const char* str)
{
    if (str == NULL) {
        return NULL;
    }

    char* ret = new char[strlen(str) + 1];
    strcpy(ret, str);

    return ret;
}

void read_all(int fd, off_t from, void* in_dest, size_t size);
void write_all(int fd, off_t to, const void* in_src, uint size);

bool is_amd_processor();

int8_t checksum8(const void *start, uint size);

#endif

