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

#ifndef _H_UTILS
#define _H_UTILS

#include <exception>
#include <string>

#include "common.h"

#define ALIGN(a, b) (((a) + ((b) - 1)) & ~((b) - 1))

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

class AutoFD {
public:
    AutoFD() : _fd (-1) {}
    AutoFD(int fd) : _fd (fd) {}
    ~AutoFD() { close();}

    int get() { return _fd;}
    void reset(int fd) { close(); _fd = fd; }
    bool is_valid() { return _fd != -1;}

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
    AutoRef(T* obj) : _obj (obj) {}
    ~AutoRef() { _obj->unref();}

    T* get() {return _obj;}
    T* operator -> () {return _obj;}

private:
    T* _obj;
};


void sprintf(std::string& str, const char* format, ...);
void wsprintf(std::wstring& str, const wchar_t* format, ...);
bool str_to_uint32(const char *str, uint32_t& num);

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

#endif

