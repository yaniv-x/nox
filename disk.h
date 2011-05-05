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

#ifndef _H_DISK
#define _H_DISK

#include "common.h"
#include "non_copyable.h"


class ATADevice: public NonCopyable {
public:
    virtual bool is_ATAPI() = 0;
    virtual uint64_t get_size() = 0;
    virtual bool read(uint64_t sector, uint8_t* buf) = 0;
    virtual bool write(uint64_t sector, const uint8_t* buf) = 0;
};


class ATAPIDevice: public ATADevice {
public:
    ATAPIDevice(const char* file_name);

    virtual bool is_ATAPI() { return true;}

    virtual uint64_t get_size() { return _size;}
    virtual bool read(uint64_t sector, uint8_t* buf);
    virtual bool write(uint64_t sector, const uint8_t* buf) { THROW("");}

private:
    AutoFD _file;
    uint64_t _size;
};

class Disk: public ATADevice {
public:
    Disk(const char* file_name, uint read_cache = 0, uint write_cache = 0);
    virtual ~Disk();

    virtual bool is_ATAPI() { return false;}
    virtual uint64_t get_size() { return _size;}

    virtual bool read(uint64_t sector, uint8_t* buf);
    virtual bool write(uint64_t sector, const uint8_t* buf);

    //void enable_cache();
    //void disable_cache();
    //void set_queue_size();
    //void get_io_buf();
    //void put_io_buf();
private:
    AutoFD _file;
    uint64_t _size;

    typedef std::map<uint64_t, uint8_t*> SectorsMap;
    SectorsMap _sectors;
};


#endif

