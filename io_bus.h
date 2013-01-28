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

#ifndef _H_IO_BUS
#define _H_IO_BUS

#include "vm_part.h"

class NoxVM;

class IORegion;

class IOBus: public VMPart {
public:
    IOBus(NoxVM& nox);
    ~IOBus();

    void read_byte(uint16_t port, uint8_t* dest, uint32_t n);
    void read_word(uint16_t port, uint16_t* dest, uint32_t n);
    void read_dword(uint16_t port, uint32_t* dest, uint32_t n);

    void write_byte(uint16_t port, uint8_t* src, uint32_t n);
    void write_word(uint16_t port, uint16_t* src, uint32_t n);
    void write_dword(uint16_t port, uint32_t* src, uint32_t n);

    IORegion* register_region(VMPart& owner, uint16_t start, uint16_t size, void* opaque,
                              io_read_byte_proc_t rb, io_write_byte_proc_t wb = NULL,
                              io_read_word_proc_t rw = NULL, io_write_word_proc_t ww = NULL,
                              io_read_dword_proc_t rd = NULL, io_write_dword_proc_t wd = NULL);
    void remap_region(IORegion* region);
    void unregister_region(IORegion* region);

    virtual void reset();
    virtual bool start() { return true;}
    virtual bool stop() { return true;}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    struct MapItem {
        uint32_t start;
        uint32_t end;

        void* opaque;
        io_read_byte_proc_t read_byte;
        io_read_word_proc_t read_word;
        io_read_dword_proc_t read_dword;
        io_write_byte_proc_t write_byte;
        io_write_word_proc_t write_word;
        io_write_dword_proc_t write_dword;

        IORegion* region;
    };

private:
    typedef std::list<IORegion*> RegionList;
    typedef std::map<uint16_t, MapItem> IOMap;
    typedef std::pair<uint16_t, MapItem> IOMapPair;

    void fill_gap(uint32_t from, uint32_t n);
    void fill_gaps();
    void unmap_range(uint32_t from, uint32_t n);
    void clear_invalid_regions();

    MapItem& find_mapping(uint16_t port);
    RegionList::iterator find_region(IORegion* region);

private:
    IOMap _io_map;
    RegionList _regions;
};

extern IOBus* io_bus;

#endif

