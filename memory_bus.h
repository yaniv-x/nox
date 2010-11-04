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

#ifndef _H_MEMORY_BUS
#define _H_MEMORY_BUS

#include <map>

#include "vm_part.h"

class NoxVM;
class PhysicalRam;
class MMIORegion;
class MapSection;


#define USE_C_CALLBACKS


class MemoryBus: public VMPart{
public:
    MemoryBus(NoxVM& nox);
    ~MemoryBus();

    void init();

    void read(uint64_t src, uint64_t length, void* dest);
    void write(const void* src, uint64_t length, uint64_t dest);

#ifdef USE_C_CALLBACKS
    MMIORegion* register_mmio(page_address_t address, uint64_t num_pages,
                              read_mem_proc_t read, write_mem_proc_t write,
                              void* opaque, VMPart& owner);
#else
    class IO;
    MMIORegion* register_mmio(page_address_t address, uint64_t num_pages, IO& io);
#endif

    void unregister_mmio(MMIORegion* mmio);

    void map_physical_ram(PhysicalRam* ram, page_address_t address, bool rom);
    void map_physical_ram_section(PhysicalRam* ram, page_address_t address, bool rom,
                                  page_address_t section_start_page, uint64_t section_pages);
    void unmap_physical_ram(PhysicalRam* ram);

    PhysicalRam* alloc_physical_ram(VMPart& owner, uint64_t num_pages, const char* name);
    void release_physical_ram(PhysicalRam* ram);
    uint8_t* get_physical_ram_ptr(PhysicalRam* ram);
    uint64_t get_physical_ram_size(PhysicalRam* ram);

    MapSection* map_section(PhysicalRam* ram, page_address_t address, bool rom,
                            page_address_t start_page, uint64_t num_pages);
    void release_section(MapSection* section);

    NoxVM& get_nox() { return *(NoxVM*)get_container();}

    virtual void reset() {}
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    // for now no A20 support (solve conflict with kvm)
    void enable_address_line_20() { /*_address_mask |= (1ULL << 20);*/}
    void disable_address_line_20() { /*_address_mask &= ~(1ULL << 20);*/}
    bool line_20_is_set() { return !!(_address_mask & (1ULL << 20));}

    enum {
        ADDRESS_BITS = 52,
    };

    class Internal;

private:

    struct MappedMemory {
        MappedMemory(page_address_t start, uint64_t size,
#ifdef USE_C_CALLBACKS
                     read_mem_proc_t read_cb,
                     write_mem_proc_t write_cb,
                     void* in_opaue,
#else
                     IO& in_io,
#endif
                     Internal& in_internal)
            : start_page (start)
            , num_pages (size)
#ifdef USE_C_CALLBACKS
            , read (read_cb)
            , write (write_cb)
            , opaque (in_opaue)
#else
            , io (in_io)
#endif
            , internal (in_internal)

        {
        }

        page_address_t start_page;
        uint64_t num_pages;
#ifdef USE_C_CALLBACKS
        read_mem_proc_t read;
        write_mem_proc_t write;
        void* opaque;
#else
        IO& io;
#endif
        Internal& internal;
    };

    typedef std::map<page_address_t, MappedMemory> MemoryMap;
    typedef std::pair<page_address_t, MappedMemory> MemoryMapPair;

    typedef std::list<PhysicalRam*> PhysicalRamList;
    typedef std::list<MapSection*> SectionsList;

    typedef std::list<MMIORegion*> MMIORegionList;

    void clear_area(page_address_t start, uint64_t num_pages);
    void clear_handler_mapping(Internal* handler);
    void fill_gap(page_address_t start, uint64_t size);
    void fill_gaps();

    void read_from_pages(uint64_t src, uint64_t length, uint8_t* dest);
    void write_to_pages(const uint8_t* src, uint64_t length, uint64_t dest);

    PhysicalRamList::iterator find_physical(PhysicalRam* ram);



private:
    MemoryMap _memory_map;
    PhysicalRamList _pysical_list;
    SectionsList _sections_list;
    MMIORegionList _mmio_list;
    uint64_t _address_mask;
};

#ifndef USE_C_CALLBACKS
class MemoryBus::IO {
public:
    virtual VMPart& get_part() = 0;

    virtual void read(uint64_t src, uint64_t length, uint8_t* dest) = 0;
    virtual void write(const uint8_t* src, uint64_t length, uint64_t dest) = 0;
};
#endif


extern MemoryBus* memory_bus;

#endif

