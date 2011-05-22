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

#include "io_bus.h"
#include "nox_vm.h"


IOBus* io_bus = NULL;

/*

quote from Intel® 64 and IA-32 Architectures Software Developer's Manual

"Any two consecutive 8-bit ports can be treated as a 16-bit port, and any four consec-
utive ports can be a 32-bit port. In this manner, the processor can transfer 8, 16, or
32 bits to or from a device in the I/O address space. Like words in memory, 16-bit
ports should be aligned to even addresses (0, 2, 4, ...) so that all 16 bits can be
transferred in a single bus cycle. Likewise, 32-bit ports should be aligned to
addresses that are multiples of four (0, 4, 8, ...). The processor supports data trans-
fers to unaligned ports, but there is a performance penalty because one or more
extra bus cycle must be used."

*/


/*

quote from Intel® 64 and IA-32 Architectures Software Developer's Manual

"If hardware or software requires that I/O ports be written to in a particular order, that order
must be specified explicitly. For example, to load a word-length I/O port at address 2H and then
another word port at 4H, two word-length writes must be used, rather than a single
doubleword write at 2H."

*/


// Not sure if splitting in/out is the right way to go. Splitting was added
// because vgabios is using one outw instead of two outb. Need better
// understanding of the io bus in order to take the right decision.


static const uint32_t num_ports = 1 << 16;
static const uint32_t max_port = max_port - 1;

#ifdef NOX_DEBUG

static bool is_valid_port_range(uint16_t start, uint16_t size)
{
    return uint32_t(start) + size <= num_ports;
}

#endif

static uint8_t default_read_byte(void*, uint16_t port)
{
    D_MESSAGE("unmapped port 0x%x", port);
    return ~0;
}


uint16_t default_read_word(void*, uint16_t port)
{
    //D_MESSAGE("unmapped port 0x%x", port);

    uint8_t b1;
    uint8_t b2;

    io_bus->read_byte(port, &b1, 1);
    io_bus->read_byte(port + 1, &b2, 1);

    return uint16_t(b2) << 8 | b1;
}


uint32_t default_read_dword(void*, uint16_t port)
{
    //D_MESSAGE("unmapped port 0x%x", port);

    uint16_t w1;
    uint16_t w2;

    io_bus->read_word(port, &w1, 1);
    io_bus->read_word(port + 2, &w2, 1);

    return uint32_t(w2) << 16 | w1;
}


void default_write_byte(void*, uint16_t port, uint8_t val)
{
    D_MESSAGE("unmapped port 0x%x val %u (0x%x)", port, val, val);
}


void default_write_word(void*, uint16_t port, uint16_t val)
{
    //D_MESSAGE("unmapped port 0x%x val %u (0x%x)", port, val, val);

    io_bus->write_byte(port, (uint8_t*)&val, 1);
    io_bus->write_byte(port + 1, ((uint8_t*)&val) + 1, 1);
}


void default_write_dword(void*, uint16_t port, uint32_t val)
{
    //D_MESSAGE("unmapped port 0x%x val %u (0x%x)", port, val, val);

    io_bus->write_word(port, (uint16_t*)&val, 1);
    io_bus->write_word(port + 2, ((uint16_t*)&val) + 1, 1);
}

class IORegion: public IOBus::MapItem {
public:
    IORegion(VMPart& part, uint32_t in_start, uint32_t in_size, void* in_opaque,
             io_read_byte_proc_t rb, io_write_byte_proc_t wb,
             io_read_word_proc_t rw, io_write_word_proc_t ww,
             io_read_dword_proc_t rd, io_write_dword_proc_t wd)
        : _part (part)
    {
        start = in_start;
        size = in_size;
        read_byte = rb ? rb : default_read_byte;
        read_word = rw ? rw : default_read_word;
        read_dword = rd ? rd : default_read_dword;
        write_byte = wb ? wb : default_write_byte;
        write_word = ww ? ww : default_write_word;
        write_dword = wd ? wd : default_write_dword;
        opaque = in_opaque;
        region = this;
    }

    IORegion(VMPart& part, uint32_t in_start, uint32_t in_size)
        : _part (part)
    {
        start = in_start;
        size = in_size;
        read_byte = default_read_byte;
        read_word = default_read_word;
        read_dword = default_read_dword;
        write_byte = default_write_byte;
        write_word = default_write_word;
        write_dword = default_write_dword;
        opaque = NULL;
        region = NULL;
    }

private:
    VMPart& _part;
};


IOBus::IOBus(NoxVM& nox)
    : VMPart("io bus", nox)
{
    fill_gap(0, num_ports);

    io_bus = this;
}


IOBus::~IOBus()
{
    clear_invalid_regions();
    ASSERT(_io_map.empty());
    ASSERT(_regions.empty());
}


void IOBus::clear_invalid_regions()
{
    IOMap::iterator now = _io_map.begin();

    while (now != _io_map.end()) {
        if ((*now).second.region == NULL) {
            IOMap::iterator to_remove = now++;
            _io_map.erase(to_remove);
        } else {
            now++;
        }
    }
}


IOBus::MapItem& IOBus::find_mapping(uint16_t port)
{
    IOMap::iterator iter = _io_map.lower_bound(port);

    ASSERT((*iter).second.start <= port && (*iter).second.start + (*iter).second.size > port);
    return (*iter).second;
}


IOBus::RegionList::iterator IOBus::find_region(IORegion* region)
{
    RegionList::iterator iter = _regions.begin();

    for (; iter != _regions.end(); iter++) {
        if (*iter == region) {
            break;
        }
    }

    return iter;
}


void IOBus::read_byte(uint16_t port, uint8_t* dest, uint32_t n)
{
    MapItem& map = find_mapping(port);
    port -= map.start;
    uint8_t* end = dest + n;

    ASSERT(get_state() == VMPart::RUNNING);

    for (; dest < end; dest++) {
        *dest = map.read_byte(map.opaque, port + map.start);
    }
}


void IOBus::read_word(uint16_t port, uint16_t* dest, uint32_t n)
{
    MapItem& map = find_mapping(port);
    port -= map.start;

    ASSERT(get_state() == VMPart::RUNNING);

    if ((map.size - port) < 2) {
        D_MESSAGE("boundry test failed, port 0x%x range start 0x%x size %u",
                  port + map.start, map.start, map.size);

        read_byte(port, (uint8_t*)dest, 1);
        read_byte(port + 1, (uint8_t*)dest + 1, 1);

        return;
    }

    uint16_t* end = dest + n;

    for (; dest < end; dest++) {
        *dest = map.read_word(map.opaque, port + map.start);
    }
}


void IOBus::read_dword(uint16_t port, uint32_t* dest, uint32_t n)
{
    MapItem& map = find_mapping(port);
    port -= map.start;

    ASSERT(get_state() == VMPart::RUNNING);

    if ((map.size - port) < 4) {
        D_MESSAGE("boundry test failed, port 0x%x range start 0x%x size %u",
                  port + map.start, map.start, map.size);

        read_word(port, (uint16_t*)dest, 1);
        read_word(port + 2, (uint16_t*)dest + 1, 1);

        return;
    }

    uint32_t* end = dest + n;

    for (; dest < end; dest++) {
        *dest = map.read_dword(map.opaque, port + map.start);
    }
}


void IOBus::write_byte(uint16_t port, uint8_t* src, uint32_t n)
{
    MapItem& map = find_mapping(port);
    port -= map.start;
    uint8_t* end = src + n;

    ASSERT(get_state() == VMPart::RUNNING);

    for (; src < end; src++) {
         map.write_byte(map.opaque, port + map.start, *src);
    }
}


void IOBus::write_word(uint16_t port, uint16_t* src, uint32_t n)
{
    MapItem& map = find_mapping(port);
    port -= map.start;

    ASSERT(get_state() == VMPart::RUNNING);

    if ((map.size - port) < 2) {
        D_MESSAGE("boundry test failed, port 0x%x range start 0x%x size %u",
                  port + map.start, map.start, map.size);

        write_byte(port, (uint8_t*)src, 1);
        write_byte(port + 1, (uint8_t*)src + 1, 1);

        return;
    }

    uint16_t* end = src + n;

    for (; src < end; src++) {
         map.write_word(map.opaque, port + map.start, *src);
    }
}


void IOBus::write_dword(uint16_t port, uint32_t* src, uint32_t n)
{
    MapItem& map = find_mapping(port);
    port -= map.start;

    ASSERT(get_state() == VMPart::RUNNING);

    if ((map.size - port) < 4) {
        D_MESSAGE("boundry test failed, port 0x%x range start 0x%x size %u",
                  port + map.start, map.start, map.size);

        write_word(port, (uint16_t*)src, 1);
        write_word(port + 2, (uint16_t*)src + 1, 1);

        return;
    }

    uint32_t* end = src + n;

    for (; src < end; src++) {
         map.write_dword(map.opaque, port + map.start, *src);
    }
}


void IOBus::fill_gap(uint32_t from, uint32_t n)
{
    _io_map.insert(IOMapPair(from + n - 1, IORegion(*this, from, n)));
}


void IOBus::fill_gaps()
{
    uint32_t from = 0;
    uint32_t n = num_ports;

    IOMap::iterator now = _io_map.begin();

    for (; now != _io_map.end(); ++now) {
        uint32_t gap_size = (*now).second.start - from;

        if (gap_size) {
            fill_gap(from, gap_size);
        }

        uint32_t skip = gap_size + (*now).second.size;
        from += skip;
        n -= skip;
    }

    if (n) {
        fill_gap(from, n);
    }
}


void IOBus::unmap_range(uint32_t from, uint32_t n)
{
    do {
        IOMap::iterator now = _io_map.lower_bound(from);
        uint32_t offset = from - (*now).second.start;
        uint32_t in_range = MIN((*now).second.size - offset, n);
        n -= in_range;
        from += in_range;
        _io_map.erase(now);
    } while (n);
}


IORegion* IOBus::register_region(VMPart& part, uint16_t start, uint16_t size, void* opaque,
                                 io_read_byte_proc_t rb, io_write_byte_proc_t wb,
                                 io_read_word_proc_t rw, io_write_word_proc_t ww,
                                 io_read_dword_proc_t rd, io_write_dword_proc_t wd)
{
    ASSERT(size > 0);
    ASSERT(is_valid_port_range(start, size));

    EXCLISIC_EXEC();

    unmap_range(start, size);
    IORegion* region = new IORegion(part, start, size, opaque, rb, wb, rw, ww, rd, wd);
    _regions.push_back(region);
    _io_map.insert(IOMapPair(start + size - 1, *region));
    fill_gaps();

    return region;
}


void IOBus::remap_region(IORegion* region)
{
    EXCLISIC_EXEC();

    IOBus::RegionList::iterator region_iter = IOBus::find_region(region);

    if (region_iter == _regions.end()) {
        PANIC("not found");
    }

    unmap_range(region->start, region->size);
    _io_map.insert(IOMapPair(region->start + region->size - 1, *region));
    fill_gaps();
}


void IOBus::unregister_region(IORegion* region)
{
    if (!region) {
        return;
    }

    EXCLISIC_EXEC();

    IOBus::RegionList::iterator region_iter = IOBus::find_region(region);

    if (region_iter == _regions.end()) {
        PANIC("not found");
    }

    IOMap::iterator now = _io_map.begin();

    for (; now != _io_map.end(); ++now) {
        if ((*now).second.region == region) {
            unmap_range((*now).second.start, (*now).second.size);
            fill_gap((*now).second.start, (*now).second.size);
            break;
        }
    }

    _regions.erase(region_iter);
    delete region;
}


void IOBus::reset()
{
    unmap_range(0, num_ports);
    fill_gap(0, num_ports);
}

