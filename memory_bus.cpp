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

#include <malloc.h>
#include <sys/user.h>
#include "memory_bus.h"
#include "nox_vm.h"
#include "common.h"
#include "kvm.h"
#include "threads.h"

MemoryBus* memory_bus = NULL;
static __thread uint mem_enter_count = 0;

static const page_address_t address_mask = (page_address_t(1) << MemoryBus::ADDRESS_BITS) - 1;
static const uint64_t max_pages = (address_mask >> GUEST_PAGE_SHIFT) + 1;


#define EXCLISIC_EXEC()                     \
    Lock lock(_gate);                       \
                                            \
    ++_exclucive_request;                   \
                                            \
    while (_clients != mem_enter_count) {   \
        _gate_condition.wait(_gate);        \
    }                                       \
                                            \
    --_exclucive_request;


static inline bool is_valid_page_address(page_address_t addr)
{
    return addr <= address_mask >> GUEST_PAGE_SHIFT;
}


static inline bool is_valid_page_range(page_address_t start, uint64_t num_pages)
{
    return num_pages && is_valid_page_address(start) && num_pages <= max_pages - start;
}


class MemoryBus::Internal {
public:
    virtual ~Internal() {}
    virtual void on_unmapped() = 0;
    virtual DirectAccess* get_direct(uint64_t offset) = 0;
};


class UmnapedAreaHandler: public MemoryBus::Internal
#ifndef USE_C_CALLBACKS
    , public MemoryBus::IO
#endif
        {
public:
    UmnapedAreaHandler(MemoryBus& bus, page_address_t start)
        : _bus (bus)
        , _start (start << GUEST_PAGE_SHIFT)
    {
    }

    virtual VMPart& get_part()
    {
        return _bus;
    }

    virtual void read(uint64_t src, uint64_t length, uint8_t* dest)
    {
        D_MESSAGE_SOME(100, "0x%016lx length %lu", _start + src, length);
        memset(dest, 0xff, length);
    }

    virtual void write(const uint8_t* src, uint64_t length, uint64_t dest)
    {
        D_MESSAGE_SOME(100, "0x%016lx length %lu", _start + dest, length);
    }


    void on_unmapped()
    {
        delete this;
    }

    virtual DirectAccess* get_direct(uint64_t offset)
    {
        return NULL;
    }

private:
    MemoryBus& _bus;
    address_t _start;
};

class MMIORegion: public MemoryBus::Internal {
public:
    MMIORegion()
        : _mapped (true)
    {
    }

    bool is_mapped() { return _mapped;}

    virtual void on_unmapped()
    {
        ASSERT(_mapped);
        _mapped = false;
    }

    virtual DirectAccess* get_direct(uint64_t offset)
    {
        return NULL;
    }

private:
    bool _mapped;
};


class PhysicalRam: public VMPart, public MemoryBus::Internal
#ifndef USE_C_CALLBACKS
                 , public MemoryBus::IO
#endif
    {
public:
    PhysicalRam(VMPart& owner, const char* name, uint64_t num_pages, KVM& kvm);

    PhysicalRam* ref() { _refs.inc(); return this;}
    uint32_t unref();

    VMPart& get_part() { return *this;}

    void read(uint64_t src, uint64_t length, uint8_t* dest);
    void write(const uint8_t* src, uint64_t length, uint64_t dest);
    virtual DirectAccess* get_direct(uint64_t offset);

    uint8_t* get_ptr() { return _ptr;}
    bool is_mapped() { return _mapped;}
    uint64_t get_num_pages() { return _num_pages;}

    void reset() {}
    bool start() { return true;}
    bool stop() { return true;}
    void save(OutStream& stream) {}
    void load(InStream& stream) {}

    virtual void map(page_address_t address);
    virtual void on_unmapped();

    KvmMapRef map_section(page_address_t address, uint64_t start_page, uint64_t num_pages);
    void unmap_section(KvmMapRef _ref);

private:
    ~PhysicalRam();

public:
    Atomic _refs;
    uint64_t _num_pages;
    uint8_t* _ptr;
    bool _mapped;
    KvmMapRef _map_ref;
    KVM& _kvm;
};


PhysicalRam::PhysicalRam(VMPart& owner, const char* name, uint64_t num_pages, KVM& kvm)
    : VMPart (name, owner)
    , _refs (1)
    , _num_pages (num_pages)
    , _ptr ((uint8_t*)memalign(GUEST_PAGE_SIZE, _num_pages * GUEST_PAGE_SIZE))
    , _mapped (false)
    , _map_ref (INVALID_KVM_MAP_REF)
    , _kvm (kvm)
{
    if (!_ptr) {
        THROW_ERROR(ERROR_NO_MEM, "physical ram alloc failed");
    }
}


PhysicalRam::~PhysicalRam()
{
    free(_ptr);
}


uint32_t PhysicalRam::unref()
{
    uint32_t num_refs = _refs.dec();

    if (num_refs == 0) {
        memory_bus->remove_physical(this);
        delete this;
    }

    return num_refs;
}


void PhysicalRam::read(uint64_t src, uint64_t length, uint8_t* dest)
{
    ASSERT(src + length <= (_num_pages << GUEST_PAGE_SHIFT));
    memcpy(dest, _ptr + src, length);
}


void PhysicalRam::write(const uint8_t* src, uint64_t length, uint64_t dest)
{
    ASSERT(dest + length <= (_num_pages << GUEST_PAGE_SHIFT));
    memcpy(_ptr + dest, src, length);
}


class Direct : public DirectAccess {
public:
    Direct(PhysicalRam* ram, uint64_t offset)
        : DirectAccess(ram->get_ptr() + offset)
        , _ram (ram->ref())
    {
    }

    virtual ~Direct()
    {
        _ram->unref();
    }

private:
    PhysicalRam* _ram;
};


DirectAccess* PhysicalRam::get_direct(uint64_t offset)
{
    ASSERT(offset < _num_pages * GUEST_PAGE_SIZE);
    return new Direct(this, offset);
}


void PhysicalRam::map(page_address_t address)
{
    ASSERT(!is_mapped());

    _map_ref = _kvm.map_mem_slot(address, _num_pages, page_address_t(_ptr) >> 12);

    if (_map_ref == INVALID_KVM_MAP_REF) {
        THROW("kvm map mem failed");
    }

    _mapped = true;
}


void PhysicalRam::on_unmapped()
{
    ASSERT(is_mapped());

    _kvm.unmap_mem_slot(_map_ref);
    _mapped = false;
}

KvmMapRef PhysicalRam::map_section(page_address_t address, uint64_t start_page, uint64_t num_pages)
{
    ASSERT(start_page + num_pages > start_page);
    ASSERT(start_page + num_pages <= _num_pages);
    KvmMapRef map_ref = _kvm.map_mem_slot(address, num_pages,
                                          (page_address_t(_ptr) >> 12) + start_page);

    if (map_ref == INVALID_KVM_MAP_REF) {
        THROW("kvm map mem failed");
    }

    return map_ref;
}

void PhysicalRam::unmap_section(KvmMapRef map_ref)
{
    _kvm.unmap_mem_slot(map_ref);
}


class MemBusGate {
public:
    inline MemBusGate(MemoryBus& bus)
        : _bus  (bus)
    {
        _bus.bus_enter();
    }

    inline ~MemBusGate()
    {
        _bus.bus_exit();
    }

private:
    MemoryBus& _bus;
};


MemoryBus::MemoryBus(NoxVM& nox)
    : VMPart ("Memory bus", nox)
    , _clients (0)
    , _exclucive_request (0)
{
    fill_gap(0, max_pages);
    reset();
    memory_bus = this;
}


MemoryBus::~MemoryBus()
{
    clear_area(0, max_pages);
    ASSERT(_memory_map.empty());
    ASSERT(_pysical_list.empty());
    ASSERT(_sections_list.empty());
    ASSERT(_mmio_list.empty());
}


void MemoryBus::bus_enter()
{
    Lock lock(_gate);

    ++mem_enter_count;
    ++_clients;
}


void MemoryBus::bus_exit()
{
    Lock lock(_gate);
    --_clients;
    --mem_enter_count;

    if (_exclucive_request) {
         _gate_condition.signal();
    }
}


void MemoryBus::read_from_pages(uint64_t src, uint64_t length, uint8_t* dest)
{
    uint64_t n = GUEST_PAGE_SIZE - (src & GUEST_PAGE_OFFSET_MASK);
    ASSERT(n < length);

    read(src, n, dest);
    src += n;
    dest += n;
    length -= n;

    for (;;) {
        n = MIN(GUEST_PAGE_SIZE, length);
        read(src, n, dest);
        length -= n;

        if (length == 0) {
            return;
        }
        src += n;
        dest += n;
    }
}


void MemoryBus::read(uint64_t src, uint64_t length, void* dest)
{
    ASSERT(get_state() == VMPart::RUNNING || get_state() == VMPart::DEBUGGING);

    if ((src & ~((1ULL << ADDRESS_BITS) - 1)) || src + length < src ||
        ((src + length)  & ~((1ULL << ADDRESS_BITS) - 1))) {
        throw MachinErrorException();
    }

    uint64_t offset = src & GUEST_PAGE_OFFSET_MASK;

    if (length > GUEST_PAGE_SIZE - offset) {
        read_from_pages(src, length, (uint8_t*)dest);
        return;
    }

    page_address_t page = (src & _address_mask) >> GUEST_PAGE_SHIFT;

    MemBusGate gate(*this);

    MemoryMap::iterator now = _memory_map.lower_bound(page);
    uint64_t page_start_address = (page - (*now).second.start_page) << GUEST_PAGE_SHIFT;

#ifdef USE_C_CALLBACKS
    (*now).second.read((*now).second.opaque, page_start_address + offset, length, (uint8_t*)dest);
#else
    (*now).second.io.read(page_start_address + offset, length, (uint8_t*)dest);
#endif
}


void MemoryBus::write_to_pages(const uint8_t* src, uint64_t length, uint64_t dest)
{
    uint64_t n = GUEST_PAGE_SIZE - (dest & GUEST_PAGE_OFFSET_MASK);
    ASSERT(n < length);

    write(src, n, dest);
    src += n;
    dest += n;
    length -= n;

    for (;;) {
        n = MIN(GUEST_PAGE_SIZE, length);
        write(src, n, dest);
        length -= n;

        if (length == 0) {
            return;
        }
        src += n;
        dest += n;
    }
}


void MemoryBus::write(const void* src, uint64_t length, uint64_t dest)
{
    ASSERT(get_state() == VMPart::RUNNING || get_state() == VMPart::DEBUGGING);

    if ((dest & ~((1ULL << ADDRESS_BITS) - 1)) || dest + length < dest ||
        ((dest + length)  & ~((1ULL << ADDRESS_BITS) - 1))) {
        throw MachinErrorException();
    }

    uint64_t offset = dest & GUEST_PAGE_OFFSET_MASK;

    if (length > GUEST_PAGE_SIZE - offset) {
        write_to_pages((uint8_t*)src, length, dest);
        return;
    }

    page_address_t page = (dest & _address_mask) >> GUEST_PAGE_SHIFT;

    MemBusGate gate(*this);

    MemoryMap::iterator now = _memory_map.lower_bound(page); //return >=

    uint64_t page_start_address = (page - (*now).second.start_page) << GUEST_PAGE_SHIFT;
#ifdef USE_C_CALLBACKS
    (*now).second.write((*now).second.opaque, (uint8_t*)src, length, page_start_address + offset);
#else
    (*now).second.io.write((uint8_t*)src, length, page_start_address + offset);
#endif
}


DirectAccess* MemoryBus::get_direct(uint64_t address, uint64_t size)
{
    ASSERT(get_state() == VMPart::RUNNING);

    if ((address & ~((1ULL << ADDRESS_BITS) - 1)) || address + size < address ||
        ((address + size)  & ~((1ULL << ADDRESS_BITS) - 1)) ) {
        throw MachinErrorException();
    }

    if (!line_20_is_set() && (address >> 20) != ((address + size) >> 20)) {
        return NULL;
    }

    page_address_t page = (address & _address_mask) >> GUEST_PAGE_SHIFT;

    MemBusGate gate(*this);

    MappedMemory& map = (*_memory_map.lower_bound(page)).second;

    uint64_t map_start = map.start_page << GUEST_PAGE_SHIFT;
    uint64_t map_end = map_start + (map.num_pages << GUEST_PAGE_SHIFT);

    if (map_end < address + size) {
        return NULL;
    }

    return map.internal.get_direct(address - map_start);
}


void MemoryBus::clear_area(page_address_t start, uint64_t num_pages)
{
    do {
        MemoryMap::iterator now = _memory_map.lower_bound(start); //return >=
        page_address_t offset = start - (*now).second.start_page;
        uint64_t in_range = MIN((*now).second.num_pages - offset, num_pages);
        num_pages -= in_range;
        start += in_range;
        MemoryBus::Internal& internal = (*now).second.internal;
        _memory_map.erase(now);
        internal.on_unmapped();
    } while (num_pages);
}


void MemoryBus::fill_gap(page_address_t start, uint64_t size)
{
    UmnapedAreaHandler *handler = new UmnapedAreaHandler(*this, start);
#ifdef USE_C_CALLBACKS
    MappedMemory map(start, size,
                     (read_mem_proc_t)&UmnapedAreaHandler::read,
                     (write_mem_proc_t)&UmnapedAreaHandler::write,
                     handler, *handler);
#else
    MappedMemory map(start, size, *handler, *handler);
#endif
    _memory_map.insert(MemoryMapPair(start + size - 1, map));
}


void MemoryBus::fill_gaps()
{
    page_address_t start = 0;
    uint64_t num_pages = max_pages;

    MemoryMap::iterator now = _memory_map.begin();

    for (; now != _memory_map.end(); ++now) {
        page_address_t gap_size = (*now).second.start_page - start;

        if (gap_size) {
            fill_gap(start, gap_size);
        }

        uint64_t skip = gap_size + (*now).second.num_pages;
        start += skip;
        num_pages -= skip;
    }

    if (num_pages) {
        fill_gap(start, num_pages);
    }
}

void MemoryBus::clear_handler_mapping(Internal* internal)
{
    MemoryMap::iterator now = _memory_map.begin();

    for (; now != _memory_map.end(); ++now) {
        if (&(*now).second.internal == internal) {
            clear_area((*now).second.start_page, (*now).second.num_pages);
            fill_gaps();
            return;
        }
    }

    D_MESSAGE("not found");
}


MMIORegion* MemoryBus::register_mmio(page_address_t address, uint64_t num_pages,
#ifdef USE_C_CALLBACKS
                                     read_mem_proc_t read, write_mem_proc_t write,
                                     void* opaque, VMPart& owner)
#else
                                     IO& io)
#endif
{
    ASSERT(is_valid_page_range(address, num_pages));

    EXCLISIC_EXEC();

    clear_area(address, num_pages);

    MMIORegion* mmio = new MMIORegion();
    _mmio_list.push_back(mmio);
#ifdef USE_C_CALLBACKS
    MappedMemory map(address, num_pages, read, write ,opaque, *mmio);
#else
    MappedMemory map(address, num_pages, io, *mmio);
#endif
    _memory_map.insert(MemoryMapPair(address + num_pages - 1, map));

    fill_gaps();

    return mmio;
}


void MemoryBus::unregister_mmio(MMIORegion* mmio)
{
    if (!mmio) {
        return;
    }

    EXCLISIC_EXEC();

    MMIORegionList::iterator iter = _mmio_list.begin();

    for (; iter != _mmio_list.end() ; iter++) {

        if (*iter == mmio) {

            if (mmio->is_mapped()) {
                clear_handler_mapping(mmio);
            }

            _mmio_list.erase(iter);
            delete mmio;
            return;
        }
    }

    PANIC("not found");
}


MemoryBus::PhysicalRamList::iterator MemoryBus::find_physical(PhysicalRam* ram)
{
    PhysicalRamList::iterator iter = _pysical_list.begin();

    for (; iter != _pysical_list.end(); iter++) {
        if (*iter == ram) {
            break;
        }
    }

    return iter;
}

void MemoryBus::map_physical_ram(PhysicalRam* ram, page_address_t address, bool is_rom)
{
    ASSERT(ram);
    ASSERT(is_valid_page_range(address, ram->_num_pages));

    EXCLISIC_EXEC();

    PhysicalRamList::iterator iter = find_physical(ram);

    if (iter == _pysical_list.end()) {
        PANIC("invalid physical ram");
    }


    if (ram->is_mapped()) {
        D_MESSAGE("calling unmap");
        clear_handler_mapping(ram);
    }

    if (is_rom) {
        D_MESSAGE("for now map as ROM is not supported");
    }

    uint64_t num_pages = ram->get_num_pages();

    clear_area(address, num_pages);
    ram->map(address);

#ifdef USE_C_CALLBACKS
    MappedMemory map(address, num_pages, (read_mem_proc_t)&PhysicalRam::read,
                     (write_mem_proc_t)&PhysicalRam::write, ram, *ram);
#else
    MappedMemory map(address, num_pages, *ram, *ram);
#endif
    _memory_map.insert(MemoryMapPair(address + num_pages - 1, map));

    fill_gaps();
}


void MemoryBus::unmap_physical_ram(PhysicalRam* ram)
{
    if (!ram) {
        return;
    }

    EXCLISIC_EXEC();

    PhysicalRamList::iterator iter = find_physical(ram);

    if (iter == _pysical_list.end()) {
        PANIC("invalid physical ram");
    }

    if (ram->is_mapped()) {
        clear_handler_mapping(ram);
    }
}


PhysicalRam* MemoryBus::alloc_physical_ram(VMPart& owner, uint64_t num_pages, const char* name)
{
    ASSERT(is_valid_page_address(num_pages));

    EXCLISIC_EXEC();

    PhysicalRam* ram = new PhysicalRam(owner, name, num_pages, get_nox().get_kvm());

    _pysical_list.push_back(ram);

    return ram;
}


PhysicalRam* MemoryBus::ref_physical_ram(PhysicalRam* ram)
{
    return ram->ref();
}


void MemoryBus::unref_physical_ram(PhysicalRam* ram)
{
    if (!ram) {
        return;
    }

    ram->unref();
}


void MemoryBus::remove_physical(PhysicalRam *ram)
{
    EXCLISIC_EXEC();

    PhysicalRamList::iterator iter = find_physical(ram);

    if (iter == _pysical_list.end()) {
        PANIC("not found");
    }

    if (ram->is_mapped()) {
        D_MESSAGE("calling unmap");
        clear_handler_mapping(ram);
    }

    _pysical_list.erase(iter);
}


uint8_t* MemoryBus::get_physical_ram_ptr(PhysicalRam* ram)
{
    EXCLISIC_EXEC();

    PhysicalRamList::iterator iter = find_physical(ram);

    if (iter == _pysical_list.end()) {
        PANIC("not found");
    }

    return ram->get_ptr();
}


uint64_t MemoryBus::get_physical_ram_size(PhysicalRam* ram)
{
    EXCLISIC_EXEC();

    PhysicalRamList::iterator iter = find_physical(ram);

    if (iter == _pysical_list.end()) {
        PANIC("not found");
    }

    return ram->get_num_pages() << GUEST_PAGE_SHIFT;
}

class MapSection : public MemoryBus::Internal {
public:
    MapSection(PhysicalRam* ram, page_address_t start_page, uint64_t num_pages)
        : _ram (ram->ref())
        , _offset (start_page * GUEST_PAGE_SIZE)
        , _count (num_pages)
        , _mapped (false)
        , _map_ref (INVALID_KVM_MAP_REF)
    {
    }

    ~MapSection()
    {
        _ram->unref();
    }

    void read(uint64_t src, uint64_t length, uint8_t* dest)
    {
        _ram->read(src + _offset, length, dest);
    }

    void write(const uint8_t* src, uint64_t length, uint64_t dest)
    {
        _ram->write(src, length, dest + _offset);
    }

    virtual void map(page_address_t address)
    {
        ASSERT(!is_mapped());

        _map_ref = _ram->map_section(address, _offset / GUEST_PAGE_SIZE, _count);

        if (_map_ref == INVALID_KVM_MAP_REF) {
            THROW("kvm map mem failed");
        }

        _mapped = true;
    }

    virtual void on_unmapped()
    {
        ASSERT(is_mapped());
        _ram->unmap_section(_map_ref);
        _mapped = false;
    }

    bool is_mapped()
    {
        return _mapped;
    }

    virtual DirectAccess* get_direct(uint64_t offset)
    {
        return _ram->get_direct(offset + _offset);
    }

private:
    PhysicalRam* _ram;
    uint64_t _offset;
    uint64_t _count;
    bool _mapped;
    KvmMapRef _map_ref;
};

MapSection* MemoryBus::map_section(PhysicalRam* ram, page_address_t address, bool rom,
                                page_address_t start_page, uint64_t num_pages)
{
    EXCLISIC_EXEC();

    PhysicalRamList::iterator iter = find_physical(ram);

    if (iter == _pysical_list.end()) {
        PANIC("not found");
    }

    ASSERT(start_page + num_pages > start_page);
    ASSERT(start_page + num_pages <=  ram->_num_pages);

    MapSection *section = new MapSection(ram, start_page, num_pages);

    _sections_list.push_back(section);

    clear_area(address, num_pages);

    section->map(address);

    MappedMemory map(address, num_pages, (read_mem_proc_t)&MapSection::read,
                     (write_mem_proc_t)&MapSection::write, section, *section);

    _memory_map.insert(MemoryMapPair(address + num_pages - 1, map));

    fill_gaps();

    return section;
}


void MemoryBus::release_section(MapSection* section)
{
    if (!section) {
        return;
    }

    EXCLISIC_EXEC();

    SectionsList::iterator iter = _sections_list.begin();

    for (; iter != _sections_list.end(); iter++) {
        if (*iter == section) {
            break;
        }
    }

    if (iter == _sections_list.end()) {
        PANIC("not found");
    }

    if (section->is_mapped()) {
        clear_handler_mapping(section);
    }

    _sections_list.erase(iter);

    delete section;
}

void MemoryBus::reset()
{
    _address_mask = ~(1ULL << 20);
    _address_mask = (1ULL << ADDRESS_BITS) - 1; // for now no A20 support (solve conflict with kvm)
    clear_area(0, max_pages);
    fill_gap(0, max_pages);
}

