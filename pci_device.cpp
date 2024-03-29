/*
    Copyright (c) 2013-2017 Yaniv Kamay,
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

#include "pci_device.h"
#include "pci_bus.h"
#include "utils.h"
#include "io_bus.h"
#include "memory_bus.h"
#include "wire.h"
#include "nox.h"
#include "pic.h"
#include "application.h"
#include "firmware_file.h"
#include "pci.h"


// The following section, from PCI spec, explain the mechanism for getting
// 64 bit region size:
//
// "64-bit (memory) Base Address registers can be handled the same, except that the second
// 32-bit register is considered an extension of the first; i.e., bits 32-63. Software writes
// 0FFFFFFFFh to both registers, reads them back, and combines the result into a 64-bit value.
// Size calculation is done on the 64-bit value."
//
// In practice linux preform the following steps:
//
// read low bar
// write ~0 to low bar
// read low bar
// write low bar
//
// read high bar
// write ~0 to high bar
// read high bar
// write high bar
//
// define PCI_STRICT to conform to PCI spec or undefine PCI_STRICT to support linux probing
// todo: test prob under windows


//#define PCI_STRICT


#define  PCI_MEM_MAX_SIZE_32 (1U << 31)
#define  PCI_MEM_MAX_SIZE_64 (1ULL << 63)


class PCIDevice::Region {
public:
    Region()
        : size (0)
#ifdef PCI_STRICT
        , read_back (false)
#endif
        , base_address (0)
        , _fix_address (0)
    {}

    virtual ~Region() {}

    virtual bool is_io() = 0;
    virtual bool is_64bit() = 0;
    virtual bool is_low_bar(uint bar_index) = 0;
    virtual void map(PCIDevice& owner, uint64_t start) = 0;
    virtual void unmap() = 0;

    bool is_high_bar(uint bar_index) {return !is_low_bar(bar_index);}
    uint64_t get_mask() { return ~(size - 1);}
    bool is_mem() { return !is_io();}
    address_t get_address() { return base_address;}
    bool is_fixed() { return _fix_address != 0;}
    void set_fix_address(address_t address) { _fix_address = address;}
    address_t get_fix_address() { return _fix_address;}
    void reset();

public:
    uint64_t size;
#ifdef PCI_STRICT
    bool read_back;
#endif

protected:
    address_t base_address;

private:
    address_t _fix_address;
};

void PCIDevice::Region::reset()
{
    unmap();
#ifdef PCI_STRICT
    read_back = false;
#endif
}

class IOMap: public PCIDevice::Region {
public:
    IOMap(uint16_t size, void* opaque,
          io_read_byte_proc_t rb, io_write_byte_proc_t wb,
          io_read_word_proc_t rw, io_write_word_proc_t ww,
          io_read_dword_proc_t rd, io_write_dword_proc_t wd);

    virtual ~IOMap();

    virtual bool is_io() { return true;}
    virtual bool is_64bit() { return false;}
    virtual bool is_low_bar(uint bar_index) {return true;}
    virtual void map(PCIDevice& owner, uint64_t start);
    virtual void unmap();

private:
    void* _opaque;
    io_read_byte_proc_t _rb;
    io_write_byte_proc_t _wb;
    io_read_word_proc_t _rw;
    io_write_word_proc_t _ww;
    io_read_dword_proc_t _rd;
    io_write_dword_proc_t _wd;
    IORegion* _io_region;
    IORegion* _io_gap;
    uint16_t effective_size;
};

// Devices that map control functions into I/O Space must not consume more than 256 bytes
// per I/O Base Address register

IOMap::IOMap(uint16_t in_size, void* opaque,
             io_read_byte_proc_t rb, io_write_byte_proc_t wb,
             io_read_word_proc_t rw, io_write_word_proc_t ww,
             io_read_dword_proc_t rd, io_write_dword_proc_t wd)
    : _opaque (opaque)
    , _rb (rb)
    , _wb (wb)
    , _rw (rw)
    , _ww (ww)
    , _rd (rd)
    , _wd (wd)
    , _io_region (NULL)
    , _io_gap (NULL)
{
    if (in_size < PCI_IO_MIN_SIZE || in_size > PCI_IO_MAX_SIZE) {
        THROW("invalid size");
    }

    int msb = find_msb(in_size);

    uint map_size = 1 << msb;

    if (map_size < in_size) {
        map_size <<= 1;
    }

    size = map_size;
    effective_size = in_size;
}


IOMap::~IOMap()
{
    unmap();
}


void IOMap::map(PCIDevice& owner, uint64_t start)
{
    if (start == base_address) {
        return;
    }

    unmap();

    base_address = start;

    if (start + size > (1 << 16)) {
        W_MESSAGE("bad mapping start 0x%lx size %lu", start, size);
        return;
    }

    _io_region = io_bus->register_region(owner, start, effective_size, _opaque,
                                         _rb, _wb, _rw, _ww, _rd, _wd);

    if (effective_size < size) {
        _io_gap = io_bus->register_region(owner, start + effective_size, size - effective_size,
                                          NULL, NULL);
    }
}


void IOMap::unmap()
{
    base_address = 0;

    if (!_io_region) {
        return;
    }

    io_bus->unregister_region(_io_region);

    _io_region = NULL;

    if (_io_gap) {
        io_bus->unregister_region(_io_gap);
        _io_gap = NULL;;
    }
}


class MemMap: public PCIDevice::Region {
public:
    MemMap(uint64_t in_size, bool bits64, uint base_bar)
        : _bits64 (bits64)
        , _base_bar (base_bar)
        , _gap (NULL)
    {
        if (!in_size || (in_size & GUEST_PAGE_OFFSET_MASK) ||
            (in_size > PCI_MEM_MAX_SIZE_32 && !bits64) ||
            in_size > PCI_MEM_MAX_SIZE_64 ) {

            THROW("invalid size");
        }

        int msb = find_msb(in_size);

        uint64_t map_size = 1ULL << msb;

        if (map_size < in_size) {
            map_size <<= 1;
        }

        size = map_size;
        _effective_size = in_size;
    }

    virtual ~MemMap()
    {
        unmap_gap();
    }

    virtual bool is_io() { return false;}
    virtual bool is_64bit() { return _bits64;}
    virtual bool is_low_bar(uint bar_index) {return bar_index == _base_bar;}

    void map_gap(PCIDevice& owner, uint64_t start)
    {
        if (_effective_size == size) {
            return;
        }

        _gap = memory_bus->register_mmio((start + _effective_size) >> GUEST_PAGE_SHIFT,
                                         (size - _effective_size) >> GUEST_PAGE_SHIFT,
                                         (read_mem_proc_t)&MemMap::nop_read,
                                         (write_mem_proc_t)&MemMap::nop_write,
                                         this, owner);
    }

    void unmap_gap()
    {
        if (!_gap) {
            return;
        }

        memory_bus->unregister_mmio(_gap);
        _gap = NULL;
    }

    void nop_read(uint64_t src, uint64_t length, uint8_t* dest)
    {
        D_MESSAGE("");
        memset(dest, 0xff, length);
    }

    void nop_write(const uint8_t* src, uint64_t length, uint64_t dest)
    {
        D_MESSAGE("");
    }

    uint64_t get_effective_size() { return _effective_size;}

private:
    bool _bits64;
    uint _base_bar;
    uint64_t _effective_size;
    MMIORegion* _gap;
};


class PhysicalMap: public MemMap {
public:
    PhysicalMap(PhysicalRam* physical, bool bits64, uint low_bar)
        : MemMap(memory_bus->get_physical_ram_size(physical), bits64, low_bar)
        , _physical (memory_bus->ref_physical_ram(physical))
        , _mapped (false)
    {
    }

    virtual ~PhysicalMap()
    {
        if (_mapped) {
            memory_bus->unmap_physical_ram(_physical);
        }

        memory_bus->unref_physical_ram(_physical);
    }

    virtual void map(PCIDevice& owner, uint64_t start)
    {
        if (start == base_address) {
            return;
        }

        unmap();
        base_address = start;

        if (start + size > (1ULL << MemoryBus::ADDRESS_BITS) || start + size < start
            || (!is_64bit() && start + size > (1ULL << 32)) ) {
            W_MESSAGE("bad mapping start 0x%lx size %lu", start, size);
            return;
        }

        memory_bus->map_physical_ram(_physical, start >> GUEST_PAGE_SHIFT, false);
        _mapped = !_mapped;
        map_gap(owner, start);
    }

    virtual void unmap()
    {
        base_address = 0;

        if (!_mapped) {
            return;
        }

        _mapped = !_mapped;
        memory_bus->unmap_physical_ram(_physical);
        unmap_gap();
    }

private:
    PhysicalRam* _physical;
    bool _mapped;
};


class PhysicalSection: public MemMap {
public:
    PhysicalSection(PhysicalRam* physical, page_address_t start_page,
                    uint64_t num_pages, bool bits64, uint low_bar)
        : MemMap(num_pages << GUEST_PAGE_SHIFT, bits64, low_bar)
        , _physical (memory_bus->ref_physical_ram(physical))
        , _start_page (start_page)
        , _num_pages (num_pages)
        , _section (NULL)
    {
    }

    virtual ~PhysicalSection()
    {
        if (_section) {
            memory_bus->release_section(_section);
            _section = NULL;
        }

        memory_bus->unref_physical_ram(_physical);
    }

    virtual void map(PCIDevice& owner, uint64_t start)
    {
        if (start == base_address) {
            return;
        }

        unmap();

        base_address = start;

        if (start + size > (1ULL << MemoryBus::ADDRESS_BITS) || start + size < start
            || (!is_64bit() && start + size > (1ULL << 32)) ) {
            W_MESSAGE("bad mapping start 0x%lx size %lu", start, size);
            return;
        }

        _section = memory_bus->map_section(_physical, start >> GUEST_PAGE_SHIFT, false,
                                           _start_page, _num_pages);
        map_gap(owner, start);
    }

    virtual void unmap()
    {
        base_address = 0;

        if (!_section) {
            return;
        }

        memory_bus->release_section(_section);
        _section = NULL;
        unmap_gap();
    }

private:
    PhysicalRam* _physical;
    page_address_t _start_page;
    uint64_t _num_pages;
    MapSection* _section;
};


class MMIOMap: public MemMap {
public:
    MMIOMap(uint64_t size, bool bits64, uint low_bar, void* opaque,
            read_mem_proc_t read, write_mem_proc_t write)
        : MemMap(size, bits64, low_bar)
        , _opaque (opaque)
        , _read (read)
        , _write (write)
        , _map (NULL)
    {
    }

    virtual ~MMIOMap()
    {
        if (_map) {
            memory_bus->unregister_mmio(_map);
        }
    }
    virtual void map(PCIDevice& owner, uint64_t start)
    {
        if (start == base_address) {
            return;
        }

        unmap();

        base_address = start;

        if (start + size > (1ULL << MemoryBus::ADDRESS_BITS) || start + size < start
            || (!is_64bit() && start + size > (1ULL << 32)) ) {
            W_MESSAGE("bad mapping start 0x%lx size %lu", start, size);
            return;
        }

        _map = memory_bus->register_mmio(start >> GUEST_PAGE_SHIFT,
                                         get_effective_size() >> GUEST_PAGE_SHIFT,
                                         _read, _write, _opaque, owner);

        map_gap(owner, start);
    }

    virtual void unmap()
    {
        base_address = 0;

        if (!_map) {
            return;
        }

        memory_bus->unregister_mmio(_map);
        _map = NULL;
        unmap_gap();
    }

private:
    void* _opaque;
    read_mem_proc_t _read;
    write_mem_proc_t _write;
    MMIORegion* _map;
};


class PCIFirmware : public NonCopyable {
public:
    PCIFirmware(PCIDevice& device, FirmwareFile *file, PhysicalRam* ram)
        : _device (device)
        , _file (file)
        , _ram (memory_bus->ref_physical_ram(ram))
        , _mmio (NULL)
        , _mapped (false)
    {
        uint32_t size = memory_bus->get_physical_ram_size(_ram);

        int msb = find_msb(size);

        _map_size = 1ULL << msb;

        if (_map_size < size) {
            _map_size <<= 1;
        }
    }


    virtual ~PCIFirmware()
    {
        unmap();
        delete _file;
        memory_bus->unref_physical_ram(_ram);
    }

    void reload()
    {
        _file->read_all(memory_bus->get_physical_ram_ptr(_ram));
    }

    void unmap()
    {
        if (!_mapped) {
            return;
        }

        _mapped = false;
        memory_bus->unmap_physical_ram(_ram);

        if (_mmio) {
            memory_bus->unregister_mmio(_mmio);
            _mmio = NULL;
        }
    }

    void map(uint32_t address)
    {
        unmap();

        memory_bus->map_physical_ram(_ram, address >> GUEST_PAGE_SHIFT, true);

        uint32_t size = memory_bus->get_physical_ram_size(_ram);

        if (size != _map_size) {
            _mmio = memory_bus->register_mmio((address + size) >> GUEST_PAGE_SHIFT,
                                              (_map_size - size) >> GUEST_PAGE_SHIFT,
                                              (read_mem_proc_t)&PCIFirmware::nop_read,
                                              (write_mem_proc_t)&PCIFirmware::nop_write,
                                              this, _device);
        }

        _mapped = true;
    }

    uint32_t get_size()
    {
        return  _map_size;
    }

    void nop_read(uint64_t src, uint64_t length, uint8_t* dest)
    {
        memset(dest, 0xff, length);
    }

    void nop_write(const uint8_t* src, uint64_t length, uint64_t dest)
    {
    }


private:
    PCIDevice& _device;
    FirmwareFile *_file;
    PhysicalRam* _ram;
    uint32_t _map_size;
    MMIORegion* _mmio;
    bool _mapped;
};


PCIDevice::PCIDevice(const char* name, PCIBus& bus, uint16_t vendor, uint16_t device,
                     uint8_t revision, uint32_t class_code, bool with_interrupt)
    : VMPart(name, bus)
    , _interrupt_line (*this)
    , _firmware (NULL)
{
    if (vendor == 0 || vendor == ~0) {
        THROW("invalid vendor id");
    }

    memset(_config_space, 0, sizeof(_config_space));
    memset(_regions, 0, sizeof(_regions));

    *reg16(PCI_CONF_VENDOR) = vendor;
    *reg16(PCI_CONF_DEVICE) = device;
    *reg8(PCI_CONF_REVISION) = revision;
    *reg8(PCI_CONF_CLASS) = class_code;
    *reg16(PCI_CONF_CLASS + 1) = class_code >> 8;

    *reg16(PCI_CONF_SUBSYS_VENDOR) = vendor;
    *reg16(PCI_CONF_SUBSYS_ID) = device;

    *reg16(PCI_CONF_STATUS) |= PCI_STATUS_66MHZ_MASK;

    if (with_interrupt) {
        *reg8(PCI_CONF_INTERRUPT_PIN) = PCI_INTTERUPT_PIN_A;
    }

    load_firmware(vendor, device, revision);
}


PCIDevice::~PCIDevice()
{
    ((PCIBus*)get_container())->remove_device(*this);

    for (int i = 0; i < NUM_BARS; i++) {
        PCIDevice::Region* region = _regions[i];

        if (!region) {
            continue;
        }

        if (region->is_64bit()) {
            i++;
        }

        delete region;
    }

    delete _firmware;
}


void PCIDevice::load_firmware(uint16_t vendor, uint16_t device, uint8_t revision)
{
    std::vector< std::string> names(2);
    std::string file_name;

    sprintf(names[0], "pci-%.4x-%.4x-%.2x.bin", vendor, device, revision);
    sprintf(names[1], "pci-%.4x-%.4x.bin", vendor, device);

    if (!Application::find_firmware(file_name, names)) {
        return;
    }

    std::unique_ptr<FirmwareFile> file(new FirmwareFile());

    file->open(file_name.c_str());

    if (!file->is_valid()) {
        return;
    }

    PhysicalRam* ram = memory_bus->alloc_physical_ram(*this, file->num_pages(), "firmware");

    _firmware = new PCIFirmware(*this, file.release(), ram);

    memory_bus->unref_physical_ram(ram);
}


void PCIDevice::set_region(uint index, PCIDevice::Region* region)
{
    if (index >= NUM_BARS || (region->is_64bit() && NUM_BARS - index < 2)) {
        PANIC("no space");
    }

    uint reg_index = (PCI_CONF_BAR_0 >> 2) + index;

    if (region->is_io()) {
        _config_space[reg_index] = PCI_BAR_IO_MASK;
    } else {
        // Any device that has a range that behaves like normal memory should mark the range as
        // prefetchable. A linear frame buffer in a graphics device is an example of a range that
        // should be marked prefetchable.

        _config_space[reg_index] = PCI_BAR_PREFATCH_MASK;
    }

    if (_regions[index]) {
        PANIC("bar is in use");
    }

    _regions[index] = region;

    if (region->is_64bit()) {
        _config_space[reg_index] |= PCI_BAR_MEM_TYPE_64BITS << PCI_BAR_MEM_TYPE_SHIFT;

        if (_regions[index + 1]) {
            PANIC("bar is in use");
        }

        _regions[index + 1] = _regions[index];
    }
}


void PCIDevice::add_mmio_region(uint bar, uint64_t size, void* opaque, read_mem_proc_t read,
                                write_mem_proc_t write, bool bits64)
{
    set_region(bar, new MMIOMap(size, bits64, bar, opaque, read, write));
}


void PCIDevice::add_mem_region(uint bar, PhysicalRam* physical, bool bits64)
{
    set_region(bar, new PhysicalMap(physical, bits64, bar));
}


void PCIDevice::add_mem_region(uint bar, PhysicalRam* physical, page_address_t start_page,
                               uint64_t num_pages, bool bits64)
{
    set_region(bar, new PhysicalSection(physical, start_page, num_pages, bits64, bar));
}


void PCIDevice::add_io_region(uint bar, uint16_t size, void* opaque,
                              io_read_byte_proc_t rb, io_write_byte_proc_t wb,
                              io_read_word_proc_t rw, io_write_word_proc_t ww,
                              io_read_dword_proc_t rd, io_write_dword_proc_t wd)
{
    set_region(bar, new IOMap(size, opaque, rb, wb, rw, ww, rd, wd));
}


address_t PCIDevice::get_region_address(uint bar)
{
    if (bar >= NUM_BARS || !_regions[bar] || _regions[bar]->is_high_bar(bar)) {
        return 0;
    }

    return _regions[bar]->get_address();
}


void PCIDevice::set_interrupt_level(uint level)
{
    if (!(*reg16(PCI_CONF_STATUS) & PCI_STATUS_INTERRUPT_MASK) == !level) {
        return;
    }

    Lock lock(_mutex);

    if (level) {
        *reg16(PCI_CONF_STATUS) |= PCI_STATUS_INTERRUPT_MASK;
    } else {
        *reg16(PCI_CONF_STATUS) &= ~PCI_STATUS_INTERRUPT_MASK;
    }

    if (*reg16(PCI_CONF_COMMAND) & PCI_COMMAND_DISABLE_INTERRUPT) {
        return;
    }

    if (level) {
        _interrupt_line.raise();
    } else {
        _interrupt_line.drop();
    }
}


uint8_t PCIDevice::read_config_byte(uint index, uint offset)
{
    ASSERT(offset < 4);

    offset += (index << 2);
    return (offset < CONFIG_SIZE) ? *reg8(offset) : 0;
}


uint16_t PCIDevice::read_config_word(uint index, uint offset)
{
    ASSERT(offset < 3);
    return (uint16_t(read_config_byte(index, offset + 1)) << 8) | read_config_byte(index, offset);
}


uint32_t PCIDevice::read_config_dword(uint index)
{
    return (uint32_t(read_config_word(index, 2)) << 16) | read_config_word(index, 0);
}


void PCIDevice::map_io()
{
    for (int i = 0; i < NUM_BARS; i++) {
        if (!_regions[i] || !_regions[i]->is_io()) {
            continue;
        }

        uint16_t start =  _config_space[(PCI_CONF_BAR_0 >> 2) + i] & _regions[i]->get_mask();

        if (start == 0) {
            // the bios enable io before it finish configuring all io bars, it looks like a
            // BIOS bug.
            continue;
        }

        _regions[i]->map(*this, start);
        D_MESSAGE("%s: 0x%lx 0x%lx", get_name().c_str(), _regions[i]->get_address(),
                  ~_regions[i]->get_mask() + 1);
    }
}


void PCIDevice::unmap_io()
{
    for (int i = 0; i < NUM_BARS; i++) {
        if (!_regions[i] || !_regions[i]->is_io()) {
            continue;
        }

        _regions[i]->unmap();
    }
}


void PCIDevice::map_mem()
{
    for (int i = 0; i < NUM_BARS; i++) {
        if (!_regions[i] || !_regions[i]->is_mem() || _regions[i]->is_high_bar(i)) {
            continue;
        }

        uint64_t start;

        if (_regions[i]->is_64bit()) {
            start = *reg64(PCI_CONF_BAR_0 + (i << 2));
        } else {
            start = _config_space[(PCI_CONF_BAR_0 >> 2) + i];
        }

        start &= _regions[i]->get_mask();

        if (!start) {
            // the bios enable mem before it finish configuring all mem bars, it looks like a
            // BIOS bug.
            continue;
        }

        _regions[i]->map(*this, start);
        D_MESSAGE("%s: 0x%lx 0x%lx", get_name().c_str(), _regions[i]->get_address(),
                  ~_regions[i]->get_mask() + 1);
    }
}


void PCIDevice::unmap_mem()
{
    for (int i = 0; i < NUM_BARS; i++) {
        if (!_regions[i] || !_regions[i]->is_mem() || _regions[i]->is_high_bar(i)) {
            continue;
        }

        _regions[i]->unmap();
    }
}


void PCIDevice::disable_interrupt()
{
    _interrupt_line.drop();
}


void PCIDevice::enable_interrupt()
{
    if (*reg16(PCI_CONF_STATUS) & PCI_STATUS_INTERRUPT_MASK) {
        _interrupt_line.raise();
    }
}


uint64_t* PCIDevice::reg64(uint offset)
{
    return (uint64_t*)((uint8_t*)_config_space + offset);
}


uint16_t* PCIDevice::reg16(uint offset)
{
    return (uint16_t*)((uint8_t*)_config_space + offset);
}


uint8_t* PCIDevice::reg8(uint offset)
{
    return (uint8_t*)_config_space + offset;
}


void PCIDevice::write_command(uint16_t val)
{
    Lock lock(_mutex);

    uint16_t* command_reg = reg16(PCI_CONF_COMMAND);
    uint16_t old_val = *command_reg;

    val &= PCI_COMMAND_MASK;
    *command_reg = val;

    if ((val & PCI_COMMAND_ENABLE_IO) != (old_val & PCI_COMMAND_ENABLE_IO)) {

        if (val & PCI_COMMAND_ENABLE_IO) {
            map_io();
            on_io_enabled();
        } else {
            unmap_io();
            on_io_disabled();
        }
    }

    if ((val & PCI_COMMAND_ENABLE_MEM) != (old_val & PCI_COMMAND_ENABLE_MEM)) {

        if (val & PCI_COMMAND_ENABLE_MEM) {
            map_mem();
            map_rom();
        } else {
            unmap_mem();
            unmap_rom();
        }
    }

    if ((val & PCI_COMMAND_DISABLE_INTERRUPT) != (old_val & PCI_COMMAND_DISABLE_INTERRUPT)) {

        if (val & PCI_COMMAND_DISABLE_INTERRUPT) {
            disable_interrupt();
        } else {
            enable_interrupt();
        }
    }
}


void PCIDevice::write_status(uint16_t val)
{
    Lock lock(_mutex);

    uint16_t* status_reg = reg16(PCI_CONF_STATUS);
    val &= PCI_STATUS_RESET_MASK;
    *status_reg &= ~val;
}


void PCIDevice::write_config_byte(uint index, uint offset, uint8_t val)
{
    offset += index << 2;

    switch (offset) {
    case PCI_CONF_CACHE_LINE:
    case PCI_CONF_CACHE_LATENCY:
    case PCI_CONF_INTERRUPT_LINE:
        *reg8(offset) = val;
        break;
    default:
        D_MESSAGE("nop %u val %u", offset, val);
    }
}


void PCIDevice::write_config_word(uint index, uint offset, uint16_t val)
{
    offset += index << 2;

    switch (offset) {
    case PCI_CONF_COMMAND:
        write_command(val);
        break;
    case PCI_CONF_STATUS:
        write_status(val);
        break;
    default:
        write_config_byte(index, 0, val);
        write_config_byte(index, 1, val >> 8);
    };
}


void PCIDevice::set_io_address(uint bar, uint16_t address, bool fix)
{
    ASSERT(_regions[bar]);
    ASSERT(_regions[bar]->is_io());
    ASSERT(address && (address & ~_regions[bar]->get_mask()) == 0);

    uint reg_index = (PCI_CONF_BAR_0 >> 2) + bar;

    _config_space[reg_index] &= ~_regions[bar]->get_mask();
    _config_space[reg_index] |= address;

    if (fix) {
        _regions[bar]->set_fix_address(address);
    }
}

void PCIDevice::write_bar(uint reg_index, uint32_t val)
{
    Lock lock(_mutex);

    uint bar = reg_index - (PCI_CONF_BAR_0 >> 2);
    uint64_t mask;
    uint64_t test_mask;

    if (!_regions[bar]) {
        return;
    }

    if (_regions[bar]->is_io() && (*reg16(PCI_CONF_COMMAND) & PCI_COMMAND_ENABLE_IO)) {
        D_MESSAGE("write to io bar while io is enabled");
    }

    if (_regions[bar]->is_mem() && (*reg16(PCI_CONF_COMMAND) & PCI_COMMAND_ENABLE_MEM)) {
        D_MESSAGE("write to mem bar while mem is enabled");
    }

    if (val == ~0) {
#ifdef PCI_STRICT
        if (!_regions[bar]->is_64bit()) {
            _config_space[reg_index] |= _regions[bar]->get_mask();
            return;
        }

        if (_regions[bar]->is_low_bar(bar)) {
            _regions[bar]->read_back = true;
            return;
        }

        if (_regions[bar - 1]->read_back) {
            uint64_t* reg = reg64((reg_index - 1) << 2);

            _regions[bar - 1]->read_back = false;
            *reg |= _regions[bar]->get_mask();
            return;
        }
#else
        if (_regions[bar]->is_low_bar(bar)) {
            _config_space[reg_index] |= _regions[bar]->get_mask();
            return;
        }

        _config_space[reg_index] |= _regions[bar]->get_mask() >> 32;

        return;
#endif
    }

#ifdef PCI_STRICT
    _regions[bar]->read_back = false;
#endif

    if (_regions[bar]->is_fixed()) {
        D_MESSAGE("fixed 0x%x", val);
        val = _regions[bar]->get_fix_address();
    }

    if (_regions[bar]->is_io()) {
        mask = _regions[bar]->get_mask() & ((1 << 16) - 1);
        test_mask = mask | 0x3;
        _config_space[reg_index] &= (1 << 16) - 1;
    } else if (_regions[bar]->is_low_bar(bar)) {
        mask = _regions[bar]->get_mask();
        test_mask = mask | 0xf;
    } else {
        mask = _regions[bar]->get_mask() >> 32;
        test_mask = mask;
    }

    if (val & ~test_mask) {
        W_MESSAGE("test mask failed. %s %d %s val 0x%x test 0x%lx size 0x%lx",
                  _regions[bar]->is_io() ? "IO" : "MEM",
                  _regions[bar]->is_64bit() ? 64 : 32,
                  _regions[bar]->is_low_bar(bar) ? "low" : "high",
                  val,
                  test_mask,
                  _regions[bar]->size);
    }

    _config_space[reg_index] &= ~mask;
    _config_space[reg_index] |= (val & mask);

    //for now
    if (*reg16(PCI_CONF_COMMAND) & PCI_COMMAND_ENABLE_MEM) {
        map_mem();
    }

    //for now
    if ((*reg16(PCI_CONF_COMMAND) & PCI_COMMAND_ENABLE_IO)) {
        map_io();
    }
}


void PCIDevice::unmap_rom()
{
    if (_firmware) {
        _firmware->unmap();
    }
}


void PCIDevice::map_rom()
{
    if (!(*reg16(PCI_CONF_COMMAND) & PCI_COMMAND_ENABLE_MEM) ||
        !((_config_space[PCI_CONF_ROM_ADDRESS >> 2]) & PCI_ROM_ENABLE_MASK)) {
        return;
    }

    D_MESSAGE("%s: 0x%x 0x%x", get_name().c_str(),
              _config_space[PCI_CONF_ROM_ADDRESS >> 2] & ~PCI_ROM_ENABLE_MASK,
              get_rom_size());

    if (_firmware) {
        _firmware->map(_config_space[PCI_CONF_ROM_ADDRESS >> 2] & ~PCI_ROM_ENABLE_MASK);
    }
}


uint32_t PCIDevice::get_rom_size()
{
    if (!_firmware) {
        return 0;
    }

    return _firmware->get_size();
}


void PCIDevice::write_rom(uint32_t val)
{
    unmap_rom();

    if (val == ~1) {
        _config_space[PCI_CONF_ROM_ADDRESS >> 2] = ~(get_rom_size() - 1);
        return;
    }

    _config_space[PCI_CONF_ROM_ADDRESS >> 2] = val & PCI_ROM_MASK;
    map_rom();
}


void PCIDevice::write_config_dword(uint index, uint32_t val)
{
    switch (index << 2) {
    case PCI_CONF_BAR_0:
    case PCI_CONF_BAR_1:
    case PCI_CONF_BAR_2:
    case PCI_CONF_BAR_3:
    case PCI_CONF_BAR_4:
    case PCI_CONF_BAR_5:
        write_bar(index, val);
        break;
    case PCI_CONF_ROM_ADDRESS:
        write_rom(val);
        break;
    default:
        write_config_word(index, 0, val);
        write_config_word(index, 2, val >> 16);
    }
}


void PCIDevice::reset_config_space()
{
    uint16_t vendor = *reg16(PCI_CONF_VENDOR);
    uint16_t device = *reg16(PCI_CONF_DEVICE);
    uint8_t revision = *reg8(PCI_CONF_REVISION);
    uint8_t class_code1 = *reg8(PCI_CONF_CLASS);
    uint16_t class_code2 = *reg16(PCI_CONF_CLASS + 1);
    uint16_t sub_vendor = *reg16(PCI_CONF_SUBSYS_VENDOR);
    uint16_t sub_device = *reg16(PCI_CONF_SUBSYS_ID);
    uint8_t interrupt_pin = *reg8(PCI_CONF_INTERRUPT_PIN);

    memset(_config_space, 0, sizeof(_config_space));

    *reg16(PCI_CONF_VENDOR) = vendor;
    *reg16(PCI_CONF_DEVICE) = device;
    *reg8(PCI_CONF_REVISION) = revision;
    *reg8(PCI_CONF_CLASS) = class_code1;
    *reg16(PCI_CONF_CLASS + 1) = class_code2;
    *reg16(PCI_CONF_SUBSYS_VENDOR) = sub_vendor;
    *reg16(PCI_CONF_SUBSYS_ID) = sub_device;
    *reg16(PCI_CONF_STATUS) = PCI_STATUS_66MHZ_MASK;
    *reg8(PCI_CONF_INTERRUPT_PIN) = interrupt_pin;
}


void PCIDevice::reset()
{
    reset_config_space();

    for (uint i = 0; i < NUM_BARS; i++) {
        PCIDevice::Region* region = _regions[i];

        if (!region) {
            continue;
        }

        uint reg_index = (PCI_CONF_BAR_0 >> 2) + i;

        if (region->is_io()) {
             _config_space[reg_index] = PCI_BAR_IO_MASK;
        } else {
            _config_space[reg_index] = PCI_BAR_PREFATCH_MASK;

            if (region->is_64bit()) {
                i++;
                _config_space[reg_index] |= PCI_BAR_MEM_TYPE_64BITS << PCI_BAR_MEM_TYPE_SHIFT;
            }
        }

        if (region->is_fixed()) {
            _config_space[reg_index] |= region->get_fix_address();
        }

        region->reset();
    }

    _interrupt_line.reset();

    if (_firmware) {
        _firmware->reload();
    }
}

