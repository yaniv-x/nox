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

#include <fcntl.h>
#include <sys/stat.h>

#include "cpu.h"
#include "application.h"
#include "nox_vm.h"
#include "memory_bus.h"
#include "io_bus.h"
#include "kvm.h"
#include "dma.h"
#include "place_holder.h"
#include "pic.h"
#include "pit.h"
#include "pci_bus.h"
#include "ata_host.h"
#include "ata_disk.h"
#include "atapi_cdrom.h"
#include "cmos.h"
#include "keyboard.h"
#include "vga.h"
#include "display.h"
#include "admin_server.h"
#include "pci.h"

enum {
    IO_PORT_MISC = 0x61,
    IO_PORT_POST_DIAGNOSTIC = 0x80,
    IO_PORT_A20 = 0x92,
    IO_PORT_BOCHS_PANIC_1 = 0x400,
    IO_PORT_BOCHS_PANIC_2 = 0x401,
    IO_PORT_BOCHS_INFO = 0x402,
    IO_PORT_BOCHS_DEBUG = 0x403,
    IO_PORT_BOCHS_MY_TEST = 0x404,

    IO_PORT_VGA_BIOS_MESSAGE = 0x500,

    LOW_RAM_SIZE = 640 * KB,
    MID_RAM_START = 768 * KB,
    MID_RAM_MAX_ADDRESS = 0xc0000000, // is hardcoded in bios
    MID_RAM_MAX = MID_RAM_MAX_ADDRESS - MID_RAM_START,

    BIOS_FOOTER_SIZE = 16,
};

#define NOX_PCI_DEV_HOST_BRIDGE_REV 1
#define NOX_PCI_DEV_ISA_BRIDGE_REV 1

enum {
    PLATFORM_IO_LOCK = 0x00,
    PLATFORM_IO_INDEX = 0x01,
    PLATFORM_IO_LOG = 0x02,
    PLATFORM_IO_ERROR = 0x04,
    PLATFORM_IO_DATA = 0x08,
    PLATFORM_IO_END = 0x0c,

    PLATFORM_RAM_PAGES = 1,
    PLATFORM_LOG_BUF_SIZE = 1024,
};

enum {
    PLATFORM_REG_BELOW_1M_USED_PAGES,
    PLATFORM_REG_ABOVE_1M_PAGES,
    PLATFORM_REG_BELOW_4G_PAGES,
    PLATFORM_REG_BELOW_4G_USED_PAGES,
    PLATFORM_REG_ABOVE_4G_PAGES,
};


class PCIHost: public PCIDevice {
public:
    PCIHost(PCIBus& bus)
        : PCIDevice("host-bridge", bus , NOX_PCI_VENDOR_ID, NOX_PCI_DEV_ID_HOST_BRIDGE,
                NOX_PCI_DEV_HOST_BRIDGE_REV, mk_pci_class_code(PCI_CLASS_BRIDGE,
                                                                 PCI_SUBCLASS_BRIDGE_HOST, 0),
                false)
        , _ram (memory_bus->alloc_physical_ram(*this, PLATFORM_RAM_PAGES, "platform ram"))
    {
        ASSERT(PLATFORM_LOG_BUF_SIZE <= (PLATFORM_RAM_PAGES << GUEST_PAGE_SHIFT));
        add_io_region(0, PLATFORM_IO_END, this, (io_read_byte_proc_t)&PCIHost::io_read_byte,
                      (io_write_byte_proc_t)&PCIHost::io_write_byte,
                      NULL, NULL,
                      (io_read_dword_proc_t)&PCIHost::io_read_dword,
                      (io_write_dword_proc_t)&PCIHost::io_write_dword);

        add_mem_region(1, _ram, false);

        bus.add_device(*this);
    }

    ~PCIHost()
    {
        memory_bus->release_physical_ram(_ram);
    }

    virtual uint get_hard_id() { return 0;}


    uint8_t* get_ram_ptr() { return memory_bus->get_physical_ram_ptr(_ram);}

private:
    uint8_t io_read_byte(uint16_t port)
    {
        return get_nox().platform_port_read_byte(port - get_region_address(0));
    }

    void io_write_byte(uint16_t port, uint8_t val)
    {
        get_nox().platform_port_write_byte(port - get_region_address(0), val);
    }

    uint32_t io_read_dword(uint16_t port)
    {
        return get_nox().platform_port_read_dword(port - get_region_address(0));
    }

    void io_write_dword(uint16_t port, uint32_t val)
    {
        get_nox().platform_port_write_dword(port - get_region_address(0), val);
    }

private:
    PhysicalRam* _ram;
};


class EISABridge : public PCIDevice {
public:
    EISABridge(PCIBus& bus)
        : PCIDevice("eisa-bridge", bus, NOX_PCI_VENDOR_ID,
                                NOX_PCI_DEV_ID_ISA_BRIDGE,
                                NOX_PCI_DEV_ISA_BRIDGE_REV,
                                mk_pci_class_code(PCI_CLASS_BRIDGE, PCI_SUBCLASS_BRIDGE_ISA, 0),
                                false)
    {
        bus.add_device(*this);
    }

    virtual uint get_hard_id() { return 1;}
};


class NoxVM::StateChangeRequest: public NonCopyable {
public:
    StateChangeRequest(NoxVM::compleation_routin_t cb, void* opaque)
        : _cb (cb)
        , _opaque (opaque)
        , _started (false)
    {
    }

    void notify(bool ok)
    {
        if (_cb) {
            _cb(_opaque, ok);
        }
    }

    bool execute(NoxVM& vm)
    {
        if (_started) {
            return cont(vm);
        }

        _started = true;
        return start(vm);
    }

    bool verify_start_condition(NoxVM& vm)
    {
        if (_started) {
            return true;
        }

        return test(vm);
    }

protected:
    virtual bool test(NoxVM& vm) = 0;
    virtual bool start(NoxVM& vm) = 0;
    virtual bool cont(NoxVM& vm) = 0;

private:
    NoxVM::compleation_routin_t _cb;
    void* _opaque;
    bool _started;
};



NoxVM::NoxVM()
    : VMPart ("Nox")
    , _kvm (new KVM())
    , _io_bus (new IOBus(*this))
    , _mem_bus (new MemoryBus(*this))
    , _holder (new PlaceHolder(*this))
    , _pic (new PIC(*this))
    , _pci (new PCIBus(*this))
    , _pci_host (new PCIHost(*_pci.get()))
    , _eisa_bridge (new EISABridge(*_pci.get()))
    , _cmos (new CMOS(*this))
    , _dma (new DMA(*this))
    , _pit (new PIT(*this))
    , _kbd (new KbdController(*this))
    , _ata_host (new ATAHost())
    , _vga (new VGA(*this))
    , _low_ram (NULL)
    , _mid_ram (NULL)
    , _high_bios (NULL)
    , _high_ram (NULL)
    , _ram_size (MB)
    , _free_high_bios_pages (0)
    , _num_cpus (1)
    , _ro_hard_disk_file (false)
    , _hard_disk_size (0)
    , _cdrom (false)
    , _boot_from_cdrom (false)
{
    add_io_region(_io_bus->register_region(*this, IO_PORT_A20, 1, this,
                                           (io_read_byte_proc_t)&NoxVM::a20_port_read,
                                           (io_write_byte_proc_t)&NoxVM::a20_port_write));

    add_io_region(_io_bus->register_region(*this, IO_PORT_BOCHS_PANIC_1, 5, this,
                                           NULL,
                                           (io_write_byte_proc_t)&NoxVM::bochs_port_write));

    add_io_region(_io_bus->register_region(*this, IO_PORT_POST_DIAGNOSTIC, 1, this,
                                           NULL,
                                           (io_write_byte_proc_t)&NoxVM::post_diagnostic));


    add_io_region(_io_bus->register_region(*this, IO_PORT_VGA_BIOS_MESSAGE, 1, this,
                                           NULL,
                                           (io_write_byte_proc_t)&NoxVM::vgabios_port_write));

    add_io_region(_io_bus->register_region(*this, IO_PORT_MISC, 1, this,
                                           (io_read_byte_proc_t)&NoxVM::misc_port_read,
                                           (io_write_byte_proc_t)&NoxVM::misc_port_write));

    register_admin_commands();
}


NoxVM::~NoxVM()
{
    while (!_dynamic_parts.empty()) {
        delete _dynamic_parts.front();
        _dynamic_parts.pop_front();
    }

    while (!_stat_change_req_list.empty()) {
        delete _stat_change_req_list.front();
        _stat_change_req_list.pop_front();
    }

    _mem_bus->unmap_physical_ram(_low_ram);
    _mem_bus->release_physical_ram(_low_ram);
    _mem_bus->unmap_physical_ram(_mid_ram);
    _mem_bus->release_physical_ram(_mid_ram);
    _mem_bus->unmap_physical_ram(_high_bios);
    _mem_bus->release_physical_ram(_high_bios);
    _mem_bus->unmap_physical_ram(_high_ram);
    _mem_bus->release_physical_ram(_high_ram);

    unregister_regions();
}


void NoxVM::register_admin_commands()
{
    AdminServer* admin = application->get_admin();

    va_type_list_t output_args(2);

    output_args[0] = VA_UINT32_T;     // result
    output_args[1] = VA_UTF8_T;       // error string

    va_names_list_t output_names(2);

    output_names[0] = "result";
    output_names[1] = "error-string";


    admin->register_command("suspend", "suspend virtual machin execution", "???",
                            empty_va_type_list, empty_names_list, output_args, output_names,
                            (admin_command_handler_t)&NoxVM::suspend_command, this);

    admin->register_command("resume", "resume virtual machin execution", "???",
                            empty_va_type_list, empty_names_list, output_args, output_names,
                            (admin_command_handler_t)&NoxVM::resume_command, this);

    admin->register_command("restart", "restart the virtual machin", "???",
                            empty_va_type_list, empty_names_list, output_args, output_names,
                            (admin_command_handler_t)&NoxVM::restart_command, this);

    admin->register_command("terminate", "terminate virtual machin execution", "???",
                            empty_va_type_list, empty_names_list, output_args, output_names,
                            (admin_command_handler_t)&NoxVM::terminate_command, this);
}


void NoxVM::reset_bios_stuff()
{
    // todo: let the bios handle all the following stuff

    _cmos->host_write(0x10, 0);      // no floppy

    //equipment byte
    _cmos->host_write(0x14, (1 << 1 /* math coproccssor*/) | (1 << 2) /* mouse port*/);

    //num of cpus
    _cmos->host_write(0x5f, _num_cpus - 1);

    //century (BCD)
    _cmos->host_write(0x32, 0x20); // IBM
    _cmos->host_write(0x37, 0x20); // PS2

    /****************************************** mem ***********************************************/
    //640k base memory
    _cmos->host_write(0x15, (LOW_RAM_SIZE / KB));
    _cmos->host_write(0x16, (LOW_RAM_SIZE / KB) >> 8);

    //extended memory
    uint64_t ram_size_kb = _ram_size / 1024;
    ram_size_kb -= 1024;
    ram_size_kb = (ram_size_kb > ((1 << 16) - 1)) ? ((1 << 16) - 1) : ram_size_kb;
    _cmos->host_write(0x17, ram_size_kb);
    _cmos->host_write(0x18, ram_size_kb >> 8);
    _cmos->host_write(0x30, ram_size_kb);
    _cmos->host_write(0x31, ram_size_kb >> 8);

    uint64_t below_4G;
    uint64_t above_4G;
    if (_ram_size > MID_RAM_MAX_ADDRESS) {
        below_4G = MID_RAM_MAX_ADDRESS;
        above_4G = _ram_size - MID_RAM_MAX_ADDRESS;
    } else {
        below_4G = _ram_size;
        above_4G = 0;
    }

    //above 16MB
    uint64_t above_16M;

    if (below_4G > 16 * MB) {
        // (64k blocks)
        above_16M = (below_4G - 16 * MB) >> 16;
    } else {
        above_16M = 0;
    }

    _cmos->host_write(0x34, above_16M);
    _cmos->host_write(0x35, above_16M >> 8);

    //abov 4g (64k blocks)
    _cmos->host_write(0x5b, above_4G >> 16);
    _cmos->host_write(0x5c, above_4G >> 24);
    _cmos->host_write(0x5d, above_4G >> 32);

    /****************************************** mem end *******************************************/

    if (_boot_from_cdrom) {
        _cmos->host_write(0x3d, 0x03); //first boot device is CD
    } else {
        _cmos->host_write(0x3d, 0x02); //first boot device is first HD
    }


    /**************************************** hard disk *******************************************/

    enum {
        MAX_HEADS = 16,
        MAX_SECTORS_PER_CYL = 63,
        MAX_CYL = 16383,
        SECTOR_SIZE = 512,
    };

    if (_hard_disk_size) {
        _cmos->host_write(0x12, 0xf0);   // hard disk 0 in extended cmos
        _cmos->host_write(0x19, 47);     // user define hd (params start in 0x1b)

        uint cyl = MIN(MAX_CYL, _hard_disk_size / MAX_HEADS / MAX_SECTORS_PER_CYL / SECTOR_SIZE);

        // hd 0 params as in AMI BIOS
        _cmos->host_write(0x1b, cyl);
        _cmos->host_write(0x1c, cyl >> 8);
        _cmos->host_write(0x1d, MAX_HEADS);
        _cmos->host_write(0x1e, 0xff); // precompensation
        _cmos->host_write(0x1f, 0xff); // precompensation
        _cmos->host_write(0x20, 0xc0 | ((MAX_SECTORS_PER_CYL > 8) << 3));
        _cmos->host_write(0x21, cyl); // landing zone
        _cmos->host_write(0x22, cyl >> 8); // landing zone
        _cmos->host_write(0x23, MAX_SECTORS_PER_CYL);
    } else {
        _cmos->host_write(0x12, 0);
        _cmos->host_write(0x19, 0);
        _cmos->host_write(0x1b, 0);
        _cmos->host_write(0x1c, 0);
        _cmos->host_write(0x1d, 0);
        _cmos->host_write(0x1e, 0);
        _cmos->host_write(0x1f, 0);
        _cmos->host_write(0x20, 0);
        _cmos->host_write(0x21, 0);
        _cmos->host_write(0x22, 0);
        _cmos->host_write(0x23, 0);
    }

    _cmos->host_write(0x39, 0x55); //use LBA translation

    /************************************** hard disk done ****************************************/
}


void NoxVM::reset()
{
    ASSERT(_state == INIT_DONE || _state == READY || _state == STOPPED);

    _state = RESETING;

    reset_childrens();

    remap_io_regions();

    _a20_port_val = 0;
    _misc_port  = 0x01;
    _platform_lock = 0;
    _platform_reg_index = 0;
    _nmi_mask = true;

    _mem_bus->map_physical_ram(_low_ram, 0, false);
    _mem_bus->map_physical_ram(_mid_ram, MID_RAM_START >> GUEST_PAGE_SHIFT, false);
    _mem_bus->map_physical_ram(_high_bios, ((4ULL * GB) - MB) >> GUEST_PAGE_SHIFT, false);

    if (_high_ram) {
        _mem_bus->map_physical_ram(_high_ram, (4ULL * GB) >> GUEST_PAGE_SHIFT, false);
    }

    load_bios();
    reset_bios_stuff();

    _state = READY;
}


class CommandReply {
public:
    CommandReply(AdminReplyContext* context)
        : _context (context)
    {

    }

    void reply(bool ok)
    {
        _context->command_reply(ok ? 0 : 1, ok ? "": "failed");
        delete this;
    }

private:
    AdminReplyContext* _context;
};


void NoxVM::suspend_command(AdminReplyContext* context)
{
    D_MESSAGE("");

    CommandReply* r = new CommandReply(context);

    vm_stop((compleation_routin_t)&CommandReply::reply, r);
}


void NoxVM::resume_command(AdminReplyContext* context)
{
    D_MESSAGE("");

    CommandReply* r = new CommandReply(context);

    vm_start((compleation_routin_t)&CommandReply::reply, r);
}


void NoxVM::restart_command(AdminReplyContext* context)
{
    D_MESSAGE("");

    CommandReply* r = new CommandReply(context);

    vm_restart((compleation_routin_t)&CommandReply::reply, r);
}


class TerminateReply {
public:
    TerminateReply(AdminReplyContext* context)
        : _context (context)
    {

    }

    void reply(bool ok)
    {
        if (ok) {
            _context->command_reply(0, "");
            application->quit();
        } else {
            _context->command_reply(1, "failed");
        }

        delete this;
    }

private:
    AdminReplyContext* _context;
};


void NoxVM::terminate_command(AdminReplyContext* context)
{
    D_MESSAGE("");

    TerminateReply* r = new TerminateReply(context);

    vm_down((compleation_routin_t)&TerminateReply::reply, r);
}


void NoxVM::a20_port_write(uint16_t port, uint8_t val)
{
    ASSERT(port == IO_PORT_A20);
    if (val & 1) {
        throw ResetException();
    }

    bool enable_A20 = !!(val & 2);

    if (enable_A20) {
        _mem_bus->enable_address_line_20();
    } else {
        _mem_bus->disable_address_line_20();
    }
}


uint8_t NoxVM::a20_port_read(uint16_t port)
{
    ASSERT(port == 0x92);

    return _mem_bus->line_20_is_set() ? 0x02 : 0;
}

void NoxVM::misc_port_write(uint16_t port, uint8_t val)
{
    _misc_port = val & 0x0f;
    _pit->set_gate_level(2, !!(_misc_port & 0x1));
}


uint8_t NoxVM::misc_port_read(uint16_t port)
{
    _misc_port ^= 0x10;
    //todo: get gate level from pit
    return _misc_port | (_pit->get_output_level(2) ? 0x20 : 0);
}


void NoxVM::bochs_port_write(uint16_t port, uint8_t val)
{
    switch (port) {
    case 0x400:
    case 0x401:
        D_MESSAGE("0x%x", val);
        PANIC("bochs panic");
        break;
    case 0x402:
    case 0x403:
        printf("%c", val);

        if (val == '\n') {
            fflush(stdout);
        }

        break;
    case IO_PORT_BOCHS_MY_TEST:
        D_MESSAGE("my %u", val);
    };
}


void NoxVM::vgabios_port_write(uint16_t port, uint8_t val)
{
    if (val == '\r') {
        return;
    }

    printf("%c", val);

    if (val == '\n') {
        fflush(stdout);
    }
}


void NoxVM::post_diagnostic(uint16_t port, uint8_t val)
{
    ASSERT(port == IO_PORT_POST_DIAGNOSTIC);

    //printf("POST 0x%x\n", val);
}


uint8_t NoxVM::platform_port_read_byte(uint16_t port)
{
    switch (port) {
    case PLATFORM_IO_LOCK:
        return _platform_lock;
    default:
        D_MESSAGE("unexpected port 0x%x", port);
        return 0xff;
    }
}

static void replace_none_printable(char* str)
{
    for (; *str; str++) {
        if (!isprint(*str)) {
            *str = '?';
        }
    }
}


void NoxVM::platform_port_write_byte(uint16_t port, uint8_t val)
{
    switch (port) {
    case PLATFORM_IO_LOCK:
        _platform_lock = 1;
        break;
    case PLATFORM_IO_INDEX:
        _platform_reg_index = val;
        break;
    case PLATFORM_IO_LOG: {
        AutoArray<uint8_t> str_copy(new uint8_t[PLATFORM_LOG_BUF_SIZE]);
        strncpy((char*)str_copy.get(), (char*)_pci_host->get_ram_ptr(), PLATFORM_LOG_BUF_SIZE - 1);
        replace_none_printable((char*)str_copy.get());
        str_copy[PLATFORM_LOG_BUF_SIZE - 1] = 0;
        D_MESSAGE("guest: %s", str_copy.get());
        break;
    }
    default:
        D_MESSAGE("unexpected port 0x%x val %u", port, val);
    }
}


uint32_t NoxVM::platform_port_read_dword(uint16_t port)
{
    switch (port) {
    case PLATFORM_IO_DATA:
        switch (_platform_reg_index) {
        case PLATFORM_REG_BELOW_1M_USED_PAGES:
            return _bios_pages;
        case PLATFORM_REG_ABOVE_1M_PAGES: {
            uint64_t uma_ram = (MB - MID_RAM_START);
            return (_mem_bus->get_physical_ram_size(_mid_ram) - uma_ram) >> GUEST_PAGE_SHIFT;
        }
        case PLATFORM_REG_BELOW_4G_PAGES:
            return _mem_bus->get_physical_ram_size(_high_bios) >> GUEST_PAGE_SHIFT;
        case PLATFORM_REG_BELOW_4G_USED_PAGES: {
            uint32_t pages = _mem_bus->get_physical_ram_size(_high_bios) >> GUEST_PAGE_SHIFT;
            return pages - _free_high_bios_pages;
        }
        case PLATFORM_REG_ABOVE_4G_PAGES:
            return _high_ram ? _mem_bus->get_physical_ram_size(_high_ram) >> GUEST_PAGE_SHIFT : 0;
        default:
            return 0;
        }
    default:
        D_MESSAGE("unexpected port 0x%x", port);
        return 0xffffffff;
    }
}


void NoxVM::platform_port_write_dword(uint16_t port, uint32_t val)
{
    switch (port) {
    case PLATFORM_IO_ERROR:
        D_MESSAGE("platform error code 0x%x", val);
        return;
    default:
        D_MESSAGE("unexpected port 0x%x val %u", port, val);
    }
}


void NoxVM::alloc_high_bios_pages(uint num_pages/*, uint8_t** ptr, page_address_t* address*/)
{
    if (_free_high_bios_pages < num_pages) {
        THROW("bios pages oom");
    }

    _free_high_bios_pages--;
}


void NoxVM::init_ram()
{
    _low_ram = _mem_bus->alloc_physical_ram(*this, LOW_RAM_SIZE >> GUEST_PAGE_SHIFT, "low ram");

    uint64_t ram_size = _ram_size - MB; // "lost" 128K (vga fb hole @ 0xa0000-0xc0000)

    uint64_t uma_ram_size = MB - MID_RAM_START;
    uint64_t mid_range_size = MIN(ram_size, MID_RAM_MAX_ADDRESS - MB) + uma_ram_size;

    _mid_ram = _mem_bus->alloc_physical_ram(*this, mid_range_size >> GUEST_PAGE_SHIFT, "mid ram");
    ram_size -= mid_range_size - uma_ram_size;

    if (ram_size) {
        _high_ram = _mem_bus->alloc_physical_ram(*this, ram_size >> GUEST_PAGE_SHIFT,
                                                 "high ram");
    }

    _high_bios = _mem_bus->alloc_physical_ram(*this, MB >> GUEST_PAGE_SHIFT, "high bios");
    _free_high_bios_pages = MB >> GUEST_PAGE_SHIFT;

    alloc_high_bios_pages(1);
}


void NoxVM::load_bios()
{
    // BIOS scan for existence of valid expansion ROMS.
    //   Video ROM:   from 0xC0000..0xC7FFF in 2k increments
    //   General ROM: from 0xC8000..0xDFFFF in 2k increments
    //   System  ROM: only 0xE0000
    //
    // Header:
    //   Offset    Value
    //   0         0x55
    //   1         0xAA
    //   2         ROM length in 512-byte blocks
    //   3         ROM initialization entry point (FAR CALL)

    uint8_t* ptr = _mem_bus->get_physical_ram_ptr(_mid_ram);
    ptr += MB - MID_RAM_START;

    std::string pc_bios_file = application->get_nox_dir() + "/firmware/pc-bios.bin";

    AutoFD _bios_fd(::open(pc_bios_file.c_str(), O_RDONLY));

    if (!_bios_fd.is_valid()) {
        THROW("open failed");
    }
    struct stat stat;

    if (fstat(_bios_fd.get(), &stat) == -1) {
        THROW("fstat failed");
    }

    if (stat.st_size > MB - MID_RAM_START) {
        THROW("bad bios size");
    }

    _bios_pages = ALIGN(stat.st_size, GUEST_PAGE_SIZE) >> GUEST_PAGE_SHIFT;
    ptr -= stat.st_size;

    if (read(_bios_fd.get(), ptr, stat.st_size) != stat.st_size) {
        THROW("read failed");
    }

    uint8_t* footer_ptr = _mem_bus->get_physical_ram_ptr(_high_bios);
    footer_ptr += _mem_bus->get_physical_ram_size(_high_bios);
    footer_ptr -= BIOS_FOOTER_SIZE;

    memcpy(footer_ptr, ptr + stat.st_size - BIOS_FOOTER_SIZE, BIOS_FOOTER_SIZE);

    std::string vga_bios_file = application->get_nox_dir() + "/firmware/vga-bios.bin";

    AutoFD _vga_fd(::open(vga_bios_file.c_str(), O_RDONLY));

    if (fstat(_vga_fd.get(), &stat) == -1) {
        THROW("fstat failed");
    }

    ptr = _mem_bus->get_physical_ram_ptr(_mid_ram);
    ptr += 0xc0000 - MID_RAM_START;
    if (read(_vga_fd.get(), ptr, stat.st_size) != stat.st_size) {
        THROW("read failed");
    }
}


void NoxVM::init_cpus()
{
    for (int i = 0; i < _num_cpus; i++) {
        _dynamic_parts.push_back(new CPU(*this, i));
    }
}


void NoxVM::set_ram_size(uint32_t ram_size_mb)
{
    ASSERT(ram_size_mb);
    ASSERT(_state == INIT);

    uint64_t ram_size = ram_size_mb;
    ram_size *= MB;
    _ram_size = ram_size;
}


void NoxVM::set_hard_disk(const char* file_name, bool read_only)
{
    ASSERT(_state == INIT);
    _hard_disk_file_name = file_name ? file_name : "";
    _ro_hard_disk_file = read_only;
}


void NoxVM::init_hard_disk()
{
    if (!_hard_disk_file_name.length()) {
        return;
    }

    ATADiskFactory factory(_hard_disk_file_name.c_str(), _ro_hard_disk_file);
    _ata_host->set_device_0(factory);
    _hard_disk_size = factory.get_size();
}


void NoxVM::set_cdrom(const char* file_name)
{
    ASSERT(_state == INIT);

    _cdrom = true;
    _cdrom_file_name = file_name ? file_name : "";
}


void NoxVM::init_cdrom()
{
    if (!_cdrom) {
        return;
    }

    ATAPICdromFactory factory(_cdrom_file_name.c_str());
    _ata_host->set_device_1(factory);
}


void NoxVM::set_boot_device(bool from_cdrom)
{
    ASSERT(_state == INIT);
    _boot_from_cdrom = from_cdrom;
}


void NoxVM::init()
{
    ASSERT(_state == INIT);

    new NoxDisplay(*_vga.get(), *_kbd.get());

    init_ram();
    init_hard_disk();
    init_cdrom();
    init_cpus();

    _state = INIT_DONE;
}


bool NoxVM::start()
{
    _state = STARTING;

    if (!start_childrens()) {
        return false;
    }

    _state = RUNNING;
    return true;
}


bool NoxVM::stop()
{
    _state = STOPPING;

    if (!stop_childrens()) {
        return false;
    }

    _state = STOPPED;
    return true;
}


void NoxVM::power()
{

}


void NoxVM::save(OutStream& stream)
{

}


void NoxVM::load(InStream& stream)
{

}


class StateChngeTask: public Task {
public:
    StateChngeTask(NoxVM& vm) : Task(), _vm (vm) {}

    virtual void execute()
    {
        _vm.handle_state_request();
    }

private:
    NoxVM& _vm;
};


class StartRequest: public NoxVM::StateChangeRequest {
public:
    StartRequest(NoxVM::compleation_routin_t cb, void* opaque);

    virtual bool test(NoxVM& vm);
    virtual bool start(NoxVM& vm);
    virtual bool cont(NoxVM& vm);
};


StartRequest::StartRequest(NoxVM::compleation_routin_t cb, void* opaque)
    : NoxVM::StateChangeRequest(cb, opaque)
{
}

bool StartRequest::test(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::READY:
    case VMPart::STOPPED:
    case VMPart::RUNNING:
        return true;
    default:
        return false;
    }
}

bool StartRequest::start(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::READY:
    case VMPart::STOPPED:
        return vm.start();
    case VMPart::SHUTING_DOWN:
    case VMPart::DOWN:
    case VMPart::RUNNING:
        return true;
    default:
        PANIC("unexpected state");
        return false;
    }
}


bool StartRequest::cont(NoxVM& vm)
{
    return vm.start();
}


void NoxVM::vm_start(NoxVM::compleation_routin_t cb, void* opaque)
{
    Lock lock(_vm_state_mutex);

    _stat_change_req_list.push_back(new StartRequest(cb, opaque));

    if (_stat_change_req_list.size() == 1) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


class StopRequest: public NoxVM::StateChangeRequest {
public:
    StopRequest(NoxVM::compleation_routin_t cb, void* opaque);

    virtual bool test(NoxVM& vm);
    virtual bool start(NoxVM& vm);
    virtual bool cont(NoxVM& vm);
};


StopRequest::StopRequest(NoxVM::compleation_routin_t cb, void* opaque)
    : NoxVM::StateChangeRequest(cb, opaque)
{
}

bool StopRequest::test(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::READY:
    case VMPart::STOPPED:
    case VMPart::SHUTING_DOWN:
    case VMPart::DOWN:
    case VMPart::RUNNING:
        return true;
    default:
        return false;
    }
}

bool StopRequest::start(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::READY:
    case VMPart::STOPPED:
    case VMPart::SHUTING_DOWN:
    case VMPart::DOWN:
        return true;
    case VMPart::RUNNING:
        return vm.stop();
    default:
        PANIC("unexpected state");
        return false;
    }
}


bool StopRequest::cont(NoxVM& vm)
{
    return vm.stop();
}


void NoxVM::vm_stop(NoxVM::compleation_routin_t cb, void* opaque)
{
    Lock lock(_vm_state_mutex);

    _stat_change_req_list.push_back(new StopRequest(cb, opaque));

    if (_stat_change_req_list.size() == 1) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


class DownRequest: public NoxVM::StateChangeRequest {
public:
    DownRequest(NoxVM::compleation_routin_t cb, void* opaque)
        : NoxVM::StateChangeRequest(cb, opaque)
    {
    }

    virtual bool test(NoxVM& vm);
    virtual bool start(NoxVM& vm) { vm.set_down(); return true;}
    virtual bool cont(NoxVM& vm) { PANIC("unexpected") return false;}
};


bool DownRequest::test(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::STOPPED:
    case VMPart::DOWN:
        return true;
    default:
        return false;
    }
}


void NoxVM::set_down()
{
    down_all();
}


void NoxVM::vm_down(compleation_routin_t cb, void* opaque)
{
    Lock lock(_vm_state_mutex);

    _stat_change_req_list.push_back(new StopRequest(NULL, NULL));
    _stat_change_req_list.push_back(new DownRequest(cb, opaque));

    if (_stat_change_req_list.size() == 2) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


void NoxVM::resume_mode_change()
{
    AutoRef<StateChngeTask> task(new StateChngeTask(*this));
    application->add_task(task.get());
}


void NoxVM::handle_state_request()
{
    Lock lock(_vm_state_mutex);

    if (_stat_change_req_list.empty()) {
        D_MESSAGE("empty");
        return;
    }

    StateChangeRequest* request = _stat_change_req_list.front();

    if (!request->verify_start_condition(*this)) {
        request->notify(false);
    } else {
        if (!request->execute(*this)) {
            return;
        }

        request->notify(true);
    }

    _stat_change_req_list.pop_front();
    delete request;

    if (!_stat_change_req_list.empty()) {
         AutoRef<StateChngeTask> task(new StateChngeTask(*this));
         application->add_task(task.get());
    }
}


void NoxVM::vm_reset()
{
    Lock lock(_vm_state_mutex);
    reset();
}


class ResetRequest: public NoxVM::StateChangeRequest {
public:
    ResetRequest()
        : NoxVM::StateChangeRequest(NULL, NULL)
    {
    }

    virtual bool test(NoxVM& vm);
    virtual bool start(NoxVM& vm) { vm.reset(); return true;}
    virtual bool cont(NoxVM& vm) { PANIC("unexpected") return false;}
};



bool ResetRequest::test(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::INIT_DONE:
    case VMPart::READY:
    case VMPart::STOPPED:
        return true;
    default:
        return false;
    }
}


void NoxVM::vm_restart(compleation_routin_t cb, void* opaque)
{
    Lock lock(_vm_state_mutex);
    _stat_change_req_list.push_back(new StopRequest(NULL, NULL));
    _stat_change_req_list.push_back(new ResetRequest());
    _stat_change_req_list.push_back(new StartRequest(cb, opaque));

    if (_stat_change_req_list.size() == 3) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}

