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
#include "io_apic.h"
#include "pit.h"
#include "pci_bus.h"
#include "ata_host.h"
#include "ata_disk.h"
#include "atapi_cdrom.h"
#include "cmos.h"
#include "keyboard.h"
#include "vga.h"
#include "pm_controller.h"
#include "display.h"
#include "admin_server.h"
#include "pci.h"
#include "speaker.h"
#include "nox.h"
#include "firmware_file.h"

enum {
    IO_PORT_MISC = 0x61,
    IO_PORT_POST_DIAGNOSTIC = 0x300,
    IO_PORT_A20 = 0x92,

    LOW_RAM_SIZE = 640 * KB,
    MID_RAM_START = 768 * KB,
    MID_RAM_MAX_ADDRESS = 0xc0000000, // is dynamic in newb bios
    MID_RAM_MAX = MID_RAM_MAX_ADDRESS - MID_RAM_START,
    HIGH_BIOS_SIZE = MB,
    BIOS_FOOTER_SIZE = 16,
};


#define NOX_PCI_DEV_ISA_BRIDGE_REV 1


class PCIHost: public PCIDevice {
public:
    PCIHost(PCIBus& bus)
        : PCIDevice("host-bridge", bus , NOX_PCI_VENDOR_ID, NOX_PCI_DEV_ID_HOST_BRIDGE,
                NOX_PCI_DEV_HOST_BRIDGE_REV, mk_pci_class_code(PCI_CLASS_BRIDGE,
                                                                 PCI_SUBCLASS_BRIDGE_HOST, 0),
                false)
        , _ram (memory_bus->alloc_physical_ram(*this, PLATFORM_MEM_PAGES, "platform ram"))
    {
        ASSERT(PLATFORM_LOG_BUF_SIZE <= (PLATFORM_MEM_PAGES << GUEST_PAGE_SHIFT));
        add_io_region(0, PLATFORM_IO_END, this, (io_read_byte_proc_t)&PCIHost::io_read_byte,
                      (io_write_byte_proc_t)&PCIHost::io_write_byte,
                      NULL, NULL,
                      (io_read_dword_proc_t)&PCIHost::io_read_dword,
                      (io_write_dword_proc_t)&PCIHost::io_write_dword);

        add_mem_region(1, _ram, false);

        bus.add_device(*this);

        ASSERT(NOX_HOST_BRIDGE_BIOS_OFFSET == (CONFIG_BASE_INDEX + CONFIG_WORD_BIOS) * 4);
        ASSERT(NOX_HOST_BRIDGE_STEERING_OFFSET == (CONFIG_BASE_INDEX + CONFIG_WORD_STEERING) * 4);
        ASSERT(NOX_STEERING_LINK == LINK);
        ASSERT(NOX_STEERING_IRQ == IRQ);
        ASSERT(NOX_STEERING_STATE == STATE);
        ASSERT(NOX_STEERING_DISABLE == DISABLE);
        ASSERT(NOX_STEERING_ERROR_MASK == STEERING_ERROR_MASK);
        ASSERT(NOX_STEERING_ENABLE_MASK == STEERING_ENABLE_MASK);
    }

    ~PCIHost()
    {
        memory_bus->unref_physical_ram(_ram);
    }

    virtual uint get_hard_id() { return HOST_BRIDGE_SLOT;}

    uint8_t* get_ram_ptr() { return memory_bus->get_physical_ram_ptr(_ram);}

    enum {
        CONFIG_BASE_OFFSET = PCIDevice::CONFIG_SIZE,
        CONFIG_BASE_INDEX = CONFIG_BASE_OFFSET / 4,

        CONFIG_WORD_STEERING = 0,
        CONFIG_WORD_BIOS,
        NUM_CONFIG_WORDS,

        LINK = 0,
        IRQ,
        STATE,
        DISABLE,

        STEERING_ERROR_MASK = 0x01,
        STEERING_ENABLE_MASK = 0x02,
    };

protected:
    virtual uint8_t read_config_byte(uint index, uint offset)
    {
        if (index != CONFIG_BASE_INDEX + CONFIG_WORD_STEERING) {
            return PCIDevice::read_config_byte(index, offset);
        }

        ASSERT(offset < 4);

        return ((uint8_t*)&_conf_regs[CONFIG_WORD_STEERING])[offset];
    }

    virtual void write_config_byte(uint index, uint offset, uint8_t val)
    {
        if (index != CONFIG_BASE_INDEX + CONFIG_WORD_STEERING) {
            PCIDevice::write_config_byte(index, offset, val);
            return;
        }

        uint32_t current = _conf_regs[CONFIG_WORD_STEERING];
        uint8_t* regs = (uint8_t*)&current;

        switch (offset) {
        case LINK: {
            if (val >= NOX_PCI_NUM_INT_LINKS) {
                regs[STATE] |= STEERING_ERROR_MASK;
                break;
            }

            regs[LINK] = val;
            regs[STATE] &= ~(STEERING_ERROR_MASK | STEERING_ENABLE_MASK);

            PCIBus* bus = (PCIBus*)get_container();
            uint8_t irq;
            bool enabled;

            bus->get_link_state(val, irq, enabled);

            if (enabled) {
                regs[STATE] |= STEERING_ENABLE_MASK;
            }

            regs[IRQ] = irq;
            break;
        }
        case IRQ: {
            PCIBus* bus = (PCIBus*)get_container();

            if (!bus->set_link_irq(regs[LINK], val)) {
                regs[STATE] |= STEERING_ERROR_MASK;
                break;
            }

            regs[IRQ] = val;
            regs[STATE] &= ~STEERING_ERROR_MASK;
            regs[STATE] |= STEERING_ENABLE_MASK;
            break;
        }
        case DISABLE: {
            if (val != 0) {
                W_MESSAGE("DISABLE: ignoring 0x%x", val);
                break;
            }

            PCIBus* bus = (PCIBus*)get_container();
            bus->disable_link(regs[LINK]);
            regs[STATE] &= ~(STEERING_ERROR_MASK | STEERING_ENABLE_MASK);
            break;
        }
        default:
            PANIC("invalid offset");
        }

        _conf_regs[CONFIG_WORD_STEERING] = current;
    }

    virtual void write_config_dword(uint index, uint32_t val)
    {
        if (index != CONFIG_BASE_INDEX + CONFIG_WORD_BIOS) {
            PCIDevice::write_config_dword(index, val);
            return;
        }

        _conf_regs[CONFIG_WORD_BIOS] = val;
    }

    virtual uint32_t read_config_dword(uint index)
    {
        if (index != CONFIG_BASE_INDEX + CONFIG_WORD_BIOS) {
            return PCIDevice::read_config_dword(index);
        }

        return _conf_regs[CONFIG_WORD_BIOS];
    }

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

    virtual void reset()
    {
        PCIDevice::reset();
        memset(_conf_regs, 0, sizeof(_conf_regs));
    }

private:
    PhysicalRam* _ram;
    uint32_t _conf_regs[NUM_CONFIG_WORDS];
};



class ISABridge : public PCIDevice {
public:
    ISABridge(PCIBus& bus)
        : PCIDevice("isa-bridge", bus, NOX_PCI_VENDOR_ID,
                                NOX_PCI_DEV_ID_ISA_BRIDGE,
                                NOX_PCI_DEV_ISA_BRIDGE_REV,
                                mk_pci_class_code(PCI_CLASS_BRIDGE, PCI_SUBCLASS_BRIDGE_ISA, 0),
                                false)
    {
        bus.add_device(*this);
    }

    virtual uint get_hard_id() { return ISA_BRIDGE_SLOT;}
};


class NoxVM::StateChangeRequest: public NonCopyable {
public:
    StateChangeRequest(NoxVM::compleation_routin_t cb, void* opaque)
        : _cb (cb)
        , _opaque (opaque)
        , _started (false)
        , _lock (NULL)
    {
    }

    virtual ~StateChangeRequest()
    {
        delete _lock;
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

    void lock(NoxVM& vm)
    {
        if (_lock) {
            return;
        }

        _lock = new WLock(vm.get_state_lock());
    }

    void unlock()
    {
        delete _lock;
        _lock = NULL;
    }

protected:
    virtual bool test(NoxVM& vm) = 0;
    virtual bool start(NoxVM& vm) = 0;
    virtual bool cont(NoxVM& vm) = 0;

private:
    NoxVM::compleation_routin_t _cb;
    void* _opaque;
    bool _started;
    WLock* _lock;
};



NoxVM::NoxVM(const char* name)
    : VMPart (name)
    , _kvm (new KVM())
    , _io_bus (new IOBus(*this))
    , _mem_bus (new MemoryBus(*this))
    , _holder (new PlaceHolder(*this))
    , _pic (new PIC(*this))
    , _io_apic (new IOApic(*this))
    , _pci (new PCIBus(*this))
    , _pci_host (new PCIHost(*_pci.get()))
    , _eisa_bridge (new ISABridge(*_pci.get()))
    , _pm_controller (new PMController(*this))
    , _cmos (new CMOS(*this))
    , _dma (new DMA(*this))
    , _pit (new PIT(*this))
    , _kbd (new KbdController(*this))
    , _ata_host (new ATAHost())
    , _vga (new VGA(*this))
    , _speaker (new Speaker(*this))
    , _bios_file (new FirmwareFile())
    , _display (NULL)
    , _low_ram (NULL)
    , _mid_ram (NULL)
    , _high_bios (NULL)
    , _high_ram (NULL)
    , _ram_size (MB)
    , _free_high_bios_pages (0)
    , _num_cpus (1)
    , _ro_hard_disk_file (false)
    , _cdrom (false)
    , _boot_from_cdrom (false)
    , _sleep_on_start (false)
{
    add_io_region(_io_bus->register_region(*this, IO_PORT_A20, 1, this,
                                           (io_read_byte_proc_t)&NoxVM::a20_port_read,
                                           (io_write_byte_proc_t)&NoxVM::a20_port_write));

    add_io_region(_io_bus->register_region(*this, IO_PORT_POST_DIAGNOSTIC, 1, this,
                                           NULL,
                                           (io_write_byte_proc_t)&NoxVM::post_diagnostic));

    add_io_region(_io_bus->register_region(*this, IO_PORT_MISC, 1, this,
                                           (io_read_byte_proc_t)&NoxVM::misc_port_read,
                                           (io_write_byte_proc_t)&NoxVM::misc_port_write));

    register_admin_commands();
}


NoxVM::~NoxVM()
{
    delete _display;

    while (!_dynamic_parts.empty()) {
        delete _dynamic_parts.front();
        _dynamic_parts.pop_front();
    }

    while (!_stat_change_req_list.empty()) {
        delete _stat_change_req_list.front();
        _stat_change_req_list.pop_front();
    }

    _mem_bus->unmap_physical_ram(_low_ram);
    _mem_bus->unref_physical_ram(_low_ram);
    _mem_bus->unmap_physical_ram(_mid_ram);
    _mem_bus->unref_physical_ram(_mid_ram);
    _mem_bus->unmap_physical_ram(_high_bios);
    _mem_bus->unref_physical_ram(_high_bios);
    _mem_bus->unmap_physical_ram(_high_ram);
    _mem_bus->unref_physical_ram(_high_ram);

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
#ifdef NOX_DEBUG
    admin->register_command("v-to-p", "translate virtual address to physical adress", "???",
                            admin_types(2, VA_UINT32_T, VA_UINT64_T),
                            admin_names(2, "cpu-id", "virtual-address"),
                            admin_types(2, VA_UINT32_T, VA_UINT64_T),
                            admin_names(2, "result", "physical-address"),
                            (admin_command_handler_t)&NoxVM::translate_command, this);
    admin->register_command("ram-dump", "dump vm ram", "???",
                            admin_types(1, VA_UTF8_T),
                            admin_names(1, "file name"),
                            admin_types(2, VA_UINT32_T, VA_UTF8_T),
                            admin_names(2, "result", "error-string"),
                            (admin_command_handler_t)&NoxVM::dump_ram, this);
#endif
}


void NoxVM::reset()
{
    ASSERT(_state == INIT_DONE || _state == READY || _state == FREEZED);

    _state = RESETING;

    reset_childrens();

    remap_io_regions();

    _a20_port_val = 0;
    _misc_port  = 0x01;
    _platform_lock = 0;
    _platform_reg_index = 0;
    _platform_write_pos = 0;
    _platform_read_pos = 0;
    _nmi_mask = true;
    _sleep_on_start = false;

    _mem_bus->map_physical_ram(_low_ram, 0, false);
    _mem_bus->map_physical_ram(_mid_ram, MID_RAM_START >> GUEST_PAGE_SHIFT, false);
    _mem_bus->map_physical_ram(_high_bios, ((4ULL * GB) - MB) >> GUEST_PAGE_SHIFT, false);

    if (_high_ram) {
        _mem_bus->map_physical_ram(_high_ram, (4ULL * GB) >> GUEST_PAGE_SHIFT, false);
    }

    load_bios();
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

    vm_freeze((compleation_routin_t)&CommandReply::reply, r);
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

#ifdef NOX_DEBUG
void NoxVM::translate_command(AdminReplyContext* context, uint32_t cpu_id, uint64_t address)
{
    RLock lock(_state_lock);

    uint64_t physical;
    CPU* cpu;

    if ((get_state() != VMPart::FREEZED && get_state() != VMPart::DEBUGGING) ||
                              !(cpu = get_cpu(cpu_id)) || !cpu->translate(address, physical)) {
        context->command_reply(0, 0);
        return;
    }

    context->command_reply(1, physical);
}


static uint8_t zero_buf[KB] = {0};

void NoxVM::dump_ram(AdminReplyContext* context, const char* file_name)
{
    RLock lock(_state_lock);

    if ((get_state() != VMPart::FREEZED && get_state() != VMPart::DEBUGGING)) {
        context->command_reply(1, "ERROR");
        return;
    }

    uint8_t* ptr = _mem_bus->get_physical_ram_ptr(_low_ram);
    uint64_t size = _mem_bus->get_physical_ram_size(_low_ram);
    uint8_t* mid_ptr =_mem_bus->get_physical_ram_ptr(_mid_ram);
    uint64_t mid_size = _mem_bus->get_physical_ram_size(_mid_ram);


    int fd = open(file_name, O_CREAT | O_EXCL | O_WRONLY, S_IRUSR | S_IWUSR);
    if (fd == -1) {
        E_MESSAGE("create dump file %s filed", file_name);
        context->command_reply(1, "ERROR");
    } else {
        write_all(fd, 0, ptr, size);

        uint64_t pos = size;

        while (pos != MID_RAM_START) {
            uint n = MIN(MID_RAM_START - pos, KB);
            write_all(fd, pos, zero_buf, n);
            pos += n;
        }

        write_all(fd, MID_RAM_START, mid_ptr, mid_size);

        close(fd);
    }


    context->command_reply(0, "OK");
}
#endif


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
    _pit->set_gate_level(2, _misc_port & 0x01);
    _speaker->set_level((_misc_port >> 1) & 0x01, _misc_port & 0x01);
}


uint8_t NoxVM::misc_port_read(uint16_t port)
{
    _misc_port ^= 0x10;
    return _misc_port | (_pit->get_output_level(2) ? 0x20 : 0);
}


void NoxVM::post_diagnostic(uint16_t port, uint8_t val)
{
    ASSERT(port == IO_PORT_POST_DIAGNOSTIC);

    D_MESSAGE("POST 0x%x (%u)", val, vcpu->get_id());
}


uint8_t NoxVM::platform_port_read_byte(uint16_t port)
{
    switch (port) {
    case PLATFORM_IO_BYTE:
        if (_platform_read_pos >= (PLATFORM_MEM_PAGES << GUEST_PAGE_SHIFT)) {
            W_MESSAGE_SOME(10, "read position is out of bound");
            break;
        }

        return _pci_host->get_ram_ptr()[_platform_read_pos++];
    case PLATFORM_IO_LOCK:
        return _platform_lock;
    default:
        D_MESSAGE("unexpected port 0x%x", port);
    }

    return 0xff;
}

static void replace_none_printable(char* str)
{
    for (; *str; str++) {
        if (!isprint(*str)) {
            *str = '?';
        }
    }
}


void NoxVM::do_platform_command(uint8_t val)
{
    switch (val) {
    case PLATFORM_CMD_SET_PCI_IRQ: {
        PCmdSetIRQ* args = (PCmdSetIRQ*)_pci_host->get_ram_ptr();
        W_MESSAGE("PLATFORM_CMD_SET_PCI_IRQ is obsolete");
        args->ret_val = pci_bus->set_irq(args->bus, args->device, args->pin, args->irq);
        break;
    }
    case PLATFORM_CMD_DELAY: {
        uint32_t* args = (uint32_t*)_pci_host->get_ram_ptr();
        W_MESSAGE("PLATFORM_CMD_DELAY is abnormal and temporary");
        usleep(*args);
        break;
    }
    default:
        W_MESSAGE("invalid command 0x%x", val);
    }
}


void NoxVM::platform_port_write_byte(uint16_t port, uint8_t val)
{
    switch (port) {
    case PLATFORM_IO_BYTE:
        if (_platform_write_pos >= (PLATFORM_MEM_PAGES << GUEST_PAGE_SHIFT)) {
            W_MESSAGE_SOME(10, "write position is out of bound 0x%x", val);
            break;
        }
        _pci_host->get_ram_ptr()[_platform_write_pos++] = val;
        break;
    case PLATFORM_IO_LOCK:
        _platform_lock = 1;
        break;
    case PLATFORM_IO_SELECT:
        _platform_reg_index = val;
        break;
    case PLATFORM_IO_LOG: {
        AutoArray<uint8_t> str_copy(new uint8_t[PLATFORM_LOG_BUF_SIZE]);
        strncpy((char*)str_copy.get(), (char*)_pci_host->get_ram_ptr(), PLATFORM_LOG_BUF_SIZE - 1);
        str_copy[PLATFORM_LOG_BUF_SIZE - 1] = 0;
        replace_none_printable((char*)str_copy.get());
        D_MESSAGE("guest[%u]: %s", vcpu->get_id(), str_copy.get());
        break;
    }
    case PLATFORM_IO_CMD:
        do_platform_command(val);
        break;
    default:
        D_MESSAGE("unexpected port 0x%x val %u", port, val);
    }
}


uint32_t NoxVM::platform_port_read_dword(uint16_t port)
{
    switch (port) {
    case PLATFORM_IO_REGISTER:
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
        case PLATFORM_REG_NUM_CPUS:
            return _num_cpus;
        default:
            D_MESSAGE("unexpected register 0x%x", _platform_reg_index);
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
        break;
    case PLATFORM_IO_REGISTER:
        switch(_platform_reg_index) {
        case PLATFORM_REG_WRITE_POS:
            if (val >= (PLATFORM_MEM_PAGES << GUEST_PAGE_SHIFT)) {
                W_MESSAGE_SOME(10, "write position is out of bound 0x%x", val);
            }

            _platform_write_pos = val;
            break;
        case PLATFORM_REG_READ_POS:
            if (val >= (PLATFORM_MEM_PAGES << GUEST_PAGE_SHIFT)) {
                W_MESSAGE_SOME(10, "read position is out of bound 0x%x", val);
            }

            _platform_read_pos = val;
            break;
        default:
            D_MESSAGE("unexpected register 0x%x", _platform_reg_index);
        }
        break;
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

    ASSERT(MID_RAM_MAX_ADDRESS < IO_APIC_ADDRESS);

    _mid_ram = _mem_bus->alloc_physical_ram(*this, mid_range_size >> GUEST_PAGE_SHIFT, "mid ram");
    ram_size -= mid_range_size - uma_ram_size;

    if (ram_size) {
        _high_ram = _mem_bus->alloc_physical_ram(*this, ram_size >> GUEST_PAGE_SHIFT,
                                                 "high ram");
    }

    _high_bios = _mem_bus->alloc_physical_ram(*this,
                                              HIGH_BIOS_SIZE >> GUEST_PAGE_SHIFT,
                                              "high bios");
    _free_high_bios_pages = HIGH_BIOS_SIZE >> GUEST_PAGE_SHIFT;

    ASSERT(4ULL * GB - HIGH_BIOS_SIZE  > LOCAL_APIC_ADDRESS);

    alloc_high_bios_pages(1);
}


void NoxVM::load_bios()
{
    uint8_t* ptr = _mem_bus->get_physical_ram_ptr(_mid_ram);
    ptr += MB - MID_RAM_START;

    uint size = (_bios_file->num_pages() << GUEST_PAGE_SHIFT);
    ptr -= size;
    _bios_file->read_all(ptr);

    uint8_t* footer_ptr = _mem_bus->get_physical_ram_ptr(_high_bios);
    footer_ptr += _mem_bus->get_physical_ram_size(_high_bios);
    footer_ptr -= BIOS_FOOTER_SIZE;

    memcpy(footer_ptr, ptr + size - BIOS_FOOTER_SIZE, BIOS_FOOTER_SIZE);
}


void NoxVM::init_cpus()
{
    if (_num_cpus > MAX_CPUS) {
        THROW("too many cpus %u", _num_cpus);
    }

    for (int i = 0; i < _num_cpus; i++) {
        _dynamic_parts.push_back(new CPU(*this));
    }
}


CPU* NoxVM::get_cpu(uint id)
{
    VMParts::iterator iter = _dynamic_parts.begin();

    for (; iter != _dynamic_parts.end(); ++iter) {
        VMPart* part = *iter;

        if (part->get_name() != "cpu") {
            continue;
        }

        CPU* cpu = (CPU*)*iter;

        if (cpu->get_id() == id) {
            return cpu;
        }
    }

    return NULL;
}


void NoxVM::set_ram_size(uint32_t ram_size_mb)
{
    ASSERT(ram_size_mb);
    ASSERT(_state == INIT);

    uint64_t ram_size = ram_size_mb;
    ram_size *= MB;
    _ram_size = ram_size;
}


void NoxVM::set_num_cpus(uint num_cpus)
{
    if (num_cpus > _kvm->get_max_cpus()) {
        THROW("requested cpu-count %u is higher than kvm limit (%u)",
              num_cpus, _kvm->get_max_cpus());
    }

    _num_cpus = num_cpus;
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


void NoxVM::init_bios()
{
    std::vector<std::string> names(3);

    sprintf(names[0], "bios-%.4x-%.4x-%.2x.bin", NOX_PCI_VENDOR_ID,
            NOX_PCI_DEV_ID_HOST_BRIDGE, NOX_PCI_DEV_HOST_BRIDGE_REV);

    sprintf(names[1], "bios-%.4x-%.4x.bin", NOX_PCI_VENDOR_ID,
            NOX_PCI_DEV_ID_HOST_BRIDGE);

    names[2] = "bios.bin";

    std::string file_name;

    if (!Application::find_firmware(file_name, names)) {
        THROW("open bios - failed");
    }

    _bios_file->open(file_name.c_str());

    if (!_bios_file->is_valid()) {
        THROW("open bios - failed");
    }

    if ((_bios_file->num_pages() << GUEST_PAGE_SHIFT) != 128 * KB) {
        THROW("bad bios file size");
    }
}


void NoxVM::init()
{
    ASSERT(_state == INIT);

    _display = new NoxDisplay(get_name().c_str(), *_vga.get(), *_kbd.get());

    init_bios();
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


bool NoxVM::stop(bool freeze)
{
    return stop_all((freeze) ? FREEZING : ABOUT_TO_SLEEP,
                    (freeze) ? FREEZED : SLEEPING);
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
    case VMPart::FREEZED:
    case VMPart::RUNNING:
    case VMPart::SLEEPING:
        return true;
    default:
        return false;
    }
}

bool StartRequest::start(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::FREEZED:
        if (vm._sleep_on_start) {
            vm._sleep_on_start = false;
            vm.unfreeze_vm();
            return true;
        }
    case VMPart::READY:
    case VMPart::SLEEPING:
        return vm.start();
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
    Lock lock(_state_request_mutex);

    _stat_change_req_list.push_back(new StartRequest(cb, opaque));

    if (_stat_change_req_list.size() == 1) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


class FreezeRequest: public NoxVM::StateChangeRequest {
public:
    FreezeRequest(NoxVM::compleation_routin_t cb, void* opaque);

    virtual bool test(NoxVM& vm);
    virtual bool start(NoxVM& vm);
    virtual bool cont(NoxVM& vm);
};


FreezeRequest::FreezeRequest(NoxVM::compleation_routin_t cb, void* opaque)
    : NoxVM::StateChangeRequest(cb, opaque)
{
}

bool FreezeRequest::test(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::READY:
    case VMPart::FREEZED:
    case VMPart::SHUTING_DOWN:
    case VMPart::DOWN:
    case VMPart::RUNNING:
    case VMPart::SLEEPING:
        return true;
    default:
        return false;
    }
}

bool FreezeRequest::start(NoxVM& vm)
{
    switch (vm.get_state()) {
    case VMPart::READY:
    case VMPart::FREEZED:
    case VMPart::SHUTING_DOWN:
    case VMPart::DOWN:
        return true;
    case VMPart::SLEEPING:
        vm._sleep_on_start = true;
        vm.freeze_vm();
        return true;
    case VMPart::RUNNING:
        return vm.stop(true);
    default:
        PANIC("unexpected state");
        return false;
    }
}


bool FreezeRequest::cont(NoxVM& vm)
{
    return vm.stop(true);
}


void NoxVM::vm_freeze(NoxVM::compleation_routin_t cb, void* opaque)
{
    Lock lock(_state_request_mutex);

    _stat_change_req_list.push_back(new FreezeRequest(cb, opaque));

    if (_stat_change_req_list.size() == 1) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


class DebugRequest: public NoxVM::StateChangeRequest {
public:
    DebugRequest(NoxVM::compleation_routin_t cb, void* opaque)
        : NoxVM::StateChangeRequest(cb, opaque)
    {
    }

    virtual bool test(NoxVM& vm)
    {
        switch (vm.get_state()) {
        case VMPart::FREEZED:
        case VMPart::DEBUGGING:
            return true;
        default:
            return false;
        }
    }

    virtual bool start(NoxVM& vm)
    {
        switch (vm.get_state()) {
        case VMPart::FREEZED:
            vm.set_debug();
            return true;
        case VMPart::DEBUGGING:
            return true;
        default:
            PANIC("unexpected");
            return false;
        }
    }

    virtual bool cont(NoxVM& vm)
    {
        PANIC("unexpected");
        return false;
    }
};


void NoxVM::set_debug()
{
    debug_all();
}


void NoxVM::vm_debug(NoxVM::compleation_routin_t cb, void* opaque)
{
    Lock lock(_state_request_mutex);

    _stat_change_req_list.push_back(new FreezeRequest(NULL, NULL));
    _stat_change_req_list.push_back(new DebugRequest(cb, opaque));

    if (_stat_change_req_list.size() == 2) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


class DebugContRequest: public NoxVM::StateChangeRequest {
public:
    DebugContRequest(NoxVM::compleation_routin_t cb, void* opaque)
        : NoxVM::StateChangeRequest(cb, opaque)
    {
    }

    virtual bool test(NoxVM& vm)
    {
        switch (vm.get_state()) {
        case VMPart::DEBUGGING:
        case VMPart::RUNNING:
            return true;
        default:
            return false;
        }
    }

    virtual bool start(NoxVM& vm)
    {
        switch (vm.get_state()) {
        case VMPart::DEBUGGING:
            vm.set_freeze_state();
            return true;
        case VMPart::RUNNING:
            return true;
        default:
            PANIC("unexpected");
            return false;
        }
    }

    virtual bool cont(NoxVM& vm)
    {
        PANIC("unexpected");
        return false;
    }
};


void NoxVM::vm_debug_cont(NoxVM::compleation_routin_t cb, void* opaque)
{
    Lock lock(_state_request_mutex);

    _stat_change_req_list.push_back(new DebugContRequest(NULL, NULL));
    _stat_change_req_list.push_back(new StartRequest(cb, opaque));

    if (_stat_change_req_list.size() == 2) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


void NoxVM::set_freeze_state()
{
    set_state_all(VMPart::FREEZED);
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
    case VMPart::FREEZED:
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
    Lock lock(_state_request_mutex);

    _stat_change_req_list.push_back(new FreezeRequest(NULL, NULL));
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
    Lock lock(_state_request_mutex);

    if (_stat_change_req_list.empty()) {
        D_MESSAGE("empty");
        return;
    }

    StateChangeRequest* request = _stat_change_req_list.front();

    lock.unlock();

    request->lock(*this);

    if (!request->verify_start_condition(*this)) {
        request->notify(false);
    } else {
        if (!request->execute(*this)) {
            return;
        }

        request->notify(true);
    }

    request->unlock();

    lock.lock();

    _stat_change_req_list.pop_front();
    delete request;

    if (!_stat_change_req_list.empty()) {
         AutoRef<StateChngeTask> task(new StateChngeTask(*this));
         application->add_task(task.get());
    }
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
    case VMPart::FREEZED:
        return true;
    default:
        return false;
    }
}


void NoxVM::vm_reset()
{
    Lock lock(_state_request_mutex);
    _stat_change_req_list.push_back(new ResetRequest());

    if (_stat_change_req_list.size() == 1) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


void NoxVM::vm_restart(compleation_routin_t cb, void* opaque)
{
    Lock lock(_state_request_mutex);
    _stat_change_req_list.push_back(new FreezeRequest(NULL, NULL));
    _stat_change_req_list.push_back(new ResetRequest());
    _stat_change_req_list.push_back(new StartRequest(cb, opaque));

    if (_stat_change_req_list.size() == 3) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}


static void ower_off_compleation(void *, bool ok)
{
    if (!ok) {
        W_MESSAGE("power off failed");
        return;
    }

    application->quit();
}


void NoxVM::vm_power_off()
{
    vm_down(ower_off_compleation, NULL);
}


class SleepRequest: public NoxVM::StateChangeRequest {
public:
    SleepRequest(CPU& initiator);

    virtual bool test(NoxVM& vm);
    virtual bool start(NoxVM& vm);
    virtual bool cont(NoxVM& vm);

private:
    CPU& _initiator;
};


SleepRequest::SleepRequest(CPU& initiator)
    : NoxVM::StateChangeRequest(NULL, NULL)
    , _initiator (initiator)
{
}

bool SleepRequest::test(NoxVM& vm)
{
    return vm.get_state() == VMPart::RUNNING &&  _initiator.pending_sleep_request();
}

bool SleepRequest::start(NoxVM& vm)
{
    if (vm.get_state() != VMPart::RUNNING) {
        PANIC("unexpected state");
    }

    return cont(vm);
}


bool SleepRequest::cont(NoxVM& vm)
{
    if (!vm.stop(false)) {
        return false;
    }

    _initiator.clear_sleep_request();
    return true;
}

void NoxVM::vm_sleep(CPU& initiator)
{
    Lock lock(_state_request_mutex);

    _stat_change_req_list.push_back(new SleepRequest(initiator));

    if (_stat_change_req_list.size() == 1) {
        AutoRef<StateChngeTask> task(new StateChngeTask(*this));
        application->add_task(task.get());
    }
}

