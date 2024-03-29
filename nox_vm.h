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

#ifndef _H_NOX_VM
#define _H_NOX_VM

#include "vm_part.h"
#include "threads.h"

class KVM;
class MemoryBus;
class PhysicalRam;
class DMA;
class PlaceHolder;
class PIC;
class IOApic;
class PIT;
class PCIBus;
class PCIHost;
class ISABridge;
class RTC;
class ATAHost;
class KbdController;
class IOBus;
class VGA;
class AdminReplyContext;
class Speaker;
class FirmwareFile;
class CPU;
class PMController;
class NoxDisplay;


struct NICInitInfo {
    mac_addr_t address;
    std::string interface;
};


class NoxVM: public VMPart {
public:
    NoxVM(const char* vm_name);
    ~NoxVM();

    void init();

    KVM& get_kvm() { return *_kvm.get();}
    IOBus& get_io_bus() { return *_io_bus.get();}
    CPU* get_cpu(uint id);
    uint get_cpu_count() { return _num_cpus;}
    PIT& get_pit() { return *_pit.get();}
    PMController& get_pm_controller() { return *_pm_controller.get();}
    void set_nmi_mask(bool mask) { _nmi_mask = mask;}
    RWLock& get_state_lock() { return _state_lock;}

    void set_ram_size(uint32_t ram_size);
    void set_num_cpus(uint num_cpus);
    void set_hard_disk(const char* file_name, bool read_only);
    void set_cdrom(const char* file_name);
    void add_nic(const NICInitInfo& info);

    typedef void (*compleation_routin_t)(void *, bool ok);
    void vm_reset();
    void vm_power_off();
    void vm_sleep(CPU& initiator);
    void vm_wakeup();
    void vm_start(compleation_routin_t cb, void* opaque);
    void vm_freeze(compleation_routin_t cb, void* opaque);
    void vm_restart(compleation_routin_t cb, void* opaque);
    void vm_down(compleation_routin_t cb, void* opaque);
    void vm_debug(NoxVM::compleation_routin_t cb, void* opaque);
    void vm_debug_cont(NoxVM::compleation_routin_t cb, void* opaque);

    void resume_mode_change();
    void handle_state_request();

    class StateChangeRequest;

protected:
    virtual void reset();
    virtual bool start();
    virtual bool stop() { return true;}
    virtual void save(OutStream& stream);
    virtual void load(InStream& stream);

private:
    void init_bios();
    void init_ram();
    void init_hard_disk();
    void init_cdrom();
    void load_bios();
    void init_cpus();

    void set_down();
    void set_debug();
    void set_freeze_state();
    void freeze_vm() { VMPart::freeze_all(); }
    void unfreeze_vm() { VMPart::unfreeze_all(); }
    bool stop(bool freeze);

    void a20_port_write(uint16_t port, uint8_t val);
    uint8_t a20_port_read(uint16_t port);
    void misc_port_write(uint16_t port, uint8_t val);
    uint8_t misc_port_read(uint16_t port);
    void post_diagnostic(uint16_t port, uint8_t val);
    uint8_t platform_port_read_byte(uint16_t port);
    void do_platform_command(uint8_t val);
    void platform_port_write_byte(uint16_t port, uint8_t val);
    uint32_t platform_port_read_dword(uint16_t port);
    void platform_port_write_dword(uint16_t port, uint32_t val);

    void register_admin_commands();
    void suspend_command(AdminReplyContext* context);
    void resume_command(AdminReplyContext* context);
    void restart_command(AdminReplyContext* context);
    void terminate_command(AdminReplyContext* context);
#ifdef NOX_DEBUG
    void translate_command(AdminReplyContext* context, uint32_t cpu, uint64_t physical_address);
    void dump_ram(AdminReplyContext* context, const char* file_name);
#endif

    void alloc_high_bios_pages(uint num_pages);

private:
    RWLock _state_lock;
    Mutex _state_request_mutex;
    std::unique_ptr<KVM> _kvm;
    std::unique_ptr<IOBus> _io_bus;
    std::unique_ptr<MemoryBus> _mem_bus;
    std::unique_ptr<PlaceHolder> _holder;
    std::unique_ptr<PIC> _pic;
    std::unique_ptr<IOApic> _io_apic;
    std::unique_ptr<PCIBus> _pci;
    std::unique_ptr<PCIHost> _pci_host;
    std::unique_ptr<ISABridge> _eisa_bridge;
    std::unique_ptr<PMController> _pm_controller;
    std::unique_ptr<RTC> _rtc;
    std::unique_ptr<DMA> _dma;
    std::unique_ptr<PIT> _pit;
    std::unique_ptr<KbdController> _kbd;
    std::unique_ptr<ATAHost> _ata_host;
    std::unique_ptr<VGA> _vga;
    std::unique_ptr<Speaker> _speaker;
    std::unique_ptr<FirmwareFile> _bios_file;
    NoxDisplay* _display;
    VMParts _dynamic_parts;
    PhysicalRam* _low_ram;
    PhysicalRam* _mid_ram;
    PhysicalRam* _high_bios;
    PhysicalRam* _high_ram;
    uint8_t _a20_port_val;
    uint64_t _ram_size;
    uint32_t _free_high_bios_pages;
    uint32_t _blow_high_bios_pages;
    uint _num_cpus;
    std::string _hard_disk_file_name;
    bool _ro_hard_disk_file;
    bool _cdrom;
    std::string _cdrom_file_name;
    bool _sleep_on_start;

    bool _nmi_mask;
    uint8_t _misc_port;
    uint8_t _platform_lock;
    uint8_t _platform_reg_index;
    uint32_t _platform_write_pos;
    uint32_t _platform_read_pos;

    std::list<StateChangeRequest*> _stat_change_req_list;

    friend class FreezeRequest;
    friend class SleepRequest;
    friend class WakeupRequest;
    friend class StartRequest;
    friend class ResetRequest;
    friend class DownRequest;
    friend class DebugRequest;
    friend class DebugContRequest;
    friend class PCIHost;
};


class MachinErrorException: public std::exception {
public:
    virtual const char* what() const throw () {return "machin error exception: implement me!!!";}
};

class ResetException: public std::exception {
public:
    virtual const char* what() const throw () {return "reset exception";}
};

class SoftOffException: public std::exception {
public:
    virtual const char* what() const throw () {return "soft off exception";}
};

class SleepException: public std::exception {
public:
    virtual const char* what() const throw () {return "sleep exception";}
};

#endif

