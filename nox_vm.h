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
class PIT;
class PCIBus;
class PCIHost;
class ISABridge;
class CMOS;
class ATAHost;
class KbdController;
class IOBus;
class VGA;
class AdminReplyContext;
class Speaker;
class FirmwareFile;
class CPU;


class NoxVM: public VMPart {
public:
    NoxVM();
    ~NoxVM();

    void init();

    KVM& get_kvm() { return *_kvm.get();}
    IOBus& get_io_bus() { return *_io_bus.get();}
    CPU* get_cpu(uint id);
    PIT& get_pit() { return *_pit.get();}
    void set_nmi_mask(bool mask) { _nmi_mask = mask;}

    void set_ram_size(uint32_t ram_size);
    void set_hard_disk(const char* file_name, bool read_only);
    void set_cdrom(const char* file_name);
    void set_boot_device(bool from_cdrom);

    typedef void (*compleation_routin_t)(void *, bool ok);
    void vm_reset();
    void vm_start(compleation_routin_t cb, void* opaque);
    void vm_stop(compleation_routin_t cb, void* opaque);
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
    virtual bool stop();
    virtual void power();
    virtual void save(OutStream& stream);
    virtual void load(InStream& stream);

private:
    void init_bios();
    void init_ram();
    void init_hard_disk();
    void init_cdrom();
    void load_bios();
    void init_cpus();
#ifdef WITH_BOCHS_BIOS
    void reset_bios_stuff();
#endif
    void set_down();
    void set_debug();
    void set_stopped();

    void a20_port_write(uint16_t port, uint8_t val);
    uint8_t a20_port_read(uint16_t port);
    void misc_port_write(uint16_t port, uint8_t val);
    uint8_t misc_port_read(uint16_t port);
#ifdef WITH_BOCHS_BIOS
    void bochs_port_write(uint16_t port, uint8_t val);
#endif
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

    void alloc_high_bios_pages(uint num_pages/*, uint8_t** ptr, page_address_t* address*/);

private:
    Mutex _vm_state_mutex;
    std::auto_ptr<KVM> _kvm;
    std::auto_ptr<IOBus> _io_bus;
    std::auto_ptr<MemoryBus> _mem_bus;
    std::auto_ptr<PlaceHolder> _holder;
    std::auto_ptr<PIC> _pic;
    std::auto_ptr<PCIBus> _pci;
    std::auto_ptr<PCIHost> _pci_host;
    std::auto_ptr<ISABridge> _eisa_bridge;
    std::auto_ptr<CMOS> _cmos;
    std::auto_ptr<DMA> _dma;
    std::auto_ptr<PIT> _pit;
    std::auto_ptr<KbdController> _kbd;
    std::auto_ptr<ATAHost> _ata_host;
    std::auto_ptr<VGA> _vga;
    std::auto_ptr<Speaker> _speaker;
    std::auto_ptr<FirmwareFile> _bios_file;
    VMParts _dynamic_parts;
    PhysicalRam* _low_ram;
    PhysicalRam* _mid_ram;
    PhysicalRam* _high_bios;
    PhysicalRam* _high_ram;
    uint8_t _a20_port_val;
    uint64_t _ram_size;
    uint32_t _free_high_bios_pages;
    uint32_t _bios_pages;
    uint _num_cpus;
    std::string _hard_disk_file_name;
    bool _ro_hard_disk_file;
#ifdef WITH_BOCHS_BIOS
    uint64_t _hard_disk_size;
#endif
    bool _cdrom;
    std::string _cdrom_file_name;
    bool _boot_from_cdrom;

    bool _nmi_mask;
    uint8_t _misc_port;
    uint8_t _platform_lock;
    uint8_t _platform_reg_index;
    uint32_t _platform_write_pos;
    uint32_t _platform_read_pos;

    std::list<StateChangeRequest*> _stat_change_req_list;

    friend class StopRequest;
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
    virtual const char* what() const throw () {return "reset exception: implement me!!!";}
};

#endif

