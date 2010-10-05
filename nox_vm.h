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
#include "kvm.h"
#include "memory_bus.h"
#include "dma.h"
#include "pit.h"
#include "keyboard.h"

class KVM;
class PlaceHolder;
class PIC;
class IORegion;
class PCIBus;
class CMOS;
class ATAController;

class NoxVM: public VMPart {
public:
    NoxVM();
    ~NoxVM();

    bool init();

    KVM& get_kvm() { return _kvm;}
    IOBus& get_io_bus() { return _io_bus;}

    virtual void reset();
    virtual void start();
    virtual void stop();
    virtual void power();
    virtual void save(OutStream& stream);
    virtual void load(InStream& stream);

private:
    void init_ram();
    void init_bios();
    void init_cpus();

    void a20_port_write(uint16_t port, uint8_t val);
    uint8_t a20_port_read(uint16_t port);
    void bochs_port_write(uint16_t port, uint8_t val);
    void post_diagnostic(uint16_t port, uint8_t val);

private:
    KVM _kvm;
    IOBus _io_bus;
    MemoryBus _mem_bus;
    std::auto_ptr<PIC> _pic;
    std::auto_ptr<PCIBus> _pci;
    std::auto_ptr<CMOS> _cmos;
    PhysicalRam* _ram;
    PhysicalRam* _high_bios;
    PhysicalRam* _high_ram;
    IORegion* _a20_io_region;
    uint8_t _a20_port_val;
    DMA _dma;
    IORegion* _bochs_io_region;
    PIT _pit;
    IORegion* _post_diagnostic;
    KbdController _kbd;
    std::auto_ptr<PlaceHolder> _holder;
    std::auto_ptr<ATAController> _ata;
};


#endif

