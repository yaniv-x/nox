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

#include "nox_vm.h"
#include "memory_bus.h"
#include "io_bus.h"
#include "kvm.h"
#include "dma.h"
#include "place_holder.h"
#include "pic.h"
#include "pit.h"
#include "pci_bus.h"
#include "ata_controller.h"
#include "cmos.h"
#include "disk.h"
#include "keyboard.h"
#include "vga.h"
#include "display.h"

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

    ATA0_IRQ = 14,

    LOW_RAM_SIZE = 640 * KB,
    MID_RAM_START = 768 * KB,
    MID_RAM_MAX_ADDRESS = 0xe0000000,
    MID_RAM_MAX = MID_RAM_MAX_ADDRESS - MID_RAM_START,
};


NoxVM::NoxVM()
    : VMPart ("Nox")
    , _kvm (new KVM())
    , _io_bus (new IOBus(*this))
    , _mem_bus (new MemoryBus(*this))
    , _holder (new PlaceHolder(*this))
    , _pic (new PIC(*this))
    , _pci (new PCIBus(*this))
    , _cmos (new CMOS(*this))
    , _low_ram (NULL)
    , _mid_ram (NULL)
    , _high_bios (NULL)
    , _high_ram (NULL)
    , _a20_io_region (NULL)
    , _a20_port_val (0)
    , _dma (new DMA(*this))
    , _bochs_io_region (NULL)
    , _pit (new PIT(*this))
    , _kbd (new KbdController(*this))
    , _ata (new ATAController(*this, ATA0_IRQ))
    , _vga (new VGA(*this))
    , _nmi_mask (false)
    , _misc_port (0x01)
{
    _a20_io_region = _io_bus->register_region(*this, IO_PORT_A20, 1, this,
                                             (io_read_byte_proc_t)&NoxVM::a20_port_read,
                                             (io_write_byte_proc_t)&NoxVM::a20_port_write);

    _bochs_io_region = _io_bus->register_region(*this, IO_PORT_BOCHS_PANIC_1, 5, this,
                                               NULL,
                                               (io_write_byte_proc_t)&NoxVM::bochs_port_write);

    _post_diagnostic = _io_bus->register_region(*this, IO_PORT_POST_DIAGNOSTIC, 1, this,
                                               NULL,
                                               (io_write_byte_proc_t)&NoxVM::post_diagnostic);


    add_io_region(_io_bus->register_region(*this, IO_PORT_VGA_BIOS_MESSAGE, 1, this,
                                           NULL,
                                           (io_write_byte_proc_t)&NoxVM::vgabios_port_write));

    add_io_region(_io_bus->register_region(*this, IO_PORT_MISC, 1, this,
                                           (io_read_byte_proc_t)&NoxVM::misc_port_read,
                                           (io_write_byte_proc_t)&NoxVM::misc_port_write));
}


NoxVM::~NoxVM()
{
    _io_bus->unregister_region(_a20_io_region);
    _io_bus->unregister_region(_bochs_io_region);
    _io_bus->unregister_region(_post_diagnostic);

    _mem_bus->unmap_physical_ram(_low_ram);
    _mem_bus->release_physical_ram(_low_ram);
    _mem_bus->unmap_physical_ram(_mid_ram);
    _mem_bus->release_physical_ram(_mid_ram);
    _mem_bus->unmap_physical_ram(_high_bios);
    _mem_bus->release_physical_ram(_high_bios);
    _mem_bus->unmap_physical_ram(_high_ram);
    _mem_bus->release_physical_ram(_high_ram);
}

void NoxVM::a20_port_write(uint16_t port, uint8_t val)
{
    ASSERT(port == 0x92);
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
}


void NoxVM::post_diagnostic(uint16_t port, uint8_t val)
{
    ASSERT(port == IO_PORT_POST_DIAGNOSTIC);

    //printf("POST 0x%x\n", val);
}


void NoxVM::init_ram()
{
    uint64_t ram_size = 512 * MB;

    _ram_size = ram_size;

    _low_ram = _mem_bus->alloc_physical_ram(*this, LOW_RAM_SIZE >> GUEST_PAGE_SHIFT, "low ram");
    _mem_bus->map_physical_ram(_low_ram, 0, false);

    ram_size -= LOW_RAM_SIZE;

    uint64_t ext_ram_size = MIN(ram_size, MID_RAM_MAX);
    _mid_ram = _mem_bus->alloc_physical_ram(*this, ext_ram_size >> GUEST_PAGE_SHIFT, "mid ram");
    _mem_bus->map_physical_ram(_mid_ram, MID_RAM_START >> GUEST_PAGE_SHIFT, false);

    ram_size -= ext_ram_size;

    if (ram_size) {
        _high_ram = _mem_bus->alloc_physical_ram(*this, ram_size >> GUEST_PAGE_SHIFT,
                                                 "high ram");
        _mem_bus->map_physical_ram(_high_ram, (4ULL * GB) >> GUEST_PAGE_SHIFT, false);
    }

    _high_bios = _mem_bus->alloc_physical_ram(*this, MB >> GUEST_PAGE_SHIFT, "high bios");
    _mem_bus->map_physical_ram(_high_bios, ((4ULL * GB) - MB) >> GUEST_PAGE_SHIFT, false);
}


void NoxVM::init_bios()
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

    uint8_t jump[] = {0xea, 0x5b, 0xe0, 0x00, 0xf0};
    uint8_t* ptr = _mem_bus->get_physical_ram_ptr(_high_bios);
    ptr += MB;
    ptr -= 16;

    memcpy(ptr, jump, sizeof(jump));

    ptr = _mem_bus->get_physical_ram_ptr(_mid_ram);
    ptr += MB - MID_RAM_START;
    AutoFD _bios_fd(::open("/home/yaniv/bochs-2.4.5/bios/BIOS-bochs-latest", O_RDONLY));

    if (!_bios_fd.is_valid()) {
        THROW("open failed");
    }
    struct stat stat;

    if (fstat(_bios_fd.get(), &stat) == -1) {
        THROW("fstat failed");
    }

    ptr -= stat.st_size;

    if (read(_bios_fd.get(), ptr, stat.st_size) != stat.st_size) {
        THROW("read failed");
    }

    AutoFD _vga_fd(::open("/home/yaniv/vgabios-0.6c/VGABIOS-lgpl-latest.bin", O_RDONLY));

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
    CPU* cpu = new CPU(*this, 0);
    cpu->start();
}

enum {
    DISK_SIZE = 1 * GB,
    MAX_HEADS = 16,
    MAX_SECTORS_PER_CYL = 63,
    MAX_CYL = 16383, //this is BOCHS limit, why?
                     // BIOS (cyl 1024, head 256, sec 63)
                     // IDE (cyl 65536, head 16, sec 256)

};

bool NoxVM::init()
{
    new NoxDisplay(*_vga.get(), *_kbd.get());

    init_ram();
    //todo: register local apic mmio
//    init_pci_bus();
//    init_usb_bus();
    init_bios();

    _cmos->host_write(0x10, 0);      // no floppy
    _cmos->host_write(0x12, 0xf0);   // hard disk 0 in extended cmos
    _cmos->host_write(0x19, 47);     // user define hd (params start in 0x1b)

    uint cyl = MIN(MAX_CYL, DISK_SIZE / MAX_HEADS / MAX_SECTORS_PER_CYL);

    // hd 0 params
    _cmos->host_write(0x1b, cyl);
    _cmos->host_write(0x1c, cyl >> 0);
    _cmos->host_write(0x1d, MAX_HEADS);
    _cmos->host_write(0x23, MAX_SECTORS_PER_CYL);

    //boot device
    _cmos->host_write(0x3d, 0x02); //first boot device is first HD
    _ata->set_disk(new Disk("/home/yaniv/images/f13_64.raw"));

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
    if (_ram_size > 0xe0000000) {
        below_4G = 0xe0000000;
        above_4G = _ram_size - 0xe0000000;
    } else {
        below_4G = _ram_size;
        above_4G = 0;
    }

    //above 16MB
    uint64_t above_16M;

    if (below_4G > 16 * MB) {
        above_16M = (below_4G - 16 * MB) / (1 << 16);
    } else {
        above_16M = 0;
    }

    _cmos->host_write(0x34, above_16M);
    _cmos->host_write(0x35, above_16M >> 8);

    //abov 4g
    _cmos->host_write(0x5b, above_4G >> 16);
    _cmos->host_write(0x5c, above_4G >> 24);
    _cmos->host_write(0x5d, above_4G >> 32);

    //num of cpus (is num of cpus - 1)
    _cmos->host_write(0x5f, 0);

    init_cpus();

    return true;
}


void NoxVM::reset()
{

}


void NoxVM::start()
{

}


void NoxVM::stop()
{

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

