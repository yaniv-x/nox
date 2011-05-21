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

#ifndef _H_PCI_DEVICE
#define _H_PCI_DEVICE

#include "vm_part.h"
#include "threads.h"
#include "wire.h"

class PhysicalRam;
class PCIBus;


// supporting only single function device (multi function device
// will be implemented only in case of a real need. supporting
// multiple buses will be preferred in case of PCI slot shortage)


class PCIDevice: public VMPart {
public:
    PCIDevice(const char* name, PCIBus& bus, uint16_t vendor, uint16_t device,
              uint8_t revision, uint32_t class_code, bool with_interrupt);
    virtual ~PCIDevice();

    virtual void reset();
    virtual bool start() { return true;}
    virtual bool stop() { return true;}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void add_mmio_region(uint bar, uint64_t size, void* opaque,
                         read_mem_proc_t read, write_mem_proc_t write,
                         bool bits64);
    void add_mem_region(uint bar, PhysicalRam* physical, bool bits64);
    void add_mem_region(uint bar, PhysicalRam* physical, page_address_t start_page,
                        uint64_t num_pages, bool bits64);
    void add_io_region(uint bar, uint16_t size, void* opaque,
                       io_read_byte_proc_t rb, io_write_byte_proc_t wb = NULL,
                       io_read_word_proc_t rw = NULL, io_write_word_proc_t ww = NULL,
                       io_read_dword_proc_t rd = NULL, io_write_dword_proc_t wd = NULL);
    uint64_t get_region_address(uint bar);
    void set_io_address(uint bar, uint16_t address, bool fixed);

    virtual uint get_hard_id() { return ~0;}
    uint get_preferd_id() { return ~0;}

    void set_interrupt_level(uint level);

    class Region;

protected:
    virtual void on_io_enabled() {}
    virtual void on_io_disabled() {}

private:
    uint8_t read_config_byte(uint index, uint offset);
    uint16_t read_config_word(uint index, uint offset);
    uint32_t read_config_dword(uint index);

    void write_config_byte(uint index, uint offset, uint8_t val);
    void write_config_word(uint index, uint offset, uint16_t val);
    void write_config_dword(uint index, uint32_t val);
    void write_bar(uint reg_index, uint32_t val);

    void reset_config_space();
    void map_io();
    void unmap_io();
    void map_mem();
    void unmap_mem();
    void disable_interrupt();
    void enable_interrupt();
    void write_command(uint16_t val);
    void write_status(uint16_t val);

    uint64_t* reg64(uint offset);
    uint16_t* reg16(uint offset);
    uint8_t* reg8(uint offset);
    void set_region(uint index, PCIDevice::Region* region);
    Wire& get_wire() { return _interrupt_line;}

     enum {
        CONFIG_SPACE_SIZE = 64,
        NUM_BARS = 6,
    };

private:
    Mutex _mutex;
    uint32_t _config_space[CONFIG_SPACE_SIZE];
    Region* _regions[NUM_BARS];
    Wire _interrupt_line;

    friend class PCIBus;
};

static inline uint32_t mk_pci_class_code(uint8_t base_class, uint8_t sub_class, uint8_t interface)
{
    return (uint32_t(base_class) << 16) | (uint32_t(sub_class) << 8) | interface;
}

#endif

