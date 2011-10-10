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

#include "pci_bus.h"
#include "nox_vm.h"
#include "io_bus.h"
#include "pci_device.h"
#include "pci.h"

PCIBus* pci_bus = NULL;

enum {
    IO_PCI_CONFIG_ADDRESS = 0xcf8,
    IO_PCI_CONFIG_DATA = 0xcfc,

    ADDRESS_INDEX_SHIFT = 2,
    ADDRESS_INDEX_BITS = 6,
    ADDRESS_FUNCTION_SHIFT = 8,
    ADDRESS_FUNCTION_BITS = 3,
    ADDRESS_DEVICE_SHIFT = 11,
    ADDRESS_DEVICE_BITS = 5,
    ADDRESS_BUS_SHIFT = 16,
    ADDRESS_BUS_BITS = 8,
    ADDRESS_ENABLED_MASK = (1 << 31),
    ADDRESS_MASK = ~((1 << 2) - 1),

};


PCIBus::PCIBus(NoxVM& nox)
    : VMPart ("pci", nox)
{
    ASSERT(pci_bus == NULL);

    IOBus& bus = nox.get_io_bus();

    add_io_region(bus.register_region(*this, IO_PCI_CONFIG_ADDRESS, 4, this,
                                      NULL, NULL,
                                      NULL, NULL,
                                      (io_read_dword_proc_t)&PCIBus::io_get_config_address,
                                      (io_write_dword_proc_t)&PCIBus::io_set_config_address));
    add_io_region(bus.register_region(*this, IO_PCI_CONFIG_DATA, 4, this,
                                      (io_read_byte_proc_t)&PCIBus::io_read_config_byte,
                                      (io_write_byte_proc_t)&PCIBus::io_write_config_byte,
                                      (io_read_word_proc_t)&PCIBus::io_read_config_word,
                                      (io_write_word_proc_t)&PCIBus::io_write_config_word,
                                      (io_read_dword_proc_t)&PCIBus::io_read_config_dword,
                                      (io_write_dword_proc_t)&PCIBus::io_write_config_dword));

    memset(_devices, 0, sizeof(_devices));

    pci_bus = this;
}


PCIBus::~PCIBus()
{
    delete _devices[0];
    delete _devices[1];

    for (uint i = 1; i < PCI_MAX_DEVICES; i++) {
        if (_devices[i]) {
            D_MESSAGE("leak");
        }
    }

    pci_bus = NULL;
}


uint32_t PCIBus::io_get_config_address(uint16_t port)
{
    return _config_address;
}


void PCIBus::io_set_config_address(uint16_t port, uint32_t val)
{
    val &= ADDRESS_MASK;
    _config_address = val;
}


bool PCIBus::is_valid_address()
{
    return !!(_config_address & ADDRESS_ENABLED_MASK);
}


PCIDevice* PCIBus::get_target()
{
    uint bus = (_config_address >> ADDRESS_BUS_SHIFT) &
               ((1 << ADDRESS_BUS_BITS) - 1);

    if (bus != 0) {
        return NULL;
    }

    uint device = (_config_address >> ADDRESS_DEVICE_SHIFT) &
                  ((1 << ADDRESS_DEVICE_BITS) - 1);

    if (!_devices[device]) {
         return NULL;
    }

    uint function = (_config_address >> ADDRESS_FUNCTION_SHIFT) &
                    ((1 << ADDRESS_FUNCTION_BITS) - 1);

    if (function != 0) {
        return NULL;
    }

    return _devices[device];
}


static inline uint config_index(uint32_t config_address)
{
    return  (config_address >> ADDRESS_INDEX_SHIFT) & ((1 << ADDRESS_INDEX_BITS) - 1);
}


uint8_t PCIBus::io_read_config_byte(uint16_t port)
{
    PCIDevice* target;

    if (!is_valid_address() || !(target = get_target())) {
        return ~0;
    }

    return target->read_config_byte(config_index(_config_address), port - IO_PCI_CONFIG_DATA);
}


void PCIBus::io_write_config_byte(uint16_t port, uint8_t val)
{
    PCIDevice* target;

    if (!is_valid_address() || !(target = get_target())) {
        return;
    }

    target->write_config_byte(config_index(_config_address), port - IO_PCI_CONFIG_DATA, val);
}

uint16_t PCIBus::io_read_config_word(uint16_t port)
{
    PCIDevice* target;

    if (!is_valid_address() || !(target = get_target())) {
        return ~0;
    }

    return target->read_config_word(config_index(_config_address), port - IO_PCI_CONFIG_DATA);
}


void PCIBus::io_write_config_word(uint16_t port, uint16_t val)
{
    PCIDevice* target;

    if (!is_valid_address() || !(target = get_target())) {
        return;
    }

    target->write_config_word(config_index(_config_address), port - IO_PCI_CONFIG_DATA, val);
}


uint32_t PCIBus::io_read_config_dword(uint16_t port)
{
    PCIDevice* target;

    if (!is_valid_address() || !(target = get_target())) {
        return ~0;
    }

    return target->read_config_dword(config_index(_config_address));
}


void PCIBus::io_write_config_dword(uint16_t port, uint32_t val)
{
    PCIDevice* target;

    if (!is_valid_address() || !(target = get_target())) {
        return;
    }

    target->write_config_dword(config_index(_config_address), val);
}


static inline bool is_reserved_slot(uint id)
{
    // reserving some slots
    // 0 is the host-bridge, 1 is eisa-bridge, others are reservd for future usus
    return id == 2 || id > 29;
}


void PCIBus::add_device(PCIDevice& device)
{
    ASSERT(get_state() == VMPart::INIT);

    uint index = device.get_hard_id();

    if (index < PCI_MAX_DEVICES) {
        if (is_reserved_slot(index) || _devices[index]) {
            THROW("can't allocate hardwired slot");
        }

        _devices[index] = &device;

        return;
    }

    index = device.get_preferd_id();

    if (index < PCI_MAX_DEVICES && !is_reserved_slot(index) && !_devices[index]) {
        _devices[index] = &device;
        return;
    }

    for (index = 0; index < PCI_MAX_DEVICES; index++) {
        if (!_devices[index] && !is_reserved_slot(index)) {
            if (device.get_preferd_id() < PCI_MAX_DEVICES) {
                I_MESSAGE("can't allocate preferd slot");
            }

            _devices[index] = &device;
            return;
        }
    }

    THROW("out if pci slots");
}


void PCIBus::remove_device(PCIDevice& device)
{
    ASSERT(get_state() == VMPart::DOWN);

    for (uint i = 0; i < PCI_MAX_DEVICES; i++) {
        if (_devices[i] == &device) {
            _devices[i] = NULL;
            return;
        }
    }
}


void PCIBus::reset()
{
    remap_io_regions();
    _config_address = 0;
}

