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


enum {
    IO_PCI_CONFIG_ADDRESS = 0xcf8,
    IO_PCI_CONFIG_DATA = 0xcfc,

    ADDRESS_INDEX_SHIFT = 2,
    ADDRESS_INDEX_BITS = 6,
    ADDRESS_FUNCTUIN_SHIFT = 8,
    ADDRESS_FUNCTUIN_BITS = 3,
    ADDRESS_DEVICE_SHIFT = 11,
    ADDRESS_DEVICE_BITS = 5,
    ADDRESS_BUS_SHIFT = 16,
    ADDRESS_BUS_BITS = 8,
    ADDRESS_ENABLED_MASK = (1 << 31),
    ADDRESS_MASK = ~(1 << 2),

};


PCIBus::PCIBus(NoxVM& nox)
    : VMPart ("pci", nox)
{
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
}


PCIBus::~PCIBus()
{
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

uint8_t PCIBus::io_read_config_byte(uint16_t port)
{
    if (!is_valid_address()) {
        return ~0;
    }

    return ~0;
}


void PCIBus::io_write_config_byte(uint16_t port, uint8_t val)
{
    if (!is_valid_address()) {
        return;
    }
}

uint16_t PCIBus::io_read_config_word(uint16_t port)
{
    if (!is_valid_address()) {
        return ~0;
    }

    return ~0;
}


void PCIBus::io_write_config_word(uint16_t port, uint16_t val)
{
    if (!is_valid_address()) {
        return;
    }
}


uint32_t PCIBus::io_read_config_dword(uint16_t port)
{
    if (!is_valid_address()) {
        return ~0;
    }

    return ~0;
}


void PCIBus::io_write_config_dword(uint16_t port, uint32_t val)
{
    if (!is_valid_address()) {
        return;
    }
}

