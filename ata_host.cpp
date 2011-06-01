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

#include "ata_host.h"
#include "wire.h"
#include "pci.h"
#include "pci_bus.h"
#include "ata_device.h"
#include "pic.h"

enum {
    ATA_PCI_REVISION = 1,

    ATA0_IRQ = 14,
    ATA0_IO_BASE = 0x01f0,
    ATA0_IO_CONTROL_BASE = 0x03f6,
    ATA0_IO_CONTROL_MAP_BASE = 0x03f4,

    ATA1_IRQ = 15,
    ATA1_IO_BASE = 0x0170,
    ATA1_IO_CONTROL_BASE = 0x0376,
    ATA1_IO_CONTROL_MAP_BASE = 0x0374,

    IO_NUM_PORTS = 8,

    RET_VAL_WHILE_NO_DEVICE = 0x7f,
};


ATAHost::ATAHost()
    : PCIDevice("ide-pci", *pci_bus, NOX_PCI_VENDOR_ID, NOX_PCI_DEV_ID_IDE, ATA_PCI_REVISION,
                mk_pci_class_code(PCI_CLASS_MASS_STORAGE, PCI_MASS_STORAGE_SUBCLASS_IDE, 0), false)
    , _channel_0_wire (new Wire(*this))
    , _channel_1_wire (new Wire(*this))
{
    add_io_region(0, IO_NUM_PORTS, this,
                  (io_read_byte_proc_t)&ATAHost::io_channel_0_read,
                  (io_write_byte_proc_t)&ATAHost::io_channel_0_write,
                  (io_read_word_proc_t)&ATAHost::io_channel_0_read_word,
                  (io_write_word_proc_t)&ATAHost::io_channel_0_write_word);

    add_io_region(1, PCI_IO_MIN_SIZE, this,
                  (io_read_byte_proc_t)&ATAHost::io_channel_0_alt_status,
                  (io_write_byte_proc_t)&ATAHost::io_channel_0_control);

    set_io_address(0, ATA0_IO_BASE, true);
    set_io_address(1, ATA0_IO_CONTROL_MAP_BASE, true);

    add_io_region(2, IO_NUM_PORTS, this,
                  (io_read_byte_proc_t)&ATAHost::io_channel_1_read,
                  (io_write_byte_proc_t)&ATAHost::io_channel_1_write,
                  (io_read_word_proc_t)&ATAHost::io_channel_1_read_word,
                  (io_write_word_proc_t)&ATAHost::io_channel_1_write_word);

    add_io_region(3, PCI_IO_MIN_SIZE, this,
                  (io_read_byte_proc_t)&ATAHost::io_channel_1_alt_status,
                  (io_write_byte_proc_t)&ATAHost::io_channel_1_control);

    set_io_address(2, ATA1_IO_BASE, true);
    set_io_address(3, ATA1_IO_CONTROL_MAP_BASE, true);

    pci_bus->add_device(*this);
}


void ATAHost::set_device_0(ATADeviceFactory& factory)
{
    ASSERT(!_channel_0.get());
    _channel_0.reset(factory.creat_device(*this, *_channel_0_wire));
}


void ATAHost::set_device_1(ATADeviceFactory& factory)
{
    ASSERT(!_channel_1.get());
    _channel_1.reset(factory.creat_device(*this, *_channel_1_wire));
}


uint8_t ATAHost::io_channel_0_alt_status(uint16_t port)
{
    if (port != ATA0_IO_CONTROL_BASE) {
        return 0xff;
    }

    if (!_channel_0.get()) {
        return RET_VAL_WHILE_NO_DEVICE;
    }

    return _channel_0->io_alt_status();
}


void ATAHost::io_channel_0_control(uint16_t port, uint8_t val)
{
    if (port != ATA0_IO_CONTROL_BASE || !_channel_0.get()) {
        return;;
    }

    return _channel_0->io_control(val);
}


uint8_t ATAHost::io_channel_0_read(uint16_t port)
{
    if (!_channel_0.get()) {
        return RET_VAL_WHILE_NO_DEVICE;
    }

    return _channel_0->io_read(port - ATA0_IO_BASE);
}


void ATAHost::io_channel_0_write(uint16_t port, uint8_t val)
{
    if (!_channel_0.get()) {
        return;
    }

    _channel_0->io_write(port - ATA0_IO_BASE, val);
}


uint16_t ATAHost::io_channel_0_read_word(uint16_t port)
{
    if (!_channel_0.get()) {
        return 0xffff;
    }

    return _channel_0->io_read_word(port - ATA0_IO_BASE);
}


void ATAHost::io_channel_0_write_word(uint16_t port, uint16_t data)
{
    if (!_channel_0.get()) {
        return;
    }

    _channel_0->io_write_word(port - ATA0_IO_BASE, data);
}


uint8_t ATAHost::io_channel_1_alt_status(uint16_t port)
{
    if (port != ATA1_IO_CONTROL_BASE) {
        return 0xff;
    }

    if (!_channel_1.get()) {
        return RET_VAL_WHILE_NO_DEVICE;
    }

    return _channel_1->io_alt_status();
}


void ATAHost::io_channel_1_control(uint16_t port, uint8_t val)
{
    if (port != ATA1_IO_CONTROL_BASE || !_channel_1.get()) {
        return;;
    }

    return _channel_1->io_control(val);
}


uint8_t ATAHost::io_channel_1_read(uint16_t port)
{
    if (!_channel_1.get()) {
        return RET_VAL_WHILE_NO_DEVICE;
    }

    return _channel_1->io_read(port - ATA1_IO_BASE);
}


void ATAHost::io_channel_1_write(uint16_t port, uint8_t val)
{
    if (!_channel_1.get()) {
        return;
    }

    _channel_1->io_write(port - ATA1_IO_BASE, val);
}


uint16_t ATAHost::io_channel_1_read_word(uint16_t port)
{
    if (!_channel_1.get()) {
        return 0xffff;
    }

    return _channel_1->io_read_word(port - ATA1_IO_BASE);
}


void ATAHost::io_channel_1_write_word(uint16_t port, uint16_t data)
{
    if (!_channel_1.get()) {
        return;
    }

    _channel_1->io_write_word(port - ATA1_IO_BASE, data);
}


void ATAHost::on_io_enabled()
{
    pic->wire(*_channel_0_wire, ATA0_IRQ);
    pic->wire(*_channel_1_wire, ATA1_IRQ);
}

void ATAHost::on_io_disabled()
{
    _channel_0_wire->dettach_dest();
    _channel_1_wire->dettach_dest();
}


void ATAHost::reset()
{
    _channel_0_wire->dettach_dest();
    _channel_1_wire->dettach_dest();
    PCIDevice::reset();
}

