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
#include "dma_state.h"
#include "memory_bus.h"

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

    RET_VAL_WHILE_NO_DEVICE = 0x7f, // on some systems the host return a const value (with
                                    // cleared BSY bit) and on some other systems the ret val
                                    // is (last-written-val & ~BSY)

    IO_BUS_MASTER_PRIMERY_COMMAND = 0x00,
    IO_BUS_MASTER_PRIMERY_STATUS = 0x02,
    IO_BUS_MASTER_PRIMERY_PRD = 0x04,
    IO_BUS_MASTER_SECONDARY_COMMAND = 0x08,
    IO_BUS_MASTER_SECONDARY_STATUS = 0x0a,
    IO_BUS_MASTER_SECONDARY_PRD = 0x0c,

    IO_BUS_MASTER_SIZE = 16,

    BUS_MASTER_COMMAND_START_MASK = (1 << 0),
    BUS_MASTER_COMMAND_DIRICTION_MASK = (1 << 3),
    BUS_MASTER_COMMAND_MASK = BUS_MASTER_COMMAND_START_MASK | BUS_MASTER_COMMAND_DIRICTION_MASK,

    BUS_MASTER_STATUS_ACTIVE_MASK = (1 << 0),
    BUS_MASTER_STATUS_ERROR_MASK = (1 << 1),
    BUS_MASTER_STATUS_INTERRUPT_MASK = (1 << 2),
    BUS_MASTER_STATUS_SIMPLEX_MASK = (1 << 7),
    BUS_MASTER_STATUS_RESERVED_MASK = (1 << 3) | (1 << 4),
    BUS_MASTER_STATUS_RO_MASK = BUS_MASTER_STATUS_SIMPLEX_MASK |
                                BUS_MASTER_STATUS_RESERVED_MASK |
                                BUS_MASTER_STATUS_ACTIVE_MASK,
    BUS_MASTER_STATUS_CLEAR_MASK = BUS_MASTER_STATUS_INTERRUPT_MASK |
                                   BUS_MASTER_STATUS_ERROR_MASK,


    END_OF_TABLE_MASK = (1 << 31),
    MAX_BUF_SIZE = 1 << 16,
};


class ATAHostDMA: public DMAState {
public:
    ATAHostDMA(uint8_t* status, uint32_t address)
        : _status (status)
        , _address (address)
    {
    }

    virtual ~ATAHostDMA()
    {
        while (!_direc_list.empty()) {
            DirectAccess* direct = *_direc_list.begin();
            _direc_list.pop_front();
            delete direct;
        }
    }

    virtual DirectVector * get_direct_vector(uint size);
    virtual IndirectVector * get_indirect_vector(uint size);

    virtual void done();
    virtual void error();
    virtual void nop();

private:
    uint8_t* _status;
    uint32_t _address;
    std::list<DirectAccess*> _direc_list;
};


DirectVector* ATAHostDMA::get_direct_vector(uint size)
{
    std::auto_ptr<DirectVector> vec(new DirectVector());
    uint32_t dma_PRD = _address;

    for (;; dma_PRD += 8) {
        uint32_t descriptor[2];
        memory_bus->read(dma_PRD, sizeof(descriptor), descriptor);

        struct iovec item;
        uint64_t address = descriptor[0];
        item.iov_len = descriptor[1] & 0xffff;

        if (!item.iov_len) {
            item.iov_len = MAX_BUF_SIZE;
        }

        item.iov_len = MIN(size, item.iov_len);
        size -= item.iov_len;

        DirectAccess* direct = memory_bus->get_direct(address, item.iov_len);

        if (!direct) {
            return NULL;
        }

        _direc_list.push_back(direct);

        item.iov_base = direct->get_ptr();

        vec->push_back(item);

        if (descriptor[1] & END_OF_TABLE_MASK) {
           break;
        }
    }

    return size ? NULL :vec.release();
}


IndirectVector* ATAHostDMA::get_indirect_vector(uint size)
{
    std::auto_ptr<IndirectVector> vec(new IndirectVector());
    uint32_t dma_PRD = _address;

    for (;; dma_PRD += 8) {
        uint32_t descriptor[2];
        memory_bus->read(dma_PRD, sizeof(descriptor), descriptor);

        DMAIndirectInfo item;
        item.address = descriptor[0];
        item.size = descriptor[1] & 0xffff;

        if (!item.size) {
            item.size = MAX_BUF_SIZE;
        }

        item.size = MIN(size, item.size);
        size -= item.size;

        vec->push_back(item);

        if (descriptor[1] & END_OF_TABLE_MASK) {
           break;
        }
    }

    return size ? NULL :vec.release();
}


void ATAHostDMA::done()
{
    // cleare command active bit ?
    uint8_t status = *_status & ~BUS_MASTER_STATUS_ACTIVE_MASK;
    status |= BUS_MASTER_STATUS_INTERRUPT_MASK;
    *_status = status;
    delete this;
}


void ATAHostDMA::error()
{
    // cleare command active bit ?
    uint8_t status = *_status & ~BUS_MASTER_STATUS_ACTIVE_MASK;
    status |= BUS_MASTER_STATUS_INTERRUPT_MASK | BUS_MASTER_STATUS_ERROR_MASK;
    *_status = status;
    delete this;
}


void ATAHostDMA::nop()
{
    delete this;
}


ATAHost::ATAHost()
    : PCIDevice("ide-pci", *pci_bus, NOX_PCI_VENDOR_ID, NOX_PCI_DEV_ID_IDE, ATA_PCI_REVISION,
                mk_pci_class_code(PCI_CLASS_MASS_STORAGE, PCI_MASS_STORAGE_SUBCLASS_IDE,
                                  (1 << PCI_IDE_PROGIF_BUS_MASTER_BIT)),
                false)
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

    add_io_region(4, IO_BUS_MASTER_SIZE, this,
                  (io_read_byte_proc_t)&ATAHost::io_bus_maste_read,
                  (io_write_byte_proc_t)&ATAHost::io_bus_maste_write);

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
        return;
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
        return;
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


uint8_t ATAHost::io_bus_maste_read(uint16_t port)
{
    _bus_master_io_base = get_region_address(4); // todo:   add pci map unmap notifiction
                                                 //         and update _bus_master_io_base in
                                                 //         notifiction handler
    port -= _bus_master_io_base;

    if (port >= IO_BUS_MASTER_SIZE) {
        D_MESSAGE("bad io port %u", port + _bus_master_io_base);
        return ~0;
    }

    return _bus_master_regs[port];
}


void ATAHost::set_bm_command(uint8_t val, uint8_t* reg, ATADevice* device)
{
    uint8_t prev_val = reg[0];
    reg[0] = val & BUS_MASTER_COMMAND_MASK;

    if (val & BUS_MASTER_COMMAND_START_MASK) {
        reg[2] |= BUS_MASTER_STATUS_ACTIVE_MASK;
    } else {
        reg[2] &= ~BUS_MASTER_STATUS_ACTIVE_MASK;
    }

    if (device && (prev_val & BUS_MASTER_COMMAND_START_MASK) !=
                  (val & BUS_MASTER_COMMAND_START_MASK)) {
        if (!(val & BUS_MASTER_COMMAND_START_MASK)) {
            //if (dma transfer is active) W_MESSAGE("unhandeld stop");
        } else {
            uint32_t address = *(uint32_t*)(reg + 4);

            Lock lock(_bm_mutex);

            ATAHostDMA* dma = new ATAHostDMA(reg + 2, address); //todo: alloc once in constructor
                                                                //      and keep active state
            if ((val & BUS_MASTER_COMMAND_DIRICTION_MASK)) {
                device->dma_write_start(*dma);
            } else {
                device->dma_read_start(*dma);
            }
        }
    }
}


void ATAHost::set_bm_status(uint8_t val, uint8_t* reg)
{
    uint8_t curren_val = *reg;

    curren_val &= ~(val & BUS_MASTER_STATUS_CLEAR_MASK);
    val &= ~(BUS_MASTER_STATUS_RO_MASK | BUS_MASTER_STATUS_CLEAR_MASK);
    curren_val &= (BUS_MASTER_STATUS_RO_MASK | BUS_MASTER_STATUS_CLEAR_MASK);
    *reg = curren_val| val;
}


void ATAHost::io_bus_maste_write(uint16_t port, uint8_t val)
{
    _bus_master_io_base = get_region_address(4); // todo: see ATAHost::io_bus_maste_read
    port -= _bus_master_io_base;

    switch (port) {
    case IO_BUS_MASTER_PRIMERY_COMMAND:
        set_bm_command(val, &_bus_master_regs[port], _channel_0.get());
        break;
    case IO_BUS_MASTER_SECONDARY_COMMAND:
        set_bm_command(val, &_bus_master_regs[port], _channel_1.get());
        break;
    case IO_BUS_MASTER_PRIMERY_STATUS:
    case IO_BUS_MASTER_SECONDARY_STATUS:
        set_bm_status(val, &_bus_master_regs[port]);
        break;
    case IO_BUS_MASTER_SECONDARY_PRD:
    case IO_BUS_MASTER_PRIMERY_PRD:
        _bus_master_regs[port] = val & ~0x03;
        break;
    case IO_BUS_MASTER_PRIMERY_PRD + 1:
    case IO_BUS_MASTER_PRIMERY_PRD + 2:
    case IO_BUS_MASTER_PRIMERY_PRD + 3:
    case IO_BUS_MASTER_SECONDARY_PRD + 1:
    case IO_BUS_MASTER_SECONDARY_PRD + 2:
    case IO_BUS_MASTER_SECONDARY_PRD + 3:
        _bus_master_regs[port] = val;
        break;
    default:
        D_MESSAGE("unexpected port 0x%x", port);
    }
}


void ATAHost::on_io_enabled()
{
    irq_wire(*_channel_0_wire, ATA0_IRQ);
    irq_wire(*_channel_1_wire, ATA1_IRQ);
}


void ATAHost::on_io_disabled()
{
    _channel_0_wire->detach_dest(true);
    _channel_1_wire->detach_dest(true);
    _bus_master_io_base = 0;
}


void ATAHost::reset()
{
    _channel_0_wire->detach_dest(false);
    _channel_1_wire->detach_dest(false);
    memset(_bus_master_regs, 0, sizeof(_bus_master_regs));
    _bus_master_io_base = 0;
    PCIDevice::reset();
}

