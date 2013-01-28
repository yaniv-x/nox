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
#include "nox.h"
#include "pic.h"

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


class InterruptLink: public VMPart {
public:
    typedef std::list<Wire* > WireList;

    InterruptLink(PCIBus& bus, uint8_t default_irq)
        : VMPart("interrupt-link", bus)
        , _default_irq (default_irq)
        , _irq (default_irq)
        , _enabled (false)
    {
    }

    void disable()
    {
        Lock lock(_mutex);

        if (!_enabled) {
            return;
        }

        WireList::iterator iter = _input_wires.begin();

        for (; iter != _input_wires.end(); iter++) {
            Wire* wire = *iter;
            wire->detach_dest(true);
        }

        _enabled = false;
    }

    void attach(Wire& wire)
    {
        Lock lock(_mutex);

        _input_wires.push_back(&wire);

        if (!_enabled) {
            return;
        }

        irq_wire(wire, _irq);
    }

    bool set_irq(uint8_t irq)
    {
        Lock lock(_mutex);

        if (!((1 << irq) & NOX_PCI_IRQ_EXCLUSIVE_MASK)) {
            return false;
        }

        if (_enabled && (irq == _irq)) {
            return true;
        }

        _irq = irq;

        WireList::iterator iter = _input_wires.begin();

        for (; iter != _input_wires.end(); iter++) {
            Wire* wire = *iter;
            irq_wire(*wire, _irq);
        }

        _enabled = true;

        return true;
    }

    bool is_enabled()
    {
        return _enabled;
    }

    uint8_t get_irq()
    {
        return _irq;
    }

    void assign_default_irq()
    {
    }

protected:
    virtual void reset()
    {
        WireList::iterator iter = _input_wires.begin();

        for (; iter != _input_wires.end(); iter++) {
            Wire* wire = *iter;
            wire->detach_dest(false);
        }

        _enabled =false;
        _irq = _default_irq;
    }

    virtual bool start()
    {
        return true;
    }

    virtual bool stop()
    {
        return true;
    }

    virtual void save(OutStream& stream)
    {
    }

    virtual void load(InStream& stream)
    {
    }

private:
    Mutex _mutex;
    uint8_t _default_irq;
    uint8_t _irq;
    bool _enabled;
    WireList _input_wires;
};


PCIBus::PCIBus(NoxVM& nox)
    : VMPart ("pci", nox)
    , _interrupt_links (NOX_PCI_NUM_INT_LINKS)
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

    uint32_t irq_mask = NOX_PCI_IRQ_EXCLUSIVE_MASK & ~(1 << PM_IRQ_LINE);
    uint32_t default_irq = 0;

    for (uint i = 0; i < NOX_PCI_NUM_INT_LINKS; i++) {

        do {
            default_irq = (default_irq + 1) % 32;
        } while (!((1 << default_irq) & irq_mask));

        _interrupt_links[i] = new InterruptLink(*this, default_irq);
    }

    pci_bus = this;
}


PCIBus::~PCIBus()
{
    for (uint i = 1; i < PCI_MAX_DEVICES; i++) {
        if (_devices[i]) {
            D_MESSAGE("leak");
        }
    }

    for (uint i = 0; i < NOX_PCI_NUM_INT_LINKS; i++) {
        delete _interrupt_links[i];
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
    return id == 2 || id == 3 || id > 29;
}


InterruptLink* PCIBus::pci_pin_to_link(uint slot, uint pin)
{
    uint link_id = NOX_PCI_DEV_TO_LINK(slot, pin);
    return _interrupt_links[link_id];
}


void PCIBus::attach_device(uint slot, PCIDevice& device)
{
    if (slot >= NOX_PCI_NUM_SLOTS) {
        E_MESSAGE("no slot for %s", device.get_name().c_str());
        return;
    }

    _devices[slot] = &device;

    uint8_t interrupt_pin = *device.reg8(PCI_CONF_INTERRUPT_PIN);

    if (!interrupt_pin) {
        return;
    }

    InterruptLink* link;

    if (*device.reg16(PCI_CONF_VENDOR) == NOX_PCI_VENDOR_ID &&
        *device.reg16(PCI_CONF_DEVICE) == NOX_PCI_DEV_ID_PM_CONTROLLER) {
        ASSERT(slot == PM_CONTROLLER_SLOT);
        irq_wire(device.get_wire(), PM_IRQ_LINE);
    } else {
        link = pci_pin_to_link(slot, interrupt_pin);
        link->attach(device.get_wire());
    }
}


void PCIBus::add_device(PCIDevice& device)
{
    ASSERT(get_state() == VMPart::INIT);

    uint index = device.get_hard_id();

    if (index < PCI_MAX_DEVICES) {
        if (is_reserved_slot(index) || _devices[index]) {
            THROW("can't satisfy slot requirement for %s", device.get_name().c_str());
        }
    } else if ((index = device.get_preferd_id()) >= PCI_MAX_DEVICES || is_reserved_slot(index) ||
                                                                       _devices[index]) {
        if (index < PCI_MAX_DEVICES) {
            I_MESSAGE("can't allocate preferd slotfor %s", device.get_name().c_str());
        }

        for (index = 0; index < PCI_MAX_DEVICES; index++) {
            if (!_devices[index] && !is_reserved_slot(index)) {
                break;
            }
        }

        if (index == PCI_MAX_DEVICES) {
            THROW("out if pci slots");
        }
    }

    attach_device(index, device);
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


bool PCIBus::set_irq(uint bus, uint device, uint pin, uint irq)
{
    if (bus != 0 || device >= NOX_PCI_NUM_SLOTS) {
        return false;
    }

    if (device == PM_CONTROLLER_SLOT && pin == 0x0a) {
        return (irq == PM_IRQ_LINE) ? true : false;
    }

    pin -= 0x0a;

    if (pin > 3) {
        return false;
    }

    InterruptLink* link = pci_pin_to_link(device, pin);

    return link->set_irq(irq);

}


void PCIBus::get_link_state(uint id, uint8_t& irq, bool& enabled)
{
    ASSERT(id < NOX_PCI_NUM_INT_LINKS);

    irq = _interrupt_links[id]->get_irq();
    enabled = _interrupt_links[id]->is_enabled();
}


bool PCIBus::set_link_irq(uint id, uint8_t irq)
{
    ASSERT(id < NOX_PCI_NUM_INT_LINKS);

    return _interrupt_links[id]->set_irq(irq);
}


void PCIBus::disable_link(uint id)
{
    ASSERT(id < NOX_PCI_NUM_INT_LINKS);

    _interrupt_links[id]->disable();
}


void PCIBus::reset()
{
    remap_io_regions();
    _config_address = 0;
}

