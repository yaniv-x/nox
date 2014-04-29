/*
    Copyright (c) 2013-2014 Yaniv Kamay,
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

#include "io_apic.h"
#include "nox_vm.h"
#include "memory_bus.h"
#include "nox.h"
#include "cpu.h"
#include "wire.h"


enum {
    IO_APIC_REG_SELECT = 0x00,
    IO_APIC_REG_DATA = 0x10,

    IO_APIC_VERSION = 0x11, // same as local apic version ?
    IO_APIC_REDIRECT_TABLE_SIZE = IOApic::NUM_IRQ_PINS,

    REG_ID = 0x00,
    REG_VERSION = 0x01,
    REG_ARBITRATION = 0x02,
    REG_REDIRECT_0 = IOApic::REDIRECT_TABLE_OFFSET,

    REDIRECT_DEST_SHIFT = 56,
    REDIRECT_FLAG_MASK = 1 << 16,
    REDIRECT_FLAG_LEVEL_MODE = 1 << 15,
    REDIRECT_FLAG_REMOTE_IRR = 1 << 14,
    REDIRECT_FLAG_POLOARITY = 1 << 13,
    REDIRECT_FLAG_PENDDING = 1 << 12,
    REDIRECT_FLAG_LOGICAL_DEST = 1 << 11,
    REDIRECT_DELIVERY_MODE_SHIFT = 8,
    REDIRECT_DELIVERY_MODE_MASK = 0x7 << 8,
    REDIRECT_VECTOR_MASK = 0xff,

    DELIVERY_MODE_FIXED = 0 << REDIRECT_DELIVERY_MODE_SHIFT,
    DELIVERY_MODE_LOWEST = 1 << REDIRECT_DELIVERY_MODE_SHIFT,
    DELIVERY_MODE_SMI = 2 << REDIRECT_DELIVERY_MODE_SHIFT,
    DELIVERY_MODE_NMI = 4 << REDIRECT_DELIVERY_MODE_SHIFT,
    DELIVERY_MODE_INIT = 5 << REDIRECT_DELIVERY_MODE_SHIFT,
    DELIVERY_MODE_EXT_INT = 7 << REDIRECT_DELIVERY_MODE_SHIFT,
};

IOApic* io_apic = NULL;


IOApic::IOApic(NoxVM& nox)
    : VMPart("io_apic", nox)
    , _mmio_region (NULL)
{
    io_apic = this;
}


IOApic::~IOApic()
{
    memory_bus->unregister_mmio(_mmio_region);
}

#define REDIRECT(index) _regs_64[REG_REDIRECT_0 / 2 + (index)]

void IOApic::reset()
{
    memory_bus->unregister_mmio(_mmio_region);
    _mmio_region = NULL;
    _mmio_region = memory_bus->register_mmio(IO_APIC_ADDRESS >> GUEST_PAGE_SHIFT, 1,
                                             (read_mem_proc_t)&IOApic::read_mem,
                                             (write_mem_proc_t)&IOApic::write_mem,
                                             this, *this);
    _select = 0;
    memset(_regs, 0, sizeof(_regs));
    _regs[REG_ID] = 0x0f000000;
    _regs[REG_VERSION] = ((NUM_IRQ_PINS - 1) << 16) | IO_APIC_VERSION;
    _regs[REG_ARBITRATION] = 0;

    for (uint i = 0; i < NUM_IRQ_PINS; i++) {
        REDIRECT(i) = REDIRECT_FLAG_MASK;
        _irq_pins[i] = 0;
    }
}


uint32_t IOApic::read_reg()
{
    if (_select >= NUM_REGS) {
        return 0;
    }

    return _regs[_select];
}


void IOApic::write_reg(uint32_t val)
{
    Lock lock(_mutex);

    switch (_select) {
    case REG_ID:
        _regs[REG_ID] = val & 0x0f000000;
        _regs[REG_ARBITRATION] = _regs[REG_ID];
        break;
    case REG_VERSION:
    case REG_ARBITRATION:
        break;
    default:
        if (_select < REG_REDIRECT_0 || _select >= REG_REDIRECT_0 + NUM_IRQ_PINS * 2) {
            break;
        }

        if ((_select & 1)) {
            _regs[_select] = val & 0xff000000;
        } else {
            uint pin = (_select - REG_REDIRECT_0) >> 1;

            _regs[_select] = val & (REDIRECT_VECTOR_MASK | REDIRECT_DELIVERY_MODE_MASK |
                                    REDIRECT_FLAG_LOGICAL_DEST | REDIRECT_FLAG_POLOARITY |
                                    REDIRECT_FLAG_LEVEL_MODE |REDIRECT_FLAG_MASK);

            switch ((_regs[_select] & REDIRECT_DELIVERY_MODE_MASK)) {
            case DELIVERY_MODE_FIXED:
            case DELIVERY_MODE_LOWEST:
                break;
            default:
                _regs[_select] &= ~(REDIRECT_FLAG_LEVEL_MODE | REDIRECT_VECTOR_MASK);
            }

            if ((_regs[_select] & REDIRECT_FLAG_LEVEL_MODE)) {

                if (_irq_pins[pin]) {
                    interrupt_trigger(pin);
                }
            }
        }
    }
}


void IOApic::read_mem(uint64_t src, uint64_t length, uint8_t* dest)
{
    if (length != 4) {
        W_MESSAGE("invalid access. address 0x%lx length 0x%lx", src, length);
        memset(dest, 0xff, length);
        return;
    }

    switch (src) {
    case IO_APIC_REG_SELECT:
        *(uint32_t*)dest = _select;
        break;
    case IO_APIC_REG_DATA:
        *(uint32_t*)dest = read_reg();
        break;
    default:
        D_MESSAGE("invalid @ 0x%lx", src);
        *(uint32_t*)dest = ~0;
    }
}


void IOApic::write_mem(const uint8_t* src, uint64_t length, uint64_t dest)
{
    if (length != 4) {
        W_MESSAGE("invalid access. address 0x%lx length 0x%lx", src, length);
        return;
    }

    switch (dest) {
    case IO_APIC_REG_SELECT:
        _select = *src;
        break;
    case IO_APIC_REG_DATA:
        write_reg(*(uint32_t*)src);
        break;
    default:
        D_MESSAGE("invalid @ 0x%lx", src);
    }
}


void IOApic::interrupt_trigger(uint pin)
{
    if ((REDIRECT(pin) & REDIRECT_FLAG_REMOTE_IRR) || (REDIRECT(pin) & REDIRECT_FLAG_MASK)) {
        return;
    }

    if ((REDIRECT(pin) & REDIRECT_FLAG_LEVEL_MODE)) {
        REDIRECT(pin) |= REDIRECT_FLAG_REMOTE_IRR;
    }

    uint vector = REDIRECT(pin) & REDIRECT_VECTOR_MASK;
    uint dest = REDIRECT(pin) >> REDIRECT_DEST_SHIFT;
    bool level = !!(REDIRECT(pin) & REDIRECT_FLAG_LEVEL_MODE);

    switch (REDIRECT(pin) & REDIRECT_DELIVERY_MODE_MASK) {
    case DELIVERY_MODE_FIXED:
        if ((REDIRECT(pin) & REDIRECT_FLAG_LOGICAL_DEST)) {
            CPU::apic_deliver_interrupt_logical(vector, dest, level);
        } else {
            CPU::apic_deliver_interrupt_physical(vector, dest, level);
        }
        break;
    case DELIVERY_MODE_LOWEST:
        if ((REDIRECT(pin) & REDIRECT_FLAG_LOGICAL_DEST)) {
            CPU::apic_deliver_interrupt_lowest(vector, dest, level);
        } else {
            D_MESSAGE("unexpected: lowest and physical");
            CPU::apic_deliver_interrupt_physical(vector, dest, level);
        }
        break;
    case DELIVERY_MODE_NMI: {
        if ((REDIRECT(pin) & REDIRECT_FLAG_LOGICAL_DEST)) {
            CPU::apic_deliver_nmi_logical(dest);
        } else {
            CPU::apic_deliver_nmi_physical(dest);
        }
        break;
    }
    case DELIVERY_MODE_SMI:
        W_MESSAGE("not implemented: SMI");
        break;
    case DELIVERY_MODE_INIT:
        W_MESSAGE("not implemented: INIT");
        break;
    case DELIVERY_MODE_EXT_INT:
        W_MESSAGE("not implemented: EXT_INT");
        break;
    default:
        D_MESSAGE("invalide mode 0x%lx", REDIRECT(pin));
    }
}


void IOApic::internal_eoi(uint pin)
{
    if (!(REDIRECT(pin) & REDIRECT_FLAG_REMOTE_IRR)) {
        D_MESSAGE("unexpected: eoi while irr is clear");
        return;
    }

    REDIRECT(pin) &= ~REDIRECT_FLAG_REMOTE_IRR;

    if (!(REDIRECT(pin) & REDIRECT_FLAG_LEVEL_MODE)) {
        D_MESSAGE("unexpected: trigger mode changed");
        return;
    }

    if (!_irq_pins[pin]) {
        return;
    }

    interrupt_trigger(pin);
}


void IOApic::eoi(uint vector)
{
    ASSERT(vector > 0xf && vector <= 0xff);

    Lock lock(_mutex);

    for (int pin = 0; pin < NUM_IRQ_PINS; pin++) {
        if ((REDIRECT(pin) & REDIRECT_VECTOR_MASK) == vector) {
            internal_eoi(pin);
        }
    }
}


void IOApic::raise(uint pin)
{
    ASSERT(pin < NUM_IRQ_PINS);

    Lock lock(_mutex);

    ASSERT(_irq_pins[pin] < 0xff);

    if (_irq_pins[pin]++ == 0) {
        interrupt_trigger(pin);
    }
}


void IOApic::drop(uint pin)
{
    ASSERT(pin < NUM_IRQ_PINS);

    Lock lock(_mutex);

    ASSERT(_irq_pins[pin] > 0);

    --_irq_pins[pin];
}


class IOApicPinBond {
public:
    IOApicPinBond(uint pin)
        : _pin (pin)
    {
    }

    void raised()
    {
        io_apic->raise(_pin);
    }

    void droped()
    {
        io_apic->drop(_pin);
    }

    void detach()
    {
        delete this;
    }

private:
    uint _pin;
};


void IOApic::wire(Wire& wire, uint pin)
{
    ASSERT(pin < NUM_IRQ_PINS);
    wire.set_dest(*this, new IOApicPinBond(pin),
                  (void_callback_t)&IOApicPinBond::raised,
                  (void_callback_t)&IOApicPinBond::droped,
                  (void_callback_t)&IOApicPinBond::detach);
}

