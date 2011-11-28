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

#include "pic.h"
#include "nox_vm.h"
#include "io_bus.h"
#include "wire.h"

enum {
    IO_PIC1 = 0x20,
    IO_PIC2 = 0xa0,

    IO_PIC1_TRIGGER_TYPE = 0x4d0, //ELCR
    IO_PIC2_TRIGGER_TYPE,

    ICW1_MASK = (1 << 4),

    ICW1_EXPECT_ICW4_MASK = (1 << 0),
    ICW1_SINGLE_MASK = (1 << 1),
    ICW1_LEVEL_MODE_MASK = (1 << 3),

    ICW2_VECTOR_BASE_MASK = 0xf8,

    ICW4_8086_MASK = (1 << 0),
    ICW4_AUTO_EOI_MASK = (1 << 1),
    ICW4_BUFF_MOD_MASK = (1 << 3),
    ICW4_SPECIAL_FULLY_NESTED_MASK = (1 << 4),

    SLAVE_IRQ_PIN_SHIFT = 2,
    SLAVE_IRQ_PIN = SLAVE_IRQ_PIN_SHIFT,

    REG_INDEX_IRR = 0,
    REG_INDEX_ISR,
    REG_INDEX_IMR,
    REG_INDEX_LEVEL,

    INIT_ICW2_MASK = (1 << 0),
    INIT_ICW3_MASK = (1 << 1),
    INIT_ICW4_MASK = (1 << 2),

    OCW3_MASK = (1 << 3),
    OCW3_READ_IRR_MASK = 0x02,
    OCW3_READ_ISR_MASK = 0x03,

    OCW3_POOL_MASK = 0x04,

    OCW3_SET_SPECIAL_MASK = 0x60,
    OCW3_CLEAR_SPECIAL_MASK = 0x40,

    OCW2_CMD_SHIFT = 5,
    OCW2_CMD_DISABLE_AUTO_ROTAT_IN_AEOI_MODE = 0,
    OCW2_CMD_NONSPECIFIC_EOI,
    OCW2_CMD_NOP,
    OCW2_CMD_SPECIFIC_EOI,
    OCW2_CMD_ENABLE_AUTO_ROTAT_IN_AEOI_MODE,
    OCW2_CMD_ROTATE_ON_NON_SPECIFIC_EOI,
    OCW2_CMD_SET_PRIORITY,
    OCW2_CMD_ROTATE_ON_SPECIFIC_EOI,
    OCW2_INTERRUPT_MASK = 0x07,

    PIC0_FORCE_EDGE_MASK = (1 << 0) | (1 << 1) | (1 << 2),
    PIC0_DEFAULT_ADDRESS = 0,

    PIC1_FORCE_EDGE_MASK = (1 << 0) | (1 << 5),
    PIC1_DEFAULT_ADDRESS = 7,

    MODE_FULLY_NESTED = ~0,
    MODE_SPECIAL_FULLY_NESTED = ~(1 << SLAVE_IRQ_PIN_SHIFT),
    MODE_MASK = 0,
};

PIC* pic = NULL;


PIC::PIC(NoxVM& nox)
    : VMPart("pic", nox)
    , _notify_proc (NULL)
    , _notify_opaque (NULL)
{
    ASSERT(pic == NULL);

    IOBus& bus = nox.get_io_bus();

    add_io_region(bus.register_region(*this, IO_PIC1, 1, this,
                                      (io_read_byte_proc_t)&PIC::io_read_port_0,
                                      (io_write_byte_proc_t)&PIC::io_write_port_0));

    add_io_region(bus.register_region(*this, IO_PIC2, 1, this,
                                      (io_read_byte_proc_t)&PIC::io_read_port_0,
                                      (io_write_byte_proc_t)&PIC::io_write_port_0));

    add_io_region(bus.register_region(*this, IO_PIC1 + 1, 1, this,
                                      (io_read_byte_proc_t)&PIC::io_read_port_1,
                                      (io_write_byte_proc_t)&PIC::io_write_port_1));

    add_io_region(bus.register_region(*this, IO_PIC2 + 1, 1, this,
                                      (io_read_byte_proc_t)&PIC::io_read_port_1,
                                      (io_write_byte_proc_t)&PIC::io_write_port_1));

    add_io_region(bus.register_region(*this, IO_PIC1_TRIGGER_TYPE, 2, this,
                                      (io_read_byte_proc_t)&PIC::io_read_trigger_type,
                                      (io_write_byte_proc_t)&PIC::io_write_trigger_type));

    reset();
    pic = this;
}

PIC::~PIC()
{
    pic = NULL;
}


uint8_t PIC::io_read_port_0(uint16_t port)
{
    Chip& chip = _chips[(port >> 7) & 1];
    return chip.regs[chip.read_index];
}


void PIC::reset_chip(Chip& chip)
{
    chip.mode = MODE_FULLY_NESTED;
    chip.revert_mode = chip.mode;
    memset(chip.regs, 0, sizeof(chip.regs));
    chip.vector_base = is_slave(chip) ? PIC1_DEFAULT_ADDRESS : PIC0_DEFAULT_ADDRESS;
    chip.read_index = REG_INDEX_IRR;
    chip.auto_eio = false;
    chip.auto_rotate = false;
    chip.intilization_mask = 0;
    chip.highst_priority = 0;
    chip.cascade_pin = is_slave(chip) ? ~0 : SLAVE_IRQ_PIN;
}


void PIC::intilization_start(Chip& chip, uint8_t val)
{
    reset_chip(chip);

    chip.intilization_mask = INIT_ICW2_MASK;

    if (val & ICW1_SINGLE_MASK) {
        W_MESSAGE("single chip");
    } else {
        chip.intilization_mask |= INIT_ICW3_MASK;
    }

    if (val & ICW1_LEVEL_MODE_MASK) {
        W_MESSAGE("not edge mode")
    }

    if (val & ICW1_EXPECT_ICW4_MASK) {
        chip.intilization_mask |= INIT_ICW4_MASK;
    }
}


void PIC::update_IRR(Chip& chip)
{
    for (int i = 0; i < NUM_IRQ; i++) {
        uint mask = (1 << i);
        if ((chip.regs[REG_INDEX_LEVEL] & mask) && chip.rais_count[i]) {
            set_IRR(chip, i);
        }
    }
}


void PIC::intilization_cont(Chip& chip, uint8_t val)
{
    if (chip.intilization_mask & INIT_ICW2_MASK) {
        chip.intilization_mask &= ~INIT_ICW2_MASK;
        chip.vector_base = val & ICW2_VECTOR_BASE_MASK;
    } else if (chip.intilization_mask & INIT_ICW3_MASK) {
        chip.intilization_mask &= ~INIT_ICW3_MASK;
        if (is_master(chip) && val != (1 << SLAVE_IRQ_PIN_SHIFT)) {
            W_MESSAGE("unexpeted configuration");
        }
    } else {
        ASSERT(chip.intilization_mask == INIT_ICW4_MASK);

        chip.intilization_mask = 0;

        if (!(val & ICW4_8086_MASK)) {
            W_MESSAGE("8086 is not set");
        }

        if (val & ICW4_AUTO_EOI_MASK) {
            chip.auto_eio = true;
        }

        if (val & ICW4_BUFF_MOD_MASK) {
            W_MESSAGE("bufferd mode");
        }

        if (val & ICW4_SPECIAL_FULLY_NESTED_MASK) {
            if (is_master(chip)) {
                chip.mode = MODE_SPECIAL_FULLY_NESTED;
                chip.revert_mode = chip.mode;
            }
        }
    }

    if (chip.intilization_mask) {
        update_IRR(chip);
    }
}


void PIC::set_mask_mode(Chip& chip)
{
    chip.mode = MODE_MASK;
}


void PIC::clear_mask_mode(Chip& chip)
{
    chip.mode = chip.revert_mode;
}


inline void PIC::handle_OCW3(Chip& chip, uint8_t val)
{
    uint rr_comand = val & OCW3_READ_ISR_MASK;

    if (rr_comand == OCW3_READ_ISR_MASK) {
        chip.read_index = REG_INDEX_ISR;
    } else if (rr_comand == OCW3_READ_IRR_MASK) {
        chip.read_index = REG_INDEX_IRR;
    }

    uint mode_comand = val & OCW3_SET_SPECIAL_MASK;

    if (mode_comand == OCW3_SET_SPECIAL_MASK) {
        set_mask_mode(chip);
    } else if (mode_comand == OCW3_CLEAR_SPECIAL_MASK) {
        clear_mask_mode(chip);
    }

    if (val & OCW3_POOL_MASK) {
        W_MESSAGE("pool command");
    }
}


inline void PIC::nonspecific_EOI(Chip& chip, bool rotate)
{
    if (chip.mode == MODE_MASK) {
        W_MESSAGE("non specific EOI while in MASK mode");
        return;
    }

    uint irq = chip.highst_priority;
    do {
       if (chip.regs[REG_INDEX_ISR] & (1 << irq)) {
           chip.regs[REG_INDEX_ISR] &= ~(1 << irq);

           if (rotate) {
               chip.highst_priority = (irq + 1) % NUM_IRQ;
           }

           break;
       }
       irq = (irq + 1) % NUM_IRQ;
    } while (irq != chip.highst_priority);
}


inline void PIC::handle_OCW2(Chip& chip, uint8_t val)
{
    uint8_t irq;

    switch (val >> OCW2_CMD_SHIFT ) {
    case OCW2_CMD_SPECIFIC_EOI:
        irq = val & OCW2_INTERRUPT_MASK;
        chip.regs[REG_INDEX_ISR] &= ~(1 << irq);
        break;
    case OCW2_CMD_ROTATE_ON_SPECIFIC_EOI:
        irq = val & OCW2_INTERRUPT_MASK;
        chip.regs[REG_INDEX_ISR] &= ~(1 << irq);
        chip.highst_priority = (irq + 1) % NUM_IRQ;
        break;
    case OCW2_CMD_NONSPECIFIC_EOI:
        nonspecific_EOI(chip, false);
        break;
    case OCW2_CMD_ROTATE_ON_NON_SPECIFIC_EOI:
        nonspecific_EOI(chip, true);
        break;
    case OCW2_CMD_SET_PRIORITY:
        irq = val & OCW2_INTERRUPT_MASK;
        chip.highst_priority = (irq + 1) % NUM_IRQ;
        break;
    case OCW2_CMD_ENABLE_AUTO_ROTAT_IN_AEOI_MODE:
        chip.auto_rotate = true;
        break;
    case OCW2_CMD_DISABLE_AUTO_ROTAT_IN_AEOI_MODE:
        chip.auto_rotate = false;
        break;
    }

    if (is_slave(chip)) {
        update_slave_IR();
    }
}


void PIC::io_write_port_0(uint16_t port, uint8_t val)
{
    Chip& chip = _chips[(port >> 7) & 1];

    Lock lock(_mutex);

    if (val & ICW1_MASK) {
        intilization_start(chip, val);
    } else if (val & OCW3_MASK) {
        handle_OCW3(chip, val);
    } else {
        handle_OCW2(chip, val);
    }

    update_output_pin(lock);
}


uint8_t PIC::io_read_port_1(uint16_t port)
{
    Chip& chip = _chips[(port >> 7) & 1];

    return chip.regs[REG_INDEX_IMR];
}


void PIC::io_write_port_1(uint16_t port, uint8_t val)
{
    Chip& chip = _chips[(port >> 7) & 1];

    Lock lock(_mutex);

    if (chip.intilization_mask) {
        intilization_cont(chip, val);
        return;
    }

    chip.regs[REG_INDEX_IMR] = val; //0 => enable;

    if (is_slave(chip)) {
        update_slave_IR();
    }

    update_output_pin(lock);
}


uint8_t PIC::io_read_trigger_type(uint16_t port)
{
    Chip& chip = _chips[port & 1];

    return chip.regs[REG_INDEX_LEVEL];
}


void PIC::io_write_trigger_type(uint16_t port, uint8_t val)
{
    Chip& chip = _chips[port & 1];

    Lock lock(_mutex);
    uint8_t mask = (port & 1) ? PIC0_FORCE_EDGE_MASK : PIC1_FORCE_EDGE_MASK;
    chip.regs[REG_INDEX_LEVEL] = val & ~mask;

    update_IRR(chip);

    update_output_pin(lock);
}


class PICPinBond {
public:
    PICPinBond(uint pin)
        : _pin (pin)
    {
    }

    void raised()
    {
        pic->raise(_pin);
    }

    void droped()
    {
        pic->drop(_pin);
    }

    void detach()
    {
        delete this;
    }

private:
    uint _pin;
};


void PIC::wire(Wire& wire, uint pin)
{
    wire.set_dest(*this, new PICPinBond(pin),
                  (void_callback_t)&PICPinBond::raised,
                  (void_callback_t)&PICPinBond::droped,
                  (void_callback_t)&PICPinBond::detach);
}


void PIC::attach_notify_target(void_callback_t proc, void* opaque)
{
    _notify_proc = NULL;
    __sync_synchronize();
    _notify_opaque = opaque;
    _notify_proc= proc;
}


void PIC::notify_output()
{
    if (_notify_proc) {
        _notify_proc(_notify_opaque);
    }
}


void PIC::update_output_pin(Lock& lock)
{
    bool output_pin = _get_interrupt(_chips[0], true) != INVALID_IRQ;

    if (output_pin != _output_pin && (_output_pin = output_pin)) {
        lock.unlock();
        notify_output();
    }
}


uint PIC::get_interrupt()
{
    ASSERT(get_state() == VMPart::RUNNING);

    Lock lock(_mutex);
    uint intterupt = _get_interrupt(_chips[0], false);

    if (intterupt != INVALID_IRQ) {
        update_output_pin(lock);
    } else {
        intterupt++;
        intterupt--;
    }

    return intterupt;
}


bool PIC::interrupt_test()
{
    return _output_pin;
}

//todo: optomize using bitsearch and bitrotate
uint PIC::_get_interrupt(Chip& chip, bool test)
{
    uint ready = chip.regs[REG_INDEX_IRR] & ~chip.regs[REG_INDEX_IMR];

    if (!ready) {
        return ~0;
    }

    uint8_t effective_isr = chip.regs[REG_INDEX_ISR] & chip.mode;
    uint8_t now = chip.highst_priority;

    do {
        uint mask = (1 << now);

        if (effective_isr & mask) {
            return ~0;
        }

        if (ready & mask) {
            uint8_t ret;

            if (now == chip.cascade_pin) {
                ret = _get_interrupt(_chips[1], test);

                if (test) {
                    return ret;
                }

                update_slave_IR();
            } else {
                ret = now + chip.vector_base;

                if (test) {
                    return ret;
                }
            }

            if (!chip.auto_eio) {
                chip.regs[REG_INDEX_ISR] |= mask;
            } else if (chip.auto_rotate) {
                chip.highst_priority = (now + 1) % NUM_IRQ;
            }

            if (!(chip.regs[REG_INDEX_LEVEL] & mask)) {
                 chip.regs[REG_INDEX_IRR] &= ~mask;
            }

            return ret;
        }
        now = (now + 1) % NUM_IRQ;
    } while (now != chip.highst_priority);

    return ~0;
}


bool PIC::is_slave(Chip& chip)
{
    return &chip != _chips;
}


bool PIC::is_master(Chip& chip)
{
    return &chip == _chips;
}


void PIC::update_slave_IR()
{
    Chip& chip = _chips[1];

    uint8_t ready = chip.regs[REG_INDEX_IRR] & ~chip.regs[REG_INDEX_IMR];

    if (ready && chip.mode == MODE_FULLY_NESTED) {
        uint now = chip.highst_priority;
        do {
            if (chip.regs[REG_INDEX_ISR] & (1 << now)) {
                ready = 0;
                break;
            }

            if (chip.regs[REG_INDEX_IRR] & (1 << now)) {
                break;
            }

            now = (now + 1) % NUM_IRQ;
        } while (now != chip.highst_priority);
    }

    if (ready) {
        set_IRR(_chips[0], SLAVE_IRQ_PIN);
    } else {
        clear_IRR(_chips[0], SLAVE_IRQ_PIN);
    }
}


void PIC::set_IRR(Chip& chip, uint8_t pin)
{
    chip.regs[REG_INDEX_IRR] |= (1 << pin);

    if (is_slave(chip)) {
        update_slave_IR();
    }
}

void PIC::clear_IRR(Chip& chip, uint8_t pin)
{
    chip.regs[REG_INDEX_IRR] &= ~(1 << pin);

    if (is_slave(chip)) {
        update_slave_IR();
    }
}


void PIC::raise(uint8_t pin)
{
    ASSERT(pin != SLAVE_IRQ_PIN && pin < NUM_IRQ * 2);

    Chip& chip = _chips[pin >> 3];
    pin = pin & 0x7;

    Lock lock(_mutex);

    if (++chip.rais_count[pin] == 1) {
        set_IRR(chip, pin);
        update_output_pin(lock);
    }
}


void PIC::drop(uint8_t pin)
{
    ASSERT(pin != 2 && pin < 16);
    ASSERT(get_state() == VMPart::RUNNING);

    Chip& chip = _chips[pin >> 3];
    pin = pin & 0x7;

    Lock lock(_mutex);

    if (--chip.rais_count[pin] == 0 && (chip.regs[REG_INDEX_LEVEL] & (1 << pin))) {
         clear_IRR(chip, pin);
         update_output_pin(lock);
    }
}


void PIC::reset()
{
    _output_pin = false;
    memset(_chips, 0, sizeof(_chips));
    remap_io_regions();
}

