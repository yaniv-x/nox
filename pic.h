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

#ifndef _H_PIC
#define _H_PIC

#include "vm_part.h"
#include "threads.h"

class NoxVM;
class IORegion;
class Wire;

class PIC: public VMPart {
public:
    PIC(NoxVM& nox);
    virtual ~PIC();

    enum {
        INVALID_IRQ = ~0,
    };

    virtual void reset();
    virtual bool start() { return true;}
    virtual bool stop() { return true;}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void wire(Wire& wire, uint pin);
    uint get_interrupt();
    bool interrupt_test();
    void attach_notify_target(void_callback_t proc, void* opaque);

private:
    uint8_t io_read_port_0(uint16_t port);
    void io_write_port_0(uint16_t port, uint8_t val);
    uint8_t io_read_port_1(uint16_t port);
    void io_write_port_1(uint16_t port, uint8_t val);
    uint8_t io_read_trigger_type(uint16_t port);
    void io_write_trigger_type(uint16_t port, uint8_t val);

    enum {
        NUM_CHIPS = 2,
        NUM_REGS = 4,
        NUM_IRQ = 8,
    };

    struct Chip {
        uint8_t mode;
        uint8_t regs[NUM_REGS];
        uint8_t rais_count[NUM_IRQ];
        uint8_t vector_base;
        uint8_t read_index;
        bool auto_eio;
        bool auto_rotate;
        uint8_t intilization_mask;
        uint8_t highst_priority;

        uint8_t cascade_pin;
        uint8_t saved_mode;
    };

    void intilization_start(Chip& chip, uint8_t val);
    void intilization_cont(Chip& chip, uint8_t val);
    void reset_chip(Chip& chip);

    bool is_master(Chip& chip);
    bool is_slave(Chip& chip);
    uint _get_interrupt(Chip& chip, bool test);
    void update_slave_IR();
    void update_IRR(Chip& chip);
    void set_IRR(Chip& chip, uint8_t pin);
    void clear_IRR(Chip& chip, uint8_t pin);
    void set_mask_mode(Chip& chip);
    void clear_mask_mode(Chip& chip);
    void nonspecific_EOI(Chip& chip, bool rotate);
    void handle_OCW2(Chip& chip, uint8_t val);
    void handle_OCW3(Chip& chip, uint8_t val);
    void update_output_pin(Lock& lock);
    void notify_output();

    void raise(uint8_t pin);
    void drop(uint8_t pin);

private:
    Mutex _mutex;
    Chip _chips[NUM_CHIPS];
    bool _output_pin;
    void_callback_t _notify_proc;
    void* _notify_opaque;

    friend class PICPinBond;
};

enum {
    WIRE_IN_USE_MASK = (1 << 7),
    WIRE_LEVEL_MASK = (1 << 6),
    WIRE_PIN_MASK = WIRE_LEVEL_MASK - 1,
};

extern PIC* pic;

#endif

