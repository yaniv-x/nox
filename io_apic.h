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

#ifndef _H_IO_APIC
#define _H_IO_APIC

#include "vm_part.h"
#include "threads.h"

class MMIORegion;
class Wire;

class IOApic : public VMPart {
public:
    IOApic(NoxVM& nox);
    virtual ~IOApic();

    void eoi(uint vector);

    enum {
        NUM_IRQ_PINS = 24,
        REDIRECT_TABLE_OFFSET = 0x10,
        NUM_REGS = NUM_IRQ_PINS * 2 + REDIRECT_TABLE_OFFSET,
    };

protected:
    virtual void reset();
    virtual bool start() { return true;}
    virtual bool stop() { return true;}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

private:
    void read_mem(uint64_t src, uint64_t length, uint8_t* dest);
    void write_mem(const uint8_t* src, uint64_t length, uint64_t dest);
    uint32_t read_reg();
    void write_reg(uint32_t val);
    void raise(uint pin);
    void drop(uint pin);
    void interrupt_trigger(uint pin);
    void internal_eoi(uint pin);
    void wire(Wire& wire, uint pin);

private:
    Mutex _mutex;
    MMIORegion* _mmio_region;
    uint32_t _select;

    uint32_t _regs[NUM_REGS];
    uint8_t _irq_pins[NUM_IRQ_PINS];

    friend class PICPinBond;
    friend class IOApicPinBond;
    friend void irq_wire(Wire&, uint);
};

extern IOApic* io_apic;

#endif

