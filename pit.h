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

#ifndef _H_PIT
#define _H_PIT

#include "vm_part.h"
#include "threads.h"

class NoxVM;
class IORegion;
class Timer;
class PICWire;

class PIT: public VMPart {
public:
    PIT(NoxVM& nox);
    virtual ~PIT();

    NoxVM& nox() { return *(NoxVM*)get_container();}

    virtual void reset() {}
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

private:

    struct PICTimer {
        uint mode;
        uint rw_mode;
        uint bcd;
        uint output;
        uint null;
        uint status_latch;
        uint counter_latch[2];
        uint counter_output; //if bcd is set then in bcd else == _counter
        nox_time_t start_time;
        uint64_t ticks;
        nox_time_t end_time;
        uint programed_val;
        uint read_flip;
        uint write_filp;
        uint write_lsb;

        Timer* timer;
        PICWire* irq;
    };

    enum {
        NUM_TIMERS = 3,
    };

    uint8_t io_read_byte(uint16_t port);
    void io_write_byte(uint16_t port, uint8_t val);

    void latch_counter(PICTimer& timer);
    void latch_status(PICTimer& timer);
    bool update_interval(PICTimer& timer, uint shift);
    bool update_one_shot(PICTimer& timer);
    void update_timer(PICTimer& timer);
    void read_back(uint8_t val);
    void set_counter_mode(PICTimer& timer, uint8_t val);
    void counter_write(PICTimer&t, uint val);
    void control(uint8_t val);
    void timer_proc();
    void set_one_shout_counter(PICTimer& timer, uint output);
    void set_interval_counter(PICTimer& timer, uint shift);
    void set_counter(PICTimer& timer, uint val);

private:
    Mutex _mutex;
    IORegion* _io_region;
    PICTimer _timers[NUM_TIMERS];
};

#endif

