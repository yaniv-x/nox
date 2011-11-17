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

#include "pit.h"
#include "nox_vm.h"
#include "pic.h"
#include "application.h"
#include "io_bus.h"
#include "wire.h"

//After power-up the state of the 8254 is undefined

//nt is held in the latch until it is read
//by the CPU (or until the Counter is reprogrammed)

// If a Counter is latched and then some time later
// latched again before the count is read the second
// Counter Latch Command is ignored

// The counter is automatically unlatched when read

//If both count and status of a counter are latched the
// first read operation of that counter will return latched
// status regardless of which was latched first The
// next one or two reads (depending on whether the
// counter is programmed for one or two type counts)
// return latched count Subsequent reads return un-
// latched count

enum {
    IO_TIMER_0 = 0x40,
    IO_TIMER_1,
    IO_TIMER_2,
    IO_TIMER_CONTROL,

    IO_NUM_PORTS = 4,
};

#define CLOCK_HERTZ 1193181.8181

static const double TICK = double(1000) * 1000 * 1000 / CLOCK_HERTZ; //838.095238153

enum {
    READ_BACK_COUNTER_MASK = 0x10,
    READ_BACK_STATUS_MASK = 0x20,

    STATUS_LATCH_PRESENT_MASK = 1 << (sizeof(uint) * 8 - 1),

    STATUS_OUTPUT_SHIFT = 7,
    STATUS_NULL_SHIFT,
    STATUS_RW_MODE_SHIFT = STATUS_NULL_SHIFT - 2,
    STATUS_MODE_SHIFT = STATUS_RW_MODE_SHIFT - 3,

    CONTROL_MODE_SHIFT = 1,
    CONTROL_MODE_BITS = 3,
    CONTROL_RW_SHIFT = 4,
    CONTROL_RW_BITS = 2,
    CONTROL_RW_MASK = 0x30,
    CONTROL_BCD_MASK = 0x01,
    CONTROL_SELECTOR_SHIFT = 6,
    CONTROL_SELECTOR_READ_BACK = 0x03,

    RW_MODE_LSB = 1,
    RW_MODE_MSB,
    RW_MODE_16BIT,
};


PIT::PIT(NoxVM& nox)
    : VMPart("pit", nox)
{
    add_io_region(io_bus->register_region(*this, IO_TIMER_0, IO_NUM_PORTS, this,
                                          (io_read_byte_proc_t)&PIT::io_read_byte,
                                          (io_write_byte_proc_t)&PIT::io_write_byte));

    memset(_timers, 0, sizeof(_timers));
    _timers[0].timer = application->create_timer((void_callback_t)&PIT::timer_proc, this);
    _timers[0].irq_wire = new Wire(*this);
    pic->wire(*_timers[0].irq_wire, 0);
}


PIT::~PIT()
{
    delete _timers[0].irq_wire;
    _timers[0].timer->destroy();
}


void PIT::timer_proc()
{
    ASSERT(get_state() == VMPart::RUNNING);
    Lock lock(_mutex);
    update_timer(_timers[0]);
}


inline void PIT::latch_counter(PICTimer& timer)
{
    if (timer.counter_latch[0]) {
        return;
    }

    timer.counter_latch[0] = timer.rw_mode;
    update_timer(timer);
    timer.counter_latch[1] = timer.counter_output;
}


inline void PIT::latch_status(PICTimer& timer)
{
    if (timer.status_latch) {
        return;
    }

    timer.status_latch = STATUS_LATCH_PRESENT_MASK |
                         (timer.output << STATUS_OUTPUT_SHIFT) |
                         (timer.null << STATUS_NULL_SHIFT) |
                         (timer.rw_mode << STATUS_RW_MODE_SHIFT) |
                         (timer.mode << STATUS_MODE_SHIFT) |
                         timer.bcd;
}


inline void PIT::read_back(uint8_t val)
{
    if (val & READ_BACK_COUNTER_MASK) {
        for (int i = 0; i < NUM_TIMERS; i++) {
            if (val & (0x02 << i))  {
                latch_counter(_timers[i]);
            }
        }
    }

    if (val & READ_BACK_STATUS_MASK) {
        for (int i = 0; i < NUM_TIMERS; i++) {
            if (val & (0x02 << i))  {
                latch_status(_timers[i]);
            }
        }
    }
}


inline void PIT::set_counter_mode(PICTimer& timer, uint8_t val)
{
    timer.mode = (val >> CONTROL_MODE_SHIFT) & ((1 << CONTROL_MODE_BITS) - 1);;
    timer.rw_mode = (val >> CONTROL_RW_SHIFT) & ((1 << CONTROL_RW_BITS) - 1);
    timer.bcd = val & CONTROL_BCD_MASK;
    timer.status_latch = 0;
    timer.counter_latch[0] = 0;
    timer.counter_latch[1] = 0;
    timer.counter_output = 0;
    timer.start_time = 0;
    timer.end_time = 0;
    timer.programed_val = 0;
    timer.output = timer.mode == 0 ? 0 : 1;
    timer.null = 1;
    timer.ticks = 0;

    switch (timer.rw_mode) {
    case RW_MODE_LSB:
        timer.read_flip = 0;
        break;
    case RW_MODE_MSB:
    case RW_MODE_16BIT:
        timer.read_flip = 1;
        timer.write_filp = 0;
        break;
    }

    if (timer.timer) {
        timer.timer->disarm();
    }

    if (timer.irq_wire) {
        timer.irq_wire->set_level(timer.output);
    }
}


inline void PIT::control(uint8_t val)
{
    Lock lock(_mutex);

    uint selector = (val >> CONTROL_SELECTOR_SHIFT);

    if (selector == CONTROL_SELECTOR_READ_BACK) {
        read_back(val);
        return;
    }

    if (!(val & CONTROL_RW_MASK)) {
        latch_counter(_timers[selector]);
        return;
    }

    set_counter_mode(_timers[selector], val);
}


inline bool PIT::update_one_shot(PICTimer& timer)
{
    nox_time_t now = get_monolitic_time();

    uint64_t delta = double(now - timer.start_time) / TICK;
    uint64_t counter_val = uint64_t(timer.programed_val) - delta;

    if (timer.bcd) {
        timer.counter_output = to_bcd(counter_val % 10000);
    } else {
        timer.counter_output = counter_val % (1 << 16);
    }

    if (!timer.ticks && now >= timer.end_time) {
        timer.ticks++;
        return true;
    }

    return false;
}


inline bool PIT::update_interval(PICTimer& timer, uint shift)
{
    //todo: use timer.start_time and timer.tiks in order to compensate on time lost
    nox_time_t now = get_monolitic_time();
    int64_t delta = timer.end_time - now;
    bool is_tick = delta <= 0;

    if (is_tick) {

        uint programed_val = timer.programed_val >> shift;

        timer.end_time = TICK * programed_val;
        timer.end_time += now;
        delta = programed_val;

        if (timer.null) {
            timer.start_time = now;
            timer.ticks = 0;
            timer.null = 0;

            if (timer.timer) {
                timer.timer->modify(timer.end_time - timer.start_time);
            }

        } else {
            timer.ticks++;
        }
    }

    uint counter_val = delta / TICK;
    counter_val >>= shift;
    timer.counter_output = timer.bcd ? to_bcd(counter_val) : counter_val;

    return is_tick;
}

void PIT::update_timer(PICTimer& timer)
{
    switch (timer.mode) {
    case 2:
        if (update_interval(timer, 0) && timer.irq_wire) {
            timer.irq_wire->drop();
        }
        break;
    case 0:
        if (update_one_shot(timer)) {
            timer.output = 1;
        }
        break;
    case 3:
        if (update_interval(timer, 1)) {
            timer.output ^= 1;
        }
        break;
    case 4:
        if (update_one_shot(timer) && timer.irq_wire) {
            timer.irq_wire->drop();
        }
        break;
    case 5:
    case 1:
        W_MESSAGE_SOME(10, "timer %u: mode %u is not implemnted", &timer - _timers, timer.mode);
        timer.counter_output = 0;
        break;
    default:
        D_MESSAGE("invalid timer mode %u", timer.mode);
    }

    if (timer.irq_wire) {
        timer.irq_wire->set_level(timer.output);
    }
}


void PIT::set_interval_counter(PICTimer& timer, uint shift)
{
    if (timer.start_time) {
        // new counter will be used on next load
        timer.null = 1;
        return;
    }

    timer.ticks = 0;
    timer.null = 0;
    timer.output = 1;
    timer.start_time = get_monolitic_time();
    timer.end_time = TICK * (timer.programed_val >> shift) + timer.start_time;

    if (timer.timer) {
        timer.timer->arm(timer.end_time - timer.start_time, true);
    }
}


void PIT::set_one_shout_counter(PICTimer& timer, uint output)
{
    timer.ticks = 0;
    timer.null = 0;
    timer.output = output;
    timer.start_time = get_monolitic_time();
    timer.end_time = TICK * timer.programed_val + timer.start_time;

    if (timer.timer) {
        timer.timer->arm(timer.end_time - timer.start_time, false);
    }
}


void PIT::set_counter(PICTimer& timer, uint val)
{
    timer.programed_val = timer.bcd ? from_bcd(val) : val;

    if (timer.programed_val == 0) {
        timer.programed_val = timer.bcd ? 10000 : 1 << 16;
    }

    if (timer.cb) {
        timer.cb(timer.cb_opaque, timer.mode, timer.programed_val);
    }

    switch (timer.mode) {
    case 2:
        // timer.programed_val == 1 is illegal in mode 2
        set_interval_counter(timer, 0);
        break;
    case 3:
        set_interval_counter(timer, 1);
        break;
    case 0:
        set_one_shout_counter(timer, 0);
        break;
    case 4:
        set_one_shout_counter(timer, 1);
        break;
    case 5:
    case 1: // hardware triggers will never hapend so the
            // counter will never be loaded (check if set_gate_level can trigget it?)
        break;
    default:
        D_MESSAGE("invalid timer mode %u", timer.mode);
    }

    if (timer.irq_wire) {
        timer.irq_wire->set_level(timer.output);
    }
}


void PIT::counter_write(PICTimer& timer, uint val)
{
    Lock lock(_mutex);

    switch (timer.rw_mode) {
    case RW_MODE_16BIT:
        if (timer.write_filp & 1) {
            set_counter(timer, (val << 8) + timer.write_lsb);
        } else {
            timer.write_lsb = val;
        }
        timer.write_filp++;
        break;
    case RW_MODE_LSB:
        set_counter(timer, val);
        break;
    case RW_MODE_MSB:
        set_counter(timer, val << 8);
        break;
    }
}


void PIT::io_write_byte(uint16_t port, uint8_t val)
{
    switch (port) {
    case IO_TIMER_0:
    case IO_TIMER_1:
    case IO_TIMER_2: {
        counter_write(_timers[port & 0x3], val);
        break;
    }
    case IO_TIMER_CONTROL:
        control(val);
        break;
    };
}


uint8_t PIT::io_read_byte(uint16_t port)
{
    if (port == IO_TIMER_CONTROL) {
        W_MESSAGE("unexpected read");
        return 0xff;
    }

    ASSERT(port >= IO_TIMER_0 && port < IO_TIMER_CONTROL)

    PICTimer& timer = _timers[port & 0x3];

    Lock lock(_mutex);

    if (timer.status_latch) {
        uint8_t ret = timer.status_latch;
        timer.status_latch = 0;
        return ret;
    }

    if (timer.counter_latch[0]) {
        if (timer.counter_latch[0] & 1) {
            timer.counter_latch[0] &= ~1;
            return timer.counter_latch[1];
        }

        timer.counter_latch[0] &= ~2;
        return timer.counter_latch[1] >> 8;

    }

    update_timer(timer);

    if (timer.rw_mode == RW_MODE_16BIT) {
        timer.read_flip += 1;
    }

    return (timer.read_flip & 1) ? timer.counter_output >> 8 : timer.counter_output;
}


void PIT::set_gate_level(uint timer, bool high)
{
    D_MESSAGE_SOME(10, "impliment me");
}


bool PIT::get_output_level(uint timer)
{
    ASSERT(timer < 4);

    Lock lock(_mutex);

    update_timer(_timers[timer]);
    return !!_timers[timer].output;
}


void PIT::reset(PICTimer& pic_timer)
{
    Timer* timer = pic_timer.timer;
    Wire* wire = pic_timer.irq_wire;
    state_cb_t cb = pic_timer.cb;
    void* opaque = pic_timer.cb_opaque;

    memset(&pic_timer, 0, sizeof(pic_timer));

    if (timer) {
        timer->disarm();
        pic_timer.timer = timer;
    }

    if (wire) {
        wire->reset();
        pic_timer.irq_wire = wire;
    }

    pic_timer.cb = cb;
    pic_timer.cb_opaque = opaque;
}


void PIT::set_state_callback(uint timer_id, state_cb_t cb, void *opaque)
{
    ASSERT(timer_id < NUM_TIMERS);
    _timers[timer_id].cb = cb;
    _timers[timer_id].cb_opaque = opaque;
}


void PIT::reset()
{
    for (int i = 0; i < NUM_TIMERS; i++) {
        reset(_timers[i]);
    }

    remap_io_regions();
}


void PIT::stop(PICTimer& pic_timer)
{
    if (!pic_timer.timer) {
        return;
    }

    pic_timer.timer->suspend();
}


bool PIT::stop()
{
    for (int i = 0; i < NUM_TIMERS; i++) {
        stop(_timers[i]);
    }

    return true;
}


void PIT::start(PICTimer& pic_timer)
{
    if (!pic_timer.timer) {
        return;
    }

    pic_timer.timer->resume();
}


bool PIT::start()
{
    for (int i = 0; i < NUM_TIMERS; i++) {
        start(_timers[i]);
    }

    return true;
}

