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

#include "cmos.h"
#include "nox_vm.h"
#include "io_bus.h"
#include "pic.h"
#include "application.h"
#include "pm_controller.h"

enum {
    CMOD_OFFSET_SHUTDOWN = 0x0f,
    CMOD_OFFSET_FLOPPY_TYPE = 0x10,
    CMOD_OFFSET_HD_TYPE = 0x12,
    CMOD_OFFSET_EQUIPMENT = 0x14,
    CMOD_OFFSET_BASE_MEM_LOW = 0x15,
    CMOD_OFFSET_BASE_MEM_HIGH = 0x16,
    CMOD_OFFSET_EXT_MEM_LOW = 0x17,
    CMOD_OFFSET_EXT_MEM_HIGH = 0x18,
    CMOD_OFFSET_HD_EXT_TYPE = 0x19,
    CMOD_OFFSET_HD0_T47_CYL_LOW = 0x1b,
    CMOD_OFFSET_HD0_T47_CYL_HIGH = 0x1c,
    CMOD_OFFSET_HD0_T47_HEADS = 0x1d,
    CMOD_OFFSET_HD0_T47_PRECOMP_LOW = 0x1e,
    CMOD_OFFSET_HD0_T47_PRECOMP_HIGH = 0x1f,
    CMOD_OFFSET_HD0_T47_INFO = 0x20,
    CMOD_OFFSET_HD0_T47_LENDING_LOW = 0x21,
    CMOD_OFFSET_HD0_T47_LENDING_HIGH = 0x22,
    CMOD_OFFSET_HD0_T47_SECTORS = 0x23,
    CMOD_OFFSET_EXT_MEM_LOW_ALIAS = 0x30,
    CMOD_OFFSET_EXT_MEM_HIGH_ALIAS = 0x31,
    CMOD_OFFSET_CENTURY = 0x32,
    CMOD_OFFSET_PS2_CENTURY = 0x37,
    CMOD_OFFSET_MEM_ABOVE_16M_LOW = 0x34,
    CMOD_OFFSET_MEM_ABOVE_16M_HIGH = 0x35,
    CMOD_OFFSET_BOOT_DEV_LOW = 0x3d,
    CMOD_OFFSET_BOOT_DEV_HIGH = 0x38,
    CMOD_OFFSET_HD_GEO_TRANSLATION = 0x39,
    CMOD_OFFSET_MEM_ABOVE_4G_LOW = 0x5b,
    CMOD_OFFSET_MEM_ABOVE_4G_MID = 0x5c,
    CMOD_OFFSET_MEM_ABOVE_4G_HIGH = 0x5d,
    CMOD_OFFSET_NUM_CPUS = 0x5f,
};

enum {
    RTC_IRQ = 8,
    PM_MASK = (1 << 7),

    REG_A_UPDATE_IN_PROGRESS_MASK = (1 << 7),
    REG_A_DIVIDER_SHIFT = 4,
    REG_A_DIVIDER_MASK = (0x7 << REG_A_DIVIDER_SHIFT),
    REG_A_RATE_MASK = 0x0f,

    REG_A_DIVIDER_NORMAL = 0x2,
    REG_A_DIVIDER_32 = 0x3,
    REG_A_DIVIDER_128 = 0x0,

    REG_A_RATE_DEFAULT = 6,


    REG_B_BINARY_MASK = (1 << 2),
    REG_B_24_HOURS_MASK = (1 << 1),
    REG_B_ENABLE_UPDATE_INTERRUPT_MASK = (1 << 4),
    REG_B_ENABLE_ALARM_INTERRUPT_MASK = (1 << 5),
    REG_B_ENABLE_PERIODIC_INTERRUPT_MASK = (1 << 6),
    REG_B_HALT_CLOCK_MASK = (1 << 7),
    REG_B_CLEAR_ON_RESET_MASK = REG_B_ENABLE_PERIODIC_INTERRUPT_MASK |
                                REG_B_ENABLE_ALARM_INTERRUPT_MASK |
                                REG_B_ENABLE_UPDATE_INTERRUPT_MASK | (1 << 3),


    REG_C_INTERRUPT_FLAG_MASK = (1 << 7),
    REG_C_PERIODIC_FLAG_MASK = (1 << 6),
    REG_C_ALARM_FLAG_MASK = (1 << 5),
    REG_C_UPDATE_FLAG_MASK = (1 << 4),


    REG_D_POWER_STABLE_MASK = (1 << 7),
};

enum {
    NANO_PER_MICRO = 1000,
    NANO_PER_MILI = NANO_PER_MICRO * 1000,
    NANO_PER_SEC = NANO_PER_MILI * 1000,
};

static uint32_t rates_table[] = {
    0,
    3906250,
    7812500,
    122070,
    244141,
    488281,
    976562,
    1953125,
    3906250,
    7812500,
    15625000,
    31250000,
    62500000,
    125000000,
    250000000,
    500000000,
};


CMOS::CMOS(NoxVM& vm)
    : VMPart ("cmos", vm)
    , _irq_wire (*this)
    , _period_timer (application->create_timer((void_callback_t)&CMOS::period_timer_proc, this))
    , _alarm_timer (application->create_timer((void_callback_t)&CMOS::alarm_timer_proc, this))
    , _update_timer (application->create_timer((void_callback_t)&CMOS::update_cycle, this))
    , _index (0)
{
    memset(_user_ares, 0, sizeof(_user_ares));
    add_io_region(io_bus->register_region(*this, 0x70, 2, this,
                                          (io_read_byte_proc_t)&CMOS::read_byte,
                                          (io_write_byte_proc_t)&CMOS::write_byte));
    time_t t = time(NULL);
    gmtime_r(&t, &_date);
    _date_base_time = get_monolitic_time();
    _suspend_time = _date_base_time;
    _next_update_time = _suspend_time + NANO_PER_SEC;
    _last_periodic_time = _date_base_time;

    _reg_a = (REG_A_DIVIDER_NORMAL << REG_A_DIVIDER_SHIFT) | REG_A_RATE_DEFAULT,
    _reg_b = 0;

    irq_wire(_irq_wire, RTC_IRQ);
}


CMOS::~CMOS()
{
    _period_timer->destroy();
    _alarm_timer->destroy();
    _update_timer->destroy();
}


void CMOS::reset()
{
    _period_timer->disarm();
    _alarm_timer->disarm();
    _update_timer->disarm();
    _irq_wire.reset();
    _reg_b &= ~REG_B_CLEAR_ON_RESET_MASK;
    _reg_c = 0;
    remap_io_regions();
}


inline bool CMOS::is_lazy_mode()
{
    return !(_reg_b & REG_B_ENABLE_UPDATE_INTERRUPT_MASK);
}


inline bool CMOS::is_clock_halted()
{
    return !!(_reg_b & REG_B_HALT_CLOCK_MASK);
}


inline bool CMOS::interrupt_on_alarm()
{
    return !!(_reg_b & REG_B_ENABLE_ALARM_INTERRUPT_MASK);
}


void CMOS::period_timer_proc()
{
    ASSERT(get_state() == VMPart::RUNNING);

    Lock lock(_mutex);

    if ((_reg_b & REG_A_RATE_MASK) && (_reg_b & REG_B_ENABLE_PERIODIC_INTERRUPT_MASK)) {
        _reg_c |= REG_C_INTERRUPT_FLAG_MASK | REG_C_PERIODIC_FLAG_MASK;
        _irq_wire.raise();
    }
}


void CMOS::alarm_timer_proc()
{
    Lock lock(_mutex);

    ASSERT(get_state() == VMPart::RUNNING);

    if (is_clock_halted() || !interrupt_on_alarm() || !is_lazy_mode()) {
        return;
    }

    nox_time_t update_time = get_monolitic_time();

    if (lazy_update(update_time)) {
        return;
    }

    D_MESSAGE("unexpected too early. rescheduling");
    reschedule_alarm(update_time);
}


bool CMOS::lazy_update(nox_time_t now)
{
    if (is_clock_halted() || !is_lazy_mode()) {
        return false;
    }

    ASSERT(_next_update_time);

    if (now < _next_update_time) {
        return false;
    }

    uint64_t delta = 1 + (now - _next_update_time) / NANO_PER_SEC;
    time_t time = timegm(&_date);

    if (_alarm_time <= time + delta) {
        if (_alarm_time < time + delta) {
            time_t shift = time + delta - _alarm_time;
            W_MESSAGE("too late guest time shift backword %lusec", shift);
            ASSERT(delta - shift > 0);
            time += delta - shift;
        } else {
            time += delta;
        }

        gmtime_r(&time, &_date);
        _next_update_time += delta * NANO_PER_SEC;

        _reg_c |= REG_C_ALARM_FLAG_MASK | REG_C_UPDATE_FLAG_MASK;

        if (interrupt_on_alarm()) {
            _reg_c |= REG_C_INTERRUPT_FLAG_MASK;
            _irq_wire.raise();
            get_nox().get_pm_controller().alarm();
        }

        reschedule_alarm(now);
        return true;
    } else {
        _reg_c |= REG_C_UPDATE_FLAG_MASK;
        time += delta;
        gmtime_r(&time, &_date);
        _next_update_time += delta * NANO_PER_SEC;
        return false;
    }
}


void CMOS::update_cycle()
{
    Lock lock(_mutex);

    ASSERT(get_state() == VMPart::RUNNING);

    if (is_clock_halted() || is_lazy_mode()) {
        return;
    }

    uint64_t now = get_monolitic_time();

    ASSERT(_next_update_time);

    if (now < _next_update_time) {
        D_MESSAGE("too early");
        _update_timer->modify(_next_update_time - now);
        return;
    }

    if (now - _next_update_time > NANO_PER_SEC) {
       nox_time_t shift = (now - _next_update_time) / NANO_PER_SEC;
        W_MESSAGE("too late. guest time shift backword %lusec", shift);
        _next_update_time += shift * NANO_PER_SEC;
    }

    time_t time = timegm(&_date) + 1;
    gmtime_r(&time, &_date);

    _next_update_time += NANO_PER_SEC;
    _update_timer->modify(_next_update_time - now);

    if (_date.tm_hour == _hours_alarm && _date.tm_min == _minutes_alarm &&
                                                          _date.tm_sec == _seconds_alarm) {
        _reg_c |= REG_C_ALARM_FLAG_MASK;

        if (interrupt_on_alarm()) {
            get_nox().get_pm_controller().alarm();
        }
    }

    _reg_c |= REG_C_INTERRUPT_FLAG_MASK | REG_C_UPDATE_FLAG_MASK;
    _irq_wire.raise();
}


void CMOS::set_reg_a(uint8_t val)
{
    val &= ~REG_A_UPDATE_IN_PROGRESS_MASK;

    if ((val & REG_A_RATE_MASK) && (_reg_b & REG_B_ENABLE_PERIODIC_INTERRUPT_MASK)) {
        _period_timer->arm(rates_table[val & REG_A_RATE_MASK], true);
    } else {
        _period_timer->disarm();
    }

    if (((val & REG_A_DIVIDER_MASK) >> REG_A_DIVIDER_SHIFT) != REG_A_DIVIDER_NORMAL) {
        W_MESSAGE("ignoring REG A divider");
    }

    _reg_a = val;
}


// called after time is up to date
void CMOS::reschedule_alarm(nox_time_t update_time)
{
    if (is_clock_halted() || !is_lazy_mode()) {
        _alarm_timer->disarm();
        return;
    }

    uint now = (_date.tm_hour * 60 + _date.tm_min) * 60 + _date.tm_sec;
    uint dest = (uint(_hours_alarm) * 60 + _minutes_alarm) * 60 + _seconds_alarm;

    if (now > dest) {
        dest = 24 * 60 * 60 - now + dest;
    } else if (now < dest){
        dest -= now;
    } else {
        dest = 24 * 60 * 60;
    }

    _alarm_time = timegm(&_date) + dest;

    if (interrupt_on_alarm()) {
        nox_time_t next_update = _next_update_time - update_time;
        _alarm_timer->arm(nox_time_t(dest -1) * NANO_PER_SEC + next_update , true);
    } else {
        _alarm_timer->disarm();
    }
}


void CMOS::set_reg_b(uint8_t val, nox_time_t now)
{
    lazy_update(now);

    if ((val & REG_B_HALT_CLOCK_MASK) != (_reg_b & REG_B_HALT_CLOCK_MASK)) {
        if (val & REG_B_HALT_CLOCK_MASK) {
            _update_timer->disarm();
            _alarm_timer->disarm();
            _suspend_time = now;
        } else {
            ASSERT(_suspend_time);
            _next_update_time = now + MAX(0, int64_t(_next_update_time - _suspend_time));
            _suspend_time = 0;
        }
    }

    _reg_b = val;

    if ((_reg_a & REG_A_RATE_MASK) && (_reg_b & REG_B_ENABLE_PERIODIC_INTERRUPT_MASK)) {
        _period_timer->arm(rates_table[_reg_a & REG_A_RATE_MASK], true);
    } else {
        _period_timer->disarm();
    }

    if (!is_clock_halted() && !is_lazy_mode()) {
        _update_timer->arm(_next_update_time - now, true);
    } else {
        _update_timer->disarm();
    }

    reschedule_alarm(now);
}


uint8_t CMOS::localize_hours(uint8_t val)
{
    if (_reg_b & REG_B_24_HOURS_MASK) {
        return localize(val) % 24;
    }

    bool pm = !!(val & PM_MASK);

    val = localize(val & ~PM_MASK);
    val = MAX(val, 1);
    val = MIN(val, 12);

    if (pm) {
        return val + 11;
    } else {
        return (val == 12) ? 0 : val;
    }
}


void CMOS::write_byte(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    if (port == 0x70) {
        get_nox().set_nmi_mask(!!(val & NMI_MASK));
        _index = val & INDEX_MASK;
    } else if (port == 0x71) {
        nox_time_t update_time = get_monolitic_time();
        lazy_update(update_time);

        switch (_index) {
        case SECONDS:
            val = localize(val);
            _date.tm_sec = val % 60;
            break;
        case SECONDS_ALARM:
            _seconds_alarm = localize(val) % 60;
            break;
        case MINUTES:
            val = localize(val);
            _date.tm_sec = val % 60;
            break;
        case MINUTES_ALARM:
            _minutes_alarm = localize(val) % 60;
            break;
        case HOURS:
            _date.tm_hour = localize_hours(val);
            break;
        case HOURS_ALARM:
            _hours_alarm = localize_hours(val);
            break;
        case DAY_OF_WEEK:
            val = localize(val) - 1;
            val %= 7;
            _date.tm_wday = val;
            break;
        case DAY_OF_MOUNTH:
            val = MAX(localize(val), 1);
            _date.tm_mday = MIN(val, 31);
            break;
        case MOUNTH:
            val = MAX(localize(val) - 1, 1);
            _date.tm_mon = MIN(val, 11);
            break;
        case YEAR:
            _date.tm_year = localize(val);
            break;
        case REGA:
            set_reg_a(val);
            return;
        case REGB:
            set_reg_b(val, update_time);
            return;
        case REGC:
        case REGD:
            return;
        default:
            D_MESSAGE("[0x%x] = 0x%x", _index, val);
            _user_ares[_index - USER_0] = val;
            return;
        }

        reschedule_alarm(update_time);

    } else {
        PANIC("unexpectd");
    }
}


inline uint CMOS::localize(uint val)
{
    if (_reg_b & REG_B_BINARY_MASK) {
        return val;
    }

    return from_bcd(val);
}


inline uint CMOS::delocalize(uint val)
{
    if (_reg_b & REG_B_BINARY_MASK) {
        return val;
    }

    return to_bcd(val);
}


uint8_t CMOS::delocalize_hours(uint8_t val)
{
    if (_reg_b & REG_B_24_HOURS_MASK) {
        return delocalize(val);
    }

    if (val < 12) {
        return delocalize(val ? val : 12);
    } else {
        return delocalize(val - 11) | PM_MASK;
    }
}


uint8_t CMOS::get_update_in_progress(nox_time_t update_time)
{
    if ((_reg_b & REG_B_HALT_CLOCK_MASK)) {
        return 0;
    }

    // according to ICH7 doc 0 => update will not start for at last 488 micro and the entire
    // update cycle dose not take more than 1984 micro
    // todo: measure the time it take to read the date and time from guest
    //       and configure it threshold accordingly
    int64_t delta = _next_update_time - update_time;
    return (delta >= 2 * NANO_PER_MILI) ? 0 : REG_A_UPDATE_IN_PROGRESS_MASK;
}


uint8_t CMOS::read_byte(uint16_t port)
{
    Lock lock(_mutex);

    if (port == 0x71) {
        nox_time_t update_time = get_monolitic_time();
        lazy_update(update_time);

        switch (_index) {
        case SECONDS:
            return delocalize(_date.tm_sec % 60);
        case SECONDS_ALARM:
            return delocalize(_seconds_alarm);
        case MINUTES:
            return delocalize(_date.tm_min);
        case MINUTES_ALARM:
            return delocalize(_minutes_alarm);
        case HOURS:
            return delocalize_hours(_date.tm_hour);
        case HOURS_ALARM:
            return delocalize_hours(_hours_alarm);
        case DAY_OF_WEEK:
            return delocalize(_date.tm_wday + 1);
        case DAY_OF_MOUNTH:
            return delocalize(_date.tm_mday);
        case MOUNTH:
            return delocalize(_date.tm_mon + 1);
        case YEAR:
            return delocalize(_date.tm_year);
        case REGA: {
            return _reg_a | get_update_in_progress(update_time);
        }
        case REGB:
            return _reg_b;
        case REGC: {
            uint8_t val = _reg_c;

            if ((_reg_a & REG_A_RATE_MASK) && !(_reg_b & REG_B_ENABLE_PERIODIC_INTERRUPT_MASK)) {
                uint32_t rate = rates_table[_reg_a & REG_A_RATE_MASK];

                if (update_time - _last_periodic_time >= rate) {
                    _last_periodic_time = update_time - (update_time % rate);
                    val |= REG_C_PERIODIC_FLAG_MASK;
                }
            }

            _reg_c = 0;
            _irq_wire.drop();
            return val;
        }
        case REGD:
            // REG_D_POWER_STABLE_MASK is set automaticaly after
            // reading this reg
            return REG_D_POWER_STABLE_MASK;
        case CMOD_OFFSET_SHUTDOWN:
        case CMOD_OFFSET_FLOPPY_TYPE:
        case CMOD_OFFSET_HD_TYPE:
        case CMOD_OFFSET_EQUIPMENT:
        case CMOD_OFFSET_BASE_MEM_LOW:
        case CMOD_OFFSET_BASE_MEM_HIGH:
        case CMOD_OFFSET_EXT_MEM_LOW:
        case CMOD_OFFSET_EXT_MEM_HIGH:
        case CMOD_OFFSET_HD_EXT_TYPE:
        case CMOD_OFFSET_HD0_T47_CYL_LOW:
        case CMOD_OFFSET_HD0_T47_CYL_HIGH:
        case CMOD_OFFSET_HD0_T47_HEADS:
        case CMOD_OFFSET_HD0_T47_SECTORS:
        case CMOD_OFFSET_HD0_T47_PRECOMP_LOW:
        case CMOD_OFFSET_HD0_T47_PRECOMP_HIGH:
        case CMOD_OFFSET_HD0_T47_INFO:
        case CMOD_OFFSET_HD0_T47_LENDING_LOW:
        case CMOD_OFFSET_HD0_T47_LENDING_HIGH:
        case CMOD_OFFSET_EXT_MEM_LOW_ALIAS:
        case CMOD_OFFSET_EXT_MEM_HIGH_ALIAS:
        case CMOD_OFFSET_MEM_ABOVE_16M_LOW:
        case CMOD_OFFSET_MEM_ABOVE_16M_HIGH:
        case CMOD_OFFSET_BOOT_DEV_LOW:
        case CMOD_OFFSET_MEM_ABOVE_4G_LOW:
        case CMOD_OFFSET_MEM_ABOVE_4G_MID:
        case CMOD_OFFSET_MEM_ABOVE_4G_HIGH:
        case CMOD_OFFSET_NUM_CPUS:
        case CMOD_OFFSET_CENTURY:
        case CMOD_OFFSET_PS2_CENTURY:
        case CMOD_OFFSET_BOOT_DEV_HIGH:
        case CMOD_OFFSET_HD_GEO_TRANSLATION:
            return _user_ares[_index - USER_0];
        default:
            W_MESSAGE("user 0x%x", _index);
            return _user_ares[_index - USER_0];
        }
    } else {
        W_MESSAGE("0x%x", port);
        return 0xff;
    }
}


void CMOS::host_write(uint index, uint value)
{
    ASSERT(get_nox().get_state() == VMPart::RESETING);
    ASSERT(index >= USER_0 && index < (1 << 8));

    _user_ares[index - USER_0] = value;
}


bool CMOS::start()
{
    nox_time_t now = get_monolitic_time();

    if (_date_base_time) {
        time_t time = timegm(&_date);
        time += (now - _date_base_time) / NANO_PER_SEC;
        gmtime_r(&time, &_date);
        _date_base_time = 0;
    }

    if ((_reg_a & REG_A_RATE_MASK) && (_reg_b & REG_B_ENABLE_PERIODIC_INTERRUPT_MASK)) {
        _period_timer->arm(rates_table[_reg_a & REG_A_RATE_MASK], true);
    }

    if (!is_clock_halted()) {
        ASSERT(_suspend_time);
        _next_update_time = now + MAX(0, int64_t(_next_update_time - _suspend_time));
        _suspend_time = 0;

        if (!is_lazy_mode()) {
            _update_timer->arm(_next_update_time - now, true);
        }
    }

    reschedule_alarm(now);

    return true;
}


bool CMOS::stop()
{
    _period_timer->disarm();
    _update_timer->disarm();
    _alarm_timer->disarm();

    if (!is_clock_halted()) {
        _suspend_time = get_monolitic_time();
    }

    return true;
}

