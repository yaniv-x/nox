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

#ifndef _H_CMOS
#define _H_CMOS

#include <time.h>
#include "vm_part.h"
#include "threads.h"
#include "wire.h"

class NoxVM;
class Timer;

class CMOS: private VMPart {
public:
    CMOS(NoxVM& vm);
    virtual ~CMOS();

    virtual void reset();
    virtual bool start();
    virtual bool stop();
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void host_write(uint index, uint value);

private:
    virtual void write_byte(uint16_t port, uint8_t val);
    virtual uint8_t read_byte(uint16_t port);
    void update_date();
    uint8_t localize_hours(uint8_t val);
    uint localize(uint val);
    uint8_t delocalize_hours(uint8_t val);
    uint delocalize(uint val);
    void period_timer_proc();
    void alarm_timer_proc();
    void update_timer_proc();
    void reschedule_alarm();
    void set_reg_a(uint8_t val);
    void set_reg_b(uint8_t val);

    enum {
        SECONDS,
        SECONDS_ALARM,
        MINUTES,
        MINUTES_ALARM,
        HOURS,
        HOURS_ALARM,
        DAY_OF_WEEK,
        DAY_OF_MOUNTH,
        MOUNTH,
        YEAR,
        REGA,
        REGB,
        REGC,
        REGD,
        USER_0,

        NMI_MASK = (1 << 7),
        INDEX_MASK = ~NMI_MASK,
        USER_SIZE = (1 << 7) - USER_0,
    };

private:
    Mutex _mutex;
    Wire _irq_wire;
    Timer* _period_timer;
    Timer* _alarm_timer;
    Timer* _update_timer;
    uint8_t _reg_a;
    uint8_t _reg_b;
    uint8_t _reg_c;
    uint8_t _user_ares[USER_SIZE];
    uint _index;
    struct tm _date;
    nox_time_t _date_base_time;
    uint8_t _seconds_alarm;
    uint8_t _minutes_alarm;
    uint8_t _hours_alarm;
    bool _alarm_on_resume;
};

#endif

