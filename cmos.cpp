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


CMOS::CMOS(NoxVM& vm)
    : VMPart ("cmos", vm)
    , _io_region (NULL)
    , _irq_is_massked (true)
    , _index (0)
{
    memset(_user_ares, 0, sizeof(_user_ares));
    _io_region = vm.get_io_bus().register_region(*this, 0x70, 2, this,
                                                 (io_read_byte_proc_t)&CMOS::read_byte,
                                                 (io_write_byte_proc_t)&CMOS::write_byte);
}


CMOS::~CMOS()
{
    nox().get_io_bus().unregister_region(_io_region);
}


void CMOS::write_byte(uint16_t port, uint8_t val)
{
    if (port == 0x70) {
        _irq_is_massked = !!(val & NMI_MASK);
        _index = val & INDEX_MASK;
    } else if (port == 0x71) {
        switch (_index) {
        case SECONDS:
        case SECONDS_ALARM:
        case MINUTS:
        case MINUTED_ALARM:
        case HOUERS:
        case HOUERS_ALARM:
        case DAY_OF_WEEK:
        case DAY_OF_MOUNTH:
        case MOUNTH:
        case YEAR:
        case REGA:
        case REGB:
        case REGC:
        case REGD:
            PANIC("implement me");
            break;
        default:
            W_MESSAGE("user 0x%x", _index);
            _user_ares[_index - USER_0] = val;
        }
    } else {
        PANIC("unexpectd");
    }
}


uint8_t CMOS::read_byte(uint16_t port)
{
    if (port == 0x71) {
        switch (_index) {
        case SECONDS: {
            struct tm tm;;
            time_t t = time(NULL);
            gmtime_r(&t, &tm);
            return tm.tm_sec;
        }
        case SECONDS_ALARM:
            PANIC("implement me");
            break;
        case MINUTS: {
            struct tm tm;;
            time_t t = time(NULL);
            gmtime_r(&t, &tm);
            return tm.tm_min;
        }
        case MINUTED_ALARM:
            PANIC("implement me");
            break;
        case HOUERS: {
            struct tm tm;;
            time_t t = time(NULL);
            gmtime_r(&t, &tm);
            return tm.tm_hour;
        }
        case HOUERS_ALARM:
        case DAY_OF_WEEK:
        case DAY_OF_MOUNTH:
        case MOUNTH:
        case YEAR:
        case REGA:
        case REGB:
        case REGC:
        case REGD:
            PANIC("implement me");
            break;
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
    ASSERT(index >= USER_0 && index < (1 << 8));

    _user_ares[index - USER_0] = value;
}

