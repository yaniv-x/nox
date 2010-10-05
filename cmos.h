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

#include "vm_part.h"

class NoxVM;
class IORegion;

class CMOS: private VMPart {
public:
    CMOS(NoxVM& vm);
    virtual ~CMOS();

    NoxVM& nox() { return *(NoxVM*)get_container();}

    virtual void reset() {}
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void host_write(uint index, uint value);

private:
    virtual void write_byte(uint16_t port, uint8_t val);
    virtual uint8_t read_byte(uint16_t port);

    enum {
        SECONDS,
        SECONDS_ALARM,
        MINUTS,
        MINUTED_ALARM,
        HOUERS,
        HOUERS_ALARM,
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
    IORegion* _io_region;
    uint8_t _user_ares[USER_SIZE];
    bool _irq_is_massked;
    uint _index;
};

#endif

