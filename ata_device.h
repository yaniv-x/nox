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

#ifndef _H_ATA_DEVICE
#define _H_ATA_DEVICE

#include "vm_part.h"
#include "threads.h"

class Wire;

class ATATask {
public:
    ATATask()
        : _refs (1)
    {
    }

    ATATask* ref() { _refs.inc(); return this;}
    void unref() { if (!_refs.dec()) delete this;}

    virtual void start() = 0;
    virtual void cancel() = 0;

protected:
    virtual ~ATATask() {}

private:
    Atomic _refs;
};

class PIODataSource {
public:
    virtual uint16_t get_word() = 0;
};

class PIODataDest {
public:
    virtual void put_word(uint16_t data) = 0;
};


class ATADevice: public VMPart {
public:
    ATADevice(const char* name, VMPart& owner, Wire& wire);
    virtual ~ATADevice();

    uint8_t io_alt_status();
    void io_control(uint8_t val);
    virtual uint8_t io_read(uint16_t port);
    void io_write(uint16_t port, uint8_t val);
    uint16_t io_read_word(uint16_t port);
    void io_write_word(uint16_t port, uint16_t val);

    void notify_command_done();

    void set_pio_source(PIODataSource* pio);
    void remove_pio_source(bool done);

    void set_pio_dest(PIODataDest* pio, bool notify = true);
    void remove_pio_dest(bool done);

protected:
    virtual void load(InStream &stream) {}
    virtual void power() {}
    virtual void reset();
    virtual void save(OutStream &stream) {}
    virtual bool start() { return true;}
    virtual bool stop();

    virtual void set_signature() = 0;
    virtual void do_command(uint8_t command) = 0;
    virtual void reset(bool cold);

    uint current();
    void raise();
    void drop();

    enum PowerState {
        POWER_ACTIVE,
        POWER_IDLE,
        POWER_STANDBY,
        POWER_SLEEP,
    };

    PowerState get_power_mode() { return _power_mode;}
    void set_power_mode(PowerState mode) { _power_mode = mode;}
    void command_abort_error();
    void start_task(ATATask* task);
    void remove_task(ATATask* task);
    ATATask* get_task();
    void dec_async_count();
    void inc_async_count() { _async_count.inc();}

private:
    void soft_reset();
    void clear_HOB();
    uint8_t byte_by_HOB(uint16_t val);

private:
    Mutex _mutex;
    Wire& _irq_wire;

    Atomic _async_count;
    Mutex _stop_mutex;
    bool _async_stop;

    uint _irq_level;
    PowerState _power_mode;

    Mutex _tasks_mutex;
    ATATask* _task;
    ATATask* _dead_task;

    PIODataSource* _pio_source;
    PIODataDest* _pio_dest;

protected:
    uint _status;
    uint _count;
    uint _control;
    uint _feature;
    uint _error;
    uint _device_reg;
    uint _lba_low;
    uint _lba_mid;
    uint _lba_high;

    bool _reverting_to_power_on_default;
};


void set_ata_str(uint16_t* start, int len, const char* str);
int8_t checksum8(void *start, uint size);


#endif

