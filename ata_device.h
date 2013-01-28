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
#include "dma_state.h"

class DMAState;
class Wire;

enum {
    ATADEV_IO_BLOCK_SIZE = 64 * KB,
};

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
    virtual bool dma_write_start(DMAState& state) { return false;}
    virtual bool dma_read_start(DMAState& state) { return false;}

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

    void set_pio_source(PIODataSource* pio);
    void remove_pio_source(bool done);

    void set_pio_dest(PIODataDest* pio, bool notify = true);
    void remove_pio_dest(bool done);

    void dma_wait();
    void dma_write_start(DMAState& state);
    void dma_read_start(DMAState& state);

protected:
    virtual void load(InStream &stream) {}
    virtual void reset();
    virtual void save(OutStream &stream) {}
    virtual bool start() { return true;}
    virtual bool stop();
    virtual void down();

    virtual void set_signature() = 0;
    void set_transfer_mode();
    virtual void do_set_features();
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
    void set_state_and_notify(uint state);
    void notify_command_done();
    void command_abort_error();
    void set_state_and_notify(uint state, DMAState& dma);
    void command_abort_error(DMAState& dma);
    void notify_command_done(DMAState& dma);
    uint get_multiword_mode() { return _multiword_mode;}
    uint get_ultra_mode() { return _ultra_mode;}
    bool revert_to_default() { return _reverting_to_power_on_default;}

    void start_task(ATATask* task);
    void remove_task(ATATask* task);
    ATATask* get_task();
    void dec_async_count();
    void inc_async_count() { _async_count.inc();}

    uint8_t* get_io_block();
    void put_io_block(uint8_t*);

private:
    void soft_reset();
    void clear_HOB();
    uint8_t byte_by_HOB(uint16_t val);

private:
    RecursiveMutex _mutex;
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

    Mutex _io_blocks_mutex;
    uint8_t* _io_blocks_data;
    std::list<uint8_t*> _free_blocks_list;

    uint _multiword_mode;
    uint _ultra_mode;
    bool _reverting_to_power_on_default;

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
};


inline void ATADevice::set_state_and_notify(uint state)
{
    Lock lock(_mutex);
    _status = state;
    raise();
}


inline void ATADevice::set_state_and_notify(uint state, DMAState& dma)
{
    Lock lock(_mutex);
    dma.done();
    _status = state;
    raise();
}


void set_ata_str(uint16_t* start, int len, const char* str);
int8_t checksum8(void *start, uint size);

#ifdef ATA_DEBUG
const char* command_name(uint8_t command);
#endif

#endif

