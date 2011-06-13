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

#include <malloc.h>

#include "ata_device.h"
#include "ata.h"
#include "wire.h"
#include "dma_state.h"

enum {
    ATA_DEV_NUM_IO_BLOCKS = 1,
};


#define ATA_LOG(format, ...)


ATADevice::ATADevice(const char* name, VMPart& owner, Wire& wire)
    : VMPart("", owner)
    , _irq_wire (wire)
    , _async_count (0)
    , _async_stop (false)
    , _irq_level (0)
    , _task (NULL)
    , _dead_task (NULL)
    , _pio_source (NULL)
    , _pio_dest (NULL)
{
    _io_blocks_data = (uint8_t*)memalign(ATADEV_IO_BLOCK_SIZE,
                                         ATADEV_IO_BLOCK_SIZE * ATA_DEV_NUM_IO_BLOCKS);

    if (!_io_blocks_data) {
        THROW("memalign failed");
    }

    for (uint i = 0; i < ATA_DEV_NUM_IO_BLOCKS; i++) {
        _free_blocks_list.push_front(_io_blocks_data + i * ATADEV_IO_BLOCK_SIZE);
    }
}


ATADevice::~ATADevice()
{
    ASSERT(_task == NULL);

    if (_dead_task) {
        _dead_task->unref();
    }

    free(_io_blocks_data);
}


uint8_t* ATADevice::get_io_block()
{
    Lock lock(_io_blocks_mutex);

    if (_free_blocks_list.empty()) {
        return NULL;
    }

    uint8_t* ret = _free_blocks_list.front();
    _free_blocks_list.pop_front();

    return ret;
}


void ATADevice::put_io_block(uint8_t* block)
{
    ASSERT(block);

    Lock lock(_io_blocks_mutex);
    _free_blocks_list.push_front(block);
}


void ATADevice::reset(bool cold)
{
    if (cold) {
        _reverting_to_power_on_default = true;
    }

    //Lock lock(_tasks_mutex);
    D_MESSAGE("");

    if (_task) {
        D_MESSAGE("active task");
        _task->cancel();
    }

    ASSERT(_task == NULL);

    if (_dead_task) {
        _dead_task->unref();
        _dead_task = NULL;
    }

    if (cold) {
        _irq_wire.reset();
    } else {
        _irq_wire.drop();
    }

    _status = 0;
    _control = 0;
    _feature = 0;
    _error = ATA_DIAGNOSTIC_D0_OK_D1_NOT_PRESENT;
    _irq_level = 0;
    set_signature();

    _pio_dest = NULL;
    _pio_source = NULL;

    _power_mode = POWER_ACTIVE;
}


void ATADevice::start_task(ATATask* task)
{
    Lock lock(_tasks_mutex);

    if (_dead_task) {
        _dead_task->unref();
        _dead_task = NULL;
    }

    ASSERT(_task == NULL);
    _task = task->ref();
    _status |= ATA_STATUS_BUSY_MASK;
    lock.unlock();
    task->start();
}


void ATADevice::remove_task(ATATask* task)
{
    Lock lock(_tasks_mutex);

    ASSERT(task == _task && _dead_task == NULL);
    _task = NULL;
    _dead_task = task;
}


ATATask* ATADevice::get_task()
{
    Lock lock(_tasks_mutex);

    if (!_task) {
        return NULL;
    }

    return _task->ref();
}


void ATADevice::dec_async_count()
{
    uint32_t count = _async_count.dec();

    if (!count) {
        Lock lock(_stop_mutex);
        if (_async_stop && !_async_count.val()) {
            _async_stop = false;
            transition_done();
        }
    }
}


bool ATADevice::stop()
{
    Lock lock(_stop_mutex);

    if (!_async_count.val()) {
        return true;
    }

    _async_stop = true;
    return false;
}


void ATADevice::reset()
{
    reset(true);
}


void ATADevice::soft_reset()
{
    reset(false);
    _status = ATA_STATUS_BUSY_MASK | ATA_INTERNAL_STATUS_RESET_MASK;
}


uint8_t ATADevice::io_alt_status()
{
    //  - not valid in sleep mode
    //  - when the BSY bit is set to one, the other bits in this register shall not be used

    if (current() == 1) {
        ATA_LOG("alt_status: slave ret 0x%x", 0);
        return 0;
    }

    ATA_LOG("alt_status: 0x%x", _status);
    return _status;
}


uint ATADevice::current()
{
    return (_device_reg >> ATA_DEVICE_SELECT_BIT) & 1;
}


void ATADevice::raise()
{
    // nIEN is the enable bit for the device assertion of INTRQ to the host. When the nIEN bit
    // is cleared to zero, and the device is selected, INTRQ shall be enabled through a driver
    // capable of a high-impedance output state and shall be asserted or negated by the device
    // as appropriate. When the nIEN bit is set to one, or the device is not selected, the device
    // shall release the INTRQ signal.

    ATA_LOG("raise");
    _irq_level = 1;

    if ((_control & ATA_CONTROL_DISABLE_INTERRUPT_MASK) || current() == 1) {
        return;
    }

    _irq_wire.raise();
}


void ATADevice::drop()
{
    ATA_LOG("drop");
    _irq_level = 0;
    _irq_wire.drop();
}


void ATADevice::command_abort_error()
{
    ATA_LOG("abort");
    _error = ATA_ERROR_ABORT;
    set_state_and_notify(_status | ATA_STATUS_ERROR_MASK);
}


void ATADevice::command_abort_error(DMAState& dma)
{
    ATA_LOG("abort");
    _error = ATA_ERROR_ABORT;
    set_state_and_notify(_status | ATA_STATUS_ERROR_MASK, dma);
}


void ATADevice::dma_wait()
{
    uint new_state = (_status | ATA_STATUS_DATA_REQUEST_MASK) & ~ATA_STATUS_BUSY_MASK;
    _status = new_state;
    // DMARQ=A ?
}


void ATADevice::dma_write_start(DMAState& state)
{
    AutoRef<ATATask> task(get_task());

    if (!task.get() || !task->dma_write_start(state)) {
        D_MESSAGE("no active dma client");
        state.done();
    }
}


void ATADevice::dma_read_start(DMAState& state)
{
    AutoRef<ATATask> task(get_task());

    if (!task.get() || !task->dma_read_start(state)) {
        D_MESSAGE("no active dma client");
        state.done();
    }
}


void ATADevice::io_control(uint8_t val)
{
    // in device 0 only configurations: write to the device control register shall complete as if
    // device 0 was the selected device;

    ATA_LOG("control: 0x%x", val);

    Lock lock(_mutex);

    // This register shall only be written when DMACK- is not asserted.
    if (false /*|| DMACK*/) {
        W_MESSAGE("DMACK");
        return;
    }

    _control = val;

    if (val & ATA_CONTROL_RESET_MASK) {

         if ((_status & ATA_INTERNAL_STATUS_RESET_MASK) &&
             (_status & ATA_STATUS_BUSY_MASK)) {
             return;
         }

         ATA_LOG("reset start");
         soft_reset();
         return;
    }

    if (_status & ATA_INTERNAL_STATUS_RESET_MASK) {
        _status = ATA_STATUS_READY_MASK | ATA_STATUS_SEEK_COMPLEAT;
        ATA_LOG("reset done");
    }

    if ((_control & ATA_CONTROL_DISABLE_INTERRUPT_MASK)) {
       _irq_wire.drop();
    } else if (_irq_level) {
        raise();
    }
}


inline void ATADevice::clear_HOB()
{
    _control &= ~ATA_CONTROL_HOB_MASK;
}


inline uint8_t ATADevice::byte_by_HOB(uint16_t val)
{
    return (_control & ATA_CONTROL_HOB_MASK) ? val >> 8 : val;
}


uint8_t ATADevice::io_read(uint16_t port)
{
    // in device 0 only configurations:
    //      1. if the device does not implement the PACKET command feature set, a read of the
    //         control block or command Block registers, other than the status or alternate status
    //         registers, shall complete as if device 0 was selected. a read of the status or
    //         alternate status register shall return the value 0
    //      2. if the device implements the PACKET Command feature set, a read of the control Block
    //         or command block registers shall return the value 0.

    uint8_t ret;

    switch (port) {
    case ATA_IO_STATUS: {
        Lock lock(_mutex);
        drop();
        ret = _status;
        break;
    }
    case ATA_IO_ERROR:
        ret = _error;
        break;
    case ATA_IO_DEVICE:
        ret = _device_reg;
        break;
    case ATA_IO_SECTOR_COUNT:
        ret = byte_by_HOB(_count);
        break;
    case ATA_IO_LBA_LOW:
        ret = byte_by_HOB(_lba_low);
        break;
    case ATA_IO_LBA_MID:
        ret = byte_by_HOB(_lba_mid);
        break;
    case ATA_IO_LBA_HIGH:
        ret = byte_by_HOB(_lba_high);
        break;
    default:
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }

    ATA_LOG("read: %s [0x%x] 0x%x", reg_offset_to_name(port, true), port, ret);
    return ret;
}


void ATADevice::io_write(uint16_t port, uint8_t val)
{
    // in device 0 only configurations:
    //      1. a write to a command block register, other than the command register, shall
    //         complete as if device 0 was selected.
    //      2. a write to the Command register shall be ignored, except for EXECUTE DEVICE
    //         DIAGNOSTIC;

    Lock lock(_mutex);

    ATA_LOG("write: %s [0x%x] 0x%x", reg_offset_to_name(port, false), port, val);

    clear_HOB();

    // abort on device not ready

    switch (port) {
    case ATA_IO_COMMAND:
        do_command(val);
        return;
    case ATA_IO_SECTOR_COUNT:
        if ((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _count = (_count << 8) | val;
        return;
    case ATA_IO_LBA_LOW:
        if ((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _lba_low = (_lba_low<< 8) | val;
        return;
    case ATA_IO_LBA_MID:
        if ((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _lba_mid = (_lba_mid << 8) | val;
        return;
    case ATA_IO_LBA_HIGH:
        if ((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _lba_high = (_lba_high << 8) | val;
        return;
    case ATA_IO_FEATURE:
        if ((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _feature = val;
        return;
    case ATA_IO_DEVICE:
        if ((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }

        // for devices implementing the PACKET Command feature set, the contents of this register
        // are valid while the device is in Sleep mode

        _device_reg = val | ATA_DEVICE_MUST_BE_ONE_MASK;

        if (!(_device_reg & ATA_DEVICE_SELECT_MASK)) {
            _status |= ATA_STATUS_READY_MASK /*| ATA_STATUS_SEEK_COMPLEAT*/;

            if (_irq_level) {
                raise();
            }
        } else {
            _status &= ~ATA_STATUS_READY_MASK;
        }

        return;
    default:
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }
}


uint16_t ATADevice::io_read_word(uint16_t port)
{
    Lock lock(_mutex);

    if (!_pio_source) {
        W_MESSAGE("unexpected read: no data source 0x%x", _status);
        return ~0;
    }

    return _pio_source->get_word();
}


void ATADevice::notify_command_done()
{
    set_state_and_notify(_status & ~(ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK));
}


void ATADevice::notify_command_done(DMAState& dma)
{
    set_state_and_notify(_status & ~(ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK), dma);
}


void ATADevice::set_pio_source(PIODataSource* pio)
{
    _pio_source = pio;

    uint new_state = (_status | ATA_STATUS_DATA_REQUEST_MASK) & ~ATA_STATUS_BUSY_MASK;
    set_state_and_notify(new_state);
}


void ATADevice::remove_pio_source(bool done)
{
    _pio_source = NULL;

    if (!done) {
        uint new_state = (_status | ATA_STATUS_BUSY_MASK) & ~ATA_STATUS_DATA_REQUEST_MASK;
        _status = new_state;
    } else {
        notify_command_done();
    }
}


void ATADevice::set_pio_dest(PIODataDest* pio, bool notify)
{
    _pio_dest = pio;

    uint new_state = (_status | ATA_STATUS_DATA_REQUEST_MASK) & ~ATA_STATUS_BUSY_MASK;

    if (notify) {
        set_state_and_notify(new_state);
    } else {
        _status = new_state;
    }
}


void ATADevice::remove_pio_dest(bool done)
{
    _pio_dest = NULL;

    if (done) {
        notify_command_done();
        return;
    }

    drop();
    uint new_state = (_status | ATA_STATUS_BUSY_MASK) & ~ATA_STATUS_DATA_REQUEST_MASK;
    _status = new_state;
}


void ATADevice::io_write_word(uint16_t port, uint16_t val)
{
    Lock lock(_mutex);

    if (!_pio_dest) {
        W_MESSAGE("unexpected read: no data source 0x%x", _status);
        return;
    }

    _pio_dest->put_word(val);
}


void set_ata_str(uint16_t* start, int len, const char* str)
{
    int n = MIN(len * 2, strlen(str));
    uint8_t* dest = (uint8_t*)start;
    int i;

    memset(dest, ' ', len * 2);

    for (i = 0; i < n; i += 2) dest[i + 1] = str[i];
    for (i = 1; i < n; i += 2) dest[i - 1] = str[i];
}


int8_t checksum8(void *start, uint size)
{
    uint8_t res = 0;
    uint8_t *now = (uint8_t*)start;
    uint8_t *end = now + size;

    for (; now < end; now++) {
        res += *now;
    }

    return -res;
}

