/*
    Copyright (c) 2013-2014 Yaniv Kamay,
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


#ifdef ATA_DEBUG
#define ATA_LOG(format, ...) D_MESSAGE(format, ## __VA_ARGS__)
#else
#define ATA_LOG(format, ...)
#endif


enum {
    MULTIWORD_DMA_MODE_MAX = 2,
    ULTRA_DMA_MODE_MAX = 5,
};

enum {
    ASYNC_FLAG_STOP = 1 << 0,
    ASYNC_FLAG_SIGNAL = 1 << 1,
};


ATADevice::ATADevice(const char* name, VMPart& owner, Wire& wire)
    : VMPart("", owner)
    , _irq_wire (wire)
    , _async_count (0)
    , _async_flags (0)
    , _irq_level (0)
    , _task (NULL)
    , _pio_source (NULL)
    , _pio_dest (NULL)
{
    _io_block = (uint8_t*)memalign(ATADEV_IO_BLOCK_SIZE, ATADEV_IO_BLOCK_SIZE);

    if (!_io_block) {
        THROW("memalign failed");
    }
}


ATADevice::~ATADevice()
{
    ASSERT(_task == NULL);

    free(_io_block);
}


void ATADevice::async_wait()
{
    Lock lock(_async_mutex);

    while  (_async_count.val()) {
        D_MESSAGE("async task");
        _async_flags |= ASYNC_FLAG_SIGNAL;
        _async_condition.wait(_async_mutex);
    }
}


void ATADevice::cancel_task()
{
    async_wait();

    if (_task) {
        D_MESSAGE("active task");
        _task->cancel();
    }

    ASSERT(_task == NULL);
}


void ATADevice::reset(bool cold)
{
    D_MESSAGE("");

    cancel_task();

    if (cold) {
        _reverting_to_power_on_default = true;
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

    if (_reverting_to_power_on_default) {
        _multiword_mode = ~0;
        _ultra_mode = ULTRA_DMA_MODE_MAX;
    }
}


void ATADevice::start_task(ATATask* task)
{
    ASSERT(_task == NULL);
    _status |= ATA_STATUS_BUSY_MASK;
    _task = task;
    _task->start();
}


void ATADevice::remove_task(ATATask* task)
{
    Lock lock(_mutex);

    ASSERT(task == _task);
    _task = NULL;
}


void ATADevice::dec_async_count()
{
    if (_async_count.dec()) {
        return;
    }

    Lock lock(_async_mutex);

    if (_async_flags && !_async_count.val()) {

        if ((_async_flags & ASYNC_FLAG_STOP)) {
            _async_flags &= ~ASYNC_FLAG_STOP;
            transition_done();
        }

        if ((_async_flags & ASYNC_FLAG_SIGNAL)) {
            _async_flags &= ~ASYNC_FLAG_SIGNAL;
            _async_condition.signal();
        }
    }
}


bool ATADevice::stop()
{
    Lock lock(_async_mutex);

    if (!_async_count.val()) {
        return true;
    }

    _async_flags |= ASYNC_FLAG_STOP;

    return false;
}


void ATADevice::down()
{
    if (!_task) {
        return;
    }

    _task->cancel();
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
    Lock lock(_mutex);

    if (!_task || !_task->dma_write_start(state)) {
        D_MESSAGE("no active dma client");
        state.done();
    }
}


void ATADevice::dma_read_start(DMAState& state)
{
    Lock lock(_mutex);

    if (!_task || !_task->dma_read_start(state)) {
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


#ifdef ATA_DEBUG
static const char* reg_offset_to_name(uint offset, bool read)
{
    switch (offset) {
    case ATA_IO_ERROR:
        return read ? "ERROR" :"FEATURE";
    case ATA_IO_SECTOR_COUNT:
        return "COUNT";
    case ATA_IO_LBA_LOW:
        return "LBA_LOW";
    case ATA_IO_LBA_MID:
        return "LBA_MID";
    case ATA_IO_LBA_HIGH:
        return "LBA_HIGH";
    case ATA_IO_DEVICE:
        return "DEVICE";
    case ATA_IO_COMMAND:
        return read ? "STATUS" : "COMMAND";
    default:
        return "???";
    }
}


const char* command_name(uint8_t command)
{
    switch (command) {
    case ATA_CMD_PACKET:
        return "PACKET";
    case ATA_CMD_READ_SECTORS:
        return "READ_SECTORS";
    case ATA_CMD_READ_SECTORS_EXT:
        return "READ_SECTORS_EXT";
    case ATA_CMD_READ_VERIFY_SECTORS:
        return "READ_VERIFY_SECTORS";
    case ATA_CMD_READ_VERIFY_SECTORS_EXT:
        return "READ_VERIFY_SECTORS_EXT";
    case ATA_CMD_SEEK:
        return "SEEK";
    case ATA_CMD_SET_FEATURES:
        return "SET_FEATURES";
    case ATA_CMD_WRITE_SECTORS:
        return "WRITE_SECTORS";
    case ATA_CMD_WRITE_SECTORS_EXT:
        return "WRITE_SECTORS_EXT";
    case ATA_CMD_WRITE_DMA:
        return "WRITE_DMA";
    case ATA_CMD_WRITE_DMA_EXT:
        return "WRITE_DMA_EXT";
    case ATA_CMD_READ_DMA:
        return "READ_DMA";
    case ATA_CMD_READ_DMA_EXT:
        return "READ_DMA_EXT";
    case ATA_CMD_READ_MULTIPLE:
        return "READ_MULTIPLE";
    case ATA_CMD_READ_MULTIPLE_EXT:
        return "READ_MULTIPLE_EXT";
    case ATA_CMD_WRITE_MULTIPLE:
        return "WRITE_MULTIPLE";
    case ATA_CMD_WRITE_MULTIPLE_EXT:
        return "WRITE_MULTIPLE_EXT";
    case ATA_CMD_FLUSH_CACHE_EXT:
        return "FLUSH_CACHE_EXT";
    case ATA_CMD_FLUSH_CACHE:
        return "FLUSH_CACHE";
    case ATA_CMD_IDENTIFY_DEVICE:
        return "IDENTIFY_DEVICE";
    case ATA_CMD_IDENTIFY_PACKET_DEVICE:
        return "IDENTIFY_PACKET_DEVICE";
    case ATA_CMD_CHECK_POWER_MODE:
        return "CHECK_POWER_MODE";
    case ATA_CMD_IDLE_IMMEDIATE:
        return "IDLE_IMMEDIATE";
    case ATA_CMD_IDLE:
        return "IDLE";
    case ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC:
        return "EXECUTE_DEVICE_DIAGNOSTIC";
    case ATA_CMD_DEVICE_RESET:
        return "DEVICE_RESET";
    case ATA_CMD_NOP:
        return "NOP";
    case ATA_CMD_SET_MULTIPLE_MODE:
        return "SET_MULTIPLE_MODE";
    case ATA_CMD_SLEEP:
        return "SLEEP";
    case ATA_CMD_STANDBY:
        return "STANDBY";
    case ATA_CMD_STANDBY_IMMEDIATE:
        return "STANDBY_IMMEDIATE";
    case ATA_CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS:
        return "INITIALIZE_DEVICE_PARAMETERS";
    default:
        return "???";
    }
}
#endif


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


void ATADevice::set_transfer_mode()
{
    uint mode = _count & 0x07;
    uint type = (_count & 0xff) >> 3;

    switch (type) {
    case 0:
        if (mode != 0 && mode != 1) {
            D_MESSAGE("invalid type 0x%x mode 0x%x", type, mode);
            command_abort_error();
            return;
        }
        //_pio_mode = default pio mode;
        break;
    case 1:
        if (mode != 3 &&  mode != 4) {
            D_MESSAGE("invalid type 0x%x mode 0x%x", type, mode);
            //command_abort_error();
        }
        //_pio_mode = mode;
        break;
    case 4:
        if (mode > MULTIWORD_DMA_MODE_MAX) {
            D_MESSAGE("invalid type 0x%x mode 0x%x", type, mode);
            command_abort_error();
            return;
        }
        _multiword_mode = mode;
        _ultra_mode = ~0;
        break;
    case 8:
        if (mode > ULTRA_DMA_MODE_MAX) {
            D_MESSAGE("invalid type 0x%x mode 0x%x", type, mode);
            command_abort_error();
            return;
        }

        _multiword_mode = ~0;
        _ultra_mode = mode;
        break;
    default:
        D_MESSAGE("invalid type 0x%x mode 0x%x", type, mode);
        command_abort_error();
        return;
    }

    raise();
}


void ATADevice::do_set_features()
{
    switch (_feature) {
    case ATA_FEATURE_DISABLE_REVERT_TO_DEFAULT:
        _reverting_to_power_on_default = false;
        raise();
        break;
    case ATA_FEATURE_ENABLE_REVERT_TO_DEFAULT:
        _reverting_to_power_on_default = true;
        raise();
        break;
    case ATA_FEATURE_SET_TRANSFER_MODE:
        set_transfer_mode();
        break;
    default:
        D_MESSAGE("unhandled 0x%x. sleeping... ", _feature);
        for (;;) sleep(2);
    }
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
        //todo: no interrupt is needed in case of successfull pio data-in
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

