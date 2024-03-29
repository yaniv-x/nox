/*
    Copyright (c) 2013-2019 Yaniv Kamay,
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
#include <fcntl.h>

#include "ata_disk.h"
#include "ata.h"
#include "application.h"
#include "block_device.h"
#include "memory_bus.h"
#include "dma_state.h"

#ifdef ATA_DEBUG
#define ATA_LOG(format, ...) D_MESSAGE(format, ## __VA_ARGS__)
#else
#define ATA_LOG(format, ...)
#endif

enum {
    SECTOR_SIZE = 512,
    SECTOR_MASK = SECTOR_SIZE - 1,

    ATA3_MAX_CYL = 16383,
    ATA3_MAX_HEAD = 16,
    ATA3_MAX_SEC = 63,

    ATADISK_BUNCH_MAX = 64 * KB / SECTOR_SIZE,
    ATADISK_BUNCH_DEFAULT = 1,
};


class ReadTask: public ATATask, public PIODataSource {
public:
    ReadTask(ATADisk& disk, uint64_t start, uint64_t end, uint bunch)
        : ATATask()
        , _disk (disk)
        , _now (start)
        , _end (end)
        , _bunch (bunch)
        , _data_now (NULL)
        , _data_end (NULL)
    {
        ASSERT(_bunch <= ATADISK_BUNCH_MAX);

        if (!(_io_block = _disk.get_io_block())) {
            THROW("age io block failed");
        }
    }

    void cancel()
    {
        _disk.remove_pio_source(false);
        _disk.remove_task(this);
    }

    void start()
    {
        get_bunch();
    }

    void start_bunch()
    {
        _data_now = (uint16_t*)_io_block;
        _data_end = (uint16_t*)(_io_block + _bunch * SECTOR_SIZE);
        _disk.set_pio_source(this);
    }

    void end()
    {
        _disk.remove_task(this);
        _disk.remove_pio_source(true);
    }

    void error()
    {
        _disk.remove_task(this);
        _disk.remove_pio_source(false);
        _disk.command_abort_error();
    }

    void redv_done(IOVec* vec, int err)
    {
        ATADisk* disk = &_disk;

        if (err) {
            D_MESSAGE("failed %d", err)
             error();
        } else {
            start_bunch();
        }

        disk->dec_async_count();
    }

    void get_bunch()
    {
        _bunch = MIN(_end - _now, _bunch);

        ASSERT(_bunch);

        _disk.inc_async_count();

        _io_vec.iov_base = _io_block;
        _io_vec.iov_len = _bunch * SECTOR_SIZE;
        new (&_iov) IOVec(_now, _bunch, &_io_vec, 1,(io_vec_cb_t)&ReadTask::redv_done, this);
        _disk._block_dev->readv(&_iov);
    }

    uint16_t get_word()
    {
        if (_data_now == _data_end) {
            PANIC("no data");
        }

        uint16_t ret = *_data_now;

        if (++_data_now == _data_end) {
            if ((_now += _bunch) == _end) {
                end();
                return ret;
            }

            _disk.remove_pio_source(false);
            get_bunch();
        }

        return ret;
    }

private:
    ATADisk& _disk;
    uint64_t _now;
    uint64_t _end;
    uint _bunch;
    uint8_t* _io_block;
    uint16_t* _data_now;
    uint16_t* _data_end;
    struct iovec _io_vec;
    IOVec _iov;
};


class ReadDMATask: public ATATask {
public:
    ReadDMATask(ATADisk& disk, uint64_t start, uint64_t end)
        : ATATask()
        , _disk (disk)
        , _start (start)
        , _end (end)
        , _dma_started (false)
        , _dma_state (NULL)
    {
    }

    virtual void cancel()
    {
        if (_dma_state) {
            _dma_state->nop();
        }
        _disk.remove_task(this);
    }

    virtual void start()
    {
        _disk.dma_wait();
    }

    void redv_done_direct(IOVec* vec, int err)
    {
        ATADisk* disk = &_disk;

        disk->remove_task(this);

        if (err) {
            disk->command_abort_error(*_dma_state);
        } else {
            disk->notify_command_done(*_dma_state);
        }

        disk->dec_async_count();
    }

    void redv_done_indirect(IOVec* vec, int err)
    {
        if (err) {
            redv_done_direct(vec, err);
            return;
        }

        IndirectVector::iterator iter = _indirect_vector->begin();
        uint8_t* src = _buf.get();

        for (; iter != _indirect_vector->end(); iter++) {
            memory_bus->write(src, (*iter).size, (*iter).address);
            src += (*iter).size;
        }

        redv_done_direct(vec, 0);
    }

    virtual bool dma_write_start(DMAState& dma)
    {
        Lock lock(_mutex);

        if (_dma_started) {
            return false;
        }

        _dma_started = true;

        uint num_blocks = _end - _start;
        uint transfer_size = num_blocks * SECTOR_SIZE;

        _direct_vector.reset(dma.get_direct_vector(transfer_size));

        if (_direct_vector.get()) {
            _dma_state = &dma;
            _disk.inc_async_count();
            new (&_iov) IOVec(_start, num_blocks, &(*_direct_vector)[0], _direct_vector->size(),
                              (io_vec_cb_t)&ReadDMATask::redv_done_direct, this);
            _disk._block_dev->readv(&_iov);
            return true;
        }

        _indirect_vector.reset(dma.get_indirect_vector(transfer_size));

        if (_indirect_vector.get()) {
            _disk.inc_async_count();
            _dma_state = &dma;
            _buf.set(new uint8_t[transfer_size]);
            _io_vec.iov_base = _buf.get();
            _io_vec.iov_len = transfer_size;
            new (&_iov) IOVec(_start, num_blocks, &_io_vec, 1,
                              (io_vec_cb_t)&ReadDMATask::redv_done_indirect, this);
            _disk._block_dev->readv(&_iov);
            return true;
        }

        D_MESSAGE("unable to obtaine transfer vector");

        _disk.remove_task(this);
        _disk.command_abort_error(dma); //todo: dma error

        return true;
    }

private:
    Mutex _mutex;
    ATADisk& _disk;
    uint64_t _start;
    uint64_t _end;
    std::unique_ptr<IndirectVector> _indirect_vector;
    struct iovec _io_vec;
    std::unique_ptr<DirectVector> _direct_vector;
    IOVec _iov;
    bool _dma_started;
    DMAState* _dma_state;
    AutoArray<uint8_t> _buf;
};


class WriteTask: public ATATask, public PIODataDest {
public:
    WriteTask(ATADisk& disk, uint64_t start, uint64_t end, uint bunch)
        : ATATask()
        , _disk (disk)
        , _now (start)
        , _end (end)
        , _bunch (bunch)
        , _data_now (NULL)
        , _data_end (NULL)
    {
        ASSERT(_bunch <= ATADISK_BUNCH_MAX);

        if (!(_io_block = _disk.get_io_block())) {
            THROW("age io block failed");
        }
    }

    void cancel()
    {
        _disk.remove_pio_dest(false);
        _disk.remove_task(this);
    }

    void start()
    {
        start_bunch();
    }

    void start_bunch()
    {
        _bunch = MIN(_end - _now, _bunch);

        ASSERT(_bunch);

        _data_now = (uint16_t*)_io_block;
        _data_end = (uint16_t*)(_io_block + _bunch * SECTOR_SIZE);

        _disk.set_pio_dest(this);
    }

    void end()
    {
        _disk.remove_task(this);
        _disk.remove_pio_dest(true);
    }

    void error()
    {
        _disk.remove_task(this);
        _disk.remove_pio_dest(false);
        _disk.command_abort_error();
    }


    void writev_done(IOVec* vec, int err)
    {
        ATADisk* disk = &_disk;

        if (err) {
            W_MESSAGE("failed %d", err)
            error();
        } else if ((_now += _bunch) == _end) {
            end();
        } else {
            start_bunch();
        }

        disk->dec_async_count();
    }

    void write_bunch()
    {
        _disk.inc_async_count();

        _io_vec.iov_base = _io_block;
        _io_vec.iov_len = _bunch * SECTOR_SIZE;
        new (&_iov) IOVec(_now, _bunch, &_io_vec, 1, (io_vec_cb_t)&WriteTask::writev_done, this);
        _disk._block_dev->writev(&_iov);
    }

    void put_word(uint16_t data)
    {
        if (_data_now == _data_end) {
            PANIC("no data");
        }

        *_data_now = data;

        if (++_data_now == _data_end) {
            _disk.remove_pio_dest(false);
            write_bunch();
        }
    }

private:
    ATADisk& _disk;
    uint64_t _now;
    uint64_t _end;
    uint _bunch;
    uint8_t* _io_block;
    uint16_t* _data_now;
    uint16_t* _data_end;
    struct iovec _io_vec;
    IOVec _iov;
};


class WriteDMATask: public ATATask {
public:
    WriteDMATask(ATADisk& disk, uint64_t start, uint64_t end)
        : ATATask()
        , _disk (disk)
        , _start (start)
        , _end (end)
        , _dma_started (false)
        , _dma_state (NULL)
    {
    }

    virtual void cancel()
    {
        if (_dma_state) {
            _dma_state->nop();
        }
        _disk.remove_task(this);
    }

    virtual void start()
    {
        _disk.dma_wait();
    }

    void writev_done(IOVec*, int err)
    {
        ATADisk* disk =&_disk;

        disk->remove_task(this);

        if (err) {
            disk->command_abort_error(*_dma_state);
        } else {
            disk->notify_command_done(*_dma_state);
        }

        disk->dec_async_count();
    }

    void copy()
    {
        IndirectVector::iterator iter = _indirect_vector->begin();
        uint8_t* dest = _buf.get();

        for (; iter != _indirect_vector->end(); iter++) {
            memory_bus->read((*iter).address, (*iter).size, dest);
            dest += (*iter).size;
        }
    }

    virtual bool dma_read_start(DMAState& dma)
    {
        Lock lock(_mutex);

        if (_dma_started) {
            return false;
        }

        _dma_started = true;

        uint num_blocks = _end - _start;
        uint transfer_size = num_blocks * SECTOR_SIZE;

        _direct_vector.reset(dma.get_direct_vector(transfer_size));

        if (_direct_vector.get()) {
            _dma_state = &dma;
            _disk.inc_async_count();
            new (&_iov) IOVec(_start, num_blocks, &(*_direct_vector)[0], _direct_vector->size(),
                              (io_vec_cb_t)&WriteDMATask::writev_done, this);
            _disk._block_dev->writev(&_iov);
            return true;
        }

        _indirect_vector.reset(dma.get_indirect_vector(transfer_size));

        if (_indirect_vector.get()) {
            _disk.inc_async_count();
            _dma_state = &dma;
            _buf.set(new uint8_t[transfer_size]);
            copy();
            _io_vec.iov_base = _buf.get();
            _io_vec.iov_len = transfer_size;
            new (&_iov) IOVec(_start, num_blocks, &_io_vec, 1,
                              (io_vec_cb_t)&WriteDMATask::writev_done,
                              this);
            _disk._block_dev->writev(&_iov);
            return true;
        }

        D_MESSAGE("unable to obtaine transfer vector");

        _disk.remove_task(this);
        _disk.command_abort_error(dma); //todo: dma error

        return true;
    }

private:
    Mutex _mutex;
    ATADisk& _disk;
    uint64_t _start;
    uint64_t _end;
    std::unique_ptr<IndirectVector> _indirect_vector;
    struct iovec _io_vec;
    std::unique_ptr<DirectVector> _direct_vector;
    IOVec _iov;
    bool _dma_started;
    DMAState* _dma_state;
    AutoArray<uint8_t> _buf;
};


class SyncTask: public ATATask {
public:
    SyncTask(ATADisk& disk)
        : ATATask()
        , _disk (disk)
        , _sync ((io_sync_cb_t)&SyncTask::sync_done, this)
    {

    }

    void cancel()
    {
        _disk.remove_task(this);
    }


    virtual void start()
    {
        _disk.inc_async_count();
        _disk._block_dev->sync(&_sync);
    }


    void sync_done(IOSync* obj, uint err)
    {
        if (err) {
            W_MESSAGE("failed %d", err);
        }

        ATADisk* disk =&_disk;
        disk->remove_task(this);
        disk->notify_command_done();
        disk->dec_async_count();
    }

private:
    ATADisk& _disk;
    IOSync _sync;
};


class IdentifyTask: public ATATask, public PIODataSource {
public:
    IdentifyTask(ATADisk& disk)
        : ATATask()
        , _disk (disk)
        , _data_now (_identity)
        , _data_end (_data_now + 256)
    {
    }

    void start()
    {
        init_identity();
        _disk.set_pio_source(this);
    }

    void cancel()
    {
        _disk.remove_pio_source(false);
        _disk.remove_task(this);
    }

    void init_identity()
    {
        memset(_identity, 0, sizeof(_identity));

        _identity[ATA_ID_OFFSET_GENERAL_CONF] = ATA_COMPAT_ID_GENERAL_CONF_NOT_REMOVABLE_MASK;
        _identity[ATA_ID_OFFSET_SPECIFIC_CONF] = 0xc837; // Device does not require SET FEATURES
                                                         // subcommand to spin-up after power-up
                                                         // and IDENTIFY DEVICE response is complete

        set_ata_str(&_identity[ATA_ID_OFFSET_SERIAL], ATA_ID_SERIAL_NUM_CHARS / 2,
                    "NxHD0000000000000001");
        set_ata_str(&_identity[ATA_ID_OFFSET_REVISION], ATA_ID_REVISION_NUM_CHARS / 2, "1.0.0");
        set_ata_str(&_identity[ATA_ID_OFFSET_MODEL], ATA_ID_MODEL_NUM_CHARS / 2, "Nox HD");

        uint32_t max_sectors = ATA3_MAX_HEAD * ATA3_MAX_SEC * ATA3_MAX_CYL;
        uint64_t sectors = MIN(_disk.get_size() / SECTOR_SIZE, max_sectors);
        uint64_t cyl = sectors / (ATA3_MAX_HEAD * ATA3_MAX_SEC);
        _identity[ATA_COMPAT_ID_OFFSET_CYL] = cyl;
        _identity[ATA_COMPAT_ID_OFFSET_HEAD] = ATA3_MAX_HEAD;
        _identity[ATA_COMPAT_ID_OFFSET_SECTORS] = ATA3_MAX_SEC;

        cyl = MIN(sectors / (_disk._heads_per_cylinder * _disk._sectors_per_track), (1 << 16) - 1);
        _identity[ATA_COMPAT_ID_OFFSET_CURRENT_CYL] = cyl;
        _identity[ATA_COMPAT_ID_OFFSET_CURRENT_HEAD] = _disk._heads_per_cylinder;
        _identity[ATA_COMPAT_ID_OFFSET_CURRENT_SECTORS] = _disk._sectors_per_track;
        uint32_t addresabel_sec =  cyl * _disk._heads_per_cylinder * _disk._sectors_per_track;
        _identity[ATA_COMPAT_ID_OFFSET_CURRENT_ADDRESABEL_SECTORS] = addresabel_sec;
        _identity[ATA_COMPAT_ID_OFFSET_CURRENT_ADDRESABEL_SECTORS + 1] = addresabel_sec >> 16;

        _identity[ATA_ID_OFFSET_MAX_SECTORS_PER_BLOCK] = 0x8000 | ATADISK_BUNCH_MAX;
        _identity[ATA_ID_OFFSET_CAP1] = ATA_ID_CAP1_DMA_MASK |
                                        ATA_ID_CAP1_LBA_MASK |
                                        ATA_ID_CAP1_IORDY_MASK/* |
                                        ATA_ID_CAP1_DISABLE_IORDY_MASK*/;

        _identity[ATA_ID_OFFSET_MULTIPLE_SETTING] = _disk._multi_mode;

        _identity[ATA_ID_OFFSET_CAP2] = ATA_ID_CAP2_MUST_SET;
        _identity[ATA_ID_OFFSET_FIELD_VALIDITY] = ATA_ID_FIELD_VALIDITY_64_70 |
                                                  ATA_ID_FIELD_VALIDITY_88 |
                                                  ATA_COMPAT_ID_FIELD_VALIDITY_54_58;

        sectors = MIN(_disk.get_size() / SECTOR_SIZE, (1 << 28) - 1);

        _identity[ATA_ID_OFFSET_ADDRESABEL_SECTORS] = sectors;
        _identity[ATA_ID_OFFSET_ADDRESABEL_SECTORS + 1] = sectors >> 16;

        uint mwdma_select = (_disk.get_multiword_mode() == ~0) ? 0 :
                                                            (1 << (_disk.get_multiword_mode() + 8));
        _identity[ATA_ID_OFFSET_MULTI_DMA] = ATA_ID_MULTI_DMA_MODE0_MASK |
                                             ATA_ID_MULTI_DMA_MODE1_MASK |
                                             ATA_ID_MULTI_DMA_MODE2_MASK |
                                             mwdma_select;

        _identity[ATA_ID_OFFSET_PIO] = ATA_IO_PIO_MODE3_MASK | ATA_IO_PIO_MODE4_MASK;

        _identity[ATA_ID_OFFSET_VERSION] = ATA_ID_VERSION_ATA3_MASK |
                                           ATA_ID_VERSION_ATA4_MASK |
                                           ATA_ID_VERSION_ATA5_MASK |
                                           ATA_ID_VERSION_ATA6_MASK;

        _identity[ATA_ID_OFFSET_CMD_SET_1] = ATA_ID_CMD_SET_1_NOP_MASK |
                                             ATA_ID_CMD_SET_1_WRITE_CACHE |
                                             ATA_ID_CMD_SET_1_POWR_MANAG;
        _identity[ATA_ID_OFFSET_CMD_SET_1_ENABLE] = ATA_ID_CMD_SET_1_NOP_MASK |
                                                    ATA_ID_CMD_SET_1_POWR_MANAG;

        if (!_disk._sync_mode) {
            _identity[ATA_ID_OFFSET_CMD_SET_1_ENABLE] |= ATA_ID_CMD_SET_1_WRITE_CACHE;
        }

        _identity[ATA_ID_OFFSET_CMD_SET_2] = ATA_ID_CMD_SET_2_ONE_MASK |
                                             ATA_ID_CMD_SET_2_FLUSH_EXT_MASK |
                                             ATA_ID_CMD_SET_2_FLUSH_MASK |
                                             ATA_ID_CMD_SET_2_48BIT_MASK /*|
                                             ATA_ID_CMD_SET_2_POWERUP_IN_STANDBY_MASK |
                                             ATA_ID_CMD_SET_2_ADVANCE_POWR_MANAG_MASK |
                                             ATA_ID_CMD_SET_2_QUAD_MASK*/;
        _identity[ATA_ID_OFFSET_CMD_SET_2_ENABLE] = ATA_ID_CMD_SET_2_48BIT_MASK |
                                                    ATA_ID_CMD_SET_2_FLUSH_EXT_MASK |
                                                    ATA_ID_CMD_SET_2_FLUSH_MASK;

        _identity[ATA_ID_OFFSET_CMD_SET_3] = ATA_ID_CMD_SET_3_ONE_MASK;
        _identity[ATA_ID_OFFSET_CMD_SET_3_ENABLE] = _identity[ATA_ID_OFFSET_CMD_SET_3];

        uint udma_select = (_disk.get_ultra_mode() == ~0) ? 0 : (1 << (_disk.get_ultra_mode() + 8));
        _identity[ATA_ID_OFFSET_UDMA] = ATA_ID_UDMA_MODE0_MASK |
                                        ATA_ID_UDMA_MODE1_MASK |
                                        ATA_ID_UDMA_MODE2_MASK |
                                        ATA_ID_UDMA_MODE3_MASK |
                                        ATA_ID_UDMA_MODE4_MASK |
                                        ATA_ID_UDMA_MODE5_MASK |
                                        udma_select;

        //_identity[ATA_ID_OFFSET_ADVANCE_POWR_MANAG] = ATA_ID_ADVANCE_POWR_MANAG_MAX_PERFORMANCE;

        _identity[ATA_ID_OFFSET_HRESET] = ATA_ID_HRESET_ONE_MASK |
                                          //ATA_ID_HRESET_PDIAG_MASK |
                                          ATA_ID_HRESET_PASS_MASK |
                                          ATA_ID_HRESET_JUMPER_MASK;

        sectors = MIN(_disk.get_size() / SECTOR_SIZE, (1ULL << 48) - 1);
        _identity[ATA_ID_OFFSET_ADDR_SECTORS_48] = sectors;
        _identity[ATA_ID_OFFSET_ADDR_SECTORS_48 + 1] = sectors >> 16;
        _identity[ATA_ID_OFFSET_ADDR_SECTORS_48 + 2] = sectors >> 32;

        _identity[ATA_ID_OFFSET_INTEGRITY] = ATA_ID_INTEGRITY_SIGNATURE;
        _identity[ATA_ID_OFFSET_INTEGRITY] |= checksum8(_identity, sizeof(_identity)) << 8;
    }

    virtual uint16_t get_word()
    {
        if (_data_now == _data_end) {
            PANIC("no data");
        }

        uint16_t ret = *_data_now;

        if (++_data_now == _data_end) {
            _disk.remove_task(this);
            _disk.remove_pio_source(true);
        }

        return ret;
    }

private:
    ATADisk& _disk;
    uint16_t _identity[256];
    uint16_t* _data_now;
    uint16_t* _data_end;
};


ATADisk::ATADisk(VMPart& owner, Wire& wire, const std::string& file_name, bool read_only)
    : ATADevice("ata-disk", owner, wire)
{
    ASSERT(ATADEV_IO_BLOCK_SIZE >= ATADISK_BUNCH_MAX * SECTOR_SIZE);

    ASSERT(sizeof(ReadTask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(ReadDMATask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(WriteTask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(WriteDMATask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(SyncTask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(IdentifyTask) <= ATADEV_TASK_STORAGE_SIZE);

    if (read_only) {
        _block_dev.reset(new ROBlockDevice(file_name, SECTOR_SIZE));
    } else {
        _block_dev.reset(new PBlockDevice(file_name, SECTOR_SIZE, false));
    }

    uint64_t size = _block_dev->get_size();

    if (size < MB) {
        THROW("invalid file size: %lu less then %u", size, MB);
    }

    uint64_t sectors = size / SECTOR_SIZE;

    if (sectors < ATA3_MAX_CYL * ATA3_MAX_HEAD * ATA3_MAX_SEC &&
                                                   (sectors % (ATA3_MAX_HEAD * ATA3_MAX_SEC))) {
        I_MESSAGE("partial cylinder: %lu is not align on %u bytes",
                  size, ATA3_MAX_HEAD * SECTOR_SIZE * SECTOR_SIZE);
    }
}


ATADisk::~ATADisk()
{
}


uint64_t ATADisk::get_size()
{
    return _block_dev->get_size();
}


void ATADisk::reset(bool cold)
{
    ATADevice::reset(cold);

    if (revert_to_default()) {
        _sync_mode = true;
        _block_dev->set_sync_mode(true);
        _heads_per_cylinder = ATA3_MAX_HEAD;
        _sectors_per_track = ATA3_MAX_SEC;
        _multi_mode = ATA_ID_MULTIPLE_SETTING_ACTIVE_MASK | ATADISK_BUNCH_DEFAULT;
    }
}


void ATADisk::set_signature()
{
    _count = 1;
    _lba_low = 1;
    _lba_mid = 0;
    _lba_high = 0;
    _device_reg = 0;
}


uint8_t ATADisk::io_read(uint16_t port)
{
    if (current() == 1 && port == ATA_IO_STATUS) {
        return 0;
    }

    return ATADevice::io_read(port);
}


uint64_t ATADisk::get_sector_address()
{
    bool lba = _device_reg & ATA_DEVICE_LBA_MASK;
    uint64_t sector;

    if (lba) {
        sector = _lba_low & 0xff;
        sector |= (_lba_mid & 0xff) << 8;
        sector |= (_lba_high & 0xff) << 16;
        sector |= (_device_reg & ATA_DEVICE_ADDRESS_MASK) << 24;
    } else {
        uint cylinder = (_lba_mid & 0xff)  + ((_lba_high & 0xff) << 8);
        uint head = _device_reg & ATA_DEVICE_ADDRESS_MASK;
        sector = (_lba_low & 0xff);
        sector = (cylinder * _heads_per_cylinder + head) * _sectors_per_track + sector - 1;
    }

    return sector;
}


uint64_t ATADisk::get_sector_address_ext()
{
    uint64_t sector;

    sector = (_lba_low & 0xff) | ((_lba_low & 0xff00) << 16);
    sector |= ((_lba_mid & 0xff) << 8) | (uint64_t(_lba_mid & 0xff00) << 24);
    sector |= ((_lba_high & 0xff) << 16) | (uint64_t(_lba_high & 0xff00) << 32);

    return sector;
}


uint ATADisk::get_sector_count()
{
    return (_count & 0xff) ? (_count & 0xff) : 256;
}


uint ATADisk::get_sector_count_ext()
{
    return (_count & 0xffff) ? (_count & 0xffff) : 65536;
}


bool ATADisk::is_valid_sectors_range(uint64_t start, uint64_t end)
{
    return start < end && end <= _block_dev->get_size() / SECTOR_SIZE;
}


void ATADisk::do_read_sectors_common(uint64_t start, uint64_t end, uint bunch)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    set_power_mode(POWER_ACTIVE);
    start_task(new (_task_storage) ReadTask(*this, start, end, bunch));
}


void ATADisk::do_read_sectors()
{
    uint64_t start = get_sector_address();
    do_read_sectors_common(start, start + get_sector_count(), 1);
}


void ATADisk::do_read_sectors_ext()
{
    uint64_t start = get_sector_address_ext();
    do_read_sectors_common(start, start + get_sector_count_ext(), 1);
}


void ATADisk::do_read_multi()
{
    if (!(_multi_mode & ATA_ID_MULTIPLE_SETTING_ACTIVE_MASK)) {
        command_abort_error();
        return;
    }

    uint64_t start = get_sector_address();
    do_read_sectors_common(start, start + get_sector_count(),
                           _multi_mode & ATA_ID_MULTIPLE_VAL_MASK);
}


void ATADisk::do_read_multi_ext()
{
    if (!(_multi_mode & ATA_ID_MULTIPLE_SETTING_ACTIVE_MASK)) {
        command_abort_error();
        return;
    }

    uint64_t start = get_sector_address_ext();
    do_read_sectors_common(start, start + get_sector_count_ext(),
                           _multi_mode & ATA_ID_MULTIPLE_VAL_MASK);
}


void ATADisk::do_read_verify_sectors()
{
    uint64_t start = get_sector_address();
    uint64_t end = start + get_sector_count();

    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    raise();
}

void ATADisk::do_read_verify_sectors_ext()
{
    uint64_t start = get_sector_address_ext();
    uint64_t end = start + get_sector_count_ext();

    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    raise();
}


void ATADisk::do_read_dma_common(uint64_t start, uint64_t end)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    set_power_mode(POWER_ACTIVE);
    start_task(new (_task_storage) ReadDMATask(*this, start, end));
}


void ATADisk::do_read_dma()
{
    uint64_t start = get_sector_address();
    do_read_dma_common(start, start + get_sector_count());
}


void ATADisk::do_read_dma_ext()
{
    uint64_t start = get_sector_address_ext();
    do_read_dma_common(start, start + get_sector_count_ext());
}


void ATADisk::do_write_sectors_common(uint64_t start, uint64_t end, uint bunch)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    set_power_mode(POWER_ACTIVE);
    start_task(new (_task_storage) WriteTask(*this, start, end, bunch));
}


void ATADisk::do_write_sectors()
{
    uint64_t start = get_sector_address();
    do_write_sectors_common(start, start + get_sector_count(), 1);
}


void ATADisk::do_write_sectors_ext()
{
    uint64_t start = get_sector_address_ext();
    do_write_sectors_common(start, start + get_sector_count_ext(), 1);
}


void ATADisk::do_write_multi()
{
    if (!(_multi_mode & ATA_ID_MULTIPLE_SETTING_ACTIVE_MASK)) {
        command_abort_error();
        return;
    }

    uint64_t start = get_sector_address();
    do_write_sectors_common(start, start + get_sector_count(),
                            _multi_mode & ATA_ID_MULTIPLE_VAL_MASK);
}


void ATADisk::do_write_multi_ext()
{
    if (!(_multi_mode & ATA_ID_MULTIPLE_SETTING_ACTIVE_MASK)) {
        command_abort_error();
        return;
    }

    uint64_t start = get_sector_address_ext();
    do_write_sectors_common(start, start + get_sector_count_ext(),
                            _multi_mode & ATA_ID_MULTIPLE_VAL_MASK);
}


void ATADisk::do_write_dma_common(uint64_t start, uint64_t end)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    set_power_mode(POWER_ACTIVE);
    start_task(new (_task_storage) WriteDMATask(*this, start, end));
}


void ATADisk::do_write_dma()
{
    uint64_t start = get_sector_address();
    do_write_dma_common(start, start + get_sector_count());
}


void ATADisk::do_write_dma_ext()
{
    uint64_t start = get_sector_address_ext();
    do_write_dma_common(start, start + get_sector_count_ext());
}


void ATADisk::do_identify_device()
{
    start_task(new (_task_storage) IdentifyTask(*this));
}


void ATADisk::do_set_multi_mode()
{
    uint val = _count & 0xff;
    int msb = find_msb(0);
    msb = find_msb(val);

    if (msb == -1) {
        _multi_mode = 0;
        raise();
        return;
    }

    if ((1 << msb) != val || val > ATADISK_BUNCH_MAX) {
        command_abort_error();
        return;
    }

    _multi_mode = val | ATA_ID_MULTIPLE_SETTING_ACTIVE_MASK;

    raise();
}


void ATADisk::do_set_features()
{
    switch (_feature) {
    case ATA_FEATURE_ENABLE_CACHE:
        D_MESSAGE("enable cache");
        _sync_mode = false;
        _block_dev->set_sync_mode(false);
        raise();
        break;
    case ATA_FEATURE_DISABLE_CACHE: {
        D_MESSAGE("disable cache");
        _sync_mode = true;
        _block_dev->set_sync_mode(true);
        start_task(new (_task_storage) SyncTask(*this));
        break;
    }
    case ATA_FEATURE_ENABLE_LOOK_AHEAD:
    case ATA_FEATURE_DISABLE_LOOK_AHEAD:
        D_MESSAGE("abbort on 0x%x", _feature);
        command_abort_error();
        break;
    default:
        ATADevice::do_set_features();
    }
}


void ATADisk::do_initialize_device_parameters()
{
    uint new_sectors_per_track = _count & 0xff;
    uint new_heads_per_cylinder = (_device_reg & ATA_DEVICE_ADDRESS_MASK) + 1;

    if (_sectors_per_track != new_sectors_per_track ||
        _heads_per_cylinder != new_heads_per_cylinder) {

        if (!new_sectors_per_track || !new_heads_per_cylinder) {
            W_MESSAGE("invalid INITIALIZE_DEVICE_PARAMETERS %u %u",
                      new_sectors_per_track, new_heads_per_cylinder);
            command_abort_error();
            return;
        }

        D_MESSAGE("sectors per track %u new %u, heads %u new %u",
                  _sectors_per_track,
                  new_sectors_per_track,
                  _heads_per_cylinder,
                  new_heads_per_cylinder);

        _sectors_per_track = new_sectors_per_track;
        _heads_per_cylinder = new_heads_per_cylinder;
    }

    _status |= ATA_STATUS_SEEK_COMPLEAT; // solves hung during winxp boot
    raise();
}


void ATADisk::do_flush()
{
    start_task(new (_task_storage) SyncTask(*this));
}


void ATADisk::do_standby_immediate()
{
    set_power_mode(POWER_STANDBY);
    raise();
}


void ATADisk::do_idle_immediate()
{
    set_power_mode(POWER_IDLE);
    raise();
}


void ATADisk::do_sleep()
{
    set_power_mode(POWER_SLEEP);
    raise();
}


void ATADisk::do_check_power_mode()
{
    _count = get_power_mode();
    raise();
}


void ATADisk::do_command(uint8_t command)
{
    // in device 0 only configurations: a write to the command register shall be ignored,
    // except for EXECUTE DEVICE DIAGNOSTIC;
    if (current() == 1 &&  command != ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC) {
        ATA_LOG("ignoring command 0x%x", command);
        return;
    }


    // For all commands except DEVICE RESET, this register shall only be written when BSY
    // and DRQ are both cleared and DMACK- is not asserted
    if (((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/))  {
        // by defenition result is indeterminate
        ATA_LOG("drop command 0x%x status 0x%x", command, _status);
        return;
    }


    if (!(_status & ATA_STATUS_READY_MASK) && command != ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC &&
                                          command != ATA_CMD_IDENTIFY_PACKET_DEVICE &&
                                          command != ATA_CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS) {
        command_abort_error();
        return;
    }

    //  For a device in the Sleep mode, writing of the Command register shall be
    // ignored except for writing of the DEVICE RESET command to a device that implements the
    // PACKET Command
    if (get_power_mode() == POWER_SLEEP) {
        ATA_LOG("drop command 0x%x while in sleep mode", command);
        return;
    }

    _status &= ATA_STATUS_READY_MASK;
    _error = 0;

    ATA_LOG("command: %s (0x%x)", command_name(command), command);

    drop();

    switch (command) {
    case ATA_CMD_READ_DMA:
        do_read_dma();
        break;
    case ATA_CMD_READ_DMA_EXT:
        do_read_dma_ext();
        break;
    case ATA_CMD_WRITE_DMA:
        do_write_dma();
        break;
    case ATA_CMD_WRITE_DMA_EXT:
        do_write_dma_ext();
        break;
    case ATA_CMD_READ_SECTORS:
        do_read_sectors();
        break;
    case ATA_CMD_READ_SECTORS_EXT:
        do_read_sectors_ext();
        break;
    case ATA_CMD_READ_VERIFY_SECTORS:
        do_read_verify_sectors();
        break;
    case ATA_CMD_READ_VERIFY_SECTORS_EXT:
        do_read_verify_sectors_ext();
        break;
#if 0
    case ATA_CMD_SEEK:
        do_seek();
        break;
#endif
    case ATA_CMD_SET_FEATURES:
        do_set_features();
        break;
    case ATA_CMD_WRITE_SECTORS:
        do_write_sectors();
        break;

    case ATA_CMD_WRITE_SECTORS_EXT:
        do_write_sectors_ext();
        break;
    case ATA_CMD_READ_MULTIPLE:
        do_read_multi();
        break;
    case ATA_CMD_READ_MULTIPLE_EXT:
        do_read_multi_ext();
        break;
    case ATA_CMD_WRITE_MULTIPLE:
        do_write_multi();
        break;
    case ATA_CMD_WRITE_MULTIPLE_EXT:
        do_write_multi_ext();
        break;
    case ATA_CMD_FLUSH_CACHE_EXT:
    case ATA_CMD_FLUSH_CACHE:
        do_flush();
        break;
    case ATA_CMD_IDENTIFY_DEVICE:
        do_identify_device();
        break;
    case ATA_CMD_IDENTIFY_PACKET_DEVICE:
        command_abort_error();
        break;
    case ATA_CMD_CHECK_POWER_MODE:
        do_check_power_mode();
        break;
    case ATA_CMD_IDLE_IMMEDIATE:
        do_idle_immediate();
        break;
#if 0
    case ATA_CMD_IDLE:
        raise();
        break;
    case ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC:
        // If the host issues an EXECUTE DEVICE DIAGNOSTIC command while a device is in or going to
        // a power management mode except Sleep, then the device shall execute the EXECUTE DEVICE
        // DIAGNOSTIC sequence.
        set_signature();
        _error = DIAGNOSTIC_D0_OK_D1_NOT_PRESENT;
        _status = 0/*STATUS_READY_MASK*/;
        raise();
        break;
    case ATA_CMD_NOP:
        do_nop();
        break;
#endif
    case ATA_CMD_DEVICE_RESET:
        command_abort_error();
        break;
    case ATA_CMD_SET_MULTIPLE_MODE:
        do_set_multi_mode();
        break;
    case ATA_CMD_SLEEP:
        do_sleep();
        break;
#if 0
    case ATA_CMD_STANDBY:
        do_standby();
        break;
#endif
    case ATA_CMD_STANDBY_IMMEDIATE:
        do_standby_immediate();
        break;
    case ATA_CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS:
        do_initialize_device_parameters();
        break;
    case ATA_CMD_READ_NATIVE_MAX_ADDRESS_EXT:
    case ATA_CMD_READ_NATIVE_MAX_ADDRESS:
    case ATA_CMD_PACKET:
    case ATA_CMD_GET_MEDIA_STATUS:
    case ATA_CMD_SECURITY_FREEZE_LOCK:
    case ATA_CMD_SMART:
        command_abort_error();
        break;
    default:
        D_MESSAGE("unhandled 0x%x %u", command, command);
        command_abort_error();
    }
}


ATADiskFactory::ATADiskFactory(const std::string& file_mame, bool read_only)
    : _file_name (file_mame)
    , _device (NULL)
    , _read_only (read_only)
{
}


ATADevice* ATADiskFactory::creat_device(VMPart &owner, Wire &wire)
{
    _device = new ATADisk(owner, wire, _file_name, _read_only);
    return _device;
}

