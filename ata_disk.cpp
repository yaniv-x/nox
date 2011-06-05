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
#include <fcntl.h>

#include "ata_disk.h"
#include "ata.h"
#include "application.h"
#include "block_device.h"

#define ATA_LOG(format, ...)

enum {
    SECTOR_SIZE = 512,
    SECTOR_MASK = SECTOR_SIZE - 1,

    NUM_IO_REQUESTS = 1000,

    ATA3_MAX_CYL = 16383,
    ATA3_MAX_HEAD = 16,
    ATA3_MAX_SEC = 63,

    ATADISK_CACHE_SIZE = (2 * MB) / SECTOR_SIZE,
    ATADISK_BUNCH_MAX = 128,
    ATADISK_BUNCH_DEFAULT = 1,
    ATADISK_PRE_FETCH_MAX = ATADISK_BUNCH_MAX * 2,
};


class CachedBlock: public Block {
public:
    enum State {
        INVALID,
        INITILIZING,
        MODIFYING,
        LOADING,
        VALID,
        DIRTY,
        STORING,
    };

    CachedBlock(uint64_t address, uint8_t* in_data)
        : Block(address, in_data)
        , _refs (0)
        , _state (INVALID)
        , _obsolete (false)
    {
    }

    void reset(uint64_t block_index)
    {
        ASSERT(_refs.val() == 0)
        _state = INVALID;
        address = block_index;
        _obsolete = false;
    }

    State get_state() { return _state;}
    void set_state(State state) { _state = state;}
    bool is_valid() { return _state >= VALID;}
    bool is_obsolete() { return _obsolete;}
    void set_obsolete() { _obsolete = true;}
    bool is_in_use() { return !!_refs.val();}
    CachedBlock* ref() { _refs.inc(); return this;};
    void unref() { ASSERT(_refs.val() > 0); _refs.dec();}

private:
    Atomic _refs;
    State _state;
    bool _obsolete;
};


class ATADiskTask : public ATATask {
public:
    virtual void block_changed() {}
    virtual void sync_done(void* mark) {}
};


class ReadTask: public ATADiskTask, public PIODataSource {
public:
    ReadTask(ATADisk& disk, uint64_t start, uint64_t end, uint bunch)
        : ATADiskTask()
        , _disk (disk)
        , _start (start)
        , _now (start)
        , _end (end)
        , _bunch (bunch)
        , _data_now (NULL)
        , _data_end (NULL)
        , _wait_resorces (false)
    {
        ASSERT(_bunch <= ATADISK_BUNCH_MAX);
    }

    void cancel()
    {
        Lock lock(_mutex);

        for (uint i = 0; i < _bunch; i++) {
            if (_blocks[i]) {
                _blocks[i]->unref();
                _blocks[i] = NULL;
            }
        }

        _wait_resorces = false;
        _disk.remove_task(this);
    }

    void start()
    {
        get_bunch();
    }

    void pre_fetch()
    {
    }

    void start_block()
    {
        _data_now = (uint16_t*)_blocks[_current_block]->data;
        _data_end = (uint16_t*)(_blocks[_current_block]->data + SECTOR_SIZE);
    }

    void start_bunch()
    {
        _current_block = 0;
        start_block();
        _disk.set_pio_source(this);
    }

    void end()
    {
        _disk.remove_task(this);
        _disk.remove_pio_source(true);
    }

    void get_bunch()
    {
        _bunch = MIN(_end - _now, _bunch);

        if (_bunch == 0) {
            end();
            return;
        }

        bool ready = true;

        Lock lock(_mutex);

        for (uint i = 0; i < _bunch; i++) {
            _blocks[i] = _disk.get_for_read(_now + i);

            if (!_blocks[i]) {
                _wait_resorces = true;
                return;
            }

            if (!_blocks[i]->is_valid()) {
                ready = false;
            }
        }

        if (ready) {
            start_bunch();
        } else {
            _wait_resorces = true;
        }

        lock.unlock();
        pre_fetch();
    }


    uint16_t get_word()
    {
        if (_data_now == _data_end) {
            PANIC("no data");
        }

        uint16_t ret = *_data_now;

        if (++_data_now == _data_end) {
            _blocks[_current_block]->unref();
            _blocks[_current_block] = NULL;
            if (++_current_block == _bunch) {
                _now += _bunch;
                _disk.remove_pio_source(false);
                get_bunch();
            } else {
                start_block();
            }
        }

        return ret;
    }

    virtual void block_changed()
    {
        Lock lock(_mutex);

        if (!_wait_resorces) {
            return;
        }

        for (uint i = 0; i < _bunch; i++) {
            if (!_blocks[i]) {
                 if (!(_blocks[i] = _disk.get_for_read(_now + i))) {
                     return;
                 }
            }

            if (!_blocks[i]->is_valid()) {
                return;
            }
        }

        _wait_resorces = false;
        start_bunch();
        pre_fetch();
    }

private:
    Mutex _mutex;
    ATADisk& _disk;
    uint64_t _start;
    uint64_t _now;
    uint64_t _end;
    uint _bunch;
    CachedBlock* _blocks[ATADISK_BUNCH_MAX];
    uint16_t* _data_now;
    uint16_t* _data_end;
    uint _current_block;
    bool _wait_resorces;
};


class WriteTask: public ATADiskTask, public PIODataDest {
public:
    WriteTask(ATADisk& disk, uint64_t start, uint64_t end, uint bunch)
        : ATADiskTask()
        , _disk (disk)
        , _now (start)
        , _end (end)
        , _bunch (bunch)
        , _data_now (NULL)
        , _data_end (NULL)
        , _wait_resorces (false)
        , _wait_sync (false)
    {
        ASSERT(_bunch <= ATADISK_BUNCH_MAX);
    }

    void start()
    {
        claim_blocks();
    }

    void cancel()
    {
        Lock lock(_mutex);

        for (uint i = 0; i < _bunch; i++) {
            if (_blocks[i]) {
                _blocks[i]->unref();
                _blocks[i] = NULL;
            }
        }

        _wait_resorces = false;
        _wait_sync = false;
        _disk.remove_task(this);
    }

    void start_block()
    {
        _data_now = (uint16_t*)_blocks[_current_block]->data;
        _data_end = (uint16_t*)(_blocks[_current_block]->data + SECTOR_SIZE);
    }

    void start_bunch()
    {
        _current_block = 0;
        start_block();
        _disk.set_pio_dest(this);
    }

    void end()
    {
        _disk.remove_task(this);
        _disk.remove_pio_dest(true);
    }

    void claim_blocks()
    {
        _bunch = MIN(_end - _now, _bunch);

        if (_bunch == 0) {
            end();
            return;
        }

        Lock lock(_mutex);

        for (uint i = 0; i < _bunch; i++) {
            _blocks[i] = _disk.get_for_write(_now + i);

            if (!_blocks[i]) {
                _wait_resorces = true;
                D_MESSAGE("wait");
                _disk.remove_pio_dest(false);
                return;
            }
        }

        start_bunch();
    }

    void sync_bunch()
    {
        for (uint i = 0; i < _bunch; i++) {
            if (!_blocks[i]) {
                continue;
            }

            if (_blocks[i]->get_state() != CachedBlock::VALID) {
                _wait_sync = true;
                return;
            }

            _blocks[i]->unref();
            _blocks[i] = NULL;
        }

        _wait_sync = false;
        _now += _bunch;
        claim_blocks();
    }

    void put_word(uint16_t data)
    {
        if (_data_now == _data_end) {
            PANIC("no data");
        }

        *_data_now = data;

        if (++_data_now == _data_end) {
            _disk.store(_blocks[_current_block]);

            if (_disk._sync_mode) {
                if (++_current_block == _bunch) {
                    _disk.remove_pio_dest(false);
                    Lock lock(_mutex);
                    sync_bunch();
                } else {
                    start_block();
                }
            } else {
                _blocks[_current_block]->unref();
                _blocks[_current_block] = NULL;
                if (++_current_block == _bunch) {
                    _now += _bunch;
                    claim_blocks();
                } else {
                    start_block();
                }
            }
        }
    }

    virtual void block_changed()
    {
        Lock lock(_mutex);

        if (_wait_sync) {
            sync_bunch();
            return;
        }

        if (!_wait_resorces) {
            return;
        }

        for (uint i = 0; i < _bunch; i++) {
            if (!_blocks[i] && !(_blocks[i] = _disk.get_for_write(_now + i))) {
                return;
            }
        }
        _wait_resorces = false;
        start_bunch();
    }

private:
    RecursiveMutex _mutex;
    ATADisk& _disk;
    uint64_t _now;
    uint64_t _end;
    uint _bunch;
    CachedBlock* _blocks[ATADISK_BUNCH_MAX];
    uint16_t* _data_now;
    uint16_t* _data_end;
    uint _current_block;
    bool _wait_resorces;
    bool _wait_sync;
};


class SyncTask: public ATADiskTask {
public:
    SyncTask(ATADisk& disk)
        : ATADiskTask()
        , _disk (disk)
    {

    }

    void cancel()
    {
        _disk.remove_task(this);
    }

    void done()
    {
        _disk.remove_task(this);
        _disk.notify_command_done();
    }

    virtual void start()
    {
        _disk.sync(this);
    }

    virtual void sync_done(void* mark)
    {
        if (mark == this) {
            done();
        }
    }

private:
    ATADisk& _disk;
};


class IdentifyTask: public ATADiskTask, public PIODataSource {
public:
    IdentifyTask(ATADisk& disk)
        : ATADiskTask()
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
        _disk.remove_task(this);
    }

    void init_identity()
    {
        memset(_identity, 0, sizeof(_identity));

        _identity[ATA_ID_OFFSET_GENERAL_CONF] = ATA_COMPAT_ID_GENERAL_CONF_NOT_REMOVABLE_MASK;
        _identity[ATA_ID_OFFSET_SPECIFIC_CONF] = 0xc837; // Device does not require SET FEATURES
                                                         // subcommand to spin-up after power-up
                                                         // and IDENTIFY DEVICE response is complete

        set_ata_str(&_identity[ATA_ID_OFFSET_SERIAL], ATA_ID_SERIAL_NUM_CHARS / 2, "0");
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
                                        ATA_ID_CAP1_IORDY_MASK |
                                        ATA_ID_CAP1_DISABLE_IORDY_MASK;

        _identity[ATA_ID_OFFSET_MULTIPLE_SETTING] = _disk._multi_mode;

        _identity[ATA_ID_OFFSET_CAP2] = ATA_ID_CAP2_MUST_SET;
        _identity[ATA_ID_OFFSET_FIELD_VALIDITY] = ATA_ID_FIELD_VALIDITY_64_70 |
                                                  ATA_ID_FIELD_VALIDITY_88;

        sectors = MIN(_disk.get_size() / SECTOR_SIZE, (1 << 28) - 1);

        _identity[ATA_ID_OFFSET_ADDRESABEL_SECTORS] = sectors;
        _identity[ATA_ID_OFFSET_ADDRESABEL_SECTORS + 1] = sectors >> 16;

        _identity[ATA_ID_OFFSET_NULTI_DMA] = ATA_ID_NULTI_DMA_MODE0_MASK |
                                            ATA_ID_NULTI_DMA_MODE1_MASK |
                                            ATA_ID_NULTI_DMA_MODE2_MASK |
                                            ATA_ID_NULTI_DMA_MODE0_SELECT_MASK;

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
        _identity[ATA_ID_OFFSET_CMD_SET_2_ENABLE] = ATA_ID_CMD_SET_2_48BIT_MASK;

        _identity[ATA_ID_OFFSET_CMD_SET_3] = ATA_ID_CMD_SET_3_ONE_MASK;
        _identity[ATA_ID_OFFSET_CMD_SET_3_ENABLE] = _identity[ATA_ID_OFFSET_CMD_SET_3];

        _identity[ATA_ID_OFFSET_UDMA] = ATA_ID_UDMA_MODE0_MASK |
                                        ATA_ID_UDMA_MODE1_MASK |
                                        ATA_ID_UDMA_MODE2_MASK |
                                        ATA_ID_UDMA_MODE3_MASK |
                                        ATA_ID_UDMA_MODE4_MASK |
                                        ATA_ID_UDMA_MODE5_MASK |
                                        ATA_ID_UDMA_MODE0_SELECT_MASK;

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

        uint8_t test = checksum8(_identity, sizeof(_identity));
        D_MESSAGE("checksum test %u", (uint)test);
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
    if (read_only) {
        _block_dev.reset(new ROBlockDevice(file_name, SECTOR_SIZE, *this));
    } else {
        _block_dev.reset(new PBlockDevice(file_name, SECTOR_SIZE, *this, false));
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

    init_cache();
}


ATADisk::~ATADisk()
{
    while (!_blocks_lru.empty()) {
        delete _blocks_lru.front();
        _blocks_lru.pop_front();
    }

    free(_cache_area);
}


uint64_t ATADisk::get_size()
{
    return _block_dev->get_size();
}


void ATADisk::init_cache()
{
    _cache_area = (uint8_t*)memalign(SECTOR_SIZE, SECTOR_SIZE * ATADISK_CACHE_SIZE);

    if (!_cache_area) {
        THROW("memalign failed");
    }

    for (int i = 0; i < ATADISK_CACHE_SIZE; i++) {
        uint64_t address = ~uint64_t(0) - i;
        CachedBlock* block = new CachedBlock(address, _cache_area + i * SECTOR_SIZE);
        _blocks_lru.push_back(block);
        _cache[address] = block;
    }
}


void ATADisk::reset(bool cold)
{
    ATADevice::reset(cold);

    if (_reverting_to_power_on_default) {
        _sync_mode = true;
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


void ATADisk::block_hit(CachedBlock* block)
{
    std::list<CachedBlock*>::iterator iter = _blocks_lru.begin();

    for (;iter != _blocks_lru.end(); iter++) {
        if ((*iter)->address == block->address) {
            _blocks_lru.erase(iter);
            _blocks_lru.push_back(block);
            return;
        }
    }

    D_MESSAGE("not found");
}


CachedBlock* ATADisk::alloc_block(uint64_t block_index)
{
    std::list<CachedBlock*>::iterator iter = _blocks_lru.begin();

    for (; iter != _blocks_lru.end(); iter++) {

        if ((*iter)->is_in_use()) {
            continue;
        }

        CachedBlock* block = *iter;
        _blocks_lru.erase(iter);

        BlocksMap::iterator cache_iter = _cache.find(block->address);

        if (cache_iter != _cache.end()) {
            if ((*cache_iter).second == block) {
                _cache.erase(cache_iter);
            } else {
                ASSERT(block->is_obsolete());
            }
        }

        block->reset(block_index);
        _blocks_lru.push_back(block);
        _cache[block_index] = block;

        return block;
    }

    return NULL;
}


CachedBlock* ATADisk::get_for_read(uint64_t block_index)
{
    Lock lock(_cache_mutex);

    BlocksMap::iterator iter = _cache.find(block_index);

    CachedBlock* block;

    if (iter != _cache.end()) {
       block = (*iter).second;
       block_hit(block);
    } else if (!(block = alloc_block(block_index))) {
        return NULL;
    }

    if (block->get_state() == CachedBlock::INVALID) {
        block->set_state(CachedBlock::LOADING);
        block->ref();
        inc_async_count();
        _block_dev->read(block);
    }

    return block->ref();
}


void ATADisk::store(CachedBlock* block)
{
    Lock lock(_cache_mutex);
    block->set_state(CachedBlock::STORING);
    block->ref();
    inc_async_count();
    _block_dev->write(block);
    lock.unlock();
    notify_block_change();
}


CachedBlock* ATADisk::get_for_write(uint64_t block_index)
{
    Lock lock(_cache_mutex);

    BlocksMap::iterator iter = _cache.find(block_index);

    CachedBlock* block;

    if (iter != _cache.end()) {
       block = (*iter).second;

       if (block->is_in_use()) {
           D_MESSAGE("replace");
           CachedBlock* obsolete = block;
           obsolete->ref();

           if (!(block = alloc_block(block_index))) {
               obsolete->unref();
               return NULL;
           }

           obsolete->set_obsolete();
           obsolete->unref();
       } else {
           block_hit(block);
       }
    } else if (!(block = alloc_block(block_index))) {
        return NULL;
    }

    block->set_state((block->get_state() == CachedBlock::INVALID) ? CachedBlock::INITILIZING :
                                                                    CachedBlock::MODIFYING);
    return block->ref();
}


void ATADisk::fetch(uint64_t block_index)
{
    CachedBlock* block = get_for_read(block_index);

    if (block) {
        block->unref();
    }
}


void ATADisk::sync(void* mark)
{
    inc_async_count();
    _block_dev->sync(mark);
}


void ATADisk::notify_block_change()
{
    AutoRef<ATADiskTask> task((ATADiskTask*)get_task());

    if (!task.get()) {
        return;
    }

    task->block_changed();
}


void ATADisk::block_io_done(Block* in_block)
{
    CachedBlock* block = (CachedBlock*)in_block;

    Lock lock(_cache_mutex);

    block->unref();
    block->set_state(CachedBlock::VALID);

    lock.unlock();

    notify_block_change();

    dec_async_count();
}


void ATADisk::block_io_error(Block* in_block, int error)
{
    D_MESSAGE("todo: add error handleing");
    CachedBlock* block = (CachedBlock*)in_block;

    Lock lock(_cache_mutex);

    CachedBlock::State state = block->get_state();

    if (state == CachedBlock::STORING) {
        block->set_state(CachedBlock::DIRTY);
    } else if (state == CachedBlock::LOADING) {
        block->set_state(CachedBlock::INVALID);
    } else {
        PANIC("invalid");
    }

    block->unref();

    lock.unlock();

    notify_block_change();

    dec_async_count();
}


void ATADisk::sync_done(void* mark)
{
    AutoRef<ATADiskTask> task((ATADiskTask*)get_task());

    if (task.get()) {
        task->sync_done(mark);
    }

    dec_async_count();
}


void ATADisk::sync_failed(void*, int error)
{
    D_MESSAGE("todo: add error handling");

    dec_async_count();
}


void ATADisk::do_read_sectors_common(uint64_t start, uint64_t end, uint bunch)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    set_power_mode(POWER_ACTIVE);
    AutoRef<ATATask> autoref(new ReadTask(*this, start, end, bunch));
    start_task(autoref.get());
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


void ATADisk::do_write_sectors_common(uint64_t start, uint64_t end, uint bunch)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    set_power_mode(POWER_ACTIVE);
    AutoRef<ATATask> autoref(new WriteTask(*this, start, end, bunch));
    start_task(autoref.get());
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


void ATADisk::do_identify_device()
{
    AutoRef<ATATask> autoref(new IdentifyTask(*this));
    start_task(autoref.get());
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
        raise();
        break;
    case ATA_FEATURE_DISABLE_CACHE:
        D_MESSAGE("disable cache");
        _sync_mode = true;
        raise();
        break;
#if 0
    case 0x05: // Enable advanced power management
        //D_MESSAGE("new power mode is 0x%x", _count & 0xff);
        raise();
        break;
#endif
    case ATA_FEATURE_DISABLE_REVERT_TO_DEFAULT:
        _reverting_to_power_on_default = false;
        raise();
        break;
    case ATA_FEATURE_ENABLE_REVERT_TO_DEFAULT:
        _reverting_to_power_on_default = true;
        raise();

    default:
        D_MESSAGE("unhandled 0x%x. sleeping... ", _feature);
        for (;;) sleep(2);
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

    raise();
}


void ATADisk::do_flush()
{
    AutoRef<ATATask> autoref(new SyncTask(*this));
    start_task(autoref.get());
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
    case ATA_CMD_PACKET:
        command_abort_error();
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
#if 0
    case CMD_WRITE_DMA:
        do_write_dma();
        break;
    case CMD_WRITE_DMA_EXT:
        do_write_dma_ext();
        break;
    case CMD_READ_DMA:
        do_read_dma();
        break;
    case CMD_READ_DMA_EXT:
        do_read_dma_ext();
        break;
#endif
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
#if 0
    case ATA_CMD_CHECK_POWER_MODE:
        _status = STATUS_READY_MASK | ATA_STATUS_SEEK_COMPLEAT;
        _count = get_ata_power_state();
        raise();
        break;
#endif
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
#if 0
    case ATA_CMD_SLEEP:
        do_sleep();
        break;
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
    default:
        D_MESSAGE("unhandled 0x%x %u", command, command);
        command_abort_error();
        for (;;) sleep(2);
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

