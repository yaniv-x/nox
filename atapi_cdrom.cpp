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

#include <sys/uio.h>

#include "atapi_cdrom.h"
#include "block_device.h"
#include "ata.h"
#include "malloc.h"
#include "application.h"
#include "admin_server.h"
#include "dma_state.h"
#include "memory_bus.h"
#include "nox_vm.h"

#ifdef ATA_DEBUG
#define ATA_LOG(format, ...) D_MESSAGE(format, ## __VA_ARGS__)
#else
#define ATA_LOG(format, ...)
#endif

enum {
    MAX_PACKET_PIO_TRANSFER_SIZE = 0xfffe,
    DEFAULT_PACKET_PIO_TRANSFER_SIZE = MAX_PACKET_PIO_TRANSFER_SIZE,

    FORMATTED_TOC = 0,
    MULTI_SESSION_INFO = 1,
    RAW_TOC = 2,

    FORMATTED_TOC_SIZE = 20,
    MULTI_SESSION_SIZE = 12,
    RAW_TOC_SIZE = 48,

    REQUEST_SENSE_PEPLY_SIZE = 18,

    MAX_BUNCH = MAX_PACKET_PIO_TRANSFER_SIZE / MMC_CD_SECTOR_SIZE,

    CDROM_NUM_BLOCKS = MAX_BUNCH,
};

#define DVD9_MAX_SIZE 8547993600UL
#define CD80_MAX_SIZE 737280000
#define MAX_MEDIA_SIZE DVD9_MAX_SIZE


enum {
    LOCK_STATE_MASK = (1 << 0),
    ACTIVE_PERSISTENT_LOCK_MASK = (1 << 1),
    PERSISTENT_LOCK_MASK = (1 << 2),
    EJECT_EVENT_MASK = (1 << 3),
    TRAY_DOR_OPEN_MASK = (1 << 4),
    OPERATIONAL_EVENT_MASK = (1 << 5),
    NEW_MEDIA_EVENT_MASK = (1 << 6),
    ATTENTION_PARAMETERS_MASK = (1 << 7),
    ATTENTION_MEDIA_MASK = (1 << 8),
    ATTENTION_REMOVE_REQUEST = (1 << 9),
    POWER_CHANGED_MASK = (1 << 10),
};


static inline uint16_t revers_unit16(uint16_t val)
{
    return (val << 8) | (val >> 8);
}


static inline uint32_t revers_unit32(uint32_t val)
{
    return (val << 24) | (val >> 24) | ((val << 8) & 0x00ff0000) | ((val >> 8) & 0x0000ff00);
}


static void set_scsi_left_str(uint8_t* dest, int len, const char* str)
{
    uint8_t* end = dest + len;

    while (*str && dest < end) {
        *dest++ = *str++;
    }

    memset(dest, ' ', end - dest);
}


static void frames_to_time(uint num_frames, uint8_t& min, uint8_t& sec, uint8_t& frame)
{
    num_frames += 2 * MMC_CD_FRAMES_PER_SEC;
    frame = num_frames % MMC_CD_FRAMES_PER_SEC;
    uint num_seconds = num_frames / MMC_CD_FRAMES_PER_SEC;
    min = num_seconds / 60;
    sec = num_seconds % 60;
    ASSERT(sec < 99);
}


class CDIdentifyTask: public ATATask, public PIODataSource {
public:
    CDIdentifyTask(ATAPICdrom& cd)
        : ATATask()
        , _cd (cd)
        , _data_now (_identity)
        , _data_end (_data_now + 256)
    {

    }

    virtual void start()
    {
        init_identity();
        _cd.set_pio_source(this);
    }

    virtual void cancel()
    {
        _cd.remove_pio_source(false);
        _cd.remove_task(this);
    }

    void init_identity()
    {
        memset(_identity, 0, sizeof(_identity));

        _identity[ATA_ID_OFFSET_GENERAL_CONF] = ATA_ID_GENERAL_CONF_ATAPI_MASK;
        _identity[ATA_ID_OFFSET_GENERAL_CONF] |=
                                  ATA_ID_GENERAL_COMMAND_SET_CD << ATA_ID_GENERAL_COMMAND_SET_SHIFT;
        _identity[ATA_ID_OFFSET_GENERAL_CONF] |= 1 << ATA_ID_GENERAL_REMOVABLE_BIT;
        _identity[ATA_ID_OFFSET_GENERAL_CONF] |=
                                     ATA_ID_GENERAL_DRQ_LATENCY << ATA_ID_GENERAL_DRQ_LATENCY_SHIFT;

        _identity[ATA_ID_OFFSET_SPECIFIC_CONF] = 0xc837; // Device does not require SET FEATURES
                                                         // subcommand to spin-up after power-up
                                                         // and IDENTIFY DEVICE response is complete

        set_ata_str(&_identity[ATA_ID_OFFSET_SERIAL], ATA_ID_SERIAL_NUM_CHARS / 2,
                    "NxDVD000000000000001");
        set_ata_str(&_identity[ATA_ID_OFFSET_REVISION], ATA_ID_REVISION_NUM_CHARS / 2, "1.0.0");
        set_ata_str(&_identity[ATA_ID_OFFSET_MODEL], ATA_ID_MODEL_NUM_CHARS / 2, "Nox DVD");

        _identity[ATA_ID_OFFSET_CAP1] = ATA_ID_CAP1_DMA_MASK |
                                        ATAPI_ID_CAP1_MBZ |
                                        ATA_ID_CAP1_IORDY_MASK |
                                        ATA_ID_CAP1_DISABLE_IORDY_MASK;

        _identity[ATA_ID_OFFSET_CAP2] = ATA_ID_CAP2_MUST_SET;

        _identity[ATA_ID_OFFSET_FIELD_VALIDITY] = ATA_ID_FIELD_VALIDITY_64_70 |
                                                  ATA_ID_FIELD_VALIDITY_88;

        uint mwdma_select = (_cd.get_multiword_mode() == ~0) ? 0 :
                                                            (1 << (_cd.get_multiword_mode() + 8));
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
                                             ATA_ID_CMD_SET_1_PACKET |
                                             ATA_ID_CMD_SET_1_DEVICE_RESET;
        _identity[ATA_ID_OFFSET_CMD_SET_1_ENABLE] = _identity[ATA_ID_OFFSET_CMD_SET_1];

        _identity[ATA_ID_OFFSET_CMD_SET_2] = ATA_ID_CMD_SET_2_ONE_MASK |
                                             ATA_ID_CMD_SET_2_MEDIA_STATUS_NOTIFICTION;
        _identity[ATA_ID_OFFSET_CMD_SET_2_ENABLE] = ATA_ID_CMD_SET_2_MEDIA_STATUS_NOTIFICTION;

        _identity[ATA_ID_OFFSET_CMD_SET_3] = ATA_ID_CMD_SET_3_ONE_MASK;
        _identity[ATA_ID_OFFSET_CMD_SET_3_ENABLE] = _identity[ATA_ID_OFFSET_CMD_SET_3];

        uint udma_select = (_cd.get_ultra_mode() == ~0) ? 0 : (1 << (_cd.get_ultra_mode() + 8));
        _identity[ATA_ID_OFFSET_UDMA] = ATA_ID_UDMA_MODE0_MASK |
                                        ATA_ID_UDMA_MODE1_MASK |
                                        ATA_ID_UDMA_MODE2_MASK |
                                        ATA_ID_UDMA_MODE3_MASK |
                                        ATA_ID_UDMA_MODE4_MASK |
                                        ATA_ID_UDMA_MODE5_MASK |
                                        udma_select;

        _identity[ATA_ID_OFFSET_HRESET] = ATA_ID_HRESET_ONE_MASK |
                                          ATA_ID_HRESET_PASS_MASK |
                                          ATA_ID_HRESET_JUMPER_MASK;

        _identity[ATA_ID_OFFSET_REMOVABLE_STATUS_SUPPORT] = 1;

        _identity[ATA_ID_OFFSET_BYTE_COUNT_0_BEHAVIOR] = DEFAULT_PACKET_PIO_TRANSFER_SIZE;

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
            _cd.remove_task(this);
            _cd.remove_pio_source(true);
        }

        return ret;
    }

private:
    ATAPICdrom& _cd;
    uint16_t _identity[256];
    uint16_t* _data_now;
    uint16_t* _data_end;
};

class CDReadTask: public ATATask, public PIODataSource {
public:
    CDReadTask(ATAPICdrom& cd, uint64_t start, uint64_t end, uint bunch)
        : ATATask()
        , _cd (cd)
        , _now (start)
        , _end (end)
        , _bunch (bunch)
        , _data_now (NULL)
        , _data_end (NULL)
    {
        ASSERT(_bunch <= MAX_BUNCH);

        if (!(_io_block = _cd.get_io_block())) {
            THROW("age io block failed");
        }

        _cd.get_media();
    }

    void cancel()
    {
        _cd.put_media();
        _cd.remove_pio_source(false);
        _cd.remove_task(this);
    }

    void start()
    {
        _cd._count &= ATA_REASON_TAG_MASK;
        _cd._count |= (1 << ATA_REASON_IO_BIT);
        get_bunch();
    }

    void start_bunch()
    {
        uint length = _bunch * MMC_CD_SECTOR_SIZE;
        _cd._lba_mid = length & 0xff;
        _cd._lba_high = length >> 8;
        _data_now = (uint16_t*)_io_block;
        _data_end = (uint16_t*)(_io_block + length);
        _cd.set_pio_source(this);
    }

    void end()
    {
        _cd.put_media();
        _cd.remove_task(this);
        _cd.remove_pio_source(false);
        _cd.packet_cmd_sucess();
    }

    void error()
    {
        _cd.put_media();
        _cd.remove_task(this);
        _cd.remove_pio_source(false);
        _cd.packet_cmd_chk(SCSI_SENSE_HARDWARE_ERROR,
                           SCSI_SENSE_ADD_LOGICAL_UNIT_COMMUNICATION_FAILURE);
    }

    void redv_done(IOVec* vec, int err)
    {
        ATAPICdrom* cd = &_cd;

        if (err) {
            D_MESSAGE("failed %d", err)
             error();
        } else {
            start_bunch();
        }

        cd->dec_async_count();
    }

    void get_bunch()
    {
        _bunch = MIN(_end - _now, _bunch);

        ASSERT(_bunch);

        _cd.inc_async_count();

        _io_vec.iov_base = _io_block;
        _io_vec.iov_len = _bunch * MMC_CD_SECTOR_SIZE;
        new (&_iov) IOVec(_now, _bunch, &_io_vec, 1,(io_vec_cb_t)&CDReadTask::redv_done, this);
        _cd._mounted_media->readv(&_iov);
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

            _cd.remove_pio_source(false);
            get_bunch();
        }

        return ret;
    }

private:
    ATAPICdrom& _cd;
    uint64_t _now;
    uint64_t _end;
    uint _bunch;
    uint8_t* _io_block;
    uint16_t* _data_now;
    uint16_t* _data_end;
    struct iovec _io_vec;
    IOVec _iov;
};


class CDReadDMATask: public ATATask {
public:
    CDReadDMATask(ATAPICdrom& cd, uint64_t start, uint64_t end)
        : ATATask()
        , _cd (cd)
        , _start (start)
        , _end (end)
        , _dma_started (false)
        , _dma_state (NULL)
    {
        _cd.get_media();
    }

    virtual void cancel()
    {
        if (_dma_state) {
            _dma_state->nop();
        }

        _cd.remove_task(this);
        _cd.put_media();
    }

    virtual void start()
    {
        _cd._count &= ATA_REASON_TAG_MASK;
        _cd._count |= (1 << ATA_REASON_IO_BIT);

        _cd.dma_wait();
    }

    void redv_done_direct(IOVec* vec, int err)
    {
        ATAPICdrom* cd = &_cd;

        cd->remove_task(this);
        cd->put_media();

        if (err) {
            cd->packet_cmd_chk(SCSI_SENSE_HARDWARE_ERROR,
                               SCSI_SENSE_ADD_LOGICAL_UNIT_COMMUNICATION_FAILURE,
                               _dma_state);
        } else {
            cd->packet_cmd_sucess(_dma_state);
        }

        cd->dec_async_count();
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
        uint transfer_size = num_blocks * MMC_CD_SECTOR_SIZE;

        _direct_vector.reset(dma.get_direct_vector(transfer_size));

        if (_direct_vector.get()) {
            _dma_state = &dma;
            _cd.inc_async_count();
            new (&_iov) IOVec(_start, num_blocks, &(*_direct_vector)[0], _direct_vector->size(),
                              (io_vec_cb_t)&CDReadDMATask::redv_done_direct, this);
            _cd._mounted_media->readv(&_iov);
            return true;
        }

        _indirect_vector.reset(dma.get_indirect_vector(transfer_size));

        if (_indirect_vector.get()) {
            _cd.inc_async_count();
            _dma_state = &dma;
            _buf.set(new uint8_t[transfer_size]);
            _io_vec.iov_base = _buf.get();
            _io_vec.iov_len = transfer_size;
            new (&_iov) IOVec(_start, num_blocks, &_io_vec, 1,
                              (io_vec_cb_t)&CDReadDMATask::redv_done_indirect,
                              this);
            _cd._mounted_media->readv(&_iov);
            return true;
        }

        D_MESSAGE("unable to obtaine transfer vector");

        _cd.put_media();
        _cd.remove_task(this);
        _cd.packet_cmd_chk(SCSI_SENSE_HARDWARE_ERROR,
                           SCSI_SENSE_ADD_LOGICAL_UNIT_COMMUNICATION_FAILURE, &dma); // todo: dma
                                                                                     //       error
        return true;
    }

private:
    Mutex _mutex;
    ATAPICdrom& _cd;
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

class CDGenericTransfer: public ATATask, public PIODataSource {
public:
    CDGenericTransfer(ATAPICdrom& cd, uint size, uint max_transfer)
        : ATATask()
        , _cd (cd)
        , _size (size)
        , _transfer_size (max_transfer)
        , _data (new uint8_t[ALIGN(_size, 2)])
        , _data_now ((uint16_t*)_data)
        , _data_end ((uint16_t*)_data)
        , _dma (!!(_cd._feature & ATA_FEATURES_DMA_MASK))
    {
        ASSERT(max_transfer >= size || !(max_transfer & 1));
    }

    inline void cleanup()
    {
        delete[] _data;
        _cd.remove_task(this);
    }

    virtual void cancel()
    {
        _cd.remove_pio_source(false);
        cleanup();
    }

    virtual void start()
    {
        _cd._count &= ATA_REASON_TAG_MASK;
        _cd._count |= (1 << ATA_REASON_IO_BIT);

        if (_dma) {
            _cd.dma_wait();
            return;
        }

        _cd.set_pio_source(this);

        next_run();
    }

    void next_run()
    {
        if (!_size) {
            _cd.remove_pio_source(false);
            cleanup();
            _cd.packet_cmd_sucess();
            return;
        }

        uint n = MIN(_size, _transfer_size);

        _cd._lba_mid = n & 0xff;
        _cd._lba_high = n >> 8;

        _size -= n;

        n = ALIGN(n, 2) >> 1;

        _data_now = _data_end;
        _data_end += n;

    }

    virtual uint16_t get_word()
    {
        if (_data_now == _data_end) {
            PANIC("no data");
        }

        uint16_t ret = *_data_now;

        if (++_data_now == _data_end) {
            next_run();
        }

        return ret;
    }

    void copy(IndirectVector& vec)
    {
        IndirectVector::iterator iter = vec.begin();
        uint8_t* src = _data;

        for (; iter != vec.end(); iter++) {
            memory_bus->write(src, (*iter).size, (*iter).address);
            src += (*iter).size;
        }
    }

    virtual bool dma_write_start(DMAState& dma)
    {
        std::unique_ptr<IndirectVector> vec(dma.get_indirect_vector(_size));

        if (!vec.get()) {
            D_MESSAGE("unable to obtaine transfer vector");
            cleanup();
            _cd.packet_cmd_chk(SCSI_SENSE_HARDWARE_ERROR,
                               SCSI_SENSE_ADD_LOGICAL_UNIT_COMMUNICATION_FAILURE,
                               &dma); // todo: dma error
            return true;
        }

        copy(*vec.get());

        cleanup();
        _cd.packet_cmd_sucess(&dma);

        return true;
    }

    uint8_t* get_data() { return _data;}

private:
    ATAPICdrom& _cd;
    uint _size;
    uint _transfer_size;
    uint8_t* _data;
    uint16_t* _data_now;
    uint16_t* _data_end;
    bool _dma;
};


class PacketTask: public ATATask, public PIODataDest {
public:
    PacketTask(ATAPICdrom& cd)
        : ATATask()
        , _cd (cd)
        , _data_now ((uint16_t*)_packet)
        , _data_end ((uint16_t*)(_packet + sizeof(_packet)))
    {
    }

    virtual ~PacketTask()
    {
    }

    virtual void start()
    {
        _cd._count &= ATA_REASON_TAG_MASK;
        _cd._count |= (1 << ATA_REASON_CD_BIT);
        _cd.set_pio_dest(this, false);
    }

    virtual void cancel()
    {
        _cd.remove_pio_dest(false);
        _cd.remove_task(this);
    }

    virtual void  put_word(uint16_t val)
    {
        if (_data_now == _data_end) {
            PANIC("no data");
        }

        *_data_now = val;

        if (++_data_now == _data_end) {
            _cd.remove_task(this);
            _cd.remove_pio_dest(false);
            _cd.handle_packet(_packet);
        }
    }

private:
    ATAPICdrom& _cd;
    uint8_t _packet[ATAPI_PACKET_SIZE];
    uint16_t* _data_now;
    uint16_t* _data_end;
};


ATAPICdrom::ATAPICdrom(VMPart& owner, Wire& wire, const std::string& file_name)
    : ATADevice("atapi-cdrom", owner, wire)
    , _mounted_media (NULL)
{
    ASSERT(sizeof(CDIdentifyTask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(CDReadTask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(CDReadDMATask) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(CDGenericTransfer) <= ATADEV_TASK_STORAGE_SIZE);
    ASSERT(sizeof(PacketTask) <= ATADEV_TASK_STORAGE_SIZE);

    if (file_name.size()) {
        try {
            _media.reset(new PBlockDevice(file_name, MMC_CD_SECTOR_SIZE, true));
            if (_media->get_size() > MAX_MEDIA_SIZE) {
                 W_MESSAGE("size exceed dvd size");
                 _media.reset(NULL);
            }
        } catch (...) {
            W_MESSAGE("loading media failed");
        }
    }

    register_admin_commands();
}


ATAPICdrom::~ATAPICdrom()
{
}


void ATAPICdrom::register_admin_commands()
{
    AdminServer* admin = application->get_admin();

    admin->register_command("eject", "press eject button", "???",
                            empty_va_type_list, empty_names_list,
                            empty_va_type_list, empty_names_list,
                            (admin_command_handler_t)&ATAPICdrom::eject_command, this);

    va_type_list_t input_args(1);

    input_args[0] = VA_UTF8_T;

    va_names_list_t input_names(1);

    input_names[0] = "file-name";

    admin->register_command("set-media", "set cdrom media", "???",
                            input_args, input_names,
                            empty_va_type_list, empty_names_list,
                            (admin_command_handler_t)&ATAPICdrom::set_media_command, this);
}


class DeferSetMedia {
public:
    DeferSetMedia(ATAPICdrom& cdrom, const char* file_name, AdminReplyContext* context)
        : _cdrom (cdrom)
        , _file (copy_cstr(file_name))
        , _context (context)
        , _timer (application->create_timer((void_callback_t)&DeferSetMedia::do_set, this))
    {
        _timer->arm(uint64_t(1000) * 1000 * 1000 * 2, false);
    }

    void do_set()
    {
        _cdrom.set_media(_file);
        _context->command_reply();
        _timer->destroy();
        delete[] _file;
        delete this;
    }

public:
    ATAPICdrom& _cdrom;
    const char* _file;
    AdminReplyContext* _context;
    Timer* _timer;
};


void ATAPICdrom::set_media_command(AdminReplyContext* context, const char* name)
{
    open_tray();

    new DeferSetMedia(*this, name, context); // deferring inserting of new media in-order to let
                                             // windows detact removal of current media. The
                                             // DeferSetMedia object is unsafe and will be fixd
                                             // once  I'll know for sure that it is the right way
                                             // to go.
}


void ATAPICdrom::eject_command(AdminReplyContext* context)
{
    eject_button_press();
    context->command_reply();
}


void ATAPICdrom::update_profile()
{
    if (!_mounted_media) {
        _profile = MMC_PROFILE_NON;
        return;
    }

    _profile = (_mounted_media->get_size() > CD80_MAX_SIZE) ? MMC_PROFILE_DVDROM :
                                                              MMC_PROFILE_CDROM;
}

void ATAPICdrom::reset(bool cold)
{
    D_MESSAGE("");

    ATADevice::reset(cold);

    _sense = SCSI_SENSE_NO_SENSE;
    _sense_add = SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION;
    _cdrom_state = 0;
    _mounted_media = _media.get();
    update_profile();
}


uint ATAPICdrom::get_mmc_power_mode()
{
    switch (get_power_mode()) {
    case ATADevice::POWER_ACTIVE:
        return MMC_POWER_ACTIVE;
    case ATADevice::POWER_IDLE:
        return MMC_POWER_IDLE;
    case ATADevice::POWER_STANDBY:
        return MMC_POWER_STANDBY;
    case ATADevice::POWER_SLEEP:
        return MMC_POWER_SLEEP;
    }

    PANIC("invalid type %u", get_power_mode());
    return 0;
}


void ATAPICdrom::set_power_mode(PowerState mode)
{
    if (get_power_mode() == mode) {
        return;
    }

    set_power_mode(mode);
    _cdrom_state |= POWER_CHANGED_MASK;
}


uint8_t ATAPICdrom::io_read(uint16_t port)
{
    if (current() == 1) {
        return 0;
    }

    return ATADevice::io_read(port);
}


void ATAPICdrom::set_signature()
{
    _count = 1;
    _lba_low = 1;
    _lba_mid = 0x14;
    _lba_high = 0xeb;
    _device_reg = 0;
}


void ATAPICdrom::_packet_cmd_done(uint status, uint sense, uint sense_add, DMAState* dma)
{
    _count &= ATA_REASON_TAG_MASK;
    _count |= (1 << ATA_REASON_CD_BIT) | (1 << ATA_REASON_IO_BIT);
    _sense = sense;
    _sense_add = sense_add;

    if (dma) {
        set_state_and_notify(status & ~(ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK), *dma);
    } else {
        set_state_and_notify(status & ~(ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK));
    }
}


void ATAPICdrom::packet_cmd_abort(uint sens, uint sens_add)
{
    ATA_LOG("packet_abort: 0x%x 0x%x", sens, sens_add);
    _error = ATA_ERROR_ABORT | (sens << SCSI_SENSE_SHIFT);
    _packet_cmd_done(_status | ATA_STATUS_CHK_MASK, sens, sens_add);
}


void ATAPICdrom::packet_cmd_chk(uint sens, uint sens_add, DMAState* dma)
{
    ATA_LOG("packet_chk: 0x%x 0x%x", sens, sens_add);
    _error = (sens << SCSI_SENSE_SHIFT);
    _packet_cmd_done(_status | ATA_STATUS_CHK_MASK, sens, sens_add, dma);
}


void ATAPICdrom::packet_cmd_sucess(DMAState* dma)
{
    _packet_cmd_done(_status,
                     SCSI_SENSE_NO_SENSE,
                     SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION,
                     dma);
}


bool ATAPICdrom::handle_attention_condition()
{
    if (!(_cdrom_state & (ATTENTION_PARAMETERS_MASK | ATTENTION_MEDIA_MASK |
                          ATTENTION_REMOVE_REQUEST))) {
        return false;
    }

    if ((_cdrom_state & ATTENTION_REMOVE_REQUEST)) {
        packet_cmd_chk(SCSI_SENSE_UNIT_ATTENTION, SCSI_SENSE_ADD_OPERATOR_REMOVAL_REQUEST);
    } else if ((_cdrom_state & ATTENTION_PARAMETERS_MASK)) {
        packet_cmd_chk(SCSI_SENSE_UNIT_ATTENTION, SCSI_SENSE_ADD_PARAMETERS_CHANGED);
    } else {
        packet_cmd_chk(SCSI_SENSE_UNIT_ATTENTION, SCSI_SENSE_ADD_MEDIUM_MAY_HAVE_CHANGED);
    }

    return true;
}


uint ATAPICdrom::max_pio_transfer_bytes()
{
    uint host_count = (_lba_mid & 0xff) | ((_lba_high & 0xff) << 8);

    if (!host_count) {
        return DEFAULT_PACKET_PIO_TRANSFER_SIZE;
    }

    return MIN(host_count, MAX_PACKET_PIO_TRANSFER_SIZE);
}


uint ATAPICdrom::get_not_present_sens_add()
{
    return (_cdrom_state & TRAY_DOR_OPEN_MASK) ? SCSI_SENSE_ADD_MEDIUM_NOT_PRESENT_OPEN :
                                                 SCSI_SENSE_ADD_MEDIUM_NOT_PRESENT_CLOSED;
}


void ATAPICdrom::mmc_read_capacity(uint8_t* packet)
{
    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    uint max_transfer = max_pio_transfer_bytes();

    if (!max_transfer) {
        D_MESSAGE("max transfer bytes is zero");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if ((max_transfer & 1) && max_transfer < 8)  {
        D_MESSAGE("invalid odd transfer bytes");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if ((packet[9] & 1) /*link bit is set*/) {
        D_MESSAGE("link is set");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint32_t* ptr32 = (uint32_t*)&packet[2];

    if ((packet[1] & 1) || (packet[8] & 1) || *ptr32) {
        D_MESSAGE("mbz test failed");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    Lock lock(_media_lock);

    if (!_mounted_media) {
        D_MESSAGE("media not present");
        packet_cmd_chk(SCSI_SENSE_NOT_READY, get_not_present_sens_add());
        return;
    }

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, 8, max_transfer);

    uint32_t* data = (uint32_t*)task->get_data();
    data[0] = revers_unit32(_mounted_media->get_size() / MMC_CD_SECTOR_SIZE - 1);
    data[1] = revers_unit32(MMC_CD_SECTOR_SIZE);

    start_task(task);
}


void ATAPICdrom::mmc_read(uint8_t* packet)
{
    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    if ((packet[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if ((packet[1] & 1) /*RelAdr*/ || (packet[1] & (1 << 3)) /*DPO*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    Lock lock(_media_lock);

    if (!_mounted_media) {
        D_MESSAGE("media not present");
        packet_cmd_chk(SCSI_SENSE_NOT_READY, get_not_present_sens_add());
        return;
    }

    uint32_t* ptr32 = (uint32_t*)&packet[2];
    uint64_t start = revers_unit32(*ptr32);
    uint16_t length = revers_unit16(*(uint16_t*)&packet[7]);
    uint64_t end = start + length;

    if (start > _mounted_media->get_size() / MMC_CD_SECTOR_SIZE || start > end)  {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST,
                         SCSI_SENSE_ADD_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);
        return;
    }

    if (!length) {
        packet_cmd_sucess();
        return;
    }

    //bool force_unit_access = !!(_sector[1] & (1 << 4));

    if ((_feature & ATA_FEATURES_DMA_MASK)) {
        start_task(new (_task_storage) CDReadDMATask(*this, start, end));
    } else {
        uint max_transfer = max_pio_transfer_bytes();
        uint bunch = max_transfer / MMC_CD_SECTOR_SIZE;

        if (!bunch) {
            D_MESSAGE("invalid  transfer size %u", max_transfer);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        start_task(new (_task_storage) CDReadTask(*this, start, end, MIN(bunch, length)));
    }

    set_power_mode(ATADevice::POWER_ACTIVE);
}


void ATAPICdrom::scsi_test_unit_ready(uint8_t* packet)
{
    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    if ((packet[5] & 1) /*link bit is set*/ ) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    Lock lock(_media_lock);

    if (_mounted_media) {
        packet_cmd_sucess();
    } else {
        packet_cmd_chk(SCSI_SENSE_NOT_READY, get_not_present_sens_add());
    }
}


void ATAPICdrom::scsi_inquiry(uint8_t* packet)
{
     if ((packet[5] & 1) /*link bit is set*/ ) {
         packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
         return;
     }

     if ((packet[1] & 1) /*use page code is set*/  || packet[2] /* page code*/) {
         packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
         return;
     }

    uint16_t* ptr16 = (uint16_t*)&packet[3];
    uint length = revers_unit16(*ptr16);

    if (length < 5) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer 0x%x 0x%x", max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    length = MIN(length, SCSI_INQUIRY_STD_LENGTH);

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);

    uint8_t buf[SCSI_INQUIRY_STD_LENGTH];
    memset(buf, 0, SCSI_INQUIRY_STD_LENGTH);

    buf[0] = 5;         // MMC-4
    buf[1] = 1 << 7;    //removable medium
    buf[2] = 0x05;      // (SPC-3)
    buf[3] = 3;         // response data format
    buf[4] = SCSI_INQUIRY_STD_LENGTH - 5;

    set_scsi_left_str(&buf[8], 8, "Nox");      // VENDOR ID
    set_scsi_left_str(&buf[16], 16, "NxDVD");    // PRODUCT ID
    set_scsi_left_str(&buf[32], 4, "0");         // PRODUCT REVISION

    memcpy(task->get_data(), buf, length);
    start_task(task);
}


class DynamicBuf {
public:
    DynamicBuf(uint init_size = 64)
        : _base (new uint8_t[init_size])
        , _now (_base)
        , _size (init_size)
    {
    }

    ~DynamicBuf()
    {
        delete[] _base;
    }

    void grow()
    {
        uint new_size = MAX(_size * 2, 64);
        uint8_t* new_buf = new uint8_t[new_size];
        memcpy(new_buf, _base, _now - _base);
        delete[] _base;
        _base = new_buf;
        _size = new_size;
    }

    void put_uint8(uint8_t val)
    {
        if (_now - _base + 1 >= _size) {
            grow();
        }

        *_now++ = val;
    }

    void put_uint16(uint16_t val)
    {
        if (_now - _base + 2 >= _size) {
            grow();
        }

        *(uint16_t*)_now = val;
        _now += 2;
    }

    void put_uint32(uint32_t val)
    {
        if (_now - _base + 4 >= _size) {
            grow();
        }

        *(uint32_t*)_now = val;
        _now += 4;
    }

    uint8_t* base() { return _base;}
    uint32_t position() { return _now - _base;}

public:
    uint8_t* _base;
    uint8_t* _now;
    uint _size;
};


void ATAPICdrom::scsi_mode_sense(uint8_t* packet)
{
    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    if ((packet[1] & 1) /* DESC*/ || (packet[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint page_code = packet[2] & 0x3f;
    uint page_control = packet[2] >> 6;
    uint subpage_code = packet[3];

    switch (page_code) {
    case 0x2a: { // MM Capabilities and Mechanical Status Page

        if (page_control != 0) {
            D_MESSAGE("abort: page 0x%x control 0x%x ", page_code, page_control);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        uint16_t* ptr16 = (uint16_t*)&packet[7];
        uint length = revers_unit16(*ptr16);

        if (length == 0) {
            packet_cmd_sucess();
            return;
        }

        DynamicBuf buf(128);

        // Mode Parameters Header
        buf.put_uint16(0); // Mode Data Length
        buf.put_uint32(0); // Reserved
        buf.put_uint16(0); // Block Descriptor Length

        // Page
        buf.put_uint8(page_code); // page code and Parameters Savable is not set
        buf.put_uint8(30); // page length conflict with documentation (28). need to test on
                           //                                              physical machine
        buf.put_uint8(0);  // no read support
        buf.put_uint8(0);  // no write support
        buf.put_uint8(1 | (1 << 6));  // play audio, multi session
        buf.put_uint8(0);
        buf.put_uint8(1 | (1 << 3) | (1 << 5) | ((_cdrom_state & LOCK_STATE_MASK) ? (1 << 1) : 0));
                                                            // lock, eject, tray type
        buf.put_uint8(0);
        buf.put_uint16(0); // Obsolete
        buf.put_uint16(revers_unit16(2)); // volume levels
        buf.put_uint16(0); // buffer size
        buf.put_uint16(0); // Obsolete
        buf.put_uint8(0); // Reserved
        buf.put_uint8(0);
        buf.put_uint32(0); // Obsolete
        buf.put_uint16(0); // Copy protection
        buf.put_uint8(0); // Reserved
        buf.put_uint8(0); // Reserved
        buf.put_uint8(0); // Reserved
        buf.put_uint8(0); // Rotation Control 0
        buf.put_uint16(0); // Current Write Speed Selected
        buf.put_uint16(0); // Number of Logical Unit Write Speed

        ptr16 = (uint16_t*)buf.base();
        *ptr16 = revers_unit16(buf.position() - 2);

        length = MIN(length, buf.position());
        uint max_transfer = max_pio_transfer_bytes();

        if (length > max_transfer && (max_transfer & 1)) {
            D_MESSAGE("odd max_transfer (0x%x 0x%x 0x%x)", buf.position(), max_transfer, length);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length,
                                                                        max_transfer);
        memcpy(task->get_data(), buf.base(), length);

        start_task(task);
        break;
    }
    case 0x0e: { //CD Audio Control Page
        if (subpage_code) {
            D_MESSAGE("abort on subpage_code 0x%x",subpage_code);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        uint16_t* ptr16 = (uint16_t*)&packet[7];
        uint length = revers_unit16(*ptr16);
        uint max_transfer = max_pio_transfer_bytes();

        if (length > max_transfer && (max_transfer & 1)) {
            D_MESSAGE("odd max_transfer 0x%x 0x%x",max_transfer, length);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, 16, max_transfer);
        task->get_data()[0] = 0x0e; //page code
        task->get_data()[1] = 0x0e; //page length
        memset(task->get_data() + 2, 0, 14);
        start_task(task);
        break;
    }
    // case 0X01: for random readable feature set, mandatory if PP == 1
    // case 0X1a: for power managment feature set, mandatory (SPC-3)
    // case 0X1d: for timout feature set, mandatory
    case 0x1b: // according to MMC-4 is reserved
        D_MESSAGE("abort on page 0x%x subpage 0x%x", page_code, subpage_code);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        break;
    default:
        D_MESSAGE("abort on page 0x%x subpage 0x%x", page_code, subpage_code);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        D_MESSAGE("sleep...");
        for (;;) {
            sleep(2);
        }
    }
}


void ATAPICdrom::read_formatted_toc(uint8_t* packet)
{
    if ((packet[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint start_track = packet[6];

    if (start_track > MMC_LEADOUT_TRACK_ID) {
        D_MESSAGE("invalid start track");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint16_t* ptr16 = (uint16_t*)&packet[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer (0x%x 0x%x)", max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    bool time = !!(packet[1] & (1 << 1));

    DynamicBuf buf(128);

    buf.put_uint16(0); // toc length
    buf.put_uint8(1);  // first track
    buf.put_uint8(1);  // last track

    if (start_track <= 1) {
        buf.put_uint8(0);     // reservd
        buf.put_uint8(0x14);  // control = Data track, recorded uninterrupted
                              // Q Sub-channel encodes current position data
        buf.put_uint8(1);     // track number
        buf.put_uint8(0);     // reservd

        if (time) {
            uint8_t min, sec, frame;
            frames_to_time(0, min, sec, frame);

            buf.put_uint8(0);
            buf.put_uint8(min);
            buf.put_uint8(sec);
            buf.put_uint8(frame);
        } else {
            buf.put_uint32(0);
        }
    }

    buf.put_uint8(0);                    // reservd
    buf.put_uint8(0x16);                 // control = Data track, recorded uninterrupted
                                         // Q Sub-channel encodes current position data
    buf.put_uint8(MMC_LEADOUT_TRACK_ID); // track number
    buf.put_uint8(0);                    // reservd
    if (time) {
        uint8_t min, sec, frame;
        frames_to_time(_mounted_media->get_size() / MMC_CD_SECTOR_SIZE, min, sec, frame);
        buf.put_uint8(0);
        buf.put_uint8(min);
        buf.put_uint8(sec);
        buf.put_uint8(frame);
    } else {
        buf.put_uint32(revers_unit32(_mounted_media->get_size() / MMC_CD_SECTOR_SIZE));
    }

    ptr16 = (uint16_t*)buf.base();
    *ptr16 = revers_unit16(buf.position() - 2);

    length = MIN(buf.position(), length);

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);
    memcpy(task->get_data(), buf.base(), length);
    start_task(task);
    set_power_mode(ATADevice::POWER_ACTIVE);
}


void ATAPICdrom::read_raw_toc(uint8_t* packet)
{
    if ((packet[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint start_session = packet[6];

    uint16_t* ptr16 = (uint16_t*)&packet[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint8_t data[RAW_TOC_SIZE];

    ptr16 = (uint16_t*)&data[0];
    *ptr16 = 0;    // row toc length
    data[2] = 1;   // first session
    data[3] = 1;   // last session

    if (start_session > 1) {
        ptr16 = (uint16_t*)&data[0];
        *ptr16 = revers_unit16(2);    // row toc length

        length = MIN(length, 4);

        uint max_transfer = max_pio_transfer_bytes();

        if (length > max_transfer && (max_transfer & 1)) {
            D_MESSAGE("odd max_transfer (0x%x 0x%x)", max_transfer, length);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length,
                                                                        max_transfer);
        memcpy(task->get_data(), data, length);
        start_task(task);
        set_power_mode(ATADevice::POWER_ACTIVE);
        return;
    }

    data[4] = 1;                                 // session number
    data[5] = 0x14;  // control = Data track, recorded uninterrupted
    data[6] = 0;     //TNO;
    data[7] = 1;     //POINT ; track 1
    frames_to_time(_mounted_media->get_size() / MMC_CD_SECTOR_SIZE,
                   data[8], data[9], data[10]); // running time
    data[11] = 0; //ZERO
    data[12] = 0; // track start MIN
    data[13] = 0; // track start SEC
    data[14] = 0; // track start FRAME


    data[15] = 1;                                 // session number
    data[16] = 0x14;  // control = Data track, recorded uninterrupted
    data[17] = 0;     //TNO;
    data[18] = 0xa0;     //POINT ; lead-in
    data[19] = 0; // lead-in running time MIN
    data[20] = 0; // lead-in running time SEC
    data[21] = 0; // lead-in running time FRAME
    data[22] = 0; //ZERO
    data[23] = 1; // first track in program area
    data[24] = 0; // program area format
    data[25] = 0;

    data[26] = 1;                                 // session number
    data[27] = 0x14;  // control = Data track, recorded uninterrupted
    data[28] = 0;     //TNO;
    data[29] = 0xa1;  //POINT
    data[30] = 0;     // lead-in running time MIN
    data[31] = 0;     // lead-in running time SEC
    data[32] = 0;    // lead-in running time FRAME
    data[33] = 0;    //ZERO
    data[34] = 1;    // last track in program area
    data[35] = 0;
    data[36] = 0;

    data[37] = 1;     // session number
    data[38] = 0x14;  // control = Data track, recorded uninterrupted
    data[39] = 0;     //TNO;
    data[40] = 0xa2;  //POINT
    data[41] = 0;     // lead-in running time MIN
    data[42] = 0;     // lead-in running time SEC
    data[43] = 0;    // lead-in running time FRAME
    data[44] = 0;    //ZERO
    frames_to_time(_mounted_media->get_size() / MMC_CD_SECTOR_SIZE,
                   data[45], data[46], data[47]); // lead out start time

    ptr16 = (uint16_t*)&data[0];
    *ptr16 = revers_unit16(46);    // row toc length

    length = MIN(length, RAW_TOC_SIZE);

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer (0x%x 0x%x)", max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);
    memcpy(task->get_data(), data, length);

    start_task(task);
    set_power_mode(ATADevice::POWER_ACTIVE);
}


void ATAPICdrom::read_multi_session_info(uint8_t* packet)
{
    if ((packet[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint16_t* ptr16 = (uint16_t*)&packet[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer (0x%x 0x%x)", max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint8_t data[MULTI_SESSION_SIZE];
    memset(data, 0, sizeof(data));

    ptr16 = (uint16_t*)&data[0];
    *ptr16 = revers_unit16(sizeof(data) - 2);
    data[2] = 1; // First Complete Session Number
    data[3] = 1; // Last Complete Session Number
    data[4] = 0, // Reserved
    data[5] = 0x14; // // control = Data track, recorded uninterrupted
    data[6] = 1; // First Track Number In Last Complete Session
    data[7] = 0; // Reserved

    bool time = !!(packet[1] & (1 << 1));

    if (time) {
        data[8] = 0;
        frames_to_time(0, data[9], data[10], data[11]);
    } else {
        uint32_t* address = (uint32_t*)&data[8];
        *address = 0; // Start Address of First Track in Last Session
    }

    length = MIN(length, sizeof(data));
    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);
    memcpy(task->get_data(), data, length);
    start_task(task);
    set_power_mode(ATADevice::POWER_ACTIVE);
}


void ATAPICdrom::mmc_read_toc(uint8_t* packet)
{
    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    Lock lock(_media_lock);

    if (!_mounted_media) {
        D_MESSAGE("media not present");
        packet_cmd_chk(SCSI_SENSE_NOT_READY, get_not_present_sens_add());
        return;
    }

    uint format = packet[2] & 0x0f;

    switch (format) {
    case FORMATTED_TOC:
        read_formatted_toc(packet);
        break;
    case MULTI_SESSION_INFO:
        read_multi_session_info(packet);
        break;
    case RAW_TOC:
        read_raw_toc(packet);
        break;
    default:
        D_MESSAGE("format 0x%x, sleeping...", format);
        for (;;) sleep(2);
    }
}


void ATAPICdrom::scsi_request_sens(uint8_t* packet)
{
    uint length = packet[4];

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer (0x%x 0x%x)", max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if ((packet[1] & 1) /* DESC*/ || (packet[5] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    length = MIN(REQUEST_SENSE_PEPLY_SIZE, length);

    uint8_t data[REQUEST_SENSE_PEPLY_SIZE];

    memset(data, 0, sizeof(data));
    data[0] = (1 << 7) | 0x70; // valid and current
    data[2] = _sense;
    data[7] = 10; // ADDITIONAL SENSE LENGTH (n - 7)
    data[12] = _sense_add >> 8;
    data[13] = _sense_add;

    if (_sense == SCSI_SENSE_UNIT_ATTENTION) {
        if (_sense_add == SCSI_SENSE_ADD_PARAMETERS_CHANGED) {
            _cdrom_state &= ~ATTENTION_PARAMETERS_MASK;
        } else if (_sense_add == SCSI_SENSE_ADD_MEDIUM_MAY_HAVE_CHANGED) {
            _cdrom_state &= ~ATTENTION_MEDIA_MASK;
        } else if (_sense_add == SCSI_SENSE_ADD_OPERATOR_REMOVAL_REQUEST) {
            _cdrom_state &= ~ATTENTION_REMOVE_REQUEST;
        }
    }

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);
    memcpy(task->get_data(), data, length);

    start_task(task);
}


static bool feature_add_test(uint feature, uint start, uint rt, bool current)
{
    switch (rt) {
    case 0:
        return feature >= start;
    case 1:
        return feature >= start && current;
    case 2:
        return feature == start;
    default:
        PANIC("invalid");
        return false;
    }
}


void ATAPICdrom::mmc_get_configuration(uint8_t* packet)
{
    Lock lock(_media_lock);

    bool media = !!_mounted_media;

    if ((packet[9] & 1) /*link bit is set*/) {
        D_MESSAGE("link is set");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint16_t* ptr16 = (uint16_t*)&packet[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint rt = packet[1] & 3;

    if (rt == 3) {
        D_MESSAGE("invalid rt");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    ptr16 = (uint16_t*)&packet[2];
    uint start = revers_unit16(*ptr16);

    DynamicBuf buf(128);

    buf.put_uint32(0); //length
    buf.put_uint8(0);  //reserved
    buf.put_uint8(0);  //reserved
    buf.put_uint16(_profile);

    if (feature_add_test(MMC_FEATURE_PROFILE_LIST, start, rt, true)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_PROFILE_LIST));
        buf.put_uint8(0x3); // currenr | persistent, version == 0
        buf.put_uint8(8);  // aditional length (num profiles * 4)
        buf.put_uint16(revers_unit16(MMC_PROFILE_CDROM));
        buf.put_uint8((_profile == MMC_PROFILE_CDROM) ? 1 : 0);  // current?
        buf.put_uint8(0);  // reserved
        buf.put_uint16(revers_unit16(MMC_PROFILE_DVDROM));
        buf.put_uint8((_profile == MMC_PROFILE_DVDROM) ? 1 : 0);  // current?
        buf.put_uint8(0);  // reserved
    }

    if (feature_add_test(MMC_FEATURE_CORE, start, rt, true)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_CORE));
        buf.put_uint8(0x7); // currenr | persistent, version == 1
        buf.put_uint8(8);  // aditional length
        buf.put_uint32(revers_unit32(0x00000002)); // ATAPI interface
        buf.put_uint8(1);  // DBE is set
        buf.put_uint8(0);  // reserved
        buf.put_uint8(0);  // reserved
        buf.put_uint8(0);  // reserved
    };

    if (feature_add_test(MMC_FEATURE_MORPHING, start, rt, true)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_MORPHING));
        buf.put_uint8(0x7); // currenr | persistent, version == 1
        buf.put_uint8(4);  // aditional length
        buf.put_uint8(2);  // OCEvent is set
        buf.put_uint8(0);  // reserved
        buf.put_uint8(0);  // reserved
        buf.put_uint8(0);  // reserved
    }

    if (feature_add_test(MMC_FEATURE_REMOVABLE, start, rt, true)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_REMOVABLE));
        buf.put_uint8(0x3); // currenr | persistent, version == 0
        buf.put_uint8(4);  // aditional length
        buf.put_uint8(1 | 8 | (1 << 5));  // LOCK | EJECT | Tray type loading mechanism
        buf.put_uint8(0);  // reserved
        buf.put_uint8(0);  // reserved
        buf.put_uint8(0);  // reserved
    }

    if (feature_add_test(MMC_FEATURE_RANDOM_READABLE, start, rt, media)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_RANDOM_READABLE));
        buf.put_uint8(media ? 1 :0); // current? | not persistent, version == 0
        buf.put_uint8(8); // aditional length
        buf.put_uint32(revers_unit32(MMC_CD_SECTOR_SIZE)); // logical block
        buf.put_uint16(revers_unit16(1)); // blocking
        buf.put_uint8(1); // PP is set
        buf.put_uint8(0); // reserved
    }

    if (feature_add_test(MMC_FEATURE_CD_READ, start, rt, _profile == MMC_PROFILE_CDROM)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_CD_READ));
        buf.put_uint8((_profile == MMC_PROFILE_CDROM) ? 0x9 : 0x8); // current? | not persistent,
                                                                    // version == 2
        buf.put_uint8(4);  // aditional length
        buf.put_uint8(0); // DAP C2 and CD-Text are not set
        buf.put_uint8(0); // reserved
        buf.put_uint8(0); // reserved
        buf.put_uint8(0); // reserved
    }

    if (feature_add_test(MMC_FEATURE_DVD_READ, start, rt, _profile == MMC_PROFILE_DVDROM)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_DVD_READ));
        buf.put_uint8((_profile == MMC_PROFILE_DVDROM) ? 1 : 0);
        buf.put_uint8(0);  // aditional length
    }

    if (feature_add_test(MMC_FEATURE_POWER_MANAGMENET, start, rt, true)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_POWER_MANAGMENET));
        buf.put_uint8(0x3); // currenr | persistent, version == 0
        buf.put_uint8(0);  // aditional length
    }

    if (feature_add_test(MMC_FEATURE_TIMEOUT, start, rt, true)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_TIMEOUT));
        buf.put_uint8(0x7); // currenr | persistent, version == 1
        buf.put_uint8(4);  // aditional length
        buf.put_uint8(0); // Group 3 is not set
        buf.put_uint8(0); // reserved
        buf.put_uint16(revers_unit16(0)); // unit length (When the Group3 bit is set to 0,
                                          //              Unit Length field is not valid)
    }

    if (feature_add_test(MMC_FEATURE_RT_STREAM, start, rt, _profile == MMC_PROFILE_DVDROM)) {
        buf.put_uint16(revers_unit16(MMC_FEATURE_RT_STREAM));
        buf.put_uint8((_profile == MMC_PROFILE_DVDROM) ? 0x0d : 0x0c); // currenr ? | version == 3
        buf.put_uint8(4);  // aditional length.
        buf.put_uint8(0); // support non
        buf.put_uint8(0); // reserved
        buf.put_uint8(0); // reserved
        buf.put_uint8(0); // reserved
    }

    *(uint32_t*)buf.base() = revers_unit32(buf.position() - 8);

    length = MIN(length, buf.position());

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer (0x%x 0x%x 0x%x)", buf.position(), max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);
    memcpy(task->get_data(), buf.base(), length);

    start_task(task);
}


void ATAPICdrom::mmc_prevent_allow_removal(uint8_t* packet)
{
    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    Lock lock(_media_lock);

    switch (packet[4] & 3) {
    case 0:
        D_MESSAGE("unlock");
        _cdrom_state &= ~LOCK_STATE_MASK;
        break;
    case 1:
        D_MESSAGE("lock");
        _cdrom_state |= LOCK_STATE_MASK;
        break;
    case 2:
        D_MESSAGE("persistent allow");
        _cdrom_state &= ~(PERSISTENT_LOCK_MASK | ACTIVE_PERSISTENT_LOCK_MASK);
        break;
    case 3:
        D_MESSAGE("Persistent Prevent");
        _cdrom_state |= PERSISTENT_LOCK_MASK;

        if (_mounted_media && !(_cdrom_state & NEW_MEDIA_EVENT_MASK)) {
            _cdrom_state |= ACTIVE_PERSISTENT_LOCK_MASK;
        }
        break;
    }

    packet_cmd_sucess();
}


uint ATAPICdrom::notify_operational(uint8_t* buf)
{
    uint16_t* ptr16;

    ptr16 = (uint16_t*)&buf[0];
    *ptr16 = revers_unit16(4);
    buf[2] = MMC_NOTIFY_OPERATIONAL_CLASS;
    buf[5] = (_cdrom_state & ACTIVE_PERSISTENT_LOCK_MASK) ? (1 << 7) : 0;

    if ((_cdrom_state & OPERATIONAL_EVENT_MASK)) {
        D_MESSAGE("op");
        buf[4] = 2;                 // Logical Unit has changed Operational state
        ptr16 = (uint16_t*)&buf[6];
        *ptr16 = revers_unit16(1);  // Feature Change
        _cdrom_state &= ~OPERATIONAL_EVENT_MASK;
    }

    return 8;
}

uint ATAPICdrom::notify_power(uint8_t* buf)
{
    uint16_t* ptr16;

    ptr16 = (uint16_t*)&buf[0];
    *ptr16 = revers_unit16(4);
    buf[2] = MMC_NOTIFY_POWER_CLASS;
    buf[5] = get_mmc_power_mode();

    if ((_cdrom_state & POWER_CHANGED_MASK)) {
        D_MESSAGE("power");
        _cdrom_state &= ~POWER_CHANGED_MASK;
        buf[4] = 2; //PwrChg-Successful
    }

    return 8;
}

uint ATAPICdrom::notify_media(uint8_t* buf)
{
    uint16_t* ptr16;

    ptr16 = (uint16_t*)&buf[0];
    *ptr16 = revers_unit16(4);
    buf[2] = MMC_NOTIFY_MEDIA_CLASS;

    if ((_cdrom_state & EJECT_EVENT_MASK)) {
        D_MESSAGE("eject");
        buf[4] = 1;             // EjectRequest
        _cdrom_state &= ~EJECT_EVENT_MASK;
    } else if ((_cdrom_state & NEW_MEDIA_EVENT_MASK)){
        buf[4] = 2;             // NewMedia
        _cdrom_state &= ~NEW_MEDIA_EVENT_MASK;
        D_MESSAGE("new");
        if ((_cdrom_state & PERSISTENT_LOCK_MASK)) {
            D_MESSAGE("active");
            _cdrom_state |= ACTIVE_PERSISTENT_LOCK_MASK;
        }
    }

    buf[5] = (_mounted_media ? (1 << 1) : 0) |  ((_cdrom_state & TRAY_DOR_OPEN_MASK) ? 1 : 0);

    return 8;
}

void ATAPICdrom::mmc_get_event_status_notification(uint8_t* packet)
{
    if ((packet[9] & 1) /*link bit is set*/) {
        D_MESSAGE("link is set");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if (!(packet[1] & 1)) {
        D_MESSAGE("async is set");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint16_t* ptr16 = (uint16_t*)&packet[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer 0x%x 0x%x", max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint8_t request_mask = packet[4];

    #define MAX_NOTIFICTION_SIZE 8

    uint8_t buf[MAX_NOTIFICTION_SIZE];

    memset(buf, 0, sizeof(buf));

    buf[3] = MMC_NOTIFY_OPERATIONAL_MASK | MMC_NOTIFY_MEDIA_MASK | MMC_NOTIFY_POWER_MASK;
    request_mask &= buf[3];

    if (length <= 4 || !request_mask) {
        if (!request_mask) {
            D_MESSAGE("empty");
            buf[2] = 0x80;
        }
        length = MIN(length, 4);
    } else if ((request_mask & MMC_NOTIFY_OPERATIONAL_MASK) &&
                                                        (_cdrom_state & OPERATIONAL_EVENT_MASK)) {
        length = MIN(notify_operational(buf), length);
    } else if ((request_mask & MMC_NOTIFY_POWER_MASK) && (_cdrom_state & POWER_CHANGED_MASK)) {
        length = MIN(notify_power(buf), length);
    } else if ((request_mask & MMC_NOTIFY_MEDIA_MASK) &&
                                     (_cdrom_state & (NEW_MEDIA_EVENT_MASK | EJECT_EVENT_MASK))) {
        length = MIN(notify_media(buf), length);
    } else if ((request_mask & MMC_NOTIFY_OPERATIONAL_MASK)) {
        length = MIN(notify_operational(buf), length);
    } else if ((request_mask & MMC_NOTIFY_POWER_MASK)) {
        length = MIN(notify_power(buf), length);
    } else {
        length = MIN(notify_media(buf), length);
    }

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);
    memcpy(task->get_data(), buf, length);
    start_task(task);
}


void ATAPICdrom::mmc_start_stop_unit(uint8_t* packet)
{
    Lock lock(_media_lock);

    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    if ((packet[5] & 1) /*link bit is set*/) {
        D_MESSAGE("link is set");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    bool immediate = packet[1] & 1;
    uint power_condition = packet[4] >> 4;

    D_MESSAGE("power condition %u%s", power_condition, immediate ? " immediate" : "");

    switch (power_condition) {
    case 0: //no change
        switch (packet[4] & 0x3) {
        case 2: // eject
            if (!(_cdrom_state & LOCK_STATE_MASK)) {
                _open_tray();
            }
            break;
        case 3: // load
            if (!(_cdrom_state & LOCK_STATE_MASK)) {
                _close_tray();
            }
            break;
        case 0: // stop the disk
            break;
        case 1: // Start the disc and make ready for access
            set_power_mode(ATADevice::POWER_ACTIVE);
            break;
        }
        break;
    case 2: // idle
        set_power_mode(ATADevice::POWER_IDLE);
        break;
    case 3: // standbuy
        set_power_mode(ATADevice::POWER_STANDBY);
        break;
    case 5: // sleap
        set_power_mode(ATADevice::POWER_SLEEP);
        break;
    default:
        D_MESSAGE("invalid powr condition");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    packet_cmd_sucess();
}


void ATAPICdrom::mmc_mechanisim_status(uint8_t* packet)
{
    if (handle_attention_condition()) {
        D_MESSAGE("attention");
        return;
    }

    if ((packet[5] & 1) /*link bit is set*/) {
        D_MESSAGE("link is set");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }


    uint16_t* ptr16 = (uint16_t*)&packet[8];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint max_transfer = max_pio_transfer_bytes();

    if (length > max_transfer && (max_transfer & 1)) {
        D_MESSAGE("odd max_transfer 0x%x 0x%x", max_transfer, length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint8_t result[8];

    length = MIN(length, sizeof(result));

    memset(result, 0, sizeof(result));

    CDGenericTransfer* task = new (_task_storage) CDGenericTransfer(*this, length, max_transfer);
    memcpy(task->get_data(), result, length);
    start_task(task);
}


void ATAPICdrom::handle_packet(uint8_t* packet)
{
    // if an Initiator issues a command other than GET CONFIGURATION, GET EVENT STATUS
    // NOTIFICATION, INQUIRY or REQUEST SENSE while a unit attention condition exists for that
    // Initiator, the logical unit shall not perform the command and shall report CHECK CONDITION
    // status unless a higher priority status as defined by the logical unit is also pending.

    switch (packet[0]) {
    case MMC_CMD_READ_10:
        mmc_read(packet);
        break;
    case SCSI_CMD_TEST_UNIT_READY:
        scsi_test_unit_ready(packet);
        break;
    case SCSI_CMD_INQUIRY:
        scsi_inquiry(packet);
        break;
    case SCSI_CMD_MODE_SENSE:
        scsi_mode_sense(packet);
        break;
    case MMC_CMD_READ_TOC:
        mmc_read_toc(packet);
        break;
    case MMC_CMD_READ_CAPACITY:
        mmc_read_capacity(packet);
        break;
    case SCSI_CMD_REQUEST_SENSE:
        scsi_request_sens(packet);
        break;
    case MMC_CMD_GET_CONFIGURATION:
        mmc_get_configuration(packet);
        break;
    case MMC_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        mmc_prevent_allow_removal(packet);
        break;
    case MMC_CMD_GET_EVENT_STATUS_NOTIFICATION:
        mmc_get_event_status_notification(packet);
        break;
    case MMC_CMD_START_STOP_UNIT:
        mmc_start_stop_unit(packet);
        break;
    case MMC_CMD_MECHANISM_STATUS:
        mmc_mechanisim_status(packet);
        break;
    case MMC_CMD_SEND_EVENT:
        D_MESSAGE("MMC_CMD_SEND_EVENT is part of the morphing set, aborting");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_READ_CD:
        if (_profile == MMC_PROFILE_CDROM) {
            D_MESSAGE("MMC_CMD_READ_CD is part of the read cd set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_READ_CD_MSF:
        if (_profile == MMC_PROFILE_CDROM) {
            D_MESSAGE("MMC_CMD_READ_CD_MSF is part of the read cd set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
     case MMC_CMD_READ_12:
        if (_profile == MMC_PROFILE_DVDROM) {
            D_MESSAGE("MMC_CMD_READ_12 is part of the read dvd set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_READ_DVD_STRUCT:
        if (_profile == MMC_PROFILE_DVDROM) {
            D_MESSAGE("MMC_CMD_READ_DVD_STRUCT is part of the read dvd set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_GET_PERFORMANCE:
        if (_profile == MMC_PROFILE_DVDROM) {
            D_MESSAGE("MMC_CMD_GET_PERFORMANCE is part of the rt stream set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_SET_READ_AHEAD:
        if (_profile == MMC_PROFILE_DVDROM) {
            D_MESSAGE("MMC_CMD_SET_READ_AHEAD is part of the rt stream set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_SET_SPEED:
        if (_profile == MMC_PROFILE_DVDROM) {
            D_MESSAGE("MMC_CMD_SET_SPEED is part of the rt stream set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_SET_STREAMING:
        if (_profile == MMC_PROFILE_DVDROM) {
            D_MESSAGE("MMC_CMD_SET_STREAMING is part of the rt stream set, aborting");
        }
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case SCSI_CMD_MODE_SELECT:
        D_MESSAGE("SCSI_CMD_MODE_SELECT is part of the core set, aborting");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    case MMC_CMD_READ_DISC_INFORMATION:
    case MMC_CMD_REPORT_KEY:
    case MMC_CMD_READ_SUBCHANNEL:
    case MMC_CMD_SYNCHRONIZE_CACHE:
    case MMC_CMD_PAUSE_RESUME:
    case SCSI_CMD_MODE_SENSE_6:
        D_MESSAGE("abort command 0x%x", packet[0]);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    default:
        W_MESSAGE("unhandled command 0x%x", packet[0]);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
    }
}


void ATAPICdrom::do_identify_packet_device()
{
    start_task(new (_task_storage) CDIdentifyTask(*this));
}


void ATAPICdrom::do_packet_command()
{
    start_task(new (_task_storage) PacketTask(*this));
}


void ATAPICdrom::do_device_reset()
{
    cancel_task();

    _cdrom_state &= ~LOCK_STATE_MASK;

    drop();
    set_signature();

     // The device shall clear bit 7 in the ERROR register to zero. The device shall clear
    // bits 6, 5, 4, 3, 2, and 0 in the Status register to zero.

    _status = 0;
    _error &= ~(1 << 7); // or DIAGNOSTIC_D0_OK_D1_NOT_PRESENT (doc conflict)
}


void ATAPICdrom::do_command(uint8_t command)
{
    // in device 0 only configurations: a write to the command register shall be ignored,
    // except for EXECUTE DEVICE DIAGNOSTIC;
    if (current() == 1 &&  command != ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC) {
        D_MESSAGE("ignoring command 0x%x", command);
        //show_io = true;
        // drop ???
        // clear error ???
        //dump_page0();
        //for (;;) sleep(3);
        return;
    }

    // For all commands except DEVICE RESET, this register shall only be written when BSY
    // and DRQ are both cleared and DMACK- is not asserted
    if ((command != ATA_CMD_DEVICE_RESET) &&
         ((_status & (ATA_STATUS_BUSY_MASK | ATA_STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/))  {
        // by defenition result is indeterminate
        D_MESSAGE("drop command 0x%x status 0x%x", command, _status);
        return;
    }

    if (!(_status & ATA_STATUS_READY_MASK) && command != ATA_CMD_PACKET &&
                                              command != ATA_CMD_IDENTIFY_PACKET_DEVICE) {
        command_abort_error();
        return;
    }

    //  For a device in the Sleep mode, writing of the Command register shall be
    // ignored except for writing of the DEVICE RESET command to a device that implements the
    // PACKET Command
    if (get_power_mode() == POWER_SLEEP && command != ATA_CMD_DEVICE_RESET) {
        D_MESSAGE("drop command 0x%x while in sleep mode", command);
        return;
    }

    _status &= ATA_STATUS_READY_MASK;
    _error = 0;

    ATA_LOG("command: %s (0x%x)", command_name(command), command);

    drop();

    switch (command) {
    case ATA_CMD_PACKET:
        do_packet_command();
        break;
    case ATA_CMD_SET_FEATURES:
        do_set_features();
        break;
    case ATA_CMD_READ_SECTORS:
        command_abort_error();
        _lba_mid = 0x14;
        _lba_high = 0xeb;
        break;
    case ATA_CMD_IDENTIFY_DEVICE:
        command_abort_error();
        set_signature();
        break;
    case ATA_CMD_IDENTIFY_PACKET_DEVICE:
        do_identify_packet_device();
        break;
    case ATA_CMD_DEVICE_RESET:
        do_device_reset();
        break;
#if 0
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
         if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_nop();
        }
        break;
#endif
    case ATA_CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS:
    case ATA_CMD_READ_SECTORS_EXT:
    case ATA_CMD_READ_VERIFY_SECTORS:
    case ATA_CMD_READ_VERIFY_SECTORS_EXT:
    case ATA_CMD_SEEK:
    case ATA_CMD_WRITE_SECTORS:
    case ATA_CMD_WRITE_SECTORS_EXT:
    case ATA_CMD_WRITE_DMA:
    case ATA_CMD_WRITE_DMA_EXT:
    case ATA_CMD_READ_DMA:
    case ATA_CMD_READ_DMA_EXT:
    case ATA_CMD_READ_MULTIPLE:
    case ATA_CMD_READ_MULTIPLE_EXT:
    case ATA_CMD_WRITE_MULTIPLE:
    case ATA_CMD_WRITE_MULTIPLE_EXT:
    case ATA_CMD_FLUSH_CACHE:
    case ATA_CMD_FLUSH_CACHE_EXT:
    case ATA_CMD_SET_MULTIPLE_MODE:
    case ATA_CMD_STANDBY_IMMEDIATE:
    case ATA_CMD_STANDBY:
    case ATA_CMD_IDLE:
    case ATA_CMD_IDLE_IMMEDIATE:
    case ATA_CMD_SLEEP:
    case ATA_CMD_CHECK_POWER_MODE:
        command_abort_error();
        break;
    default:
        D_MESSAGE("unhandled 0x%x %u", command, command);
        command_abort_error();
    }
}


bool ATAPICdrom::is_tray_locked()
{
    return (_cdrom_state & (LOCK_STATE_MASK | ACTIVE_PERSISTENT_LOCK_MASK)) || _media_refs.val();
}


bool ATAPICdrom::is_tray_open()
{
    return !!(_cdrom_state & TRAY_DOR_OPEN_MASK);
}


void ATAPICdrom::_open_tray()
{
    D_MESSAGE("");

    if (_cdrom_state & TRAY_DOR_OPEN_MASK) {
        return;
    }

    D_MESSAGE("do");

    _cdrom_state |= TRAY_DOR_OPEN_MASK | OPERATIONAL_EVENT_MASK | ATTENTION_PARAMETERS_MASK;
    _cdrom_state &= ~(NEW_MEDIA_EVENT_MASK | EJECT_EVENT_MASK | ATTENTION_MEDIA_MASK |
                      ATTENTION_REMOVE_REQUEST);
    _mounted_media = NULL;
    update_profile();
}


void ATAPICdrom::_close_tray()
{
    D_MESSAGE("");

    if (!(_cdrom_state & TRAY_DOR_OPEN_MASK)) {
        return;
    }

    D_MESSAGE("do");

    _cdrom_state &= ~(TRAY_DOR_OPEN_MASK);
    _cdrom_state |= OPERATIONAL_EVENT_MASK | ATTENTION_PARAMETERS_MASK;

    if ((_mounted_media = _media.get())) {
        D_MESSAGE("new media");
        _cdrom_state |= NEW_MEDIA_EVENT_MASK | ATTENTION_MEDIA_MASK;
    }

    update_profile();
}


void ATAPICdrom::eject_button_press()
{
    RLock state_lock(get_nox().get_state_lock());

    if (get_state() != VMPart::RUNNING) {
        return;
    }

    Lock lock(_media_lock);

    if (is_tray_locked()) {
        D_MESSAGE("locked");

        if (!is_tray_open()) {
            D_MESSAGE("event");
            _cdrom_state |= EJECT_EVENT_MASK | ATTENTION_REMOVE_REQUEST;
        }

        return;
    }

    if (is_tray_open()) {
        _close_tray();
    } else {
        _open_tray();
    }
}


void ATAPICdrom::open_tray()
{
    RLock state_lock(get_nox().get_state_lock());

    if (get_state() != VMPart::RUNNING) {
        D_MESSAGE("not running");
        return;
    }

    Lock lock(_media_lock);

    if (is_tray_open()) {
        return;
    }

    if (is_tray_locked()) {
        D_MESSAGE("event");
        _cdrom_state |= EJECT_EVENT_MASK | ATTENTION_REMOVE_REQUEST;
        return;
    }

    _open_tray();
}


void ATAPICdrom::set_media(const std::string& file_name)
{
    RLock state_lock(get_nox().get_state_lock());

    if (get_state() != VMPart::RUNNING) {
        D_MESSAGE("not running");
        return;
    }

    Lock lock(_media_lock);

    if (!is_tray_open()) {
        D_MESSAGE("closed");
        return;
    }

    _media.reset(NULL);

    if (file_name == "") {
        D_MESSAGE("remove media");
        return;
    }

    try {
        _media.reset(new PBlockDevice(file_name, MMC_CD_SECTOR_SIZE, true));

        if (_media->get_size() > MAX_MEDIA_SIZE) {
            W_MESSAGE("size exceed dvd size");
            _media.reset(NULL);
            return;
        }

    } catch (...) {
        D_MESSAGE("failed");
        return;
    }

    if (!(_cdrom_state & LOCK_STATE_MASK)) {
        _close_tray();
    }

    return;
}


ATAPICdromFactory::ATAPICdromFactory(const std::string& file_name)
    : _file_name (file_name)
{
}


ATADevice* ATAPICdromFactory::creat_device(VMPart &owner, Wire &wire)
{
    return new ATAPICdrom(owner, wire, _file_name);
}

