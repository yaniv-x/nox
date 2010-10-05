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

#include "ata_controller.h"
#include "pic.h"


//Command Block registers include the
//  LBA High,
//  LBA Mid,
//  Device,
//  Sector Count,
//  Command,
//  Status,
//  Features,
//  Error,
//  Data


/*
DMACK- (DMA acknowledge)

*/




#include "nox_vm.h"

enum {
    IO_CONTROL = 0x03f6,

    IO_BASE = 0x01f0,
    IO_DATA = 0,
    IO_ERROR, IO_FEATURE = IO_ERROR,
    IO_SECTOR_COUNT,
    IO_LBA_LOW,         // IO_SECTOR_POS = ID_LBA_LOW,
    IO_LBA_MID,         // IO_CYL_LOW = ID_LBA_MID,
    IO_LBA_HIGH,        // IO_CYL_HIGH = ID_LBA_HIGH,
    IO_DEVICE,          // IO_HEAD = ID_DEVICE,
    IO_COMMAND, IO_STATUS = IO_COMMAND,
    IO_NUM_PORTS,

    CONTROL_DISABLE_INTERRUPT_MASK = (1 << 1),
    CONTROL_RESET_MASK = (1 << 2),
    CONTROL_HOB_MASK = (1 << 7),                                            //HOB

    STATUS_BUSY_MASK = (1 << 7), // is exclusive                            //BSY
    STATUS_READY_MASK = (1 << 6),                                           //DRDY
    STATUS_DATA_REQUEST_MASK = (1 << 3), //expeting data from host          //DRQ
    STATUS_ERROR_MASK = (1 << 0),                                           //ERR

    INTERNAL_STATUS_RESET_MASK = (1 << 8),
    INTERNAL_STATUS_IDENTIFY = (1 << 9),
    INTERNAL_STATUS_IRQ_PEANDING = (1 << 10),


    ERROR_ABORT = (1 << 2),                                                 //ABRT

    POWER_STANDBY = 0x00,
    POWER_IDLE = 0x80,
    POWER_ACTIVE = 0xFF,

    DIAGNOSTIC_D0_OK_D1_NOT_PRESENT = 0x01,
    DIAGNOSTIC_D0_FAILED_D1_NOT_PRESENT = 0x02,

    DEVICE_MUST_BE_ONE_MASK = 0,//(1 << 7) | (1 << 5),
};

ATAController::ATAController(NoxVM& nox, uint irq)
    : VMPart("ata", nox)
    , _irq (pic->wire(*this, irq))
{
    IOBus& bus = nox.get_io_bus();

    add_io_region(bus.register_region(*this, IO_BASE, IO_NUM_PORTS, this,
                                      (io_read_byte_proc_t)&ATAController::io_read,
                                      (io_write_byte_proc_t)&ATAController::io_write,
                                      (io_read_word_proc_t)&ATAController::io_read_word,
                                      (io_write_word_proc_t)&ATAController::io_write_word));

    add_io_region(bus.register_region(*this, IO_CONTROL, 1, this,
                                      (io_read_byte_proc_t)&ATAController::io_alt_status,
                                      (io_write_byte_proc_t)&ATAController::io_control));
}


void ATAController::reset()
{
    _status = 0;
    _control = 0;
    _error = 0;
    _device = 0;
    _count = 0;
    _lba_low = 0;
    _lba_mid = 0;
    _lba_high = 0;
    _feature = 0;
    _data_in = _data_in_end = 0;

    _irq->drop();
}

void ATAController::soft_reset()
{
    reset();
    _status = STATUS_BUSY_MASK | INTERNAL_STATUS_RESET_MASK;
}


uint8_t ATAController::io_alt_status(uint16_t port)
{

    return _status;
}

inline void ATAController::raise()
{
    if (_control & CONTROL_DISABLE_INTERRUPT_MASK) {
        return;
    }

    _irq->raise();
}

void ATAController::io_control(uint16_t port, uint8_t val)
{
    // This register shall only be written when DMACK- is not asserted.
    Lock lock(_mutex);

    _control = val;

    if (val & CONTROL_RESET_MASK) {
         if ((_status & INTERNAL_STATUS_RESET_MASK) && (_status & STATUS_BUSY_MASK)) {
             return;
         }

         soft_reset();

         return;
    } else if (_status & INTERNAL_STATUS_RESET_MASK) {
        set_signature();
        _status &= ~(STATUS_BUSY_MASK | INTERNAL_STATUS_RESET_MASK);
        _status |= STATUS_READY_MASK;
    }
}


enum {
    CMD_CHECK_POWER_MODE = 0xe5,
    CMD_DEVICE_CONFIGURATION = 0xb1,
    CMD_DEVICE_RESET = 0x08, //Use prohibited when the PACKET Command feature set is not implemented
    CMD_DOWNLOAD_MICROCODE = 0x92, //optional
    CMD_EXECUTE_DEVICE_DIAGNOSTIC = 0x90,
    CMD_FLUSH_CACHE = 0xe7,
    CMD_FLUSH_CACHE_EXT = 0xea,

    CMD_GET_MEDIA_STATUS = 0xda,    //Mandatory for devices implementing the Removable Media Status
                                    //Notification feature set.
                                    //Optional for devices implementing the Removable Media feature
                                    //set.

    CMD_IDENTIFY_DEVICE = 0xec,
};


void ATAController::command_abort_error()
{
    _status |= STATUS_ERROR_MASK;
    _error = ERROR_ABORT;

    raise();
}


void ATAController::clear_HOB()
{
    _control &= ~CONTROL_HOB_MASK;
}


enum {
    CONFIGURATION_RESTORE = 0xc0,
    CONFIGURATION_FREEZE_LOCK,
    CONFIGURATION_IDENTIFY,
    CONFIGURATION_SET,
};


void ATAController::set_config()
{
    /*switch (_features) {
    case CONFIGURATION_RESTORE:
        break;
    }*/
    D_MESSAGE("unhandled");
    for (;;) sleep(2);
}


void ATAController::set_signature()
{
    _count = 1;
    _lba_low = 1;
    _lba_mid = 0;
    _lba_high = 0;
    _device = 0;
}

enum {

    ATA3_MAX_CYL = 16383,
    ATA3_MAX_HEAD = 16,
    ATA3_MAX_SEC = 63,

    ID_OFFSET_GENERAL_CONF = 0,
    COMPAT_ID_GENERAL_CONF_NOT_REMOVABLE_MASK = (1 << 6),

    COMPAT_ID_OFFSET_CYL = 1,

    ID_OFFSET_SPECIFIC_CONF = 2,

    COMPAT_ID_OFFSET_HEAD = 3,
    COMPAT_ID_OFFSET_SECTORS = 6,

    ID_OFFSET_SERIAL = 19,
    ID_SERIAL_SIZE = 20,

    ID_OFFSET_REVISION = 23,
    ID_REVISION_SIZE = 8,

    ID_OFFSET_MAX_SECTORS_PER_BLOCK = 47,



    ID_OFFSET_CAP1 = 49,
    ID_CAP1_DMA_MASK = (1 << 8),
    ID_CAP1_LBA_MASK = (1 << 9),
    ID_CAP1_DISABLE_IORDY_MASK = (1 << 10),
    ID_CAP1_IORDY_MASK = (1 << 11),
    ID_OFFSET_CAP2 = 50,
    ID_CAP2_MUST_SET = (1 << 14),
    ID_OFFSET_FIELD_VALIDITY = 53,
    ID_FIELD_VALIDITY_64_70 = (1 << 1),
    ID_FIELD_VALIDITY_88 = (1 << 2),

    ID_OFFSET_MULTIPLE_SETTING = 59,
    ID_MULTIPLE_SETTING_ACTIVE_MASK = (1 << 8),
    ID_MULTIPLE_VAL_MASK = (1 << 8) - 1,

    ID_OFFSET_ADDRESABEL_SECTORS = 60,
    ID_OFFSET_NULTI_DMA = 63,
    ID_NULTI_DMA_MODE0_MASK = (1 << 0),
    ID_NULTI_DMA_MODE1_MASK = (1 << 1),
    ID_NULTI_DMA_MODE2_MASK = (1 << 2),
    ID_NULTI_DMA_MODE0_SELECT_MASK = (1 << 8),
    ID_NULTI_DMA_MODE1_SELECT_MASK = (1 << 9),
    ID_NULTI_DMA_MODE2_SELECT_MASK = (1 << 10),


    ID_OFFSET_PIO = 64,
    IO_PIO_MODE3_MASK = (1 << 0),
    IO_PIO_MODE4_MASK = (1 << 1),

    ID_OFFSET_QUEUE_DEPTH = 75,

    ID_OFFSET_VERSION = 80,
    ID_VERSION_ATA3_MASK = (1 << 3),
    ID_VERSION_ATA4_MASK = (1 << 4),
    ID_VERSION_ATA5_MASK = (1 << 5),
    ID_VERSION_ATA6_MASK = (1 << 6),

    ID_OFFSET_CMD_SET_1 = 82,
    ID_OFFSET_CMD_SET_1_ENABLE = 85,
    ID_CMD_SET_1_NOP_MASK = (1 << 14),
    ID_CMD_SET_1_WRITE_CACHE = (1 << 5),
    ID_CMD_SET_1_POWR_MANAG = (1 << 3),

    ID_OFFSET_CMD_SET_2 = 83,
    ID_OFFSET_CMD_SET_2_ENABLE = 86,
    ID_CMD_SET_2_ONE_MASK = (1 << 14),
    ID_CMD_SET_2_FLUSH_EXT_MASK = (1 << 13),
    ID_CMD_SET_2_FLUSH_MASK = (1 << 12),
    ID_CMD_SET_2_48BIT_MASK = (1 << 10),
    ID_CMD_SET_2_POWERUP_IN_STANDBY_MASK = (1 << 5),
    ID_CMD_SET_2_ADVANCE_POWR_MANAG_MASK = (1 << 3),
    ID_CMD_SET_2_QUAD_MASK = (1 << 1),

    ID_OFFSET_CMD_SET_3 = 84,
    ID_OFFSET_CMD_SET_3_ENABLE = 87,
    ID_CMD_SET_3_ONE_MASK = (1 << 14),

    ID_OFFSET_UDMA = 88,
    ID_UDMA_MODE0_MASK = (1 << 0),
    ID_UDMA_MODE1_MASK = (1 << 1),
    ID_UDMA_MODE2_MASK = (1 << 2),
    ID_UDMA_MODE3_MASK = (1 << 3),
    ID_UDMA_MODE4_MASK = (1 << 4),
    ID_UDMA_MODE5_MASK = (1 << 5),
    ID_UDMA_MODE0_SELECT_MASK = (1 << 8),
    ID_UDMA_MODE1_SELECT_MASK = (1 << 9),
    ID_UDMA_MODE2_SELECT_MASK = (1 << 10),
    ID_UDMA_MODE3_SELECT_MASK = (1 << 11),
    ID_UDMA_MODE4_SELECT_MASK = (1 << 12),
    ID_UDMA_MODE5_SELECT_MASK = (1 << 13),

    ID_OFFSET_ADVANCE_POWR_MANAG = 91,
    ID_ADVANCE_POWR_MANAG_MIN_CONSUMTION = 0x01,
    ID_ADVANCE_POWR_MANAG_MAX_PERFORMANCE = 0xfe,

    ID_OFFSET_HRESET = 93,
    ID_HRESET_ONE_MASK = (1 << 14) | (1 << 8),
    ID_HRESET_PDIAG_MASK = (1 << 9),
    ID_HRESET_JUMPER_MASK = (1 << 11),

    ID_OFFSET_ADDR_SECTORS_48 = 100,

    ID_OFFSET_INTEGRITY = 255,
    ID_INTEGRITY_SIGNATURE = 0xa5,
};


uint64_t ATAController::get_num_sectors()
{
    D_MESSAGE("for now");
    return 1 * GB / 512;
}


static int8_t checksum8(void *start, uint size)
{
    uint8_t res = 0;
    uint8_t *now = (uint8_t*)start;
    uint8_t *end = now + size;

    for (; now < end; now++) {
        res += *now;
    }

    return -res;
}

void ATAController::identify_device()
{
    memset(_identity, 0, sizeof(_identity));

    _identity[ID_OFFSET_GENERAL_CONF] = 0;
    _identity[ID_OFFSET_SPECIFIC_CONF] = 0xc837; // Device does not require SET FEATURES subcommand
                                                // to spin-up after power-up and IDENTIFY DEVICE
                                                // response is complete

    char* ch = (char*)&_identity[ID_OFFSET_SERIAL];
    int i = 0;

    for (i = 0; i < ID_SERIAL_SIZE; i++) {
        ch[i] = '0' + i;
    }

    ch = (char*)&_identity[ID_OFFSET_REVISION];
    for (i = 0; i < ID_REVISION_SIZE; i++) {
        ch[i] = 'a' + i;
    }

    _identity[ID_OFFSET_GENERAL_CONF] = COMPAT_ID_GENERAL_CONF_NOT_REMOVABLE_MASK;

    uint64_t sectors = get_num_sectors();
    uint64_t cyl = sectors / (ATA3_MAX_HEAD * ATA3_MAX_SEC);
    _identity[COMPAT_ID_OFFSET_CYL] = (cyl > ATA3_MAX_CYL) ? ATA3_MAX_CYL : cyl;
    _identity[COMPAT_ID_OFFSET_HEAD] = ATA3_MAX_HEAD;
    _identity[COMPAT_ID_OFFSET_SECTORS] = ATA3_MAX_SEC;

    _identity[ID_OFFSET_MAX_SECTORS_PER_BLOCK] = 0x8000 | (4096 / 512);
    _identity[ID_OFFSET_CAP1] = ID_CAP1_DMA_MASK |
                                ID_CAP1_LBA_MASK |
                                ID_CAP1_IORDY_MASK |
                                ID_CAP1_DISABLE_IORDY_MASK;

    _identity[ID_OFFSET_CAP2] = ID_CAP2_MUST_SET;
    _identity[ID_OFFSET_FIELD_VALIDITY] = ID_FIELD_VALIDITY_64_70 | ID_FIELD_VALIDITY_88;

    uint32_t sectors_28 = (sectors > (1 << 28) - 1) ? (1 << 28) - 1 : sectors;

    //*(uint32_t*)&_identity[ID_OFFSET_ADDRESABEL_SECTORS] = sectors_28;
    _identity[ID_OFFSET_ADDRESABEL_SECTORS] = sectors_28;
    _identity[ID_OFFSET_ADDRESABEL_SECTORS + 1] = sectors_28 >> 16;

    _identity[ID_OFFSET_NULTI_DMA] = ID_NULTI_DMA_MODE0_MASK |
                                     ID_NULTI_DMA_MODE1_MASK |
                                     ID_NULTI_DMA_MODE2_MASK |
                                     ID_NULTI_DMA_MODE0_SELECT_MASK;

    _identity[ID_OFFSET_PIO] = IO_PIO_MODE3_MASK | IO_PIO_MODE4_MASK;


    _identity[ID_OFFSET_VERSION] = ID_VERSION_ATA3_MASK |
                                   ID_VERSION_ATA4_MASK |
                                   ID_VERSION_ATA5_MASK |
                                   ID_VERSION_ATA6_MASK;

    _identity[ID_OFFSET_CMD_SET_1] = ID_CMD_SET_1_NOP_MASK |
                                     ID_CMD_SET_1_WRITE_CACHE |
                                     ID_CMD_SET_1_POWR_MANAG;
    _identity[ID_OFFSET_CMD_SET_1_ENABLE] = ID_CMD_SET_1_NOP_MASK | ID_CMD_SET_1_POWR_MANAG;

    _identity[ID_OFFSET_CMD_SET_2] = ID_CMD_SET_2_ONE_MASK |
                                     ID_CMD_SET_2_FLUSH_EXT_MASK |
                                     ID_CMD_SET_2_FLUSH_MASK |
                                     ID_CMD_SET_2_48BIT_MASK |
                                     ID_CMD_SET_2_POWERUP_IN_STANDBY_MASK |
                                     ID_CMD_SET_2_ADVANCE_POWR_MANAG_MASK |
                                     ID_CMD_SET_2_QUAD_MASK;
    _identity[ID_OFFSET_CMD_SET_2_ENABLE] = _identity[ID_OFFSET_CMD_SET_2];

    _identity[ID_OFFSET_CMD_SET_3] = ID_CMD_SET_3_ONE_MASK;
    _identity[ID_OFFSET_CMD_SET_3_ENABLE] = _identity[ID_OFFSET_CMD_SET_3];

    _identity[ID_OFFSET_UDMA] = ID_UDMA_MODE0_MASK |
                                ID_UDMA_MODE1_MASK |
                                ID_UDMA_MODE2_MASK |
                                ID_UDMA_MODE3_MASK |
                                ID_UDMA_MODE4_MASK |
                                ID_UDMA_MODE5_MASK |
                                ID_UDMA_MODE0_SELECT_MASK;

    _identity[ID_ADVANCE_POWR_MANAG_MAX_PERFORMANCE] = ID_ADVANCE_POWR_MANAG_MAX_PERFORMANCE;

    _identity[ID_OFFSET_HRESET] = ID_HRESET_ONE_MASK |
                                  ID_HRESET_PDIAG_MASK |
                                  ID_HRESET_JUMPER_MASK;


    _identity[ID_OFFSET_ADDR_SECTORS_48] = (sectors > (1ULL << 48) - 1) ? (1ULL << 48) - 1 :
                                                                           sectors;

    _identity[ID_OFFSET_INTEGRITY] = ID_INTEGRITY_SIGNATURE;
    _identity[ID_OFFSET_INTEGRITY] |= checksum8(_identity, sizeof(_identity)) << 8;

    uint8_t test = checksum8(_identity, sizeof(_identity));
    D_MESSAGE("checksum test %u", (uint)test);

    _status |= STATUS_DATA_REQUEST_MASK | INTERNAL_STATUS_IDENTIFY;

    _data_in = _identity;
    _data_in_end = _data_in + 256;
    raise();
}

void ATAController::do_command(uint8_t command)
{
    // For all commands except DEVICE RESET, this register shall only be written when BSY
    // and DRQ are both cleared and DMACK- is not asserted

    //  For a device in the Sleep mode, writing of the Command register shall be
    // ignored (except DEVICE RESET)

    _status &= ~STATUS_ERROR_MASK;

    _irq->drop();

    switch (command) {
    case CMD_FLUSH_CACHE_EXT:
    case CMD_FLUSH_CACHE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            //...
        }
        break;
    case CMD_IDENTIFY_DEVICE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            identify_device();
        }
        break;
    case CMD_CHECK_POWER_MODE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            _status = STATUS_READY_MASK;
            _count = POWER_ACTIVE;
            raise();
        }
        break;
    case CMD_DEVICE_CONFIGURATION: //Mandatory when the Device Configuration Overlay feature set is
                                   //implemented.
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            set_config();
        }
        break;
    case CMD_EXECUTE_DEVICE_DIAGNOSTIC:
        // If the host issues an EXECUTE DEVICE DIAGNOSTIC command while a device is in or going to
        // a power management mode except Sleep, then the device shall execute the EXECUTE DEVICE
        // DIAGNOSTIC sequence.

        _error = DIAGNOSTIC_D0_OK_D1_NOT_PRESENT;
        _status = STATUS_READY_MASK;
        raise();

        break;
    default:
        D_MESSAGE("unhandled 0x%x %u", command, command);
        command_abort_error();
        for (;;) sleep(2);
    }
}


uint8_t ATAController::io_read(uint16_t port)
{
    port -= IO_BASE;
    switch (port) {
    case IO_STATUS: {
        Lock lock(_mutex);
        _irq->drop();
        return _status;
    }
    case IO_ERROR:
        return _error;
    case IO_DEVICE:
        return _device;
    case IO_SECTOR_COUNT:
        return _count;
    case IO_LBA_LOW:
        return _lba_low;
    case IO_LBA_MID:
        return _lba_mid;
    case IO_LBA_HIGH:
        return _lba_high;
    default:
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }

    return 0xff;
}

void ATAController::io_write(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    if ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
        // by defenition result is indeterminate
        W_MESSAGE("drop port 0x%x val 0x%x _status 0x%x", port, val, _status);
        return;
    }

    // abort on device not ready

    port -= IO_BASE;

    switch (port) {
    case IO_COMMAND:
        do_command(val);
        return;
    case IO_SECTOR_COUNT:
        _count = (_count << 8) | val;
        return;
    case IO_LBA_LOW:
        _lba_low = (_lba_low<< 8) | val;
        return;
    case IO_LBA_MID:
        _lba_mid = (_lba_mid << 8) | val;
        return;
    case IO_LBA_HIGH:
        _lba_high = (_lba_high << 8) | val;
        return;
    case IO_FEATURE:
        _feature = val;
        return;
    case IO_DEVICE:
        _device = val | DEVICE_MUST_BE_ONE_MASK;
        return;
    default:
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }

    clear_HOB();
}


uint16_t ATAController::io_read_word(uint16_t port)
{
    port -= IO_BASE;

    if (port != IO_DATA) {
        W_MESSAGE("waiting 0x%x %u", port, port);
        return ~0;
    }

    if (_data_in < _data_in_end) {
        uint16_t val = *_data_in;

        if (++_data_in == _data_in_end) {
            _status &= ~STATUS_DATA_REQUEST_MASK;
        }

        return val;
    }

    W_MESSAGE("IO_DATA waiting 0x%x %u", port, port);
    for (;;) sleep(2);

    return 0;
}

void ATAController::io_write_word(uint16_t port, uint16_t val)
{
    port -= IO_BASE;

    if (port != IO_DATA) {
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }

    W_MESSAGE("IO_DATA waiting 0x%x %u", port, port);
    for (;;) sleep(2);
}

