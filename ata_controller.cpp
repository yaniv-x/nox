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
#include "disk.h"
#include "io_bus.h"
#include "pci.h"
#include "pci_device.h"
#include "pci_bus.h"


/* todo:
    6.11.3 Power modes
*/


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


/* ATA test results using my test box

ATA0 - single device
D0 => 0x02 0x00 0x00 0x54 0x91 0x00 0xe0 0x50
D1 => 0x02 0x00 0x00 0x54 0x91 0x00 0xf0 0x00

ATA1 - single device
D0 => 0x00 0x00 0x03 0x00 0x00 0x08 0xa0 0x50
D1 => 0x7f 0x04 0x7f 0x7f 0x00 0x00 0xb0 0x01

ATA1 - no device
D0 => 0x21 0x21 0x21 0x21 0x21 0x21 0x21 0x21
D1 => 0x31 0x31 0x31 0x31 0x31 0x31 0x31 0x31


the following result using my desktop

ATA0 - no device
D0 => 0x7f 0x7f 0x7f 0x7f 0x7f 0x7f 0x7f 0x7f
D1 => 0x7f 0x7f 0x7f 0x7f 0x7f 0x7f 0x7f 0x7f

ATA1 - no device
D0 => 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0x7f
D1 => 0xff 0xff 0xff 0xff 0xff 0xff 0xff 0x7f

*/



#include "nox_vm.h"

enum {
    ATA_PCI_REVISION = 1,

    ATA0_IRQ = 14,
    ATA0_IO_BASE = 0x01f0,
    ATA0_IO_CONTROL_BASE = 0x03f6,
    ATA0_IO_CONTROL_MAP_BASE = 0x03f4,

    ATA1_IRQ = 15,
    ATA1_IO_BASE = 0x0170,
    ATA1_IO_CONTROL_BASE = 0x0376,
    ATA1_IO_CONTROL_MAP_BASE = 0x0374,

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
    STATUS_CHK_MASK = STATUS_ERROR_MASK,                                    //CHK

    INTERNAL_STATUS_RESET_MASK = (1 << 8),
    INTERNAL_STATUS_IDENTIFY = (1 << 9),
    INTERNAL_STATUS_PACKET = (1 << 10),
    INTERNAL_STATUS_MMCREAD = (1 << 11),

    REASON_CD_BIT = 0,
    REASON_IO_BIT = 1,
    REASON_REL_BIT = 2,
    REASON_TAG_SHIFT = 3,
    REASON_TAG_MASK = ~((1 << REASON_TAG_SHIFT) - 1),


    ERROR_ABORT = (1 << 2),                                                 //ABRT

    POWER_STANDBY = 0x00,
    POWER_IDLE = 0x80,
    POWER_ACTIVE = 0xFF,

    DIAGNOSTIC_D0_OK_D1_NOT_PRESENT = 0x01,
    DIAGNOSTIC_D0_FAILED_D1_NOT_PRESENT = 0x02,

    DEVICE_MUST_BE_ONE_MASK = 0,//(1 << 7) | (1 << 5),
    DEVICE_SELECT_BIT = 4,
    DEVICE_SELECT_MASK = (1 << DEVICE_SELECT_BIT),
    DEVICE_LBA_MASK = (1 << 6),
    DEVICE_ADDRESS_MASK = ((1 << 4) - 1),
};

enum {
    SECTOR_SIZE = 512,

    ATA3_MAX_CYL = 16383,
    ATA3_MAX_HEAD = 16,
    ATA3_MAX_SEC = 63,
};

enum PowerState {
    _POWER_ACTIVE,
    _POWER_SLEEP,
};


enum {
    SCSI_CMD_TEST_UNIT_READY = 0x00,
    SCSI_CMD_REQUEST_SENSE = 0x03,
    SCSI_CMD_INQUIRY = 0x12,
    SCSI_CMD_GET_CONFIGURATION = 0x46,
    SCSI_CMD_GET_EVENT_STATUS_NOTIFICATION = 0x4a,
    SCSI_CMD_MODE_SENSE = 0x5a,

    //SCSI_LOGICAL_UNIT_SHIFT = 5,
    //SCSI_LOGICAL_UNIT_OFFSET = 1,

    SCSI_SENSE_SHIFT = 4,
    SCSI_SENSE_NO_SENSE = 0x00,
    SCSI_SENSE_NOT_READY = 0x02,
    SCSI_SENSE_MEDIUM_ERROR = 0x03,
    SCSI_SENSE_ILLEGAL_REQUEST = 0x05,

    SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION = 0x0000,
    SCSI_SENSE_ADD_READ_RETRIES_EXHAUSTED = 0x1101,
    SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE = 0x2000,
    SCSI_SENSE_ADD_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE = 0x2100,
    SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB = 0x2400,
    SCSI_SENSE_ADD_MEDIUM_NOT_PRESENT = 0x3a00,

    SCSI_INQUIRY_STD_LENGTH = 36,
    SCSI_FIX_SENSE_LENGTH = 18,

    DEFAULT_PACKET_PIO_TRANSFER_SIZE = 0xfffe,
};

#define ATA_LOG(format, ...)

/*
#define ATA_LOG(format, ...) {                                     \
    std::string log_message;                                       \
    sprintf(log_message, "ATA %u:%u "format"\n",                   \
            _io_base == ATA1_IO_BASE ,current(), ## __VA_ARGS__);  \
    printf(log_message.c_str());                                   \
}
*/



class ATAController: public VMPart {
public:
    ATAController(VMPart& host, uint id, Wire& irq_wire, uint io_base, uint control_base);

    virtual void reset();
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void set_device(ATADevice* device) { _ata_device = device;}

private:
    void reset(bool cold);
    void soft_reset();
    uint8_t io_alt_status(uint16_t port);
    void io_control(uint16_t port, uint8_t val);
    uint8_t io_read(uint16_t port);
    void io_write(uint16_t port, uint8_t val);
    uint16_t io_read_word(uint16_t port);
    void io_write_word(uint16_t port, uint16_t data);
    void set_signature();
    void command_abort_error();
    void clear_HOB();
    uint8_t byte_by_HOB(uint16_t val);
    //void set_config();
    void do_identify_device();
    void do_identify_packet_device();
    void do_read_sectors_common(uint64_t start, uint64_t end);
    void do_read_sectors();
    void do_read_sectors_ext();
    void do_read_verify_sectors();
    void do_read_verify_sectors_ext();
    void do_write_sectors_common(uint64_t start, uint64_t end);
    void do_write_sectors();
    void do_write_sectors_ext();
    void do_read_multi();
    void do_read_multi_ext();
    void do_write_multi();
    void do_write_multi_ext();
    void do_read_dma();
    void do_read_dma_ext();
    void do_write_dma();
    void do_write_dma_ext();
    void do_nop();
    void do_seek();
    void do_sleep();
    void do_standby();
    void do_standby_immediate();
    void do_set_fetures();
    void do_set_multi_mode();
    void do_packet_command();
    void do_command(uint8_t command);
    void _packet_cmd_done(uint sens, uint sense_add);
    void packet_cmd_abort(uint sens, uint sens_add);
    void packet_cmd_chk(uint sens, uint sense_add);
    void packet_cmd_sucess();
    uint get_host_count();
    void standard_inquiry();
    void request_sens();
    void mode_sense();
    void get_event_status_notification();
    void get_configuration();
    void read_toc();
    void read_row_toc();
    void read_capacity();
    void mmc_read();
    void handle_packet();
    void raise();
    void drop();
    uint current();
    bool is_valid_sectors_range(uint64_t start, uint64_t end);
    uint get_sector_count();
    uint get_sector_count_ext();
    uint64_t get_num_sectors();
    uint64_t get_sector_address();
    uint64_t get_sector_address_ext();

    PowerState get_power_state() { return _POWER_ACTIVE;}
    bool is_packet_device() { return _ata_device && _ata_device->is_ATAPI();}
    int get_ata_power_state();

private:
    Mutex _mutex;
    ATADevice* _ata_device;
    Wire& _irq_wire;
    uint _io_base;      // <---------------------------------------- io base logic to atat_host base
    uint _control_base; // <---------------------------------------- io base logic to atat_host base

    union {
        uint16_t _identity[256];
        uint8_t _sector[512];
        uint8_t _buf[65536];
    };

    uint _status;
    uint _control;
    uint _error;
    uint _device_reg;
    uint _count;
    uint _lba_low;
    uint _lba_mid;
    uint _lba_high;
    uint _feature;
    uint _irq_level;

    uint _heads_per_cylinder;
    uint _sectors_per_track;
    uint16_t* _data_in;
    uint16_t* _data_in_end;
    uint64_t _next_sector;
    uint64_t _end_sector;

    uint _sense;
    uint _sense_add;

    friend class ATAHost;
};

ATAController::ATAController(VMPart& host, uint id, Wire& irq_wire, uint io_base,
                             uint control_base)
    : VMPart("ata", host)
    , _ata_device (NULL)
    , _irq_wire (irq_wire)
    , _io_base (io_base)
    , _control_base (control_base)
    , _irq_level (0)
{
}

void ATAController::reset(bool cold)
{
    if (cold) {
        _irq_wire.reset();
    } else {
        _irq_wire.drop();
    }

    _status = 0;
    _control = 0;
    _feature = 0;
    _data_in = _data_in_end = NULL;
    _next_sector = _end_sector = 0;
    _error = _ata_device ? DIAGNOSTIC_D0_OK_D1_NOT_PRESENT : DIAGNOSTIC_D0_FAILED_D1_NOT_PRESENT;
    _irq_level = 0;
    set_signature();

    _sense = SCSI_SENSE_NO_SENSE;
    _sense_add = SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION;

    _heads_per_cylinder = ATA3_MAX_HEAD;
    _sectors_per_track = ATA3_MAX_SEC;
}


void ATAController::reset()
{
    reset(true);
}


void ATAController::soft_reset()
{
    reset(false);
    _status = STATUS_BUSY_MASK | INTERNAL_STATUS_RESET_MASK;
}


uint8_t ATAController::io_alt_status(uint16_t port)
{
    if (port != _control_base) {
        ATA_LOG("alt_status: invalid port 0x%x ret 0x%x", port, ~0);
        return ~0;
    }

    //  - not valid in sleep mode
    //  - when the BSY bit is set to one, the other bits in this register shall not be used

    if (current() == 1) {
        ATA_LOG("alt_status: slave ret 0x%x", 0);
        return 0;
    }

    ATA_LOG("alt_status: 0x%x", _status);
    return _status;
}


inline uint ATAController::current()
{
    return (_device_reg >> DEVICE_SELECT_BIT) & 1;
}

inline void ATAController::raise()
{
    // nIEN is the enable bit for the device assertion of INTRQ to the host. When the nIEN bit
    // is cleared to zero, and the device is selected, INTRQ shall be enabled through a driver
    // capable of a high-impedance output state and shall be asserted or negated by the device
    // as appropriate. When the nIEN bit is set to one, or the device is not selected, the device
    // shall release the INTRQ signal.

    ATA_LOG("raise");
    _irq_level = 1;

    if ((_control & CONTROL_DISABLE_INTERRUPT_MASK) || current() == 1) {
        return;
    }

    _irq_wire.raise();
}

inline void ATAController::drop()
{
    ATA_LOG("drop");
    _irq_level = 0;
    _irq_wire.drop();
}

void ATAController::io_control(uint16_t port, uint8_t val)
{
    // in device 0 only configurations: write to the device control register shall complete as if
    // device 0 was the selected device;

    ATA_LOG("control: 0x%x", val);

    if (port != _control_base) {
        ATA_LOG("invalid port");
        return;
    }

    Lock lock(_mutex);

    // This register shall only be written when DMACK- is not asserted.
    if (false /*|| DMACK*/) {
        W_MESSAGE("DMACK");
        return;
    }

    _control = val;

    if (val & CONTROL_RESET_MASK) {

         if ((_status & INTERNAL_STATUS_RESET_MASK) && (_status & STATUS_BUSY_MASK)) {
             return;
         }

         ATA_LOG("reset start");
         soft_reset();
         return;
    }

    if (_status & INTERNAL_STATUS_RESET_MASK) {
        _status &= ~(STATUS_BUSY_MASK | INTERNAL_STATUS_RESET_MASK);
        if (_ata_device) {
            _status |= STATUS_READY_MASK | (1 << 4) /* ??? */;
        }
        ATA_LOG("reset done");
    }

    if ((_control & CONTROL_DISABLE_INTERRUPT_MASK)) {
       _irq_wire.drop();
    } else if (_irq_level) {
        raise();
    }
}


enum {
    CMD_NOP = 0x00,
    CMD_READ_SECTORS = 0x20,
    CMD_READ_SECTORS_EXT = 0x24,
    CMD_WRITE_SECTORS = 0x30,
    CMD_WRITE_SECTORS_EXT = 0x34,
    CMD_IDLE_IMMEDIATE = 0xe1,
    CMD_IDLE = 0xe3,
    CMD_CHECK_POWER_MODE = 0xe5,
    //CMD_DEVICE_CONFIGURATION = 0xb1,
    CMD_DEVICE_RESET = 0x08, //Use prohibited when the PACKET Command feature set is not implemented
    //CMD_DOWNLOAD_MICROCODE = 0x92, //optional
    CMD_EXECUTE_DEVICE_DIAGNOSTIC = 0x90,
    CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS = 0x91,

    CMD_FLUSH_CACHE = 0xe7,
    CMD_FLUSH_CACHE_EXT = 0xea,

    //CMD_GET_MEDIA_STATUS = 0xda,  //Mandatory for devices implementing the Removable Media Status
                                    //Notification feature set.
                                    //Optional for devices implementing the Removable Media feature
                                    //set.

    CMD_IDENTIFY_DEVICE = 0xec,
    CMD_PACKET = 0xa0,
    CMD_IDENTIFY_PACKET_DEVICE = 0xa1,

    CMD_READ_MULTIPLE = 0xc4,
    CMD_READ_MULTIPLE_EXT = 0x29,
    CMD_READ_DMA = 0xc8,
    CMD_READ_DMA_EXT = 0x25,
    CMD_READ_VERIFY_SECTORS = 0x40,
    CMD_READ_VERIFY_SECTORS_EXT = 0x42,
    CMD_SEEK = 0x70,
    CMD_SET_FEATURES = 0xef,
    CMD_SET_MULTIPLE_MODE = 0xc6,
    CMD_SLEEP = 0xe6,
    CMD_STANDBY = 0xe2,
    CMD_STANDBY_IMMEDIATE = 0xe0,
    CMD_WRITE_DMA = 0xca,
    CMD_WRITE_DMA_EXT = 0x35,
    CMD_WRITE_MULTIPLE = 0xc5,
    CMD_WRITE_MULTIPLE_EXT = 0x39,
};


void ATAController::command_abort_error()
{
    ATA_LOG("abort");
    _status |= STATUS_ERROR_MASK;
    _error = ERROR_ABORT;

    raise();
}


void ATAController::clear_HOB()
{
    _control &= ~CONTROL_HOB_MASK;
}


uint8_t ATAController::byte_by_HOB(uint16_t val)
{
    return (_control & CONTROL_HOB_MASK) ? val >> 8 : val;
}

/*
enum {
    CONFIGURATION_RESTORE = 0xc0,
    CONFIGURATION_FREEZE_LOCK,
    CONFIGURATION_IDENTIFY,
    CONFIGURATION_SET,
};


void ATAController::set_config()
{
    switch (_features) {
    case CONFIGURATION_RESTORE:
        break;
    }
    D_MESSAGE("unhandled");
    for (;;) sleep(2);
}
*/


void ATAController::set_signature()
{
    _count = 1;
    _lba_low = 1;

    if (is_packet_device()) {
        _lba_mid = 0x14;
        _lba_high = 0xeb;
    } else {
        _lba_mid = 0;
        _lba_high = 0;
    }

    _device_reg = 0;
}

enum {

    ID_OFFSET_GENERAL_CONF = 0,
    COMPAT_ID_GENERAL_CONF_NOT_REMOVABLE_MASK = (1 << 6),

    COMPAT_ID_OFFSET_CYL = 1,

    ID_OFFSET_SPECIFIC_CONF = 2,

    COMPAT_ID_OFFSET_HEAD = 3,
    COMPAT_ID_OFFSET_SECTORS = 6,

    ID_OFFSET_SERIAL = 10,
    ID_SERIAL_NUM_CHARS = 20,

    ID_OFFSET_REVISION = 23,
    ID_REVISION_NUM_CHARS = 8,

    ID_OFFSET_MODEL = 27,
    ID_MODEL_NUM_CHARS = 40,

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
    ID_CMD_SET_1_DEVICE_RESET = (1 << 9),
    ID_CMD_SET_1_WRITE_CACHE = (1 << 5),
    ID_CMD_SET_1_PACKET = (1 << 4),
    ID_CMD_SET_1_POWR_MANAG = (1 << 3),

    ID_OFFSET_CMD_SET_2 = 83,
    ID_OFFSET_CMD_SET_2_ENABLE = 86,
    ID_CMD_SET_2_ONE_MASK = (1 << 14),
    ID_CMD_SET_2_FLUSH_EXT_MASK = (1 << 13),
    ID_CMD_SET_2_FLUSH_MASK = (1 << 12),
    ID_CMD_SET_2_48BIT_MASK = (1 << 10),
    ID_CMD_SET_2_POWERUP_IN_STANDBY_MASK = (1 << 5),
    ID_CMD_SET_2_MEDIA_STATUS_NOTIFICTION = (1 << 4),
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
    ID_HRESET_ONE_MASK = (1 << 14) | (1 << 0),
    ID_HRESET_PDIAG_MASK = (1 << 4),
    ID_HRESET_PASS_MASK = (1 << 3),
    ID_HRESET_JUMPER_MASK = (1 << 1),

    ID_OFFSET_ADDR_SECTORS_48 = 100,

    ID_OFFSET_BYTE_COUNT_0_BEHAVIOR = 125,
    ID_OFFSET_REMOVABLE_STATUS_SUPPORT = 127,

    ID_OFFSET_INTEGRITY = 255,
    ID_INTEGRITY_SIGNATURE = 0xa5,
};


uint64_t ATAController::get_num_sectors()
{
    ASSERT(_ata_device && !(_device_reg & DEVICE_SELECT_MASK));
    return _ata_device->get_size() / SECTOR_SIZE;
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


static void set_ata_str(uint16_t* start, int len, const char* str)
{
    int n = MIN(len * 2, strlen(str));
    uint8_t* dest = (uint8_t*)start;
    int i;

    memset(dest, ' ', len * 2);

    for (i = 0; i < n; i += 2) dest[i + 1] = str[i];
    for (i = 1; i < n; i += 2) dest[i - 1] = str[i];
}


void ATAController::do_identify_device()
{
    memset(_identity, 0, sizeof(_identity));

    _identity[ID_OFFSET_GENERAL_CONF] = COMPAT_ID_GENERAL_CONF_NOT_REMOVABLE_MASK;
    _identity[ID_OFFSET_SPECIFIC_CONF] = 0xc837; // Device does not require SET FEATURES subcommand
                                                 // to spin-up after power-up and IDENTIFY DEVICE
                                                 // response is complete

    set_ata_str(&_identity[ID_OFFSET_SERIAL], ID_SERIAL_NUM_CHARS / 2, "0");
    set_ata_str(&_identity[ID_OFFSET_REVISION], ID_REVISION_NUM_CHARS / 2, "1.0.0");
    set_ata_str(&_identity[ID_OFFSET_MODEL], ID_MODEL_NUM_CHARS / 2, "Nox HD");

    uint64_t sectors = get_num_sectors();
    uint64_t cyl = sectors / (ATA3_MAX_HEAD * ATA3_MAX_SEC);
    _identity[COMPAT_ID_OFFSET_CYL] = (cyl > ATA3_MAX_CYL) ? ATA3_MAX_CYL : cyl;
    _identity[COMPAT_ID_OFFSET_HEAD] = _heads_per_cylinder = ATA3_MAX_HEAD;
    _identity[COMPAT_ID_OFFSET_SECTORS] = _sectors_per_track = ATA3_MAX_SEC;

    _identity[ID_OFFSET_MAX_SECTORS_PER_BLOCK] = 0x8000 | 1; // test the performace benefit of
                                                             // making it larger
    _identity[ID_OFFSET_CAP1] = ID_CAP1_DMA_MASK |
                                ID_CAP1_LBA_MASK |
                                ID_CAP1_IORDY_MASK |
                                ID_CAP1_DISABLE_IORDY_MASK;

    _identity[ID_OFFSET_CAP2] = ID_CAP2_MUST_SET;
    _identity[ID_OFFSET_FIELD_VALIDITY] = ID_FIELD_VALIDITY_64_70 | ID_FIELD_VALIDITY_88;

    uint32_t sectors_28 = (sectors > (1 << 28) - 1) ? (1 << 28) - 1 : sectors;

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
                                     /*ID_CMD_SET_1_WRITE_CACHE |*/
                                     ID_CMD_SET_1_POWR_MANAG;
    _identity[ID_OFFSET_CMD_SET_1_ENABLE] = ID_CMD_SET_1_NOP_MASK | ID_CMD_SET_1_POWR_MANAG;

    _identity[ID_OFFSET_CMD_SET_2] = ID_CMD_SET_2_ONE_MASK |
                                     /*ID_CMD_SET_2_FLUSH_EXT_MASK |
                                     ID_CMD_SET_2_FLUSH_MASK |*/
                                     ID_CMD_SET_2_48BIT_MASK /*|
                                     ID_CMD_SET_2_POWERUP_IN_STANDBY_MASK |
                                     ID_CMD_SET_2_ADVANCE_POWR_MANAG_MASK |
                                     ID_CMD_SET_2_QUAD_MASK*/;
    _identity[ID_OFFSET_CMD_SET_2_ENABLE] = ID_CMD_SET_2_48BIT_MASK;

    _identity[ID_OFFSET_CMD_SET_3] = ID_CMD_SET_3_ONE_MASK;
    _identity[ID_OFFSET_CMD_SET_3_ENABLE] = _identity[ID_OFFSET_CMD_SET_3];

    _identity[ID_OFFSET_UDMA] = ID_UDMA_MODE0_MASK |
                                ID_UDMA_MODE1_MASK |
                                ID_UDMA_MODE2_MASK |
                                ID_UDMA_MODE3_MASK |
                                ID_UDMA_MODE4_MASK |
                                ID_UDMA_MODE5_MASK |
                                ID_UDMA_MODE0_SELECT_MASK;

    //_identity[ID_OFFSET_ADVANCE_POWR_MANAG] = ID_ADVANCE_POWR_MANAG_MAX_PERFORMANCE;

    _identity[ID_OFFSET_HRESET] = ID_HRESET_ONE_MASK |
                                  //ID_HRESET_PDIAG_MASK |
                                  ID_HRESET_PASS_MASK |
                                  ID_HRESET_JUMPER_MASK;

    uint64_t sectors_48 = (sectors > (1ULL << 48) - 1) ? (1ULL << 48) - 1 : sectors;
    _identity[ID_OFFSET_ADDR_SECTORS_48] = sectors_48;
    _identity[ID_OFFSET_ADDR_SECTORS_48 + 1] = sectors_48 >> 16;
    _identity[ID_OFFSET_ADDR_SECTORS_48 + 2] = sectors_48 >> 32;

    _identity[ID_OFFSET_INTEGRITY] = ID_INTEGRITY_SIGNATURE;
    _identity[ID_OFFSET_INTEGRITY] |= checksum8(_identity, sizeof(_identity)) << 8;

    uint8_t test = checksum8(_identity, sizeof(_identity));
    D_MESSAGE("checksum test %u", (uint)test);

    _status |= STATUS_DATA_REQUEST_MASK | INTERNAL_STATUS_IDENTIFY;

    _data_in = _identity;
    _data_in_end = _data_in + 256;
    _next_sector = _end_sector = 0;
    raise();
}


enum {
    ID_GENERAL_CONF_ATAPI_MASK = 1 << 15,
    ID_GENERAL_COMMAND_SET_CD = 0x05,
    ID_GENERAL_COMMAND_SET_SHIFT = 8,
    ID_GENERAL_REMOVABLE_BIT = 7,
    ID_GENERAL_DRQ_LATENCY = 2,
    ID_GENERAL_DRQ_LATENCY_SHIFT = 5,

    ATAPI_PACKET_SIZE = 12,


    ATAPI_ID_CAP1_MBZ = ID_CAP1_LBA_MASK,
};

void ATAController::do_identify_packet_device()
{
    memset(_identity, 0, sizeof(_identity));

    _identity[ID_OFFSET_GENERAL_CONF] = ID_GENERAL_CONF_ATAPI_MASK;
    _identity[ID_OFFSET_GENERAL_CONF] |= ID_GENERAL_COMMAND_SET_CD << ID_GENERAL_COMMAND_SET_SHIFT;
    _identity[ID_OFFSET_GENERAL_CONF] |= 1 << ID_GENERAL_REMOVABLE_BIT;
    _identity[ID_OFFSET_GENERAL_CONF] |= ID_GENERAL_DRQ_LATENCY << ID_GENERAL_DRQ_LATENCY_SHIFT;

    _identity[ID_OFFSET_SPECIFIC_CONF] = 0xc837; // Device does not require SET FEATURES subcommand
                                                 // to spin-up after power-up and IDENTIFY DEVICE
                                                 // response is complete


    set_ata_str(&_identity[ID_OFFSET_SERIAL], ID_SERIAL_NUM_CHARS / 2, "0");
    set_ata_str(&_identity[ID_OFFSET_REVISION], ID_REVISION_NUM_CHARS / 2, "1.0.0");
    set_ata_str(&_identity[ID_OFFSET_MODEL], ID_MODEL_NUM_CHARS / 2, "Nox CD");


    _identity[ID_OFFSET_CAP1] = ID_CAP1_DMA_MASK |
                                ATAPI_ID_CAP1_MBZ |
                                ID_CAP1_IORDY_MASK |
                                ID_CAP1_DISABLE_IORDY_MASK;

    _identity[ID_OFFSET_CAP2] = ID_CAP2_MUST_SET;


    _identity[ID_OFFSET_FIELD_VALIDITY] = ID_FIELD_VALIDITY_64_70 | ID_FIELD_VALIDITY_88;

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
                                     ID_CMD_SET_1_PACKET |
                                     ID_CMD_SET_1_DEVICE_RESET |
                                     ID_CMD_SET_1_POWR_MANAG;
    _identity[ID_OFFSET_CMD_SET_1_ENABLE] = _identity[ID_OFFSET_CMD_SET_1];

    _identity[ID_OFFSET_CMD_SET_2] = ID_CMD_SET_2_ONE_MASK |
                                     ID_CMD_SET_2_MEDIA_STATUS_NOTIFICTION;
    _identity[ID_OFFSET_CMD_SET_2_ENABLE] = ID_CMD_SET_2_MEDIA_STATUS_NOTIFICTION;

    _identity[ID_OFFSET_CMD_SET_3] = ID_CMD_SET_3_ONE_MASK;
    _identity[ID_OFFSET_CMD_SET_3_ENABLE] = _identity[ID_OFFSET_CMD_SET_3];

    _identity[ID_OFFSET_UDMA] = ID_UDMA_MODE0_MASK |
                                ID_UDMA_MODE1_MASK |
                                ID_UDMA_MODE2_MASK |
                                ID_UDMA_MODE3_MASK |
                                ID_UDMA_MODE4_MASK |
                                ID_UDMA_MODE5_MASK |
                                ID_UDMA_MODE0_SELECT_MASK;

    _identity[ID_OFFSET_HRESET] = ID_HRESET_ONE_MASK |
                                  ID_HRESET_PASS_MASK |
                                  ID_HRESET_JUMPER_MASK;

    _identity[ID_OFFSET_REMOVABLE_STATUS_SUPPORT] = 1;

    _identity[ID_OFFSET_BYTE_COUNT_0_BEHAVIOR] = DEFAULT_PACKET_PIO_TRANSFER_SIZE;

    _identity[ID_OFFSET_INTEGRITY] = ID_INTEGRITY_SIGNATURE;
    _identity[ID_OFFSET_INTEGRITY] |= checksum8(_identity, sizeof(_identity)) << 8;

    _status |= STATUS_DATA_REQUEST_MASK | INTERNAL_STATUS_IDENTIFY;

    _data_in = _identity;
    _data_in_end = _data_in + 256;
    _next_sector = _end_sector = 0;
    raise();
}


void ATAController::do_nop()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_packet_command()
{
    _status |= STATUS_DATA_REQUEST_MASK | INTERNAL_STATUS_PACKET;
    _count &= REASON_TAG_MASK;
    _count |= 1 << REASON_CD_BIT;

    _data_in = (uint16_t*)_sector;
    _data_in_end = (uint16_t*)(_sector + ATAPI_PACKET_SIZE);
    _next_sector = _end_sector = 0;

    //raise();
}



/*void ATAController::packet_chk_error(uint err)
{
    _status |= STATUS_ERROR_MASK;
    _error = err;

    packet_cmd_done();
}*/


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

uint ATAController::get_host_count()
{
    uint host_count = (_lba_mid & 0xff) | ((_lba_high & 0xff) << 8);

    return host_count ? host_count : DEFAULT_PACKET_PIO_TRANSFER_SIZE;
}

void ATAController::standard_inquiry()
{
    uint16_t* ptr16 = (uint16_t*)&_sector[3];
    uint length = revers_unit16(*ptr16);

    uint host_count = get_host_count();

    if (length > host_count) {
        D_MESSAGE("length conflict");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
    }

    if (length < 5) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    length = MIN(length, SCSI_INQUIRY_STD_LENGTH);

    memset(_buf, 0, sizeof(_buf));

    _sector[0] = 5;         // MMC-4
    _sector[1] = 1 << 7;    //removable medium
    _sector[2] = 0x05;      // (SPC-3)
    _sector[3] = 3;        // response data format
    _sector[4] = SCSI_INQUIRY_STD_LENGTH - 5;

    set_scsi_left_str(&_sector[8], 8, "Nox");     // VENDOR ID
    set_scsi_left_str(&_sector[16], 16, "NoxCD");    // PRODUCT ID
    set_scsi_left_str(&_sector[32], 4, "0");    // PRODUCT REVISION

    _data_in = (uint16_t*)_sector;
    _data_in_end = (uint16_t*)(_sector + length);
    _next_sector = _end_sector = 0;

    _lba_mid = length & 0xff;
    _lba_high = length >> 8;

    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_IO_BIT);
    _status |= STATUS_DATA_REQUEST_MASK;
    raise();
}

/*void ATAController::packet_chk_error(uint err, uint sense_add)
{
    _status |= STATUS_ERROR_MASK;
    _error = err;

    packet_cmd_done();
}*/


void ATAController::_packet_cmd_done(uint sense, uint sense_add)
{
    _status &= ~(INTERNAL_STATUS_PACKET | INTERNAL_STATUS_MMCREAD);
    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_CD_BIT) | (1 << REASON_IO_BIT);
    _sense = sense;
    _sense_add = sense_add;
    raise();
}


void ATAController::packet_cmd_abort(uint sens, uint sens_add)
{
    ATA_LOG("packet_abort: 0x%x 0x%x", sens, sens_add);
    _status |= STATUS_CHK_MASK;
    _error = ERROR_ABORT | (sens << SCSI_SENSE_SHIFT);
    _packet_cmd_done(SCSI_SENSE_NO_SENSE, SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION);
}

void ATAController::packet_cmd_chk(uint sens, uint sens_add)
{
    ATA_LOG("packet_chk: 0x%x 0x%x", sens, sens_add);
    _status |= STATUS_CHK_MASK;
    _error = (sens << SCSI_SENSE_SHIFT);
    _packet_cmd_done(SCSI_SENSE_NO_SENSE, SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION);
}

void ATAController::packet_cmd_sucess()
{
    _packet_cmd_done(SCSI_SENSE_NO_SENSE, SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION);
}


void ATAController::request_sens()
{
    uint length = _sector[4];
    uint host_count = (_lba_mid & 0xff) | ((_lba_high & 0xff) << 8);

    if (host_count == 0) {
        host_count = DEFAULT_PACKET_PIO_TRANSFER_SIZE;
    }

    if (length > host_count) {
        D_MESSAGE("length conflict");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
    }


    if ((_sector[1] & 1) /* DESC*/ || (_sector[5] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    /*if (length < 18) {
        D_MESSAGE("length < 18");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }*/

    length = MIN(18, length);

    memset(_sector, 0, sizeof(_sector));
    _sector[0] = (1 << 7) | 0x70; // valid and current
    _sector[2] = _sense;
    _sector[7] = 10; // ADDITIONAL SENSE LENGTH
    _sector[12] = _sense_add >> 8;
    _sector[13] = _sense_add;

    _lba_mid = length & 0xff;
    _lba_high = length >> 8;

    _data_in = (uint16_t*)_sector;
    _data_in_end = (uint16_t*)(_sector + length);
    _next_sector = _end_sector = 0;

    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_IO_BIT);
    _status |= STATUS_DATA_REQUEST_MASK;
    raise();
}

class Buf {
public:
    Buf(uint8_t* base, uint size)
        : _base (base)
        , _now (_base)
        , _size (size)
    {
    }

    void put_uint8(uint8_t val)
    {
        if (_now - _base + 1 >= _size) {
            THROW("buf overrun");
        }

        *_now++ = val;
    }

    void put_uint16(uint16_t val)
    {
        if (_now - _base + 2 >= _size) {
            THROW("buf overrun");
        }

        *(uint16_t*)_now = val;
        _now += 2;
    }

    void put_uint32(uint32_t val)
    {
        if (_now - _base + 4 >= _size) {
            THROW("buf overrun");
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


void ATAController::mode_sense()
{
   // uint length = revers_unit16(*(uint16_t*)&_sector[7]);

    if ((_sector[1] & 1) /* DESC*/ || (_sector[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint page_code = _sector[2] & 0x3f;
    uint page_control = _sector[2] >> 6;
    uint subpage_code = _sector[3];

    switch (page_code) {
    case 0x2a: { // MM Capabilities and Mechanical Status Page

        if (page_control != 0) {
            D_MESSAGE("abort: page 0x%x control 0x%x ", page_code, page_control);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        uint16_t* ptr16 = (uint16_t*)&_sector[7];
        uint length = revers_unit16(*ptr16);

        if (length == 0) {
            packet_cmd_sucess();
            return;
        }



        Buf buf(_buf, sizeof(_buf));

        // Mode Parameters Header
        buf.put_uint16(0); // Mode Data Length
        buf.put_uint32(0); // Reserved
        buf.put_uint16(0); // Block Descriptor Length

        // Page
        buf.put_uint8(page_code); // page code and Parameters Savable is not set
        buf.put_uint8(30); // page length
        buf.put_uint8(0);  // no read support
        buf.put_uint8(0);  // no write support
        buf.put_uint8(1 | (1 << 6));  // play audio, multi session
        buf.put_uint8(0);
        buf.put_uint8(1 | (1 << 3) | (1 << 5));  // lock, eject, tray type
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

        ptr16 = (uint16_t*)_buf;
        *ptr16 = revers_unit16(buf.position() - 2);

        length = MIN(length, buf.position());
        uint host_count = get_host_count();

        if (length > host_count) {
            D_MESSAGE("not implemented buf.position() > host_count (0x%x 0x%x 0x%x)",
                      buf.position(),
                      host_count,
                      length);
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        _lba_mid = length & 0xff;
        _lba_high = length >> 8;

        _data_in = (uint16_t*)_buf;
        _data_in_end = (uint16_t*)(_buf + length);
        _next_sector = _end_sector = 0;

        _count &= REASON_TAG_MASK;
        _count |= (1 << REASON_IO_BIT);
        _status |= STATUS_DATA_REQUEST_MASK;
        raise();
        break;
    }
    case 0x1b: //Reserved
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

void ATAController::get_event_status_notification()
{
    if ((_sector[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if (!(_sector[1] & 1)) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        D_MESSAGE("async");
        return;
    }

    uint16_t* ptr16 = (uint16_t*)&_sector[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint host_count = get_host_count();

    if (length > host_count) {
        D_MESSAGE("not implemented length > host_count");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    //uint notification_request = _sector[4];


    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    if (length < 5) {
        // keep event
    }

    length = MIN(length, 4);

    memset(_sector, 0, sizeof(_sector));

    ptr16 = (uint16_t*)&_sector[0];
    *ptr16 = revers_unit16(0);
    _sector[3] = (1 << 4) || (1 << 1); // support media and operational change notifiction


    _lba_mid = length & 0xff;
    _lba_high = length >> 8;

    _data_in = (uint16_t*)_sector;
    _data_in_end = (uint16_t*)(_sector + length);
    _next_sector = _end_sector = 0;

    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_IO_BIT);
    _status |= STATUS_DATA_REQUEST_MASK;
    raise();
}

enum {
    MMC_FEATURE_PROFILE_LIST = 0x0000,
    MMC_FEATURE_CORE = 0x0001,
    MMC_FEATURE_MORPHING = 0x0002,
    MMC_FEATURE_REMOVABLE = 0x0003,
    MMC_FEATURE_RANDOM_READABLE = 0x0010,
    MMC_FEATURE_CD_READ = 0x001e,
    MMC_FEATURE_POWER_MANAGMENET = 0x0100,
    MMC_FEATURE_TIMEOUT = 0x0105,

    MMC_PROFILE_CDROM = 0x0008,

    MMC_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL = 0x1e,
    MMC_READ_CAPACITY = 0x25,
    MMC_CMD_READ = 0x28,
    MMC_CMD_READ_TOC = 0x43,

    MMC_LEADOUT_TRACK_ID = 0xaa,

    MMC_CD_SECTOR_SIZE = 2048,
    MMC_CD_FRAMES_PER_SEC = 75,
};


void ATAController::get_configuration()
{
    if ((_sector[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint16_t* ptr16 = (uint16_t*)&_sector[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    uint host_count = get_host_count();

    //length = MIN(length, sizeof(_sector));
    /*if (length > 512) {
        D_MESSAGE("length 0x%x", length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }*/

#ifdef NO_MEDIUM
    uint rt = _sector[1] & 3;

    if (rt != 0) {
        D_MESSAGE("rt == 0x%x, sleep..", rt);
        for(;;) sleep(2);
    }
#endif

    ptr16 = (uint16_t*)&_sector[2];
    uint start = revers_unit16(*ptr16);

    if (start) {
        D_MESSAGE("start 0x%x", start);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }


    memset(_buf, 0, sizeof(_buf));

    Buf buf(_sector, sizeof(_sector));

    buf.put_uint32(0); //length
    buf.put_uint8(0);  //reserved
    buf.put_uint8(0);  //reserved
#ifdef NO_MEDIUM
    buf.put_uint16(0);  //current profile
#else
    buf.put_uint16(revers_unit16(MMC_PROFILE_CDROM));
#endif

    buf.put_uint16(revers_unit16(MMC_FEATURE_PROFILE_LIST));
    buf.put_uint8(0x3); // currenr | persistent, version == 0
    buf.put_uint8(4);  // aditional length

    buf.put_uint16(revers_unit16(MMC_PROFILE_CDROM));
#ifdef NO_MEDIUM
    buf.put_uint8(0);  // not current
#else
    buf.put_uint8(1);  // current
#endif
    buf.put_uint8(0);  // reserved

    buf.put_uint16(revers_unit16(MMC_FEATURE_CORE));
    buf.put_uint8(0x7); // currenr | persistent, version == 1
    buf.put_uint8(8);  // aditional length
    buf.put_uint32(0x00000002); //interface ATAPI
    buf.put_uint8(1);  // DBE is set
    buf.put_uint8(0);  // reserved
    buf.put_uint8(0);  // reserved
    buf.put_uint8(0);  // reserved


    buf.put_uint16(revers_unit16(MMC_FEATURE_MORPHING));
    buf.put_uint8(0x7); // currenr | persistent, version == 1
    buf.put_uint8(4);  // aditional length
    buf.put_uint8(2);  // OCEvent is set
    buf.put_uint8(0);  // reserved
    buf.put_uint8(0);  // reserved
    buf.put_uint8(0);  // reserved


    buf.put_uint16(revers_unit16(MMC_FEATURE_REMOVABLE));
    buf.put_uint8(0x3); // currenr | persistent, version == 0
    buf.put_uint8(4);  // aditional length
    buf.put_uint8(1 | 8 | (1 << 5));  // LOCK | EJECT | Tray type loading mechanism
    buf.put_uint8(0);  // reserved
    buf.put_uint8(0);  // reserved
    buf.put_uint8(0);  // reserved

    buf.put_uint16(revers_unit16(MMC_FEATURE_RANDOM_READABLE));
#ifdef NO_MEDIUM
    buf.put_uint8(0); // not current | not persistent, version == 0
#else
    buf.put_uint8(0x01); // current | not persistent, version == 0
#endif
    buf.put_uint8(8); // aditional length
    buf.put_uint32(revers_unit16(MMC_CD_SECTOR_SIZE)); // logical block
    buf.put_uint16(revers_unit16(1)); // blocking
    buf.put_uint8(1); // PP is set
    buf.put_uint8(0); // reserved

    buf.put_uint16(revers_unit16(MMC_FEATURE_CD_READ));
#ifdef NO_MEDIUM
    buf.put_uint8(0x8); // not current | not persistent, version == 2
#else
    buf.put_uint8(0x9); // current | not persistent, version == 2
#endif
    buf.put_uint8(4);  // aditional length
    buf.put_uint8(0); // DAP C2 and CD-Text are not set
    buf.put_uint8(0); // reserved
    buf.put_uint8(0); // reserved
    buf.put_uint8(0); // reserved


    buf.put_uint16(revers_unit16(MMC_FEATURE_POWER_MANAGMENET));
    buf.put_uint8(0x3); // currenr | persistent, version == 0
    buf.put_uint8(0);  // aditional length


    buf.put_uint16(revers_unit16(MMC_FEATURE_TIMEOUT));
    buf.put_uint8(0x7); // currenr | persistent, version == 1
    buf.put_uint8(4);  // aditional length
    buf.put_uint8(0); // Group 3 is not set
    buf.put_uint8(0); // reserved
    buf.put_uint16(revers_unit16(0)); // unit length (When the Group3 bit is set to 0,
                                      //              Unit Length field is not valid)

    *(uint32_t*)buf.base() = revers_unit32(buf.position() - 8);


    length = MIN(length, buf.position());

    if (length > host_count) {
        D_MESSAGE("not implemented buf.position() > host_count (0x%x 0x%x 0x%x)",
                  buf.position(),
                  host_count,
                  length);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    _lba_mid = length & 0xff;
    _lba_high = length >> 8;

    _data_in = (uint16_t*)_sector;
    _data_in_end = (uint16_t*)(_sector + length);
    _next_sector = _end_sector = 0;

    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_IO_BIT);
    _status |= STATUS_DATA_REQUEST_MASK;
    raise();
}


void frames_to_time(uint num_frames, uint8_t& min, uint8_t& sec, uint8_t& frame)
{
    frame = num_frames % MMC_CD_FRAMES_PER_SEC;
    uint num_seconds = num_frames / MMC_CD_FRAMES_PER_SEC;
    min = num_seconds / 60;
    sec = num_seconds % 60;
    ASSERT(sec < 99);
}

void ATAController::read_row_toc()
{
    if ((_sector[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint start_session = _sector[6];

    uint16_t* ptr16 = (uint16_t*)&_sector[7];
    uint length = revers_unit16(*ptr16);

    if (length == 0) {
        packet_cmd_sucess();
        return;
    }

    ptr16 = (uint16_t*)&_sector[0];
    *ptr16 = 0;    // row toc length
    _sector[2] = 1;                 // first session
    _sector[3] = 1;                 // last session

    if (start_session > 1) {
        ptr16 = (uint16_t*)&_sector[0];
        *ptr16 = revers_unit16(2);    // row toc length

        length = MIN(length, 4);

        uint host_count = get_host_count();

        if (length > host_count) {
            D_MESSAGE("not implemented length > host_count");
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        _lba_mid = length & 0xff;
        _lba_high = length >> 8;

        _data_in = (uint16_t*)_sector;
        _data_in_end = (uint16_t*)(_sector + length);
        _next_sector = _end_sector = 0;

        _count &= REASON_TAG_MASK;
        _count |= (1 << REASON_IO_BIT);
        _status |= STATUS_DATA_REQUEST_MASK;
        raise();
        return;
    }

    _sector[4] = 1;                                 // session number
    _sector[5] = 0x14;  // control = Data track, recorded uninterrupted
    _sector[6] = 0;     //TNO;
    _sector[7] = 1;     //POINT ; track 1
    frames_to_time(_ata_device->get_size() / MMC_CD_SECTOR_SIZE,
                   _sector[8], _sector[9], _sector[10]); // running time
    _sector[11] = 0; //ZERO
    _sector[12] = 0; // track start MIN
    _sector[13] = 0; // track start SEC
    _sector[14] = 0; // track start FRAME


    _sector[15] = 1;                                 // session number
    _sector[16] = 0x14;  // control = Data track, recorded uninterrupted
    _sector[17] = 0;     //TNO;
    _sector[18] = 0xa0;     //POINT ; lead-in
    _sector[19] = 0; // lead-in running time MIN
    _sector[20] = 0; // lead-in running time SEC
    _sector[21] = 0; // lead-in running time FRAME
    _sector[22] = 0; //ZERO
    _sector[23] = 1; // first track in program area
    _sector[24] = 0; // program area format
    _sector[25] = 0;

    _sector[26] = 1;                                 // session number
    _sector[27] = 0x14;  // control = Data track, recorded uninterrupted
    _sector[28] = 0;     //TNO;
    _sector[29] = 0xa1;  //POINT
    _sector[30] = 0;     // lead-in running time MIN
    _sector[31] = 0;     // lead-in running time SEC
    _sector[32] = 0;    // lead-in running time FRAME
    _sector[33] = 0;    //ZERO
    _sector[34] = 1;    // last track in program area
    _sector[35] = 0;
    _sector[36] = 0;

    _sector[37] = 1;     // session number
    _sector[38] = 0x14;  // control = Data track, recorded uninterrupted
    _sector[39] = 0;     //TNO;
    _sector[40] = 0xa2;  //POINT
    _sector[41] = 0;     // lead-in running time MIN
    _sector[42] = 0;     // lead-in running time SEC
    _sector[43] = 0;    // lead-in running time FRAME
    _sector[44] = 0;    //ZERO
    frames_to_time(_ata_device->get_size() / MMC_CD_SECTOR_SIZE,
                   _sector[45], _sector[46], _sector[47]); // lead out start time

    ptr16 = (uint16_t*)&_sector[0];
    *ptr16 = revers_unit16(46);    // row toc length

    length = MIN(length, 48);


    uint host_count = get_host_count();

    if (length > host_count) {
        D_MESSAGE("not implemented length > host_count");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    _lba_mid = length & 0xff;
    _lba_high = length >> 8;

    _data_in = (uint16_t*)_sector;
    _data_in_end = (uint16_t*)(_sector + length);
    _next_sector = _end_sector = 0;

    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_IO_BIT);
    _status |= STATUS_DATA_REQUEST_MASK;
    raise();
    return;
}

void ATAController::read_toc()
{
    uint format = _sector[1] & 0x0f;

    switch (format) {
    case 0: {
        if ((_sector[9] & 1) /*link bit is set*/) {
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        uint start_track = _sector[6];


        if (start_track >= MMC_LEADOUT_TRACK_ID) {
            D_MESSAGE("invalid start track");
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        if ( start_track > 1) {
            D_MESSAGE("start_track > 1");
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        uint16_t* ptr16 = (uint16_t*)&_sector[7];
        uint length = revers_unit16(*ptr16);

        if (length == 0) {
            packet_cmd_sucess();
            return;
        }

        uint host_count = get_host_count();

        length = MIN(20, length);

        if (length > host_count) {
            D_MESSAGE("not implemented length > host_count");
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        }

        bool time = !!(_sector[1] && (1 << 1));

        ptr16 = (uint16_t*)&_sector[0];
        *ptr16 = revers_unit16(18); // toc length
        _sector[2] = 1;                             // first track
        _sector[3] = 1;                             // last track

        _sector[4] = 0;     // reservd
        _sector[5] = 0x14;  // control = Data track, recorded uninterrupted
                            // Q Sub-channel encodes current position data
        _sector[6] = 1;     // track number
        _sector[7] = 0;     // reservd

        uint32_t* ptr32;

        if (time) {
            D_MESSAGE("time");
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            return;
        } else {
            ptr32 = (uint32_t*)&_sector[8];
            *ptr32 = 0;
        }

        _sector[12] = 0;                    // reservd
        _sector[13] = 0x14;                 // control = Data track, recorded uninterrupted
                                            // Q Sub-channel encodes current position data
        _sector[14] = MMC_LEADOUT_TRACK_ID; // track number
        _sector[15] = 0;     // reservd
        ptr32 = (uint32_t*)&_sector[16];
        *ptr32 = revers_unit32(_ata_device->get_size() / MMC_CD_SECTOR_SIZE);

        _lba_mid = length & 0xff;
        _lba_high = length >> 8;

        _data_in = (uint16_t*)_sector;
        _data_in_end = (uint16_t*)(_sector + length);
        _next_sector = _end_sector = 0;

        _count &= REASON_TAG_MASK;
        _count |= (1 << REASON_IO_BIT);
        _status |= STATUS_DATA_REQUEST_MASK;
        raise();
        break;
    }
    case 2:
        read_row_toc();
        break;
    default:
        D_MESSAGE("format 0x%x, sleeping...", format);
        for (;;) sleep(2);
    }
}

void ATAController::read_capacity()
{
    uint host_count = get_host_count();

    if (host_count < 8) {
        D_MESSAGE("not implemented host_count < 8");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if ((_sector[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    uint32_t* ptr32 = (uint32_t*)&_sector[2];

    if ((_sector[1] & 1) || (_sector[8] & 1) || *ptr32) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    ptr32 = (uint32_t*)&_sector[0];
    *ptr32 = revers_unit32(_ata_device->get_size() / MMC_CD_SECTOR_SIZE - 1);
    ptr32 = (uint32_t*)&_sector[4];
    *ptr32 = revers_unit32(MMC_CD_SECTOR_SIZE);

    _lba_mid = 8;
    _lba_high = 0;

    _data_in = (uint16_t*)_sector;
    _data_in_end = (uint16_t*)(_sector + 8);
    _next_sector = _end_sector = 0;

    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_IO_BIT);
    _status |= STATUS_DATA_REQUEST_MASK;
    raise();
}


void ATAController::mmc_read()
{
    uint host_count = get_host_count();

    if (host_count < 2048) {
        D_MESSAGE("not implemented host_count < 2048");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if ((_sector[9] & 1) /*link bit is set*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }

    if ((_sector[1] & 1) /*RelAdr*/ || (_sector[1] & (1 << 3)) /*DPO*/) {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        return;
    }
    uint32_t* ptr32 = (uint32_t*)&_sector[2];
    _next_sector = revers_unit32(*ptr32);
    uint16_t* ptr16 = (uint16_t*)&_sector[7];
    _end_sector = _next_sector + revers_unit16(*ptr16);

    if (_end_sector > _ata_device->get_size() / 2048)  {
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST,
                         SCSI_SENSE_ADD_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);
        return;
    }

    if (_next_sector == _end_sector) {
        packet_cmd_sucess();
        return;
    }

    bool force_unit_access = !!(_sector[1] & (1 << 4));

    if (force_unit_access) {
        W_MESSAGE("flush blocks in range");
    }

    if (!_ata_device->read(_next_sector++, _buf)) {
        packet_cmd_abort(SCSI_SENSE_MEDIUM_ERROR,
                         SCSI_SENSE_ADD_READ_RETRIES_EXHAUSTED);
        return;
    }

    _lba_mid = 2048 & 0xff;
    _lba_high = 2048 >> 8;

    _data_in = (uint16_t*)_buf;
    _data_in_end = (uint16_t*)(_buf + 2048);

    _count &= REASON_TAG_MASK;
    _count |= (1 << REASON_IO_BIT);
    _status |= STATUS_DATA_REQUEST_MASK | INTERNAL_STATUS_MMCREAD;
    raise();
}


void ATAController::handle_packet()
{
    //D_MESSAGE("-------------------------------------------------- 0x%x", (uint)_sector[0]);
    switch (_sector[0]) {
    case SCSI_CMD_TEST_UNIT_READY:
        if ((_sector[5] & 1) /*link bit is set*/ ) {
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            break;
        }
#ifdef NO_MEDIUM
        packet_cmd_chk(SCSI_SENSE_NOT_READY, SCSI_SENSE_ADD_MEDIUM_NOT_PRESENT);
#else
        packet_cmd_sucess();
#endif
        break;
    case SCSI_CMD_REQUEST_SENSE:
        request_sens();
        break;
    case SCSI_CMD_INQUIRY: // Inquiry
        if ((_sector[5] & 1) /*link bit is set*/ ) {
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
            break;
        }

        if ((_sector[1] & 1) /*use page code is set*/  || _sector[2] /* page code*/) {
            packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST, SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
        } else {
            standard_inquiry();
        }
        break;

    case SCSI_CMD_MODE_SENSE:
        mode_sense();
        break;
    case SCSI_CMD_GET_EVENT_STATUS_NOTIFICATION:
        get_event_status_notification();
        break;
    case SCSI_CMD_GET_CONFIGURATION:
        get_configuration();
        break;
    case MMC_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        switch (_sector[4] & 3) {
        case 0:
            D_MESSAGE("unlock");
            break;
        case 1:
            D_MESSAGE("lock");
            break;
        case 2:
            D_MESSAGE("persistent allow");
            break;
        case 3:
            D_MESSAGE("Persistent Prevent");
            break;
        }
        packet_cmd_sucess();
        break;
    case MMC_CMD_READ_TOC:
        read_toc();
        break;
    case MMC_CMD_READ:
        mmc_read();
        break;
    case MMC_READ_CAPACITY:
        read_capacity();
        break;
    case 0x51:
        D_MESSAGE("abort command 0x%x", _sector[0]);
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST,
                         SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        break;
    default:
        D_MESSAGE("invalid command");
        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST,
                         SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE);
        for (;;) sleep(2);
    }
}

void ATAController::do_read_verify_sectors()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_read_verify_sectors_ext()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_seek()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_set_fetures()
{
    //D_MESSAGE("subcommand 0x%x", _feature);
    switch (_feature) {
    case 0x02: // Enable write cache
        //D_MESSAGE("enable write cache");
        raise();
        break;
    case 0x05: // Enable advanced power management
        //D_MESSAGE("new power mode is 0x%x", _count & 0xff);
        raise();
        break;
    case 0x66: //Disable reverting to power-on defaults
        raise();
        break;
    default:
        D_MESSAGE("unhandled 0x%x. sleeping... ", _feature);
        for (;;) sleep(2);
    }
}

void ATAController::do_sleep()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_standby()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_standby_immediate()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_write_multi()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_write_multi_ext()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_read_multi()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_read_multi_ext()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_write_dma()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_write_dma_ext()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_read_dma()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_read_dma_ext()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}

void ATAController::do_set_multi_mode()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}



uint64_t ATAController::get_sector_address()
{
    bool lba = _device_reg & DEVICE_LBA_MASK;
    uint64_t sector;

    if (lba) {
        sector = _lba_low & 0xff;
        sector |= (_lba_mid & 0xff) << 8;
        sector |= (_lba_high & 0xff) << 16;
        sector |= (_device_reg & DEVICE_ADDRESS_MASK) << 24;
    } else {
        uint cylinder = (_lba_mid & 0xff)  + ((_lba_high & 0xff) << 8);
        uint head = _device_reg & DEVICE_ADDRESS_MASK;
        sector = (_lba_low & 0xff);
        sector = (cylinder * _heads_per_cylinder + head) * _sectors_per_track + sector - 1;
    }

    return sector;
}


uint64_t ATAController::get_sector_address_ext()
{
    uint64_t sector;

    sector = (_lba_low & 0xff) | ((_lba_low & 0xff00) << 16);
    sector |= ((_lba_mid & 0xff) << 8) | (uint64_t(_lba_mid & 0xff00) << 24);
    sector |= ((_lba_high & 0xff) << 16) | (uint64_t(_lba_high & 0xff00) << 32);

    return sector;
}


uint ATAController::get_sector_count()
{
    return (_count & 0xff) ? (_count & 0xff) : 256;
}


uint ATAController::get_sector_count_ext()
{
    return (_count & 0xffff) ? (_count & 0xffff) : 65536;
}


bool ATAController::is_valid_sectors_range(uint64_t start, uint64_t end)
{
    return start < end && end <= _ata_device->get_size() / SECTOR_SIZE;
}


void ATAController::do_read_sectors_common(uint64_t start, uint64_t end)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    _end_sector = end;

    if (!_ata_device->read(start, _sector)) {
        command_abort_error();
        return;
    }

    _data_in = (uint16_t*)_sector;
    _data_in_end = _data_in + 256;
    _next_sector = start + 1;

    _status |= STATUS_DATA_REQUEST_MASK;
    raise();
}


void ATAController::do_read_sectors()
{
    uint64_t start = get_sector_address();
    do_read_sectors_common(start, start + get_sector_count());
}


void ATAController::do_read_sectors_ext()
{
    uint64_t start = get_sector_address_ext();
    do_read_sectors_common(start, start + get_sector_count_ext());
}


void ATAController::do_write_sectors_common(uint64_t start, uint64_t end)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    _next_sector = start;
    _end_sector = end;

    _data_in = (uint16_t*)_sector;
    _data_in_end = _data_in + 256;

    _status |= STATUS_DATA_REQUEST_MASK;

    raise();
}


void ATAController::do_write_sectors()
{
    uint64_t start = get_sector_address();
    do_write_sectors_common(start, start + get_sector_count());
}


void ATAController::do_write_sectors_ext()
{
    uint64_t start = get_sector_address_ext();
    do_write_sectors_common(start, start + get_sector_count_ext());
}


int ATAController::get_ata_power_state()
{
    return POWER_ACTIVE;
}


const char* command_name(uint8_t command)
{
    switch (command) {
    case CMD_PACKET:
        return "PACKET";
    case CMD_READ_SECTORS:
        return "READ_SECTORS";
    case CMD_READ_SECTORS_EXT:
        return "READ_SECTORS_EXT";
    case CMD_READ_VERIFY_SECTORS:
        return "READ_VERIFY_SECTORS";
    case CMD_READ_VERIFY_SECTORS_EXT:
        return "READ_VERIFY_SECTORS_EXT";
    case CMD_SEEK:
        return "SEEK";
    case CMD_SET_FEATURES:
        return "SET_FEATURES";
    case CMD_WRITE_SECTORS:
        return "WRITE_SECTORS";
    case CMD_WRITE_SECTORS_EXT:
        return "WRITE_SECTORS_EXT";
    case CMD_WRITE_DMA:
        return "WRITE_DMA";
    case CMD_WRITE_DMA_EXT:
        return "WRITE_DMA_EXT";
    case CMD_READ_DMA:
        return "READ_DMA";
    case CMD_READ_DMA_EXT:
        return "READ_DMA_EXT";
    case CMD_READ_MULTIPLE:
        return "READ_MULTIPLE";
    case CMD_READ_MULTIPLE_EXT:
        return "READ_MULTIPLE_EXT";
     case CMD_WRITE_MULTIPLE:
        return "WRITE_MULTIPLE";
    case CMD_WRITE_MULTIPLE_EXT:
        return "WRITE_MULTIPLE_EXT";
    case CMD_FLUSH_CACHE_EXT:
        return "FLUSH_CACHE_EXT";
    case CMD_FLUSH_CACHE:
        return "FLUSH_CACHE";
    case CMD_IDENTIFY_DEVICE:
        return "IDENTIFY_DEVICE";
    case CMD_IDENTIFY_PACKET_DEVICE:
        return "IDENTIFY_PACKET_DEVICE";
    case CMD_CHECK_POWER_MODE:
        return "CHECK_POWER_MODE";
    case CMD_IDLE_IMMEDIATE:
        return "IDLE_IMMEDIATE";
    case CMD_IDLE:
        return "IDLE";
    case CMD_EXECUTE_DEVICE_DIAGNOSTIC:
        return "EXECUTE_DEVICE_DIAGNOSTIC";
    case CMD_DEVICE_RESET:
        return "DEVICE_RESET";
    case CMD_NOP:
        return "NOP";
    case CMD_SET_MULTIPLE_MODE:
        return "SET_MULTIPLE_MODE";
    case CMD_SLEEP:
        return "SLEEP";
    case CMD_STANDBY:
        return "STANDBY";
    case CMD_STANDBY_IMMEDIATE:
        return "STANDBY_IMMEDIATE";
    case CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS:
        return "INITIALIZE_DEVICE_PARAMETERS";
    default:
        return "???";
    }
}


void ATAController::do_command(uint8_t command)
{
    // in device 0 only configurations: a write to the command register shall be ignored,
    // except for EXECUTE DEVICE DIAGNOSTIC;
    if (current() == 1 &&  command != CMD_EXECUTE_DEVICE_DIAGNOSTIC) {
        ATA_LOG("ignoring command 0x%x", command);
        ATA_LOG("-------------------------------------------------");
        //show_io = true;
        // drop ???
        // clear error ???
        //dump_page0();
        //for (;;) sleep(3);
        return;
    }

    // For all commands except DEVICE RESET, this register shall only be written when BSY
    // and DRQ are both cleared and DMACK- is not asserted
    if ((command != CMD_DEVICE_RESET) &&
         ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/))  {
        // by defenition result is indeterminate
        ATA_LOG("drop command 0x%x status 0x%x", command, _status);
        return;
    }

    //  For a device in the Sleep mode, writing of the Command register shall be
    // ignored except for writing of the DEVICE RESET command to a device that implements the
    // PACKET Command
    if (get_power_state() == _POWER_SLEEP && !(is_packet_device() && command == CMD_DEVICE_RESET)) {
        ATA_LOG("drop command 0x%x while in sleep mode", command);
        return;
    }

    _status &= ~STATUS_ERROR_MASK;
    _error = 0;

    ATA_LOG("command: %s (0x%x)", command_name(command), command);

    drop();

    switch (command) {
    case CMD_PACKET:
        if (!is_packet_device()) {
            command_abort_error();
            break;
        }
        do_packet_command();
        break;
    case CMD_READ_SECTORS: {
        if (is_packet_device()) {
            command_abort_error();
            // use set sig ?
            _lba_mid = 0x14;
            _lba_high = 0xeb;
        } else if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_read_sectors();
        }
        break;
    }
    case CMD_READ_SECTORS_EXT: {
        if (is_packet_device()) {
            command_abort_error();
        } else if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_read_sectors_ext();
        }
        break;
    }
    case CMD_READ_VERIFY_SECTORS: {
        if (is_packet_device()) {
            command_abort_error();
        } else if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_read_verify_sectors();
        }
        break;
    }
    case CMD_READ_VERIFY_SECTORS_EXT: {
        if (is_packet_device()) {
            command_abort_error();
        } else if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_read_verify_sectors_ext();
        }
        break;
    }
    case CMD_SEEK:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_seek();
        }
        break;
    case CMD_SET_FEATURES:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_set_fetures();
        }
        break;
    case CMD_WRITE_SECTORS:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_write_sectors();
        }
        break;
    case CMD_WRITE_SECTORS_EXT:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_write_sectors_ext();
        }
        break;
    case CMD_WRITE_DMA:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_write_dma();
        }
        break;
    case CMD_WRITE_DMA_EXT:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_write_dma_ext();
        }
        break;
    case CMD_READ_DMA:
         if (is_packet_device() || !(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_read_dma();
        }
        break;
    case CMD_READ_DMA_EXT:
         if (is_packet_device() || !(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_read_dma_ext();
        }
        break;
    case CMD_READ_MULTIPLE:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)
                             /*|| !(_identity[59]) & (1 << 8)*/) {
            command_abort_error();
        } else {
            do_read_multi();
        }
        break;
    case CMD_READ_MULTIPLE_EXT:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)
                             /*|| !(_identity[59]) & (1 << 8)*/) {
            command_abort_error();
        } else {
            do_read_multi_ext();
        }
        break;
     case CMD_WRITE_MULTIPLE:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)
                             /*|| !(_identity[59]) & (1 << 8)*/) {
            command_abort_error();
        } else {
            do_write_multi();
        }
        break;
    case CMD_WRITE_MULTIPLE_EXT:
        if (is_packet_device() || !(_status & STATUS_READY_MASK)
                             /*|| !(_identity[59]) & (1 << 8)*/) {
            command_abort_error();
        } else {
            do_write_multi_ext();
        }
        break;
    case CMD_FLUSH_CACHE_EXT:
        if (is_packet_device()) {
            command_abort_error();
            break;
        }
    case CMD_FLUSH_CACHE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            //...
            raise();
        }
        break;
    case CMD_IDENTIFY_DEVICE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        }  else if (is_packet_device()) {
            command_abort_error();
            set_signature();
        } else {
            do_identify_device();
        }
        break;
    case CMD_IDENTIFY_PACKET_DEVICE:
        if (!is_packet_device()) {
            command_abort_error();
        } else {
            do_identify_packet_device();
        }
        break;
    case CMD_CHECK_POWER_MODE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            _status = STATUS_READY_MASK | (1 << 4);
            _count = get_ata_power_state();
            raise();
        }
        break;
    case CMD_IDLE_IMMEDIATE:
    case CMD_IDLE:
         if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            // idle
            raise();
        }
        break;
        /*
    case CMD_DEVICE_CONFIGURATION: //Mandatory when the Device Configuration Overlay feature set is
                                   //implemented.
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            set_config();
        }
        break;*/
    case CMD_EXECUTE_DEVICE_DIAGNOSTIC:
        // If the host issues an EXECUTE DEVICE DIAGNOSTIC command while a device is in or going to
        // a power management mode except Sleep, then the device shall execute the EXECUTE DEVICE
        // DIAGNOSTIC sequence.

        set_signature();
        _error = DIAGNOSTIC_D0_OK_D1_NOT_PRESENT;
        _status = 0/*STATUS_READY_MASK*/;
        raise();

        break;
    case CMD_DEVICE_RESET:
        if (!is_packet_device()) {
            command_abort_error();
            break;
        }
        //... reset packet device
        drop();
        set_signature();

         // The device shall clear bit 7 in the ERROR register to zero. The device shall clear
        // bits 6, 5, 4, 3, 2, and 0 in the Status register to zero.

        _status = 0;
        _error &= ~(1 << 7); // or DIAGNOSTIC_D0_OK_D1_NOT_PRESENT (doc conflict)

        break;
    case CMD_NOP:
         if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_nop();
        }
        break;
    case CMD_SET_MULTIPLE_MODE:
        if (!(_status & STATUS_READY_MASK) || !is_packet_device()) {
            command_abort_error();
        } else {
            do_set_multi_mode();
        }
        break;
    case CMD_SLEEP:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_sleep();
        }
        break;
    case CMD_STANDBY:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_standby();
        }
        break;
    case CMD_STANDBY_IMMEDIATE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            do_standby_immediate();
        }
        break;
    case CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            //The Sector Count register specifies the number of logical sectors per logical track,
            // and the Device/Head
            // register specifies the maximum head number.

            if (_sectors_per_track != (_count & 0xff) ||
                _heads_per_cylinder != (_device_reg & DEVICE_ADDRESS_MASK) + 1) {
                PANIC("implement me");
            }

            D_MESSAGE("sectors per track %u new %u, heads %u new %u",
                      _sectors_per_track,
                      _count & 0xff,
                      _heads_per_cylinder,
                      (_device_reg & DEVICE_ADDRESS_MASK) + 1)
            raise();
        }
        break;
    default:
        D_MESSAGE("unhandled 0x%x %u", command, command);
        command_abort_error();
        for (;;) sleep(2);
    }
}


const char* reg_offset_to_name(uint offset, bool r)
{
    switch (offset) {
    case IO_STATUS:
        return r ? "STATUS" : "IO_COMMAND";
    case IO_ERROR:
        return r ? "ERROR" : "IO_FEATURE";
    case IO_DEVICE:
        return "DEVICE";
    case IO_SECTOR_COUNT:
        return "COUNT";
    case IO_LBA_LOW:
        return "LBA_LOW";
    case IO_LBA_MID:
        return "LBA_MID";
    case IO_LBA_HIGH:
        return "LBA_HIGH";
    default:
        return "???";
    }
}

uint8_t ATAController::io_read(uint16_t port)
{
    uint8_t ret = 0xff;

    // in device 0 only configurations:
    //      1. if the device does not implement the PACKET command feature set, a read of the
    //         control block or command Block registers, other than the status or alternate status
    //         registers, shall complete as if device 0 was selected. a read of the status or
    //         alternate status register shall return the value 0
    //      2. if the device implements the PACKET Command feature set, a read of the control Block
    //         or command block registers shall return the value 0.

    port -= _io_base;

    if (current() == 1 && (is_packet_device() || port == IO_STATUS)) {
        /*if (port == IO_STATUS) {
            Lock lock(_mutex);
            drop();
        }*/
        // drop on port == IO_STATUS ?
        ATA_LOG("read: %s [0x%x] 0x%x", reg_offset_to_name(port, true), port, 0);
        return 0;
    }

    switch (port) {
    case IO_STATUS: {
        Lock lock(_mutex);
        drop();
        ret/*urn*/ = _status;
        break;
    }
    case IO_ERROR:
        ret/*urn*/ = _error;
        break;
    case IO_DEVICE:
        ret/*urn*/ = _device_reg;
        break;
    case IO_SECTOR_COUNT:
        ret/*urn*/ = byte_by_HOB(_count);
        break;
    case IO_LBA_LOW:
        ret/*urn*/ = byte_by_HOB(_lba_low);
        break;
    case IO_LBA_MID:
        ret/*urn*/ = byte_by_HOB(_lba_mid);
        break;
    case IO_LBA_HIGH:
        ret/*urn*/ = byte_by_HOB(_lba_high);
        break;
    default:
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }

    ATA_LOG("read: %s [0x%x] 0x%x", reg_offset_to_name(port, true), port, ret);
    return ret;
    //return 0xff;
}

void ATAController::io_write(uint16_t port, uint8_t val)
{
    // in device 0 only configurations:
    //      1. a write to a command block register, other than the command register, shall
    //         complete as if device 0 was selected.
    //      2. a write to the Command register shall be ignored, except for EXECUTE DEVICE
    //         DIAGNOSTIC;

    Lock lock(_mutex);

    port -= _io_base;

    ATA_LOG("write: %s [0x%x] 0x%x", reg_offset_to_name(port, false), port, val);

    clear_HOB();

    // abort on device not ready

    switch (port) {
    case IO_COMMAND:
        do_command(val);
        return;
    case IO_SECTOR_COUNT:
        if ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _count = (_count << 8) | val;
        return;
    case IO_LBA_LOW:
        if ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _lba_low = (_lba_low<< 8) | val;
        return;
    case IO_LBA_MID:
        if ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _lba_mid = (_lba_mid << 8) | val;
        return;
    case IO_LBA_HIGH:
        if ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _lba_high = (_lba_high << 8) | val;
        return;
    case IO_FEATURE:
        if ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }
        _feature = val;
        return;
    case IO_DEVICE:
        if ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/) {
            return;
        }

        // for devices implementing the PACKET Command feature set, the contents of this register
        // are valid while the device is in Sleep mode

        _device_reg = val | DEVICE_MUST_BE_ONE_MASK;

        if (!(_device_reg & DEVICE_SELECT_MASK)) {
            if (_ata_device) {
                _status |= STATUS_READY_MASK | (1 << 4);
            }

            if (_irq_level) {
                raise();
            }
        } else {
            _status &= ~STATUS_READY_MASK;
        }

        return;
    default:
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }
}


uint16_t ATAController::io_read_word(uint16_t port)
{
    port -= _io_base;

    if (port != IO_DATA) {
        W_MESSAGE("invalid port 0x%x", port);
        return ~0;
    }

    if (false /*|| DMACK*/) {
        W_MESSAGE("DMA");
        return ~0;
    }

    if (get_power_state() == _POWER_SLEEP) {
        W_MESSAGE("POWER_SLEEP");
        return ~0;
    }

    if (!(_status & STATUS_DATA_REQUEST_MASK)) {
        W_MESSAGE("unexpected read: data requset is clear 0x%x", _status);
        return ~0;
    }

    if (_data_in < _data_in_end) {
        uint16_t val = *_data_in;

        if (++_data_in == _data_in_end) {

            if (_status & INTERNAL_STATUS_PACKET) {

                if ((_status & INTERNAL_STATUS_MMCREAD) && _next_sector < _end_sector) {
                    if (!_ata_device->read(_next_sector, _buf)) {
                        _status &= ~STATUS_DATA_REQUEST_MASK;
                        packet_cmd_abort(SCSI_SENSE_ILLEGAL_REQUEST,
                                         SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB);
                    }
                    _data_in = (uint16_t*)_buf;
                    _next_sector++;
                    raise();
                } else {
                    _status &= ~STATUS_DATA_REQUEST_MASK;
                    packet_cmd_sucess();
                }
            } else if (_next_sector < _end_sector) {
                if (!_ata_device->read(_next_sector, _sector)) {
                    _status &= ~STATUS_DATA_REQUEST_MASK;
                    command_abort_error();
                }
                _data_in = (uint16_t*)_sector;
                _next_sector++;
                raise(); // todo: rais according to block size
            } else {
                _status &= ~STATUS_DATA_REQUEST_MASK;
            }
        }

        return val;
    }

    W_MESSAGE("unexpected. sleeping... ");
    for (;;) sleep(2);

    return 0;
}

void ATAController::io_write_word(uint16_t port, uint16_t val)
{
    port -= _io_base;

    if (port != IO_DATA) {
        W_MESSAGE("invalid port 0x%x", port);
        return;
    }

    if (false /*|| DMACK*/) {
        W_MESSAGE("DMA");
        return;
    }

    if (get_power_state() == _POWER_SLEEP) {
        W_MESSAGE("POWER_SLEEP");
        return;
    }

    if (!(_status & STATUS_DATA_REQUEST_MASK)) {
         W_MESSAGE("unexpected write: data requset is clear 0x%x", _status);
        return;
    }

    //todo: make it safe and use _data_in for read and _data_out for write
    ASSERT(_data_in < _data_in_end);

    /*if (_status & INTERNAL_STATUS_PACKET) {
        D_MESSAGE("PACKET data 0x%x 0x%x", val & 0xff, val >> 8);
    }*/

    *_data_in = val;

    if (++_data_in == _data_in_end) {

        if (_status & INTERNAL_STATUS_PACKET) {
            _status &= ~STATUS_DATA_REQUEST_MASK;
            handle_packet();
            return;
        }

        if (!_ata_device->write(_next_sector, _sector)) {
            _status &= ~STATUS_DATA_REQUEST_MASK;
            command_abort_error();
            return;
        }

        _data_in = (uint16_t*)_sector;

        if (++_next_sector == _end_sector) {
            _status &= ~STATUS_DATA_REQUEST_MASK;
        }

        raise(); // todo: rais according to block size

        return;
    }
}


ATAHost::ATAHost()
    : PCIDevice ("ide-pci", *pci_bus, NOX_PCI_VENDOR_ID, NOX_PCI_DEV_ID_IDE, ATA_PCI_REVISION,
                 mk_pci_class_code(PCI_CLASS_MASS_STORAGE, PCI_MASS_STORAGE_SUBCLASS_IDE, 0), false)
    , _irq_wire_0 (*this)
    , _channel_0 (new ATAController(*this, 0, _irq_wire_0, ATA0_IO_BASE, ATA0_IO_CONTROL_BASE))
    , _irq_wire_1 (*this)
    , _channel_1 (new ATAController(*this,1, _irq_wire_1, ATA1_IO_BASE, ATA1_IO_CONTROL_BASE))
{
    add_io_region(0, IO_NUM_PORTS, _channel_0.get(),
                  (io_read_byte_proc_t) &ATAController::io_read,
                  (io_write_byte_proc_t)&ATAController::io_write,
                  (io_read_word_proc_t)&ATAController::io_read_word,
                  (io_write_word_proc_t)&ATAController::io_write_word);

    add_io_region(1, PCI_IO_MIN_SIZE, _channel_0.get(),
                  (io_read_byte_proc_t)&ATAController::io_alt_status,
                  (io_write_byte_proc_t)&ATAController::io_control);

    set_io_address(0, ATA0_IO_BASE, true);
    set_io_address(1, ATA0_IO_CONTROL_MAP_BASE, true);

    add_io_region(2, IO_NUM_PORTS, _channel_1.get(),
                  (io_read_byte_proc_t) &ATAController::io_read,
                  (io_write_byte_proc_t)&ATAController::io_write,
                  (io_read_word_proc_t)&ATAController::io_read_word,
                  (io_write_word_proc_t)&ATAController::io_write_word);

    add_io_region(3, PCI_IO_MIN_SIZE, _channel_1.get(),
                  (io_read_byte_proc_t)&ATAController::io_alt_status,
                  (io_write_byte_proc_t)&ATAController::io_control);

    //add_io_region(4, 16, this,
    //              (io_read_byte_proc_t)&ATAHost::io_bus_master_read,
    //              (io_write_byte_proc_t)&ATAHost::io_bus_master_write);

    set_io_address(2, ATA1_IO_BASE, true);
    set_io_address(3, ATA1_IO_CONTROL_MAP_BASE, true);

    pci_bus->add_device(*this);
}


void ATAHost::set_device_0(ATADevice* device)
{
    _channel_0->set_device(device);
}

void ATAHost::set_device_1(ATADevice* device)
{
    _channel_1->set_device(device);
}

void ATAHost::on_io_enabled()
{
    pic->wire(_irq_wire_0, ATA0_IRQ);
    pic->wire(_irq_wire_1, ATA1_IRQ);
}

void ATAHost::on_io_disabled()
{
    _irq_wire_0.dettach_dest();
    _irq_wire_1.dettach_dest();
}

uint8_t ATAHost::io_bus_master_read(uint16_t port)
{
    D_MESSAGE("");
    return ~0;
}

void ATAHost::io_bus_master_write(uint16_t port, uint8_t val)
{
    D_MESSAGE("");
}


void ATAHost::reset()
{
    PCIDevice::reset();
    remap_io_regions();
}

