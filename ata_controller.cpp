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
    DEVICE_SELECT_BIT = 4,
    DEVICE_SELECT_MASK = (1 << DEVICE_SELECT_BIT),
    DEVICE_LBA_MASK = (1 << 6),
    DEVICE_ADDRESS_MASK = ((1 << 4) - 1),
};

enum {
    SECTOR_SIZE = 512,
};

enum PowerState {
    _POWER_ACTIVE,
    _POWER_SLEEP,
};


class ATAController: public VMPart {
public:
    ATAController(VMPart& host, uint id, Wire& irq_wire, uint io_base, uint control_base);

    virtual void reset();
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void set_disk(Disk* disk) { _disk = disk;}

private:
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
    bool is_packet_device() { return false;}
    int get_ata_power_state();

private:
    Mutex _mutex;
    Disk* _disk;
    Wire& _irq_wire;
    uint _io_base;
    uint _control_base;

    union {
        uint16_t _identity[256];
        uint8_t _sector[512];
    };

    uint _status;
    uint _control;
    uint _error;
    uint _device;
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

    friend class ATAHost;
};

ATAController::ATAController(VMPart& host, uint id, Wire& irq_wire, uint io_base,
                             uint control_base)
    : VMPart("ata", host)
    , _disk (NULL)
    , _irq_wire (irq_wire)
    , _io_base (io_base)
    , _control_base (control_base)
    , _irq_level (0)
{
}

void ATAController::reset()
{
    _irq_wire.drop();

    _status = 0;
    _control = 0;
    _feature = 0;
    _data_in = _data_in_end = 0;
    _error = _disk ? DIAGNOSTIC_D0_OK_D1_NOT_PRESENT : DIAGNOSTIC_D0_FAILED_D1_NOT_PRESENT;
    _irq_level = 0;
    set_signature();
}

void ATAController::soft_reset()
{
    reset();
    _status = STATUS_BUSY_MASK | INTERNAL_STATUS_RESET_MASK;
}


uint8_t ATAController::io_alt_status(uint16_t port)
{
    //  - not valid in sleep mode
    //  - when the BSY bit is set to one, the other bits in this register shall not be used

    if (current() == 1) {
        return 0;
    }

    return _status;
}


inline uint ATAController::current()
{
    return (_device >> DEVICE_SELECT_BIT) & 1;
}

inline void ATAController::raise()
{
    // nIEN is the enable bit for the device assertion of INTRQ to the host. When the nIEN bit
    // is cleared to zero, and the device is selected, INTRQ shall be enabled through a driver
    // capable of a high-impedance output state and shall be asserted or negated by the device
    // as appropriate. When the nIEN bit is set to one, or the device is not selected, the device
    // shall release the INTRQ signal.

    _irq_level = 1;

    if ((_control & CONTROL_DISABLE_INTERRUPT_MASK) || current() == 1) {
        return;
    }

    _irq_wire.raise();
}

inline void ATAController::drop()
{
    _irq_level = 0;
    _irq_wire.drop();
}

void ATAController::io_control(uint16_t port, uint8_t val)
{
    // in device 0 only configurations: write to the device control register shall complete as if
    // device 0 was the selected device;

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

         soft_reset();
         return;
    }

    if (_status & INTERNAL_STATUS_RESET_MASK) {
        _status &= ~(STATUS_BUSY_MASK | INTERNAL_STATUS_RESET_MASK);
        if (_disk) {
            _status |= STATUS_READY_MASK;
        }
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
    ID_HRESET_ONE_MASK = (1 << 14) | (1 << 0),
    ID_HRESET_PDIAG_MASK = (1 << 4),
    ID_HRESET_PASS_MASK = (1 << 3),
    ID_HRESET_JUMPER_MASK = (1 << 1),

    ID_OFFSET_ADDR_SECTORS_48 = 100,

    ID_OFFSET_INTEGRITY = 255,
    ID_INTEGRITY_SIGNATURE = 0xa5,
};


uint64_t ATAController::get_num_sectors()
{
    ASSERT(_disk && !(_device & DEVICE_SELECT_MASK));
    return _disk->get_size() / SECTOR_SIZE;
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


void ATAController::do_identify_packet_device()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_nop()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
}


void ATAController::do_packet_command()
{
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
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
    D_MESSAGE("unhandled. sleeping...");
    for (;;) sleep(2);
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
    bool lba = _device & DEVICE_LBA_MASK;
    uint64_t sector;

    if (lba) {
        sector = _lba_low & 0xff;
        sector |= (_lba_mid & 0xff) << 8;
        sector |= (_lba_high & 0xff) << 16;
        sector |= (_device & DEVICE_ADDRESS_MASK) << 24;
    } else {
        uint cylinder = (_lba_mid & 0xff)  + ((_lba_high & 0xff) << 8);
        uint head = _device & DEVICE_ADDRESS_MASK;
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
    return start < end && end <= _disk->get_size() / SECTOR_SIZE;
}


void ATAController::do_read_sectors_common(uint64_t start, uint64_t end)
{
    if (!is_valid_sectors_range(start, end)) {
        command_abort_error();
        return;
    }

    _end_sector = end;

    if (!_disk->read(start, _sector)) {
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

void ATAController::do_command(uint8_t command)
{
    // in device 0 only configurations: a write to the command register shall be ignored,
    // except for EXECUTE DEVICE DIAGNOSTIC;
    if (current() == 1 &&  command != CMD_EXECUTE_DEVICE_DIAGNOSTIC) {
        W_MESSAGE("ignoring command 0x%x while devce 1 is slected ", command);
        // drop ???
        // clear error ???
        return;
    }

    // For all commands except DEVICE RESET, this register shall only be written when BSY
    // and DRQ are both cleared and DMACK- is not asserted
    if ((command != CMD_DEVICE_RESET) &&
         ((_status & (STATUS_BUSY_MASK | STATUS_DATA_REQUEST_MASK)) /*|| DMACK*/))  {
        // by defenition result is indeterminate
        W_MESSAGE("drop command 0x%x status 0x%x", command, _status);
        return;
    }

    //  For a device in the Sleep mode, writing of the Command register shall be
    // ignored except for writing of the DEVICE RESET command to a device that implements the
    // PACKET Command
    if (get_power_state() == _POWER_SLEEP && !(is_packet_device() && command == CMD_DEVICE_RESET)) {
        W_MESSAGE("drop command 0x%x while in sleep mode", command);
        return;
    }

    _status &= ~STATUS_ERROR_MASK;

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
        if (!(_status & STATUS_READY_MASK) || !is_packet_device()) {
            command_abort_error();
        } else {
            do_identify_packet_device();
        }
        break;
    case CMD_CHECK_POWER_MODE:
        if (!(_status & STATUS_READY_MASK)) {
            command_abort_error();
        } else {
            _status = STATUS_READY_MASK;
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
        _status = STATUS_READY_MASK;
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
    default:
        D_MESSAGE("unhandled 0x%x %u", command, command);
        command_abort_error();
        for (;;) sleep(2);
    }
}


uint8_t ATAController::io_read(uint16_t port)
{
    // in device 0 only configurations:
    //      1. if the device does not implement the PACKET command feature set, a read of the
    //         control block or command Block registers, other than the status or alternate status
    //         registers, shall complete as if device 0 was selected. a read of the status or
    //         alternate status register shall return the value 0
    //      2. if the device implements the PACKET Command feature set, a read of the control Block
    //         or command block registers shall return the value 0.

    port -= _io_base;

    if (current() == 1 && (is_packet_device() || port == IO_STATUS)) {
        // drop on port == IO_STATUS ?
        return 0;
    }

    switch (port) {
    case IO_STATUS: {
        Lock lock(_mutex);
        drop();
        return _status;
    }
    case IO_ERROR:
        return _error;
    case IO_DEVICE:
        return _device;
    case IO_SECTOR_COUNT:
        return byte_by_HOB(_count);
    case IO_LBA_LOW:
        return byte_by_HOB(_lba_low);
    case IO_LBA_MID:
        return byte_by_HOB(_lba_mid);
    case IO_LBA_HIGH:
        return byte_by_HOB(_lba_high);
    default:
        W_MESSAGE("waiting 0x%x %u", port, port);
        for (;;) sleep(2);
    }

    return 0xff;
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

        _device = val | DEVICE_MUST_BE_ONE_MASK;

        if (!(_device & DEVICE_SELECT_MASK)) {
            if (_disk) {
                _status |= STATUS_READY_MASK;
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
            raise(); // todo: rais according to block size
            if (_next_sector < _end_sector) {
                if (!_disk->read(_next_sector, _sector)) {
                    _status &= ~STATUS_DATA_REQUEST_MASK;
                    command_abort_error();
                }
                _data_in = (uint16_t*)_sector;
                _next_sector++;

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

    *_data_in = val;

    if (++_data_in == _data_in_end) {
        if (!_disk->write(_next_sector, _sector)) {
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

    set_io_address(2, ATA1_IO_BASE, true);
    set_io_address(3, ATA1_IO_CONTROL_MAP_BASE, true);

    pci_bus->add_device(*this);
}


void ATAHost::set_device_0(Disk* disk)
{
    _channel_0->set_disk(disk);
}

void ATAHost::set_device_1(Disk* disk)
{
    _channel_1->set_disk(disk);
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

