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

#ifndef _H_ATA
#define _H_ATA


enum {
    ATA_DIAGNOSTIC_D0_OK_D1_NOT_PRESENT = 0x01,
    ATA_DIAGNOSTIC_D0_FAILED_D1_NOT_PRESENT = 0x02,

    ATA_IO_DATA = 0,
    ATA_IO_ERROR, ATA_IO_FEATURE = ATA_IO_ERROR,
    ATA_IO_SECTOR_COUNT,
    ATA_IO_LBA_LOW,         // IO_SECTOR_POS = ID_LBA_LOW,
    ATA_IO_LBA_MID,         // IO_CYL_LOW = ID_LBA_MID,
    ATA_IO_LBA_HIGH,        // IO_CYL_HIGH = ID_LBA_HIGH,
    ATA_IO_DEVICE,          // IO_HEAD = ID_DEVICE,
    ATA_IO_COMMAND, ATA_IO_STATUS = ATA_IO_COMMAND,

    ATA_CONTROL_DISABLE_INTERRUPT_MASK = (1 << 1),
    ATA_CONTROL_RESET_MASK = (1 << 2),
    ATA_CONTROL_HOB_MASK = (1 << 7),                                            //HOB

    ATA_STATUS_BUSY_MASK = (1 << 7), // is exclusive                            //BSY
    ATA_STATUS_READY_MASK = (1 << 6),                                           //DRDY
    ATA_STATUS_SEEK_COMPLEAT = (1 << 4),                                        //DSC

    ATA_STATUS_DATA_REQUEST_MASK = (1 << 3), //expeting data from host          //DRQ
    ATA_STATUS_ERROR_MASK = (1 << 0),                                           //ERR
    ATA_STATUS_CHK_MASK = ATA_STATUS_ERROR_MASK,                                //CHK

    ATA_INTERNAL_STATUS_RESET_MASK = (1 << 8),

    ATA_DEVICE_MUST_BE_ONE_MASK = 0,//(1 << 7) | (1 << 5),
    ATA_DEVICE_SELECT_BIT = 4,
    ATA_DEVICE_SELECT_MASK = (1 << ATA_DEVICE_SELECT_BIT),
    ATA_DEVICE_LBA_MASK = (1 << 6),
    ATA_DEVICE_ADDRESS_MASK = ((1 << 4) - 1),

    ATA_ERROR_ABORT = (1 << 2),                                                 //ABRT

    ATA_REASON_CD_BIT = 0,
    ATA_REASON_IO_BIT = 1,
    ATA_REASON_REL_BIT = 2,
    ATA_REASON_TAG_SHIFT = 3,
    ATA_REASON_TAG_MASK = ~((1 << ATA_REASON_TAG_SHIFT) - 1),
};

enum {
    ATA_CMD_NOP = 0x00,
    ATA_CMD_READ_SECTORS = 0x20,
    ATA_CMD_READ_SECTORS_EXT = 0x24,
    ATA_CMD_WRITE_SECTORS = 0x30,
    ATA_CMD_WRITE_SECTORS_EXT = 0x34,
    ATA_CMD_IDLE_IMMEDIATE = 0xe1,
    ATA_CMD_IDLE = 0xe3,
    ATA_CMD_CHECK_POWER_MODE = 0xe5,
    //ATA_CMD_DEVICE_CONFIGURATION = 0xb1,
    ATA_CMD_DEVICE_RESET = 0x08, // Use prohibited when the PACKET Command feature set
                                 // is not implemented
    ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC = 0x90,
    ATA_CMOMPAT_CMD_INITIALIZE_DEVICE_PARAMETERS = 0x91,

    ATA_CMD_FLUSH_CACHE = 0xe7,
    ATA_CMD_FLUSH_CACHE_EXT = 0xea,

    //ATA_CMD_GET_MEDIA_STATUS = 0xda,  // Mandatory for devices implementing the Removable
                                        // Media Status Notification feature set. Optional for
                                        // devices implementing the Removable Media feature set.

    ATA_CMD_IDENTIFY_DEVICE = 0xec,
    ATA_CMD_PACKET = 0xa0,
    ATA_CMD_IDENTIFY_PACKET_DEVICE = 0xa1,

    ATA_CMD_READ_MULTIPLE = 0xc4,
    ATA_CMD_READ_MULTIPLE_EXT = 0x29,
    ATA_CMD_READ_DMA = 0xc8,
    ATA_CMD_READ_DMA_EXT = 0x25,
    ATA_CMD_READ_VERIFY_SECTORS = 0x40,
    ATA_CMD_READ_VERIFY_SECTORS_EXT = 0x42,
    ATA_CMD_SEEK = 0x70,
    ATA_CMD_SET_FEATURES = 0xef,
    ATA_CMD_SET_MULTIPLE_MODE = 0xc6,
    ATA_CMD_SLEEP = 0xe6,
    ATA_CMD_STANDBY = 0xe2,
    ATA_CMD_STANDBY_IMMEDIATE = 0xe0,
    ATA_CMD_WRITE_DMA = 0xca,
    ATA_CMD_WRITE_DMA_EXT = 0x35,
    ATA_CMD_WRITE_MULTIPLE = 0xc5,
    ATA_CMD_WRITE_MULTIPLE_EXT = 0x39,
};

enum {
    ATA_ID_OFFSET_GENERAL_CONF = 0,
    ATA_COMPAT_ID_GENERAL_CONF_NOT_REMOVABLE_MASK = (1 << 6),

    ATA_COMPAT_ID_OFFSET_CYL = 1,

    ATA_ID_OFFSET_SPECIFIC_CONF = 2,

    ATA_COMPAT_ID_OFFSET_HEAD = 3,
    ATA_COMPAT_ID_OFFSET_SECTORS = 6,

    ATA_ID_OFFSET_SERIAL = 10,
    ATA_ID_SERIAL_NUM_CHARS = 20,

    ATA_ID_OFFSET_REVISION = 23,
    ATA_ID_REVISION_NUM_CHARS = 8,

    ATA_ID_OFFSET_MODEL = 27,
    ATA_ID_MODEL_NUM_CHARS = 40,

    ATA_ID_OFFSET_MAX_SECTORS_PER_BLOCK = 47,

    ATA_ID_OFFSET_CAP1 = 49,
    ATA_ID_CAP1_DMA_MASK = (1 << 8),
    ATA_ID_CAP1_LBA_MASK = (1 << 9),
    ATA_ID_CAP1_DISABLE_IORDY_MASK = (1 << 10),
    ATA_ID_CAP1_IORDY_MASK = (1 << 11),

    ATA_ID_OFFSET_CAP2 = 50,
    ATA_ID_CAP2_MUST_SET = (1 << 14),

    ATA_ID_OFFSET_FIELD_VALIDITY = 53,
    ATA_ID_FIELD_VALIDITY_64_70 = (1 << 1),
    ATA_ID_FIELD_VALIDITY_88 = (1 << 2),

    ATA_COMPAT_ID_OFFSET_CURRENT_CYL = 54,
    ATA_COMPAT_ID_OFFSET_CURRENT_HEAD = 55,
    ATA_COMPAT_ID_OFFSET_CURRENT_SECTORS = 56,
    ATA_COMPAT_ID_OFFSET_CURRENT_ADDRESABEL_SECTORS = 57,

    ATA_ID_OFFSET_MULTIPLE_SETTING = 59,
    ATA_ID_MULTIPLE_SETTING_ACTIVE_MASK = (1 << 8),
    ATA_ID_MULTIPLE_VAL_MASK = (1 << 8) - 1,

    ATA_ID_OFFSET_ADDRESABEL_SECTORS = 60,
    ATA_ID_OFFSET_NULTI_DMA = 63,
    ATA_ID_NULTI_DMA_MODE0_MASK = (1 << 0),
    ATA_ID_NULTI_DMA_MODE1_MASK = (1 << 1),
    ATA_ID_NULTI_DMA_MODE2_MASK = (1 << 2),
    ATA_ID_NULTI_DMA_MODE0_SELECT_MASK = (1 << 8),
    ATA_ID_NULTI_DMA_MODE1_SELECT_MASK = (1 << 9),
    ATA_ID_NULTI_DMA_MODE2_SELECT_MASK = (1 << 10),

    ATA_ID_OFFSET_PIO = 64,
    ATA_IO_PIO_MODE3_MASK = (1 << 0),
    ATA_IO_PIO_MODE4_MASK = (1 << 1),

    ATA_ID_OFFSET_QUEUE_DEPTH = 75,

    ATA_ID_OFFSET_VERSION = 80,
    ATA_ID_VERSION_ATA3_MASK = (1 << 3),
    ATA_ID_VERSION_ATA4_MASK = (1 << 4),
    ATA_ID_VERSION_ATA5_MASK = (1 << 5),
    ATA_ID_VERSION_ATA6_MASK = (1 << 6),

    ATA_ID_OFFSET_CMD_SET_1 = 82,
    ATA_ID_OFFSET_CMD_SET_1_ENABLE = 85,
    ATA_ID_CMD_SET_1_NOP_MASK = (1 << 14),
    ATA_ID_CMD_SET_1_DEVICE_RESET = (1 << 9),
    ATA_ID_CMD_SET_1_WRITE_CACHE = (1 << 5),
    ATA_ID_CMD_SET_1_PACKET = (1 << 4),
    ATA_ID_CMD_SET_1_POWR_MANAG = (1 << 3),

    ATA_ID_OFFSET_CMD_SET_2 = 83,
    ATA_ID_OFFSET_CMD_SET_2_ENABLE = 86,
    ATA_ID_CMD_SET_2_ONE_MASK = (1 << 14),
    ATA_ID_CMD_SET_2_FLUSH_EXT_MASK = (1 << 13),
    ATA_ID_CMD_SET_2_FLUSH_MASK = (1 << 12),
    ATA_ID_CMD_SET_2_48BIT_MASK = (1 << 10),
    ATA_ID_CMD_SET_2_POWERUP_IN_STANDBY_MASK = (1 << 5),
    ATA_ID_CMD_SET_2_MEDIA_STATUS_NOTIFICTION = (1 << 4),
    ATA_ID_CMD_SET_2_ADVANCE_POWR_MANAG_MASK = (1 << 3),
    ATA_ID_CMD_SET_2_QUAD_MASK = (1 << 1),

    ATA_ID_OFFSET_CMD_SET_3 = 84,
    ATA_ID_OFFSET_CMD_SET_3_ENABLE = 87,
    ATA_ID_CMD_SET_3_ONE_MASK = (1 << 14),

    ATA_ID_OFFSET_UDMA = 88,
    ATA_ID_UDMA_MODE0_MASK = (1 << 0),
    ATA_ID_UDMA_MODE1_MASK = (1 << 1),
    ATA_ID_UDMA_MODE2_MASK = (1 << 2),
    ATA_ID_UDMA_MODE3_MASK = (1 << 3),
    ATA_ID_UDMA_MODE4_MASK = (1 << 4),
    ATA_ID_UDMA_MODE5_MASK = (1 << 5),
    ATA_ID_UDMA_MODE0_SELECT_MASK = (1 << 8),
    ATA_ID_UDMA_MODE1_SELECT_MASK = (1 << 9),
    ATA_ID_UDMA_MODE2_SELECT_MASK = (1 << 10),
    ATA_ID_UDMA_MODE3_SELECT_MASK = (1 << 11),
    ATA_ID_UDMA_MODE4_SELECT_MASK = (1 << 12),
    ATA_ID_UDMA_MODE5_SELECT_MASK = (1 << 13),

    ATA_ID_OFFSET_ADVANCE_POWR_MANAG = 91,
    ATA_ID_ADVANCE_POWR_MANAG_MIN_CONSUMTION = 0x01,
    ATA_ID_ADVANCE_POWR_MANAG_MAX_PERFORMANCE = 0xfe,

    ATA_ID_OFFSET_HRESET = 93,
    ATA_ID_HRESET_ONE_MASK = (1 << 14) | (1 << 0),
    ATA_ID_HRESET_PDIAG_MASK = (1 << 4),
    ATA_ID_HRESET_PASS_MASK = (1 << 3),
    ATA_ID_HRESET_JUMPER_MASK = (1 << 1),

    ATA_ID_OFFSET_ADDR_SECTORS_48 = 100,

    ATA_ID_OFFSET_BYTE_COUNT_0_BEHAVIOR = 125,
    ATA_ID_OFFSET_REMOVABLE_STATUS_SUPPORT = 127,

    ATA_ID_OFFSET_INTEGRITY = 255,
    ATA_ID_INTEGRITY_SIGNATURE = 0xa5,
};


enum {
    ATA_ID_GENERAL_CONF_ATAPI_MASK = 1 << 15,
    ATA_ID_GENERAL_COMMAND_SET_CD = 0x05,
    ATA_ID_GENERAL_COMMAND_SET_SHIFT = 8,
    ATA_ID_GENERAL_REMOVABLE_BIT = 7,
    ATA_ID_GENERAL_DRQ_LATENCY = 2,
    ATA_ID_GENERAL_DRQ_LATENCY_SHIFT = 5,

    ATAPI_PACKET_SIZE = 12,

    ATAPI_ID_CAP1_MBZ = ATA_ID_CAP1_LBA_MASK,
};

enum {
    ATA_FEATURE_ENABLE_CACHE = 0x02,
    ATA_FEATURE_DISABLE_CACHE = 0x82,
    ATA_FEATURE_DISABLE_REVERT_TO_DEFAULT = 0x66,
    ATA_FEATURE_ENABLE_REVERT_TO_DEFAULT = 0xcc,
};


enum {
    SCSI_CMD_TEST_UNIT_READY = 0x00,
    SCSI_CMD_REQUEST_SENSE = 0x03,
    SCSI_CMD_INQUIRY = 0x12,
    SCSI_CMD_MODE_SENSE = 0x5a,

    SCSI_SENSE_SHIFT = 4,
    SCSI_SENSE_NO_SENSE = 0x00,
    SCSI_SENSE_NOT_READY = 0x02,
    SCSI_SENSE_MEDIUM_ERROR = 0x03,
    SCSI_SENSE_ILLEGAL_REQUEST = 0x05,
    SCSI_SENSE_UNIT_ATTENTION = 0x06,

    SCSI_SENSE_ADD_NO_ADDITIONAL_SENSE_INFORMATION = 0x0000,
    SCSI_SENSE_ADD_READ_RETRIES_EXHAUSTED = 0x1101,
    SCSI_SENSE_ADD_INVALID_COMMAND_OPERATION_CODE = 0x2000,
    SCSI_SENSE_ADD_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE = 0x2100,
    SCSI_SENSE_ADD_INVALID_FIELD_IN_CDB = 0x2400,
    SCSI_SENSE_ADD_MEDIUM_NOT_PRESENT = 0x3a00,
    SCSI_SENSE_ADD_MEDIUM_NOT_PRESENT_CLOSED = 0x3a01,
    SCSI_SENSE_ADD_MEDIUM_NOT_PRESENT_OPEN = 0x3a02,
    SCSI_SENSE_ADD_MEDIUM_MAY_HAVE_CHANGED = 0x2800,
    SCSI_SENSE_ADD_PARAMETERS_CHANGED = 0x2a00,
    SCSI_SENSE_ADD_OPERATOR_REMOVAL_REQUEST = 0x5a01,

    SCSI_INQUIRY_STD_LENGTH = 36,
    SCSI_FIX_SENSE_LENGTH = 18,
};


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
    MMC_CMD_READ_CAPACITY = 0x25,
    MMC_CMD_READ = 0x28,
    MMC_CMD_READ_TOC = 0x43,
    MMC_CMD_GET_CONFIGURATION = 0x46,
    MMC_CMD_GET_PERFORMANCE = 0xac,
    MMC_CMD_READ_DISC_INFORMATION = 0x51,
    MMC_CMD_GET_EVENT_STATUS_NOTIFICATION = 0x4a,
    MMC_CMD_MECHANISM_STATUS = 0xbd,
    MMC_CMD_START_STOP_UNIT = 0x1b,

    MMC_LEADOUT_TRACK_ID = 0xaa,

    MMC_CD_SECTOR_SIZE = 2048,
    MMC_CD_FRAMES_PER_SEC = 75,

    MMC_NOTIFY_OPERATIONAL_CLASS = 1,
    MMC_NOTIFY_OPERATIONAL_MASK = (1 << MMC_NOTIFY_OPERATIONAL_CLASS),
    MMC_NOTIFY_MEDIA_CLASS = 4,
    MMC_NOTIFY_MEDIA_MASK = (1 << MMC_NOTIFY_MEDIA_CLASS),
};

#endif

