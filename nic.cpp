/*
    Copyright (c) 2014 Yaniv Kamay,
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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <arpa/inet.h>

#include "nic.h"
#include "pci_bus.h"
#include "pci.h"
#include "memory_bus.h"

// Emulates Intel 82574 GbE (82574l-gbe-controller-datasheet.pdf)

#ifdef DO_NIC_LOG
#define NIC_LOG(format, ...) \
    D_MESSAGE("NIC[%lu]: %lu "format, pthread_self(), get_monolitic_time(), ## __VA_ARGS__)
#else
    #define NIC_LOG(format, ...)
#endif

#define NIC_VENDOR_ID 0x8086
#define NIC_DEV_ID 0x10D3
#define NIC_DEV_REVISION 0

#define NIC_CSR_BAR 0
#define NIC_CSR_SPACE_SIZE (128 * KB)

#define NIC_FLASH_BAR 1
#define NIC_FLASH_SPACE_SIZE (4 * KB)

#define NIC_IO_BAR 2
#define NIC_IO_SPACE_SIZE 32
#define NIC_IO_ADDR_PORT 0
#define NIC_IO_DATA_PORT 4


enum {
    NIC_REG_CTRL = 0x00,
    NIC_REG_CTRL_ALIAS = 0x04,
    NIC_REG_STATUS = 0x08,
    NIC_REG_NVM_CTRL = 0x10,
    NIC_REG_EEPROM_READ = 0x14,
    NIC_REG_EXT_CTRL = 0x18,
    NIC_REG_MDI_CTRL = 0x20,
    NIC_REG_FLOW_CTRL_ADDR_LOW = 0x28,
    NIC_REG_FLOW_CTRL_ADDR_HIGH = 0x2c,
    NIC_REG_FLOW_CTRL = 0x30,
    NIC_REG_VLAN_ETHER_TYPE = 0x38,
    NIC_REG_INT_CAUSE_READ = 0xc0,
    NIC_REG_INT_THROTTLING = 0xc4,
    NIC_REG_INT_CAUSE_SET = 0xc8,
    NIC_REG_INT_MASK_SET = 0xd0,
    NIC_REG_INT_MASK_CLEAR = 0xd8,
    NIC_REG_INT_AUTO_MASK = 0xe0,
    NIC_REG_RX_CTRL = 0x100,
    NIC_REG_FLOW_CTRL_TTV = 0x170,
    NIC_REG_TX_CTRL = 0x400,
    NIC_REG_ADAPT_IFS_THROT = 0x458,
    NIC_REG_LED_CTRL = 0xe00,
    NIC_REG_EXT_CONF_CTRL = 0xf00,
    NIC_REG_PACKET_BUFF_ALLOC = 0x1000,
    NIC_REG_PACKET_BUFF_SIZE = 0x1008,
    NIC_REG_FLOW_CTRL_RT_LOW = 0x02160,
    NIC_REG_FLOW_CTRL_RT_HIGH = 0x02168,
    NIC_REG_RX_DESCRIPTOR_ADDRESS_LOW_0 = 0x02800,
    NIC_REG_RX_DESCRIPTOR_ADDRESS_HIGH_0 = 0x02804,
    NIC_REG_RX_DESCRIPTOR_LENGTH_0 = 0x02808,
    NIC_REG_RX_DESCRIPTOR_HEAD_0 = 0x02810,
    NIC_REG_RX_DESCRIPTOR_TAIL_0 = 0x02818,
    NIC_REG_RX_INT_DELAY_VAL = 0x02820,
    NIC_REG_RX_DESCRIPTOR_CTRL_0 = 0x02828,
    NIC_REG_RX_INT_ABS_DELAY_VAL = 0x0282c,
    NIC_REG_RX_DESCRIPTOR_ADDRESS_LOW_1 = NIC_REG_RX_DESCRIPTOR_ADDRESS_LOW_0 + 0x100,
    NIC_REG_RX_DESCRIPTOR_ADDRESS_HIGH_1 = NIC_REG_RX_DESCRIPTOR_ADDRESS_HIGH_0 + 0x100,
    NIC_REG_RX_DESCRIPTOR_LENGTH_1 = NIC_REG_RX_DESCRIPTOR_LENGTH_0 + 0x100,
    NIC_REG_RX_DESCRIPTOR_HEAD_1 = NIC_REG_RX_DESCRIPTOR_HEAD_0 + 0x100,
    NIC_REG_RX_DESCRIPTOR_TAIL_1 = NIC_REG_RX_DESCRIPTOR_TAIL_0 + 0x100,
    NIC_REG_RX_DESCRIPTOR_CTRL_1 = NIC_REG_RX_DESCRIPTOR_CTRL_0 + 0x100,
    NIC_REG_TX_DESCRIPTOR_ADDRESS_LOW_0 = 0x03800,
    NIC_REG_TX_DESCRIPTOR_ADDRESS_HIGH_0 = 0x03804,
    NIC_REG_TX_DESCRIPTOR_LENGTH_0 = 0x03808,
    NIC_REG_TX_DESCRIPTOR_HEAD_0 = 0x03810,
    NIC_REG_TX_DESCRIPTOR_TAIL_0 = 0x03818,
    NIC_REG_TX_INT_DELAY_VAL = 0x03820,
    NIC_REG_TX_DESCRIPTOR_CTRL_0 = 0x3828,
    NIC_REG_TX_INT_ABS_DELAY_VAL = 0x0382c,
    NIC_REG_TX_ARBITRATION_COUNT_0 = 0x3840,
    NIC_REG_TX_DESCRIPTOR_ADDRESS_LOW_1 = NIC_REG_TX_DESCRIPTOR_ADDRESS_LOW_0 + 0x100,
    NIC_REG_TX_DESCRIPTOR_ADDRESS_HIGH_1 = NIC_REG_TX_DESCRIPTOR_ADDRESS_HIGH_0 + 0x100,
    NIC_REG_TX_DESCRIPTOR_LENGTH_1 = NIC_REG_TX_DESCRIPTOR_LENGTH_0 + 0x100,
    NIC_REG_TX_DESCRIPTOR_HEAD_1 = NIC_REG_TX_DESCRIPTOR_HEAD_0 + 0x0100,
    NIC_REG_TX_DESCRIPTOR_TAIL_1 = NIC_REG_TX_DESCRIPTOR_TAIL_0 + 0x0100,
    NIC_REG_TX_DESCRIPTOR_CTRL_1 = NIC_REG_TX_DESCRIPTOR_CTRL_0 + 0x0100,
    NIC_REG_TX_ARBITRATION_COUNT_1 = NIC_REG_TX_ARBITRATION_COUNT_0 + 0x100,

    NIC_REG_STAT_START = 0x04000,
    NIC_REG_STAT_END = 0x04100,

    NIC_REG_RX_CHECKSUM_CTRL = 0x05000,
    NIC_REG_RX_FILTER_CTRL = 0x05008,
    NIC_REG_MULTICAST_TABLE_START = 0x05200,
    NIC_REG_MULTICAST_TABLE_END = NIC_REG_MULTICAST_TABLE_START + (MULTICAST_TABLE_SIZE - 1) * 4,
    NIC_REG_RECEIVE_ADDR_START = 0x05400,
    NIC_REG_RECEIVE_ADDR_END = NIC_REG_RECEIVE_ADDR_START + NUM_RECEIVE_ADDR * 8 - 4,
    NIC_REG_VLAN_FILTER_TABLE_ARRAY = 0x5600,
    NIC_REG_WAKEUP_CTRL = 0x5800,
    NIC_REG_WAKEUP_FILTER = 0x5808,
    NIC_REG_MULTI_RQ_CMD = 0x5818,
    NIC_REG_MNG_CTRL = 0x5820,
    NIC_REG_3GIO_CTRL_1 = 0x05B00,
    NIC_REG_SOFT_SEM = 0x05B50,
    NIC_REG_3GIO_CTRL_2 = 0x05B64,
    NIC_REG_REDIRECTION_START = 0x5c00,
    NIC_REG_REDIRECTION_END = 0x5c00 + NIC_REDIRECTION_TABLE_SIZE - 4,
    NIC_REG_RX_RSS_KEY_START = 0x5c80,
    NIC_REG_RX_RSS_KEY_END = NIC_REG_RX_RSS_KEY_START + NIC_RSS_KEY_SIZE - 4,
    NIC_REG_TIME_SYNC_SYSTEM_TIME_LOW = 0x0b600, // latch High
    NIC_REG_TIME_SYNC_SYSTEM_TIME_HIGH = 0x0b604,
    NIC_REG_TIME_SYNC_INC_ATTRIB = 0x0b608,
    NIC_REG_TIME_SYNC_TX_CTRL = 0x0b614,
    NIC_REG_TIME_SYNC_TX_STAMP_LOW = 0x0b618,
    NIC_REG_TIME_SYNC_TX_STAMP_HIGH = 0x0b61c,
    NIC_REG_TIME_SYNC_RX_CTRL = 0x0b620,
    NIC_REG_TIME_SYNC_STAMP_LOW = 0x0b624,
    NIC_REG_TIME_SYNC_STAMP_HIGH = 0x0b628,
    NIC_REG_TIME_SYNC_MESS_TYPE = 0x0b634,
    NIC_REG_TIME_SYNC_UDP_PORT = 0x0b638,

    NIC_SOFT_SEM_LOCK = (1 << 0),
    NIC_SOFT_SEM_EEPROM_LOCK = (1 << 1),
    NIC_SOFT_SEM_MASK = NIC_SOFT_SEM_LOCK | NIC_SOFT_SEM_EEPROM_LOCK,

    NIC_NVM_CTRL_TYPE = (1 << 23),
    NIC_NVM_CTRK_ADDRESS_SIZE_SHIFT = 15,
    NIC_NVM_CTRL_SIZE_SHIFT = 11,
    NIC_NVM_CTRL_SIZE_MASK = (0x0f << NIC_NVM_CTRL_SIZE_SHIFT),
    NIC_NVM_CTRL_AUTO_READ_DONE = (1 << 9),
    NIC_NVM_CTRL_PRESENT = (1 << 8),
    NIC_NVM_CTRL_WRITE_CTRL_SHIFT = 4,
    NIC_NVM_CTRL_ERASE = 0,     // Enable Flash erase and block erase
    NIC_NVM_CTRL_RO = 1,        // Flash writes and Flash erase disabled
    NIC_NVM_CTRL_RW = 2,        // Flash writes enabled
    NIC_NVM_CTRL_INVALID = 3,   // Not allowed

    NIC_EXT_CONF_CTRL_MDIO_MNG = (1 << 7),
    NIC_EXT_CONF_CTRL_MDIO_HW = (1 << 6),
    NIC_EXT_CONF_CTRL_MDIO_SW = (1 << 5),

    NIC_EXT_CONF_CTRL_RESERVED_SET = (1 << 3),

    NIC_MDI_CTRL_REGADDR_SHIFT = 16,
    NIC_MDI_CTRL_DATA_MASK = (1 << NIC_MDI_CTRL_REGADDR_SHIFT) - 1,
    NIC_MDI_CTRL_REGADDR_MASK = 0x1f << NIC_MDI_CTRL_REGADDR_SHIFT,
    NIC_MDI_CTRL_REGADDR_PAGE_CMD = 0x1f,
    NIC_MDI_CTRL_ADDRESS_SHIFT = 21,
    NIC_MDI_CTRL_ADDRESS_MASK = 0x1f << NIC_MDI_CTRL_ADDRESS_SHIFT,
    NIC_MDI_CTRL_ADDRESS_GIGABIT = 1 << NIC_MDI_CTRL_ADDRESS_SHIFT,
    NIC_MDI_CTRL_ADDRESS_PCIE = 2 << NIC_MDI_CTRL_ADDRESS_SHIFT,
    NIC_MDI_CTRL_OP_SHIFT = 26,
    NIC_MDI_CTRL_OP_MASK = 0x03 << NIC_MDI_CTRL_OP_SHIFT,
    NIC_MDI_CTRL_OP_WRITE = 1 << NIC_MDI_CTRL_OP_SHIFT,
    NIC_MDI_CTRL_OP_READ = 2 << NIC_MDI_CTRL_OP_SHIFT,
    NIC_MDI_CTRL_READY = (1 << 28),
    NIC_MDI_CTRL_INTERRUPT = (1 << 29),
    NIC_MDI_CTRL_ERROR = (1 << 30),
    NIC_MDI_CTRL_RESSERVED = (1 << 31),

    NIC_MNG_CTRL_DIS_ARP_IP_CHECK = (1 << 28),

    NIC_RX_CTRL_FLXBUF_SHIFT = 27,
    NIC_RX_CTRL_FLXBUF_MASK = (0x0f << NIC_RX_CTRL_FLXBUF_SHIFT),
    NIC_RX_CTRL_SECRC = (1 << 26),
    NIC_RX_CTRL_BSEX = (1 << 25),
    NIC_RX_CTRL_CFIEN = (1 << 19),
    NIC_RX_CTRL_VFE = (1 << 18),
    NIC_RX_CTRL_BSIZE_SHIFT = 16,
    NIC_RX_CTRL_BSIZE_MASK = (0x03 << NIC_RX_CTRL_BSIZE_SHIFT),
    NIC_RX_CTRL_ACCEPT_BROADCAST = (1 << 15),
    NIC_RX_CTRL_MULTICAST_OFFSET_SHIFT = 12,
    NIC_RX_CTRL_MULTICAST_OFFSET_MASK = (0x03 << NIC_RX_CTRL_MULTICAST_OFFSET_SHIFT),
    NIC_RX_CTRL_DTYP_MASK = (0x03 << 10),
    NIC_RX_CTRL_DESCRIPTOR_THRESHOLD_SHIFT = 8,
    NIC_RX_CTRL_DESCRIPTOR_THRESHOLD_MASK = (0x03 << NIC_RX_CTRL_DESCRIPTOR_THRESHOLD_SHIFT),
    NIC_RX_CTRL_LONG_PACKET_ENABLE = (1 << 5),
    NIC_RX_CTRL_PROMISCUOUS_MULTICAST = (1 << 4),
    NIC_RX_CTRL_PROMISCUOUS_UNICAST = (1 << 3),
    NIC_RX_CTRL_ENABLE = (1 << 1),
    NIC_RX_CTRL_RESERVED = (1 << 0) | (1 << 14) | (1 << 21) | (1 << 24) | (1 << 31),

    NIC_RX_CHECKSUM_CTRL_PACKET_DISABLE = (1 << 13),
    NIC_RX_CHECKSUM_CTRL_UDP = (1 << 9),
    NIC_RX_CHECKSUM_CTRL_IP = (1 << 8),
    NIC_RX_CHECKSUM_CTRL_START_MASK = 0xff,
    NIC_RX_CHECKSUM_RESERVED = (1 << 10) | ~((1 << 14) - 1),

    NIC_RX_DESCRIPTOR_LENGTH_MASK = 0x0fff80,

    NIC_RX_FILTER_CTRL_EXSTEN = (1 << 15),
    NIC_RX_FILTER_CTRL_RESERVED = 0xffff0000,

    NIC_MULTI_RQ_CMD_ENABLE_MASK = 0x3,
    NIC_MULTI_RQ_CMD_RSS_MASK = (0xffff << 16),
    NIC_MULTI_RQ_CMD_RESERVED = ~(NIC_MULTI_RQ_CMD_ENABLE_MASK | NIC_MULTI_RQ_CMD_ENABLE_MASK),

    NIC_TX_CTRL_ENABLE = (1 << 1),
    NIC_TX_CTRL_PAD_SHORT = (1 << 3),
    NIC_TX_CTRL_COLLIS_THRESH_SHIFT = 4,
    NIC_TX_CTRL_COLLIS_THRESH_MASK = 0xff << NIC_TX_CTRL_COLLIS_THRESH_SHIFT,
    NIC_TX_CTRL_COLLIS_DIST_SHIFT = 12,
    NIC_TX_CTRL_COLLIS_DIST_MASK = 0x3ff << NIC_TX_CTRL_COLLIS_DIST_SHIFT,
    NIC_TX_CTRL_COLLIS_DIST_INIT_VAL = 0x3f,
    NIC_TX_CTRL_MULTI_REQ = (1 << 28),

    NIC_TX_CTRL_RR_THRESH_SHIFT = 29,
    NIC_TX_CTRL_RR_THRESH_MASK = 0x03 << NIC_TX_CTRL_RR_THRESH_SHIFT,
    NIC_TX_CTRL_RR_THRESH_INIT_VAL = 1,
    NIC_TX_CTRL_RESERVED = (1 << 0) | (1 << 2) | (0x1 << 31),

    NIC_TX_DESCRIPTOR_ADDRESS_LOW_MASK = ~0x0f,
    NIC_TX_DESCRIPTOR_LENGTH_MASK = 0x0fff80,

    NIC_EEPROM_READ_START = (1 << 0),
    NIC_EEPROM_READ_DONE = (1 << 1),
    NIC_EEPROM_READ_ADDR_SHIFT = 2,
    NIC_EEPROM_READ_ADDR_MASK = (0x3fff << NIC_EEPROM_READ_ADDR_SHIFT),
    NIC_EEPROM_READ_DATA_SHIFT = 16,
    NIC_EEPROM_READ_DATA_MASK = (0xffff << NIC_EEPROM_READ_DATA_SHIFT),

    NIC_RECEIVE_ADDR_HIGH_VALID = (1 << 31),

    NIC_PACKET_BUFF_SIZE_MASK = 0x3f,
    NIC_PACKET_BUFF_SIZE_MAX = 40, //40 KB packet buffer

    NIC_PACKET_BUFF_ALLOC_RX_MASK = 0x3f,
    NIC_PACKET_BUFF_ALLOC_RX_INIT_VAL = 20,
    NIC_PACKET_BUFF_ALLOC_TX_INIT_VAL = NIC_PACKET_BUFF_SIZE_MAX -
                                        NIC_PACKET_BUFF_ALLOC_RX_INIT_VAL,
    NIC_PACKET_BUFF_ALLOC_TX_MIN = 5, // Transmit packet buffer size should be configured to be
                                      // more than 4 KB
    NIC_PACKET_BUFF_ALLOC_TX_SHIFT = 16,

    NIC_INT_CAUSE_READ_ASSERTED = (1 << 31),
    NIC_INT_CAUSE_TxQ1_WRITTEN_BACK = (1 << 23),
    NIC_INT_CAUSE_TxQ0_WRITTEN_BACK = (1 << 22),
    NIC_INT_CAUSE_RxQ1_WRITTEN_BACK = (1 << 21),
    NIC_INT_CAUSE_RxQ0_WRITTEN_BACK = (1 << 20),
    NIC_INT_CAUSE_TXD_LOW = (1 << 15),
    NIC_INT_CAUSE_MDIO_COMPLETE = (1 << 9),
    NIC_INT_CAUSE_RX_TIMER = (1 << 7),
    NIC_INT_CAUSE_RX_OVERRUN = (1 << 6),
    NIC_INT_CAUSE_RX_DESCRIPTOR_THRESHOLD = (1 << 4),
    NIC_INT_CAUSE_LINK_CHANGE = (1 << 2),
    NIC_INT_CAUSE_TX_QUEUE_EMPTY = (1 << 1),
    NIC_INT_CAUSE_TX_WRITTEN_BACK = (1 << 0),

    NIC_INT_CAUSE_SET_MASK = 0x07 | (1 << 4) | (0x03 << 6) | (1 << 9) | (0x0f << 15) |
                             (0x1f << 20),
    NIC_INT_MASK_SET_MASK = NIC_INT_CAUSE_SET_MASK,
    NIC_INT_AUTO_MASK_MASK = NIC_INT_CAUSE_SET_MASK,

    NIC_EXT_CTRL_AUTO_MASK = (1 << 27),
    NIC_EXT_CTRL_PHY_PDEN = (1 << 20),
    NIC_EXT_CTRL_SPEED_BYPASS = (1 << 15),
    NIC_EXT_CTRL_EEPROM_RESET = (1 << 13),
    NIC_EXT_CTRL_SPEED_DETECTION = (1 << 12),
    NIC_EXT_CTRL_RESERVED = 0xfff | (1 << 14) | (1 << 16) | (1 << 18) | (1 << 21) |
                            (0x3 << 25) | (1 << 30),

    NIC_WAKEUP_CTRL_ENABLE = (1 << 0),
    NIC_WAKEUP_CTRL_STATUS = (1 << 2),
    NIC_WAKEUP_CTRL_ASSERT_PME = (1 << 3),
    NIC_WAKEUP_CTRL_LINK_WAKE_OVERRIDE = (1 << 5),
    NIC_WAKEUP_CTRL_RESERVED = ~0x3ff,

    NIC_WAKEUP_FILTER_RESERVED = (0x7f << 8) | (0xfff << 20),

    NIC_TX_DESCRIPTOR_CTRL_SET = (1 << 22),
    NIC_TX_DESCRIPTOR_CTRL_RESERVED = (3 << 6) | (3 << 14) | (1 << 23) | NIC_TX_DESCRIPTOR_CTRL_SET,

    NIC_TX_ARBITRATION_COUNT_INIT_COUNT = 3,
    NIC_TX_ARBITRATION_COUNT_ENABLE = (1 << 10),
    NIC_TX_ARBITRATION_COUNT_RESERVED = ~0x7ff,

    NIC_3GIO_CTRL_1_SELF_TEST = (1 << 30),
    NIC_3GIO_CTRL_1_LATENCY_DEFAULT = (3 << 25) | (1 << 23),  // 4 ms L1_Entry_Latency
    NIC_3GIO_CTRL_1_SOFT_INIT = (1 << 22),
    NIC_3GIO_CTRL_1_ADJUSTMENT = (1 << 9), // Rx_L0s_Adjustment
    NIC_3GIO_CTRL_1_RESERVED = (0x3ff << 10) | (0x07 << 6),

    NIC_3GIO_CTRL_2_SOFT_INIT = (1 << 31),
    NIC_3GIO_CTRL_2_RESERVED = ~NIC_3GIO_CTRL_2_SOFT_INIT,

    NIC_LED_CTRL_RESERVED = (0xff << 24) | (1 << 20) | (1 << 12) | (1 << 4),
};


#define NIC_RX_DESCRIPTOR_ADDRESS_MASK ~0x0f
#define NIC_RECEIVE_ADDR_VALID (1UL << 63)
#define NIC_RECEIVE_ADDR_SELECT_MASK (0x03UL << 48)
#define NIC_RECEIVE_ADDR_SELECT_DEST (0UL << 48)


enum {
    NIC_STATUS_DUPLEX = (1 << 0),
    NIC_STATUS_LINK_UP = (1 << 1),
    NIC_STATUS_SPEED_SHIFT = 6,
    NIC_STATUS_SPEED_MASK = (0x03 << NIC_STATUS_SPEED_SHIFT),
    NIC_STATUS_AUTO_SPEED_SHIFT = 8,
    NIC_STATUS_AUTO_SPEED_MASK = (0x03 << NIC_STATUS_AUTO_SPEED_SHIFT),
    NIC_STATUS_PHYRA = (1 << 10),
    NIC_STATUS_MASTER_ENABLE_STATUS = (1 << 19),
};


enum {
    NIC_CTRL_FULL_DUPLEX = (1 << 0),
    NIC_CTRL_MASTER_DISABLE = (1 << 2),
    NIC_CTRL_AUTO_SPPED = (1 << 5),
    NIC_CTRL_SET_LINK_UP = (1 << 6),
    NIC_CTRL_SPEED_SHIFT = 8,
    NIC_CTRL_SPEED_MASK = (0x03 << NIC_CTRL_SPEED_SHIFT),
    NIC_CTRL_FORCE_SPEED = (1 << 11),
    NIC_CTRL_FORCE_DUPLEX = (1 << 12),
    NIC_CTRL_ADVD3WUC = (1 << 20),
    NIC_CTRL_SOFT_RESET = (1 << 26), // self-clearing
    NIC_CTRL_PHY_RESET = (1 << 31),

    NIC_CTRL_RESREVED_SET = (1 << 3),
    NIC_CTRL_RESREVED_MASK = NIC_CTRL_RESREVED_SET | (1 << 1) | (1 << 4) | (1 << 7) |(1 << 10) |
                             (0x7f << 13) | (0x1f << 21) | (1 << 29),
};


enum {
    STAT_OFFSET_RX_MISSED_PACKETS = 0x10 / 4,
    STAT_OFFSET_RX_SIZE_ERR = 0x40 / 4,
    STAT_OFFSET_RX_64 = 0x5c / 4,
    STAT_OFFSET_RX_128 = 0x60 / 4,
    STAT_OFFSET_RX_256 = 0x64 / 4,
    STAT_OFFSET_RX_512 = 0x68 / 4,
    STAT_OFFSET_RX_1024 = 0x6c / 4,
    STAT_OFFSET_RX_GOOD_PACKETS = 0x74 / 4,
    STAT_OFFSET_RX_BROADCAST = 0x78 / 4,
    STAT_OFFSET_RX_MULTICAST = 0x7c / 4,
    STAT_OFFSET_RX_GOOD_OCTETS_LOW = 0x88 / 4,
    STAT_OFFSET_RX_UNDERSIZE = 0xa4 / 4,
    STAT_OFFSET_RX_OVERSIZE = 0xac / 4,
    STAT_OFFSET_RX_TOTAL_OCTETS_LOW = 0xc0 / 4,
    STAT_OFFSET_RX_TOTAL_PACKETS = 0xd0 / 4,
};


enum {
    PHY_REG_CTRL = 0,
    PHY_REG_STATUS = 1,
    PHY_REG_ID1 = 2,
    PHY_REG_ID2 = 3,
    PHY_REG_AUTO_NEGO_ADVERTIS = 4,
    PHY_REG_PARTNER_ABILITY = 5,
    PHY_REG_NEGO_EXP = 6,
    PHY_REG_1000_CTRL = 9,
    PHY_REG_1000_STATUS = 10,
    PHY_REG_EXT_STATUS = 15,
    PHY_REG_P0_COPPER_CTRL_1 = 16,
    PHY_REG_P0_COPPER_STATUS_1 = 17,
    PHY_REG_P0_OEM_BITS = 25,
    PHY_REG_P0_BIAS_1 = 29,
    PHY_REG_P0_BIAS_2 = 30,
    PHY_REG_ERR_COUNT = 21,
    PHY_REG_PAGE = 22,

    PHY_ID1 = 0x0141,

    PHY_ID2_OUI_LSB = 0x03,
    PHY_ID2_MODEL = 0x0b,
    PHY_ID2_REVISION = 1,
    PHY_ID2_REVISION_MASK = 0x0f,

    PHY_CTRL_RESET = (1 << 15),
    PHY_CTRL_LOOPBACK = (1 << 14),
    PHY_CTRL_SPEED_LSB_SHIFT = 13,
    PHY_CTRL_SPEED_LSB = (1 << PHY_CTRL_SPEED_LSB_SHIFT),
    PHY_CTRL_AUTO_NEGOTIATION = (1 << 12),
    PHY_CTRL_POWER_DOWN = (1 << 11),
    PHY_CTRL_RESTART_AUTO_NEGOTIATION = (1 << 9),
    PHY_CTRL_DUPLEX_MODE = (1 << 8),
    PHY_CTRL_SPEED_MSB_SHIFT = 6,
    PHY_CTRL_SPEED_MSB = (1 << PHY_CTRL_SPEED_MSB_SHIFT),
    PHY_CTRL_RESERVED = (1 << 10) | (1 << 7) | 0x3f,

    PHY_AUTO_NEGO_ASYMMETRIC_PAUSE = (1 << 11),
    PHY_AUTO_NEGO_PAUSE = (1 << 10),
    PHY_AUTO_NEGO_100BASE_T4 = (1 << 9),
    PHY_AUTO_NEGO_100FD_ADVERTIS = (1 << 8),
    PHY_AUTO_NEGO_100HD_ADVERTIS = (1 << 7),
    PHY_AUTO_NEGO_10FD_ADVERTIS = (1 << 6),
    PHY_AUTO_NEGO_10HD_ADVERTIS = (1 << 5),
    PHY_AUTO_NEGO_SELECTOR = 1,
    PHY_AUTO_NEGO_SELECTOR_MASK = 0x1f,
    PHY_AUTO_NEGO_RESERVED = (1 << 14) | (1 << 12),

    PHY_1000_CTRL_TEST_MASK = (0x07 << 13),
    PHY_1000_CTRL_1000FD_ADVERTIS = (1 << 9),
    PHY_1000_CTRL_1000HD_ADVERTIS = (1 << 8),
    PHY_1000_CTRL_RESERVED = 0xff,

    PHY_1000_STATUS_LOCAL_RECEIVER_OPERATIONAL = (1 << 13),
    PHY_1000_STATUS_REMOTE_RECEIVER_OPERATIONAL = (1 << 12),
    PHY_1000_STATUS_PARTNER_CAPABLE_1000FD = (1 << 11),
    PHY_1000_STATUS_PARTNER_CAPABLE_1000HD = (1 << 10),

    PHY_STATUS_100FD = (1 << 14),
    PHY_STATUS_100HD = (1 << 13),
    PHY_STATUS_10FD = (1 << 12),
    PHY_STATUS_10HD = (1 << 11),
    PHY_STATUS_EXT_STATUS = (1 << 8),
    PHY_STATUS_MF = (1 << 6),
    PHY_STATUS_AUTO_NEGO_COMPLETE = (1 << 5),
    PHY_STATUS_AUTO_NEGO_CAP = (1 << 3),
    PHY_STATUS_LINK_UP = (1 << 2),
    PHY_STATUS_EXT_CAP = (1 << 0),

    PHY_PARTNER_ABILITY_100FD = (1 << 8),
    PHY_PARTNER_ABILITY_100HD = (1 << 7),
    PHY_PARTNER_ABILITY_10FD = (1 << 6),
    PHY_PARTNER_ABILITY_10HD = (1 << 5),
    PHY_PARTNER_SELECTOR = 1,

    PHY_NEGO_EXP_LOCAL_NEXT = (1 << 2),
    PHY_NEGO_EXP_PARTNER_NEGO_ABLE = (1 << 0),

    PHY_EXT_STATUS_1000FD_ABLE = (1 << 13),
    PHY_EXT_STATUS_1000HD_ABLE = (1 << 12),

    PHY_P0_COPPER_CTRL_1_DISABLE_LINK_PULSES = (1 << 15),
    PHY_P0_COPPER_CTRL_1_DOWNSHIFT_COUNTER_SHIFT = 12,
    PHY_P0_COPPER_CTRL_1_FORCE_LINK_GOOD = (1 << 10),
    PHY_P0_COPPER_CTRL_1_ENABLE_EXTENDED_DISTANCE = (1 << 7),
    PHY_P0_COPPER_CTRL_1_CROSSOVER_MODE_SHIFT = 5,
    PHY_P0_COPPER_CTRL_1_TRANSMITTER_DISABLE = (1 << 3),
    PHY_P0_COPPER_CTRL_1_POWER_DOWN = (1 << 2),
    PHY_P0_COPPER_CTRL_1_POLARITY_REVERSAL_DISABLE = (1 << 1),
    PHY_P0_COPPER_CTRL_1_DISABLE_JABBER = (1 << 0),

    PHY_P0_COPPER_STATUS_1_SPEED_SHIFT = 14,
    PHY_P0_COPPER_STATUS_1_SPEED_MASK = (0x3 << PHY_P0_COPPER_STATUS_1_SPEED_SHIFT),
    PHY_P0_COPPER_STATUS_1_DUPLEX = (1 << 13),
    PHY_P0_COPPER_STATUS_1_RESOLVED = (1 << 11),
    PHY_P0_COPPER_STATUS_1_LINK_UP = (1 << 10),
    PHY_P0_COPPER_STATUS_1_MSI_CROSSOVER = (1 << 6),
    PHY_P0_COPPER_STATUS_1_GLOBAL_LINK = (1 << 3),

    PHY_P0_OEM_BITS_RESTART_NEGO = (1 << 10),
    PHY_P0_OEM_BITS_GBE_DISABLE = (1 << 6),
    PHY_P0_OEM_BITS_LPLU = (1 << 2),
};


enum {
    NIC_SPEED_10 = 0,
    NIC_SPEED_100 = 1,
    NIC_SPEED_1000 = 2,
};


enum {
    NV_WORD_ADDRESS_0 = 0x00,
    NV_WORD_ADDRESS_1 = 0x01,
    NV_WORD_ADDRESS_2 = 0x02,
    NV_WORD_INIT_CTRL_1 = 0x0a,
    NV_WORD_SUBSYS_ID = 0x0b,
    NV_WORD_SUBSYS_VENDOR = 0x0c,
    NV_WORD_DEV_ID = 0x0d,
    NV_WORD_INIT_CTRL_2 = 0x0f,
    NV_WORD_PROTECT_0 = 0x10,
    NV_WORD_PROTECT_1 = 0x11,
    NV_WORD_PROTECT_2 = 0x12,
    NV_WORD_EXT_CONF_1 = 0x14,
    NV_WORD_EXT_CONF_2 = 0x15,
    NV_WORD_EXT_CONF_3 = 0x16,
    NV_WORD_PCIE_ELECT_IDLE_DELAY = 0x17,
    NV_WORD_PCIE_INIT_CONF_1 = 0x18,
    NV_WORD_PCIE_INIT_CONF_2 = 0x19,
    NV_WORD_PCIE_INIT_CONF_3 = 0x1a,
    NV_WORD_PCIE_CTRL = 0x1b,
    NV_WORD_LED1_PHY_CONF = 0x1c,
    NV_WORD_RESRVED = 0x1d,
    NV_WORD_REV_ID = 0x1e,
    NV_WORD_LED0_LED2 = 0x1f,
    NV_WORD_POWER = 0x22,
    NV_WORD_INIT_CTRL_3 = 0x24,

    NV_WORD_SUM = 0x3f,

    NV_INIT_CTRL_1_FORCE_SPEED = (1 <<  11),
    NV_INIT_CTRL_1_FORCE_DUPLEX = (1 <<  10),
    NV_INIT_CTRL_1_ILOS = (1 << 4),
    NV_INIT_CTRL_1_LOAD_SUB_ID = (1 << 1),
    NV_INIT_CTRL_1_LOAD_DEV_ID = (1 << 0),

    NV_INIT_CTRL_1_SET = (1 << 3) | (1 << 5) | (1 << 6) | (1 << 9),

    NV_INIT_CTRL_2_WUC_APMPME = (1 <<  15),
    NV_INIT_CTRL_2_NV_TYPE = (1 <<  12),
    NV_INIT_CTRL_2_NV_SIZE_SHIFT = 8,
    NV_INIT_CTRL_2_NV_SIZE = 1, // 256 bytes
    NV_INIT_CTRL_2_NV_SIZE_MASK = (0x0f << NV_INIT_CTRL_2_NV_SIZE_SHIFT),
    NV_INIT_CTRL_2_SET = (1 << 3) | (1 << 4) | (1 << 6),

    NV_INIT_CTRL_3_WAKE_UP_ENABLE = (1 << 10),
    NV_INIT_CTRL_3_NO_PHY_RESET = (1 << 0),
    NV_INIT_CTRL_3_SET = (1 << 14) | (1 << 11) | (0x03 << 8),

    NV_PROTECT_2_SIGNATURE = 0x7e,
    NV_PROTECT_2_SIGNATURE_SHIFT = 8,
    NV_PROTECT_2_SECTOR_SIZE = 0x03, // Reserved
    NV_PROTECT_2_SET= (0x07 << 4),
    NV_PROTECT_2_SECTOR_SIZE_SHIFT = 2,

    NV_LED1_PHY_GIGA_DISABLE = (1 << 14),

    NV_LED1_PHY_ND0_NO_GB = (1 << 11),
    NV_LED1_PHY_ND0_DEC_SPEED = (1 << 10),
    NV_LED1_PHY_D0_DEC_SPEED = (1 << 9),
    NV_LED1_BLINK = (1 << 7),
    NV_LED1_INVERT = (1 << 6),
    NV_LED1_BLINK_MODE = (1 << 5),
    NV_LED1_INIT_MODE = 4,

    NV_LED1_RESERVED_SET = (1 << 13) | (1 << 8),

    NV_LED0_L2_BLINK = (1 << 15),
    NV_LED0_L2_INVERT = (1 << 14),
    NV_LED0_L2_BLINK_MODE = (1 << 13),
    NV_LED0_L2_INIT_MODE_SHIFT = 8,
    NV_LED0_L2_INIT_MODE = 7,
    NV_LED0_L0_BLINK = (1 << 7),
    NV_LED0_L0_INVERT = (1 << 6),
    NV_LED0_L0_BLINK_MODE = (1 << 5),
    NV_LED0_L0_INIT_MODE = 6,
};


enum {
    NIC_MIN_PACKET_SIZE = 64,
    NIC_MAX_PACKET_SIZE = 1522,
    NIC_MAX_LONG_PACKET_SIZE = 16384,

    ETHER_HEADER_LENGTH = 14,
    ETHER_ADDRESS_LENGTH = 6,
    ETHER_CRC_SIZE = 4,

    ETHER_TYPE_ARP = 0x0806,
    ETHER_TYPE_VLAN = 0x8100,
    ETHER_TYPE_IPV6 = 0x86dd,

    IP6_HEADER_LENGTH = 40,
};


enum {
    TR_INIT,
    TR_WAITING,
    TR_RUNNING,
    TR_TERMINATED,
};


enum {
    TR_CMD_WAIT,
    TR_CMD_RUN,
    TR_CMD_QUIT,
};

typedef struct __attribute__ ((__packed__)) CommonTxDescriptor {
    uint8_t specific_0[11];
    uint8_t command;
    uint8_t status;
    uint8_t specific_1[3];
} CommonTxDescriptor;


typedef struct __attribute__ ((__packed__)) LegacyTxDescriptor {
    uint64_t address;
    uint16_t length;
    uint8_t checksum_offset;
    uint8_t command;
    uint8_t status;
    uint8_t checksum_start;
    uint16_t vlan; /*If the packet type is 802.1q and RCTL.VME = 1b, then the VLAN Tag field
                     records the VLAN information and the four byte VLAN information is stripped
                     from the packet data storage. Otherwise, the special field contains 0000h

                     (12bit VLAN Identifier + 1bit CFI + PRI 3bit Priority)
                   */
} LegacyTxDescriptor;


enum {
    TX_DESCRIPTOR_CMD_IDE = (1 << 7),
    TX_DESCRIPTOR_CMD_VLAN = (1 << 6),
    TX_DESCRIPTOR_CMD_EXTENSION = (1 << 5),
    TX_DESCRIPTOR_CMD_REPORT_STATUS = (1 << 3),
    TX_DESCRIPTOR_CMD_TSE = (1 << 2),
    TX_DESCRIPTOR_INSERT_CHECKSUM = (1 << 2),
    TX_DESCRIPTOR_CMD_IP4 = (1 << 1),
    TX_DESCRIPTOR_INSERT_FCS = (1 << 1),
    TX_DESCRIPTOR_END_OF_PACKET = (1 << 0),
    TX_DESCRIPTOR_CMD_TCP = (1 << 0),

    TX_DESCRIPTOR_STATUS_DD = (1 << 0),
    TX_DESCRIPTOR_STATUS_EXTCMD_TS = (1 << 4),

    TX_EXT_DESC_TYPE_SHIFT = 20,
    TX_EXT_DESC_TYPE_MASK = (0x0f << TX_EXT_DESC_TYPE_SHIFT),
    TX_EXT_DESC_TYPE_CONTEXT = 0,
    TX_EXT_DESC_TYPE_DATA = 1,

    TX_DESCRIPTOR_POPTS_IXSM = (1 << 0),
    TX_DESCRIPTOR_POPTS_TXSM = (1 << 1),
};


typedef struct __attribute__ ((__packed__)) TxDataDescriptor {
    uint64_t address;
    union {
        uint32_t mix;
        uint8_t mix_v[3];
    };
    uint8_t status;
    uint8_t packet_opt;
    uint16_t vlan;
} TxDataDescriptor;


typedef struct __attribute__ ((__packed__)) LegacyRxDescriptor {
    uint64_t address;
    uint16_t length;
    uint16_t checksum;
    uint8_t status;
    uint8_t error;
    uint16_t vlan;
} LegacyRxDescriptor;


typedef struct __attribute__ ((__packed__)) RxExtReadDescriptor {
    uint64_t address;
    uint64_t status;
} RxExtReadDescriptor;


typedef struct __attribute__ ((__packed__)) RxExtWBDescriptor {
    uint32_t mrq;
    union {
        uint32_t rss;
        struct __attribute__ ((__packed__)) {
            uint16_t ip_identification;
            uint16_t packet_checksum;
        } no_rss;
    };
    uint32_t status;
    uint16_t length;
    uint16_t vlan;
} RxExtWBDescriptor;


enum {
    RX_DESCRIPTOR_STATUS_DD = (1 << 0),
    RX_DESCRIPTOR_STATUS_EOP = (1 << 1),
};


static const uint8_t mac_address[] = { 0x02, 0xf1, 0xf2, 0xf3, 0xf4, 0xff};


static uint16_t checksum16(void* start, uint size)
{
    uint16_t res = 0;
    uint16_t* now = (uint16_t*)start;
    uint16_t* end = now + size;

    for (; now < end; now++) {
        res += *now;
    }

    return res;
}


uint32_t net_sum16(uint8_t* buf, int len)
{
    uint16_t* header = (uint16_t*)buf;
    uint32_t sum = 0;

    for (int i = 0; i < len / 2; ++i) {
        sum += ntohs(header[i]);
    }

    if (len & 1) {
        sum += ((uint16_t)buf[len - 1]) << 8;
    }

    return sum;
}


static inline uint32_t combine32(uint32_t val, uint32_t mask, uint32_t store)
{
    return (val & ~mask) | (store & mask);
}


static void debug_attention()
{
    //raise(SIGTRAP);
}


static void debug_unhandled_attention()
{
    //raise(SIGTRAP);
}


void NIC::Queue::reset(uint32_t qx_cause_mask)
{
    _public_head = 0;
    _private_head = 0;
    _tail = 0;
    _length = 0;
    _address &= NIC_RX_DESCRIPTOR_ADDRESS_MASK;
    _q_size = 1;
    _qx_cause_mask = qx_cause_mask;
    _descriptor_ctrl = NIC_TX_DESCRIPTOR_CTRL_SET;
}


void NIC::Queue::begin()
{
    _q_size = MAX(1, _length / 16);
    _private_head = _public_head % _q_size;
}


void NIC::Queue::set_addr_low(uint32_t val)
{
    _address &= ~0xffffffff;
    _address |= (val & NIC_RX_DESCRIPTOR_ADDRESS_MASK);
}


void NIC::Queue::set_addr_high(uint32_t val)
{
    _address &= 0xffffffff;
    _address |= (uint64_t(val) << 32);
}


NIC::NicTimer::NicTimer(const char* name, Timer* a, Timer* b)
    : _name (name)
    , _timer (a)
    , _abs_timer (b)
{
}


NIC::NicTimer::~NicTimer()
{
    _timer->destroy();
    _abs_timer->destroy();
}


void NIC::NicTimer::reset()
{
    disarm();
    _delay_val = 0;
    _abs_delay_val = 0;
}


inline void NIC::NicTimer::disarm()
{
    NIC_LOG("%s", _name.c_str());

    _timer->disarm();
    _abs_timer->disarm();
    _abs_timer_armed = false;
}


void NIC::NicTimer::resume()
{
    _timer->resume();
    _abs_timer->resume();
}


void NIC::NicTimer::suspend()
{
    _timer->suspend();
    _abs_timer->suspend();
}


void NIC::NicTimer::set_val(uint32_t val)
{
    _delay_val = (val & 0xffff) * 1024;
}


void NIC::NicTimer::set_abs_val(uint32_t val)
{
    _abs_delay_val = (val & 0xffff) * 1024;
}


inline void NIC::NicTimer::arm()
{
    NIC_LOG("%s", _name.c_str());

    if (_delay_val) {
        NIC_LOG("%s: rel %u", _name.c_str(), _delay_val);
        _timer->arm(_delay_val, false);
    }

    if (!_abs_timer_armed && _abs_delay_val) {
        NIC_LOG("%s: abs %u", _name.c_str(), _abs_delay_val);
        _abs_timer->arm(_abs_delay_val, false);
        _abs_timer_armed = true;
    }
}


NIC::NIC(NoxVM& nox)
    : PCIDevice("nic", *pci_bus, NIC_VENDOR_ID, NIC_DEV_ID, NIC_DEV_REVISION,
                mk_pci_class_code(PCI_CLASS_NIC, PCI_SUBCLASS_ETHERNET, PCI_PROGIF_ETHERNET),
                true)
    , _tr_state (TR_INIT)
    , _tr_command (TR_CMD_WAIT)
    , _tx_trigger (_transceiver.create_event((void_callback_t)&NIC::tx_trigger_handler, this))
    , _rx_trigger (_transceiver.create_event((void_callback_t)&NIC::rx_trigger_handler, this))
    , _interface_event (NULL)
    , _tx_timer ("tx",_transceiver.create_timer((void_callback_t)&NIC::tx_timer_handler, this),
                 _transceiver.create_timer((void_callback_t)&NIC::tx_timer_handler, this))
    , _rx_timer ("rx",_transceiver.create_timer((void_callback_t)&NIC::rx_timer_handler, this),
                 _transceiver.create_timer((void_callback_t)&NIC::rx_timer_handler, this))
    , _thread ((Thread::start_proc_t)&NIC::thread_main, this)
{
    ASSERT(NUM_STATISTIC_REGS == (NIC_REG_STAT_END - NIC_REG_STAT_START) / 4 + 1);
    add_mmio_region(NIC_CSR_BAR, NIC_CSR_SPACE_SIZE, this,
                    (read_mem_proc_t)&NIC::csr_read,
                    (write_mem_proc_t)&NIC::csr_write, false);

    add_io_region(NIC_IO_BAR, NIC_IO_SPACE_SIZE, this, NULL, NULL, NULL, NULL,
                  (io_read_dword_proc_t)&NIC::io_read_dword,
                  (io_write_dword_proc_t)&NIC::io_write_dword);

    pci_bus->add_device(*this);

    init_eeprom();

    tr_wait(TR_WAITING);
}


NIC::~NIC()
{
    Lock lock(_tr_cmd_lock);
    _tr_command = TR_CMD_QUIT;
    _tr_cmd_condition.signal();
    lock.unlock();
    _transceiver.run_break();
    _thread.join();
    _tx_trigger->destroy();
    _rx_trigger->destroy();

    if (_interface_event) {
        _interface_event->destroy();
    }
}


void NIC::tr_cmd(uint cmd)
{
    Lock lock(_tr_cmd_lock);
    _tr_command = cmd;
    _tr_cmd_condition.signal();
}


void NIC::tr_wait(uint state)
{
    Lock lock(_tr_state_lock);

    while (_tr_state != state) {
        _tr_state_condition.wait(_tr_state_lock);
    }
}


void NIC::tr_set_state(int state)
{
    Lock cmd_lock(_tr_state_lock);
    _tr_state = state;
    _tr_state_condition.signal();
}


bool NIC::start()
{
    if (!PCIDevice::start()) {
        return false;
    }

    tr_cmd(TR_CMD_RUN);

    update_interrupt_level();

    _tx_timer.resume();
    _rx_timer.resume();

    return true;
}


bool NIC::stop()
{
    Lock lock(_tr_cmd_lock);
    _tr_command = TR_CMD_WAIT;
    _tr_cmd_condition.signal();
    lock.unlock();
    _transceiver.run_break();

    Lock state_lock(_tr_state_lock);

    while (_tr_state != TR_WAITING) {
        _tr_state_condition.wait(_tr_state_lock);
    }

    state_lock.unlock();

    _tx_timer.suspend();
    _rx_timer.suspend();

    return PCIDevice::stop();
}


void NIC::down()
{
    Lock lock(_tr_cmd_lock);
    _tr_command = TR_CMD_QUIT;
    _tr_cmd_condition.signal();
    lock.unlock();

    PCIDevice::down();
}


void NIC::tx_write_back(Queue& queue)
{
    if (!queue.get_wb_count()) {
        return;
    }

    NIC_LOG("q[%u] head %u private %u tail %u", &queue - _tx_queue,
            queue.get_public_head(),
            queue.get_private_head(),
            queue.get_tail());

    queue.relase_used();
    interrupt(queue.get_qx_cause_mask() | NIC_INT_CAUSE_TX_WRITTEN_BACK |
              (queue.is_empty() ? NIC_INT_CAUSE_TX_QUEUE_EMPTY : 0));
}


void NIC::tx_write_back()
{
    NIC_LOG("");

    _tx_timer.disarm();
    tx_write_back(_tx_queue[0]);
    tx_write_back(_tx_queue[1]);
}


void NIC::tx_timer_handler()
{
    Lock lock(_mutex);
    NIC_LOG("");
    tx_write_back();
}


void NIC::rx_write_back(Queue& queue, uint32_t cause)
{
    if (!queue.get_wb_count()) {
        return;
    }

    NIC_LOG("q[%u] head %u private %u tail %u", &queue - _rx_queue,
            queue.get_public_head(),
            queue.get_private_head(),
            queue.get_tail());

    queue.relase_used();
    interrupt(queue.get_qx_cause_mask() | cause);
}


void NIC::rx_write_back(uint32_t cause)
{
    NIC_LOG("");

    _rx_timer.disarm();
    rx_write_back(_rx_queue[0], cause);
    rx_write_back(_rx_queue[1], cause);
}


void NIC::rx_timer_handler()
{
    Lock lock(_mutex);
    NIC_LOG("");
    rx_write_back(NIC_INT_CAUSE_RX_TIMER);
}


inline void NIC::handle_legacy_tx(LegacyTxDescriptor& descriptor)
{
    if (!descriptor.address || !descriptor.length) {
        debug_unhandled_attention();
    }

    std::auto_ptr<DirectAccess> direct(memory_bus->get_direct(descriptor.address,
                                                              descriptor.length));

    if (!direct.get()) {
        debug_unhandled_attention();
    }

    uint8_t *buf = direct->get_ptr();

    if (!(descriptor.command & TX_DESCRIPTOR_END_OF_PACKET)) {
        debug_unhandled_attention();
    }

    if ((descriptor.command & TX_DESCRIPTOR_CMD_VLAN)) {
        debug_unhandled_attention();
    }

    if ((descriptor.command & TX_DESCRIPTOR_INSERT_CHECKSUM)) {
        debug_unhandled_attention();
    }

    if (!(descriptor.command & TX_DESCRIPTOR_INSERT_FCS)) {
        debug_unhandled_attention();
    }

    if ((descriptor.status & TX_DESCRIPTOR_STATUS_EXTCMD_TS)) {
        debug_unhandled_attention();
    }

    if (write(_interface.get(), buf, descriptor.length) != descriptor.length) {
        debug_unhandled_attention();
    }
}


inline void NIC::handle_tx_context(TxContextDescriptor& descriptor)
{
    if ((descriptor.mix_v[3] & TX_DESCRIPTOR_CMD_TSE)) {
        _seg_context = descriptor;
    } else {
        _offload_context = descriptor;
    }
}


static uint32_t insert_cheacksum(uint offset, uint start, uint end, uint8_t* buf,
                                 uint buf_length)
{
    if (offset + 2 > buf_length || start + 2 > buf_length) {
        W_MESSAGE("bad values: start %u offset %u buf length %u", start, offset, buf_length);
        return 0;
    }

    if (!end) {
        end = buf_length;
    } if (end > buf_length || end <= start) {
        W_MESSAGE("bad values: start %u end %u buf length %u", start, end, buf_length);
        return 0;
    }

    uint32_t sum = net_sum16(buf + start, end - start);

    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }

    sum = (sum) ? ~sum : 0xffff;

    uint16_t* sum_ptr = (uint16_t*)(buf + offset);

    uint32_t save = (1 << 31) | *sum_ptr;
    *sum_ptr = htons(sum);

    return save;
}


static inline void restore_cheacksum(uint32_t data, uint8_t offset, uint8_t* buf)
{
    if (!(data & (1 << 31))) {
        return;
    }

    *(uint16_t*)(buf + offset) = data;
}


inline void NIC::handle_tx_data(TxDataDescriptor& descriptor)
{
    uint length = descriptor.mix & 0xfffff;

    if (!length || !descriptor.address) {
        debug_unhandled_attention();
    }

    std::auto_ptr<DirectAccess> direct(memory_bus->get_direct(descriptor.address, length));

    if (!direct.get()) {
        debug_unhandled_attention();
    }

    if (!(descriptor.mix_v[3] & TX_DESCRIPTOR_END_OF_PACKET)) {
        debug_unhandled_attention();
    }

    if ((descriptor.mix_v[3] & TX_DESCRIPTOR_CMD_VLAN)) {
        debug_unhandled_attention();
    }

    if (!(descriptor.mix_v[3] & TX_DESCRIPTOR_INSERT_FCS)) {
        debug_unhandled_attention();
    }

    if ((descriptor.mix_v[3] & TX_DESCRIPTOR_CMD_TSE)) {
        debug_unhandled_attention();
    }

    if ((descriptor.status & TX_DESCRIPTOR_STATUS_EXTCMD_TS)) {
        debug_unhandled_attention();
    }

    uint8_t *buf = direct->get_ptr();

    uint32_t ipfix = (descriptor.packet_opt & TX_DESCRIPTOR_POPTS_IXSM) ?
                                insert_cheacksum(_offload_context.ipcso, _offload_context.ipcss,
                                                 _offload_context.ipcse, buf, length) : 0;
    uint32_t tufix = (descriptor.packet_opt & TX_DESCRIPTOR_POPTS_TXSM) ?
                                insert_cheacksum(_offload_context.tucso, _offload_context.tucss,
                                                 _offload_context.tucse, buf, length) : 0;

    uint n = write(_interface.get(), buf, length);

    restore_cheacksum(ipfix, _offload_context.ipcso, buf);
    restore_cheacksum(tufix, _offload_context.tucso, buf);

    if (n != length) {
        debug_unhandled_attention();
    }
}


bool NIC::do_tx(Queue& queue)
{
    Lock lock(_mutex);

    if (!phy_link_is_up() || !(_tx_ctrl & NIC_TX_CTRL_ENABLE)) {
        return false;
    }

    if (queue.is_empty()) {
        NIC_LOG("q[%u]: empty", &queue - _tx_queue);
        tx_write_back(queue);
        return false;;
    }

    NIC_LOG("q[%u] head %u private %u tail %u ctrl 0x%x", &queue - _tx_queue,
            queue.get_public_head(),
            queue.get_private_head(),
            queue.get_tail(),
            queue.get_descriptor_ctrl());

    CommonTxDescriptor descriptor;
    uint64_t descriptor_address = queue.head_address();

    NIC_LOG("q[%u] address  0x%lx base 0x%lx size %u", &queue - _tx_queue,
                descriptor_address,
                queue.get_base_address(),
                queue.num_items());

    memory_bus->read(descriptor_address, sizeof(descriptor), &descriptor);

    if (!(descriptor.command & TX_DESCRIPTOR_CMD_EXTENSION)) {
        handle_legacy_tx((LegacyTxDescriptor&)descriptor);
    } else {
        switch (descriptor.specific_0[10] >> 4) {
        case TX_EXT_DESC_TYPE_CONTEXT:
            handle_tx_context((TxContextDescriptor&)descriptor);
            break;
        case TX_EXT_DESC_TYPE_DATA: {
            handle_tx_data((TxDataDescriptor&)descriptor);
            break;
        }
        default:
            W_MESSAGE("invalid ext descriptor type", descriptor.specific_0[10] >> 4);
            debug_unhandled_attention();
        };
    }

    queue.pop();

    NIC_LOG("q[%u] POP head %u private %u tail %u", &queue - _tx_queue,
                queue.get_public_head(),
                queue.get_private_head(),
                queue.get_tail());

    if (descriptor.command & TX_DESCRIPTOR_CMD_REPORT_STATUS) {
        descriptor.status |= TX_DESCRIPTOR_STATUS_DD;
        memory_bus->write(&descriptor, sizeof(descriptor), descriptor_address);
        NIC_LOG("q[%u] DD", &queue - _tx_queue);
    }

    if (queue.get_low_Threshold() && queue.get_unused_count() == queue.get_low_Threshold() * 8) {
        interrupt(NIC_INT_CAUSE_TXD_LOW);
        NIC_LOG("q[%u] low Threshold hit", &queue - _tx_queue);
    }

    if ((descriptor.command & TX_DESCRIPTOR_CMD_IDE)) {
        NIC_LOG("q[%u] IDE", &queue - _tx_queue);
        _tx_timer.arm();
    }

    if (queue.get_wb_threshold()) {
        NIC_LOG("q[%u] wb threshold", &queue - _tx_queue);
        if (queue.get_wb_count() == queue.get_wb_threshold()) {
            NIC_LOG("q[%u] write back Threshold hit", &queue - _tx_queue);
            tx_write_back(queue);
        }
    } else if ((descriptor.command & TX_DESCRIPTOR_CMD_REPORT_STATUS) &&
               !(descriptor.command & TX_DESCRIPTOR_CMD_IDE)) {
        NIC_LOG("q[%u] imidiate", &queue - _tx_queue);
        tx_write_back();
    }

    return true;
}


void NIC::trancive()
{
    bool more;

    do {
        more = recive_data();
        _transceiver.run_timers();
        more = do_tx(_tx_queue[0]) || more;
        _transceiver.run_timers();
        more = do_tx(_tx_queue[1]) || more;
        _transceiver.run_timers();
        pthread_yield();
    } while (more);
}


void NIC::tx_trigger_handler()
{
    NIC_LOG("");
    trancive();
}


void NIC::rx_trigger_handler()
{
    NIC_LOG("");
    trancive();
}


static void ether_addr_to_str(std::string& str, uint8_t* mac)
{
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


static void ether_to_str(std::string& str, uint8_t* buf, uint length)
{
    if (length < ETHER_HEADER_LENGTH) {
        sprintf(str, "bad ether: too short (%u)", length);
    }

    std::string dest;
    ether_addr_to_str(dest, buf);
    std::string source;
    ether_addr_to_str(source, buf + 6);
    uint16_t type = ntohs(*(uint16_t*)(buf + 12));
    std::string type_str;
    switch (type) {
    case ETHER_TYPE_IPV6:
        if (length < ETHER_HEADER_LENGTH + IP6_HEADER_LENGTH ) {
            type_str = "ERR too short";
            break;
        }
        sprintf(type_str, "IPv6 - payload %u", ntohs(*(uint16_t*)(buf + ETHER_HEADER_LENGTH + 4)));
        break;
    case ETHER_TYPE_ARP:
        sprintf(type_str, "ARP");
        break;
    case ETHER_TYPE_VLAN:
        sprintf(type_str, "VLAN");
        break;
    default:
        type_str = "?";
    };

    sprintf(str, "length %u dest %s source %s type 0x%02x%02x: %s",
            length, dest.c_str(), source.c_str(), buf[12], buf[13],
            type_str.c_str());
}


// broadcast  FF:FF:FF:FF:FF:FF
// multicast  x1:xx:xx:xx:xx:xx
//             ^
//             bit XXXXXXX1b

static const uint8_t ether_broadcast_addr[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static const uint pointer_offset_table[] = {32, 34, 35, 36};

bool NIC::ether_filter(uint8_t* address)
{
    if ((address[0] & 1)) {
        // multicate or broadcast

        if ((_rx_ctrl & NIC_RX_CTRL_PROMISCUOUS_MULTICAST)) {
            return true;
        }

        if (memcmp(address, ether_broadcast_addr, ETHER_ADDRESS_LENGTH) == 0 &&
                                            (_rx_ctrl & NIC_RX_CTRL_ACCEPT_BROADCAST)) {
            return true;
        }

        uint pointer_offset = (_rx_ctrl & NIC_RX_CTRL_MULTICAST_OFFSET_MASK);
        pointer_offset >>= NIC_RX_CTRL_MULTICAST_OFFSET_SHIFT;
        pointer_offset = pointer_offset_table[pointer_offset];
        uint pointer = (*(uint64_t*)address >> pointer_offset) & 0xfff;

        return !!(_multicast_table[pointer >> 5] & (1 << (pointer & 0x1f)));

    }

    if ((_rx_ctrl & NIC_RX_CTRL_PROMISCUOUS_UNICAST)) {
        return true;
    }

    uint64_t* addr_table = (uint64_t*)_receive_addr_table;

    for (uint i = 0; i < NUM_RECEIVE_ADDR - 1 /*The software device driver can use only
                                                 entries 0-14. Entry 15 is reserved for
                                                 manageability firmware usage.*/; i++) {
        if ((addr_table[i] & (NIC_RECEIVE_ADDR_VALID | NIC_RECEIVE_ADDR_SELECT_MASK)) !=
                                          (NIC_RECEIVE_ADDR_VALID | NIC_RECEIVE_ADDR_SELECT_DEST)) {
            continue;
        }

        if (memcmp(address, addr_table, ETHER_ADDRESS_LENGTH) == 0) {
            return true;
        }
    }

    return false;
}


void NIC::push(uint8_t* packet, uint length)
{
    Lock lock(_mutex);

    // todo: handle receive filter control register (RFCTL)
    // todo: handle TCP ACK

    if (!phy_link_is_up() || !(_tx_ctrl & NIC_RX_CTRL_ENABLE)) {
        return;
    }

    ++_statistic[STAT_OFFSET_RX_TOTAL_PACKETS];
    uint64_t* stat_64 = (uint64_t*)&_statistic[STAT_OFFSET_RX_TOTAL_OCTETS_LOW];
    *stat_64 += length;

    if ((_multi_rq_command & NIC_MULTI_RQ_CMD_ENABLE_MASK) == 1) {
        debug_unhandled_attention();
    }

    if (_rx_queue[0].is_empty()) {
        NIC_LOG("empty")
        ++_statistic[STAT_OFFSET_RX_MISSED_PACKETS];
        interrupt(NIC_INT_CAUSE_RX_OVERRUN);
        return;
    }

    NIC_LOG("q[%u] head %u private %u tail %u ctrl 0x%x", 0,
            _rx_queue[0].get_public_head(),
            _rx_queue[0].get_private_head(),
            _rx_queue[0].get_tail(),
            _rx_queue[0].get_descriptor_ctrl());

    if (length + ETHER_CRC_SIZE < NIC_MIN_PACKET_SIZE) {
        NIC_LOG("bad packet size");
        ++_statistic[STAT_OFFSET_RX_SIZE_ERR];
        ++_statistic[STAT_OFFSET_RX_UNDERSIZE];
        return;
    }

    if((length + ETHER_CRC_SIZE > NIC_MAX_PACKET_SIZE &&
                                            !(_rx_ctrl & NIC_RX_CTRL_LONG_PACKET_ENABLE)) ||
                                            length + ETHER_CRC_SIZE > NIC_MAX_LONG_PACKET_SIZE) {
        NIC_LOG("bad packet size");
        ++_statistic[STAT_OFFSET_RX_SIZE_ERR];
        ++_statistic[STAT_OFFSET_RX_OVERSIZE];
        return;
    }

    if (length + ETHER_CRC_SIZE == 64) {
        ++_statistic[STAT_OFFSET_RX_64];
    } else if (length + ETHER_CRC_SIZE < 128) {
        ++_statistic[STAT_OFFSET_RX_128];
    } else if (length + ETHER_CRC_SIZE < 256) {
        ++_statistic[STAT_OFFSET_RX_256];
    } else if (length + ETHER_CRC_SIZE < 512) {
        ++_statistic[STAT_OFFSET_RX_512];
    } else if (length + ETHER_CRC_SIZE < 1024) {
        ++_statistic[STAT_OFFSET_RX_1024];
    }

    ++_statistic[STAT_OFFSET_RX_GOOD_PACKETS];
    stat_64 = (uint64_t*)&_statistic[STAT_OFFSET_RX_GOOD_OCTETS_LOW];
    *stat_64 += length;

    if ((_rx_ctrl & NIC_RX_CTRL_DTYP_MASK) || !(_rx_filter_ctrl & NIC_RX_FILTER_CTRL_EXSTEN)) {
        debug_unhandled_attention();
    }

    //extended Rx descriptor

    if ((_rx_ctrl & NIC_RX_CTRL_VFE)) {
        debug_unhandled_attention();
    }

    if ((_rx_ctrl & NIC_RX_CTRL_CFIEN)) {
        debug_unhandled_attention();
    }

    if (!(_rx_ctrl & NIC_RX_CTRL_SECRC)) {
        debug_unhandled_attention();
    }

    uint buff_size = ((_rx_ctrl & NIC_RX_CTRL_FLXBUF_MASK) >> NIC_RX_CTRL_FLXBUF_SHIFT);
    if (buff_size) {
        buff_size = buff_size * KB;
    }else {
        buff_size = (_rx_ctrl & NIC_RX_CTRL_BSEX) ? NIC_MAX_LONG_PACKET_SIZE * 2 : 2048;
        buff_size >>= ((_rx_ctrl & NIC_RX_CTRL_BSIZE_MASK) >> NIC_RX_CTRL_BSIZE_SHIFT);
    }

    RxExtReadDescriptor read_descriptor;
    uint64_t descriptor_address = _rx_queue[0].head_address();

    NIC_LOG("q[%u] address  0x%lx base 0x%lx size %u", 0,
                  descriptor_address,
                  _rx_queue[0].get_base_address(),
                  _rx_queue[0].num_items());

    memory_bus->read(descriptor_address, sizeof(read_descriptor), &read_descriptor);

    if (!read_descriptor.address) {
        debug_unhandled_attention();
    }

    std::auto_ptr<DirectAccess> direct(memory_bus->get_direct(read_descriptor.address, buff_size));

    if (!direct.get()) {
        debug_unhandled_attention();
    }

    ASSERT(length <= buff_size);

    memcpy(direct.get()->get_ptr(), packet, length);
    RxExtWBDescriptor wb_descriptor;
    wb_descriptor.mrq = 0;

    if ((_rx_checksum_ctrl & NIC_RX_CHECKSUM_CTRL_PACKET_DISABLE)) {
        wb_descriptor.rss = 0;
    } else {
        uint32_t start = _rx_checksum_ctrl & NIC_RX_CHECKSUM_CTRL_START_MASK;
        wb_descriptor.no_rss.packet_checksum = checksum16(packet + start,
                                                          MAX(0, (int)(length - start)));
        wb_descriptor.no_rss.ip_identification = 0; // software device driver should ignore this
                                                    // field when status.IPIDV is not set
    }

    wb_descriptor.length = length;
    wb_descriptor.status = RX_DESCRIPTOR_STATUS_DD | RX_DESCRIPTOR_STATUS_EOP;
    wb_descriptor.vlan = 0;

    memory_bus->write(&wb_descriptor, sizeof(wb_descriptor), descriptor_address);
    _rx_queue[0].pop();

     NIC_LOG("q[%u] POP head %u private %u tail %u ctrl 0x%x", 0,
             _rx_queue[0].get_public_head(),
             _rx_queue[0].get_private_head(),
             _rx_queue[0].get_tail(),
             _rx_queue[0].get_descriptor_ctrl());

    uint descriptor_threshold = ((_rx_ctrl & NIC_RX_CTRL_DESCRIPTOR_THRESHOLD_MASK) >>
                                                        NIC_RX_CTRL_DESCRIPTOR_THRESHOLD_SHIFT) + 1;
    descriptor_threshold = _rx_queue[0].num_items() >> descriptor_threshold;

    if (_rx_queue[0].get_unused_count() == descriptor_threshold) {
        NIC_LOG("descriptor threshold hit");
        rx_write_back(NIC_INT_CAUSE_RX_DESCRIPTOR_THRESHOLD);
    } else if (!_rx_timer.private_delay_val()){
        NIC_LOG("q[%u] imidiate wb", 0);
        rx_write_back(NIC_INT_CAUSE_RX_TIMER);
    } else {
        NIC_LOG("q[%u] set timers", 0);
        _rx_timer.arm();
    }
}


void NIC::drop(uint8_t *buf, ssize_t n)
{
#ifndef DO_NIC_LOG
    std::string str;
    ether_to_str(str, buf, n);
    D_MESSAGE("%s", str.c_str());
#else
    D_MESSAGE("");
#endif
}


bool NIC::recive_data()
{
    ssize_t n = read(_interface.get(), _in_buf, sizeof(_in_buf));

    if (n == -1) {
        if (errno == EAGAIN) {
            return false;
        }

        E_MESSAGE("%s (%d)", strerror(errno), errno);
        debug_unhandled_attention();
        return false;
    }

#ifdef DO_NIC_LOG
    std::string str;
    ether_to_str(str, _in_buf, n);
    D_MESSAGE("%s", str.c_str());
#endif
    if (!ether_filter(_in_buf)) {
        drop(_in_buf, n);
    } else {
        push(_in_buf, n);
    }

    return true;
}


void NIC::interface_event_handler()
{
    trancive();
}


void* NIC::thread_main()
{
    try {
        Lock cmd_lock(_tr_cmd_lock);

        for (;;) {
            switch (_tr_command) {
            case TR_CMD_WAIT:
                tr_set_state(TR_WAITING);
               _tr_cmd_condition.wait(_tr_cmd_lock);
                break;
            case TR_CMD_RUN:
                tr_set_state(TR_RUNNING);
                cmd_lock.unlock();
                _transceiver.run();
                cmd_lock.lock();
                break;
            case TR_CMD_QUIT:
                tr_set_state(TR_TERMINATED);
                cmd_lock.unlock();
                return NULL;
            default:
                THROW("invlaid command");
            }
        }
    } catch (Exception& e) {
        E_MESSAGE("unhndled exception -> %s", e.what());
    } catch (std::exception& e) {
         E_MESSAGE("unhndled exception -> %s", e.what());
    } catch (...) {
         E_MESSAGE("unhndled exception");
    }

    return NULL;
}


void NIC::init_eeprom()
{
    memset(_rom, 0, sizeof(_rom));

    _rom[NV_WORD_ADDRESS_0] = mac_address[0] | (uint16_t(mac_address[1]) << 8);
    _rom[NV_WORD_ADDRESS_1] = mac_address[2] | (uint16_t(mac_address[3]) << 8);
    _rom[NV_WORD_ADDRESS_2] = mac_address[4] | (uint16_t(mac_address[5]) << 8);

    _rom[NV_WORD_INIT_CTRL_1] = NV_INIT_CTRL_1_LOAD_SUB_ID | NV_INIT_CTRL_1_LOAD_SUB_ID |
                                NV_INIT_CTRL_1_SET | NV_INIT_CTRL_1_LOAD_SUB_ID |
                                NV_INIT_CTRL_1_LOAD_DEV_ID;

    _rom[NV_WORD_SUBSYS_ID] = NIC_DEV_ID;
    _rom[NV_WORD_SUBSYS_VENDOR] = NIC_VENDOR_ID;
    _rom[NV_WORD_DEV_ID] = NIC_DEV_ID;

    _rom[NV_WORD_INIT_CTRL_2] = NV_INIT_CTRL_2_WUC_APMPME | NV_INIT_CTRL_2_SET |
                                (NV_INIT_CTRL_2_NV_SIZE << NV_INIT_CTRL_2_NV_SIZE_SHIFT);
    _rom[NV_WORD_PROTECT_1] = 0x2001;
    _rom[NV_WORD_PROTECT_2] = (NV_PROTECT_2_SIGNATURE << NV_PROTECT_2_SIGNATURE_SHIFT) |
                              (NV_PROTECT_2_SECTOR_SIZE << NV_PROTECT_2_SECTOR_SIZE_SHIFT) |
                              NV_PROTECT_2_SET;

    _rom[NV_WORD_EXT_CONF_1] = 1 << 12;
    _rom[NV_WORD_EXT_CONF_2] = (1 << 3) | (0x03 << 6);

    _rom[NV_WORD_PCIE_ELECT_IDLE_DELAY] = (1 << 2) | (7 << 8) | (1 << 13);
    _rom[NV_WORD_PCIE_INIT_CONF_1] = 1 | (1 << 3) | (3 << 6) | (6 << 9) | (6 << 12);
    _rom[NV_WORD_PCIE_INIT_CONF_2] = 0x50 | (1 << 8) | (1 << 12) | (1 << 13);
    _rom[NV_WORD_PCIE_INIT_CONF_3] = (1 << 10) | (1 << 9) | (1 << 8) | (1 << 5) | (1 << 4) |
                                     (0x03 << 2) | (1 << 1);
    _rom[NV_WORD_PCIE_CTRL] = (1 << 14) | (1 << 10) | (0x04 << 7) | (1 << 3) | 0x03;

    _rom[NV_WORD_LED1_PHY_CONF] = NV_LED1_INIT_MODE | NV_LED1_RESERVED_SET | NV_LED1_BLINK |
                                  NV_LED1_PHY_ND0_NO_GB | NV_LED1_PHY_ND0_DEC_SPEED;
    _rom[NV_WORD_RESRVED] = (1 << 8);
    _rom[NV_WORD_REV_ID] = 0xf0;
    _rom[NV_WORD_LED0_LED2] = (NV_LED0_L2_INIT_MODE << NV_LED0_L2_INIT_MODE_SHIFT) |
                              NV_LED0_L0_INIT_MODE;

    _rom[NV_WORD_POWER] = 0x04 | (0x07 << 8);

    _rom[NV_WORD_INIT_CTRL_3] = NV_INIT_CTRL_3_WAKE_UP_ENABLE | NV_INIT_CTRL_3_SET |
                                NV_INIT_CTRL_3_NO_PHY_RESET;

    /* Words 0x30 through 0x3E (bytes 0x60 through 0x7D) have been reserved for
       configuration and version values to be used by PXE code.
    */

    _rom[NV_WORD_SUM] =  0xbaba - checksum16(_rom, sizeof(_rom) / 2);

    D_MESSAGE("sum_2 0x%x", checksum16(_rom, sizeof(_rom) / 2));
}


void NIC::common_reset()
{
    _tx_timer.reset();
    _rx_timer.reset();

    _int_cause = 0;
    _int_throttling = 0;
    _int_mask = 0;
    _auto_mask = 0;

    _ctrl = NIC_CTRL_FULL_DUPLEX | (NIC_SPEED_1000 << NIC_CTRL_SPEED_SHIFT) |
            NIC_CTRL_ADVD3WUC | NIC_CTRL_RESREVED_SET;
    _ext_ctrl = NIC_EXT_CTRL_PHY_PDEN;
    _ext_conf_ctrl = NIC_EXT_CONF_CTRL_RESERVED_SET;
    _mdi_ctrl = NIC_MDI_CTRL_ADDRESS_GIGABIT | NIC_MDI_CTRL_READY;

    _rx_ctrl = 0;
    _rx_checksum_ctrl = NIC_RX_CHECKSUM_CTRL_IP | NIC_RX_CHECKSUM_CTRL_UDP;
    memset(_rx_rss_key, 0, sizeof(_rx_rss_key));
    _multi_rq_command = 0;
    _rx_filter_ctrl = 0;

    _tx_ctrl = NIC_TX_CTRL_PAD_SHORT | NIC_TX_CTRL_MULTI_REQ |
               (NIC_TX_CTRL_RR_THRESH_INIT_VAL << NIC_TX_CTRL_RR_THRESH_SHIFT);
    _tx_arbitration_count_1 = _tx_arbitration_count_0 = NIC_TX_ARBITRATION_COUNT_INIT_COUNT |
                                                        NIC_TX_ARBITRATION_COUNT_ENABLE;

    _tx_queue[0].reset(NIC_INT_CAUSE_TxQ0_WRITTEN_BACK);
    _tx_queue[1].reset(NIC_INT_CAUSE_TxQ1_WRITTEN_BACK);
    _rx_queue[0].reset(NIC_INT_CAUSE_RxQ0_WRITTEN_BACK);
    _rx_queue[1].reset(NIC_INT_CAUSE_RxQ1_WRITTEN_BACK);

    memset(&_offload_context, 0, sizeof(_offload_context));
    memset(&_seg_context, 0, sizeof(_seg_context));

    _rom_read = 0;

    _vlan_ether_type = 0x8100;
    _adaptive_IFS_throttle = 0;

    _time_sync_tx_ctrl = 0;
    _time_sync_rx_ctrl = 0;
    _time_sync_mess_type = 0x88f7;
    _time_sync_udp_port = 0x319,

    _nvm_ctrl = (2 << NIC_NVM_CTRK_ADDRESS_SIZE_SHIFT) |
                NIC_NVM_CTRL_AUTO_READ_DONE |
                NIC_NVM_CTRL_PRESENT |
                (NIC_NVM_CTRL_RO << NIC_NVM_CTRL_WRITE_CTRL_SHIFT);
}


void NIC::reset()
{
    D_MESSAGE("");

    PCIDevice::reset();

    _io_address = 0;

    common_reset();

    _status = NIC_STATUS_MASTER_ENABLE_STATUS;
    _packet_buff_alloc = NIC_PACKET_BUFF_ALLOC_RX_INIT_VAL |
                         (NIC_PACKET_BUFF_ALLOC_TX_INIT_VAL << NIC_PACKET_BUFF_ALLOC_TX_SHIFT);
    _packet_buff_size = NIC_PACKET_BUFF_SIZE_MAX;

    _wakeup_ctrl = 0;
    _wakeup_filter = 0;
    _3gio_ctrl_1 = NIC_3GIO_CTRL_1_SELF_TEST | NIC_3GIO_CTRL_1_LATENCY_DEFAULT |
                   NIC_3GIO_CTRL_1_ADJUSTMENT;
    _3gio_ctrl_2 = 0;
    _soft_sem = 0;
    _receive_addr_table[(NUM_RECEIVE_ADDR - 1) * 2] = 0;
    _receive_addr_table[(NUM_RECEIVE_ADDR - 1) * 2 + 1] = 0;

    memset(_statistic, 0, sizeof(_statistic));

    nv_load();

    phy_reset();
}


void NIC::nv_load()
{
    if ((_rom[NV_WORD_INIT_CTRL_1] & NV_INIT_CTRL_1_LOAD_DEV_ID)) {
        //...
    }

    if ((_rom[NV_WORD_INIT_CTRL_1] & NV_INIT_CTRL_1_LOAD_SUB_ID)) {
        //...
    }

    if ((_rom[NV_WORD_INIT_CTRL_3] & NV_INIT_CTRL_3_WAKE_UP_ENABLE)) {
        _wakeup_ctrl |= NIC_WAKEUP_CTRL_ENABLE;
    } else {
        _wakeup_ctrl &= ~NIC_WAKEUP_CTRL_ENABLE;
    }


    if ((_rom[NV_WORD_INIT_CTRL_2] & NV_INIT_CTRL_2_WUC_APMPME)) {
        _wakeup_ctrl |= NIC_WAKEUP_CTRL_ASSERT_PME;
    } else {
        _wakeup_ctrl &= ~NIC_WAKEUP_CTRL_ASSERT_PME;
    }

    /*if ( ? ) {
        _wakeup_ctrl |= NIC_WAKEUP_CTRL_LINK_WAKE_OVERRIDE;
    } else {
        _wakeup_ctrl &= ~NIC_WAKEUP_CTRL_LINK_WAKE_OVERRIDE;
    }*/

    if (_rom[NV_WORD_INIT_CTRL_1] & NV_INIT_CTRL_1_FORCE_DUPLEX) {
        _ctrl |= NIC_CTRL_FULL_DUPLEX;
    } else {
        _ctrl &= ~NIC_CTRL_FULL_DUPLEX;
    }

    if (_rom[NV_WORD_INIT_CTRL_1] & NV_INIT_CTRL_1_FORCE_SPEED) {
        _ctrl |= NIC_CTRL_FORCE_SPEED;
    } else {
        _ctrl &= ~NIC_CTRL_FORCE_SPEED;
    }

    if ((_rom[NV_WORD_INIT_CTRL_3] & NV_INIT_CTRL_3_WAKE_UP_ENABLE)) {
        _ctrl |= NIC_CTRL_SET_LINK_UP;
    } else {
        _ctrl &= ~NIC_CTRL_SET_LINK_UP;
    }

    _led_ctrl = (_rom[NV_WORD_LED0_LED2] & 0xff);
    _led_ctrl |= (_rom[NV_WORD_LED1_PHY_CONF] & 0xff) << 8;
    _led_ctrl |= uint32_t(_rom[NV_WORD_LED0_LED2] & 0xff00) << 8;
    _led_ctrl &= ~NIC_LED_CTRL_RESERVED;

    _receive_addr_table[0] = (uint32_t(_rom[NV_WORD_ADDRESS_1]) << 16) | _rom[NV_WORD_ADDRESS_0];
    _receive_addr_table[1] = _rom[NV_WORD_ADDRESS_2] | NIC_RECEIVE_ADDR_HIGH_VALID;


    if ((_rom[NV_WORD_INIT_CTRL_2] & NV_INIT_CTRL_2_NV_TYPE)) {
        _nvm_ctrl |= NIC_NVM_CTRL_TYPE;
    } else {
        _nvm_ctrl &= ~NIC_NVM_CTRL_TYPE;
    }

    _nvm_ctrl &= ~NIC_NVM_CTRL_SIZE_MASK;
    _nvm_ctrl |= ((_rom[NV_WORD_INIT_CTRL_2] & NV_INIT_CTRL_2_NV_SIZE_MASK) >>
                                           NV_INIT_CTRL_2_NV_SIZE_SHIFT) << NIC_NVM_CTRL_SIZE_SHIFT;
}



void NIC::soft_reset()
{
    /* Software Reset - Software can reset the 82574 by writing the Device Reset bit of
       the Device Control (CTRL.RST) register. The 82574L re-reads the per-function NVM
       fields after a software reset. Bits that are normally read from the NVM are reset to
       their default hardware values. Note that this reset is per function and resets only
       the function that received the software reset. PCI configuration space (configuration
       and mapping) of the device is unaffected.
    */

    D_MESSAGE("");
    debug_attention();
    init_eeprom();
    common_reset();

    _receive_addr_table[0] = mac_address[0] | (uint32_t(mac_address[1]) << 8) |
                             (uint32_t(mac_address[2]) << 16) | (uint32_t(mac_address[3]) << 24);
    _receive_addr_table[1] = mac_address[4] | (uint32_t(mac_address[5]) << 8) |
                             NIC_RECEIVE_ADDR_HIGH_VALID;
}


void NIC::eeprom_reset()
{
    /* EEPROM Reset - Writing a 1b to the EEPROM Reset bit of the Extended Device
       Control (CTRL_EXT.EE_RST) register causes the 82574 to re-read the per-function
       configuration from the NVM, setting the appropriate bits in the registers loaded
       by the NVM.
    */

    D_MESSAGE("");
    nv_load();
}


void NIC::csr_read(uint64_t src, uint64_t length, uint8_t* dest)
{
    if ((src & 0x03) || (length & 0x03)) {
        W_MESSAGE("not dword 0x%lx 0x%lx", src, length);
        return;
    }

    uint32_t* dest_32 = (uint32_t*)dest;
    length = length >> 2;

    for (; length--; dest_32++, src += 4) {
        NIC_LOG("read 0x%x", src);

        Lock lock(_mutex);

        switch (src) {
        case NIC_REG_INT_CAUSE_READ:
            *dest_32 = _int_cause;
            NIC_LOG("INT_CAUSE_READ 0x%x", _int_cause);

            if (_int_mask == 0) {
                _int_cause = 0;
            } else if ((_int_cause & NIC_INT_CAUSE_READ_ASSERTED)) {
                _int_cause = 0;
                if ((_ext_ctrl & NIC_EXT_CTRL_AUTO_MASK)) {
                    _int_mask = _auto_mask;
                }
            }

            update_interrupt_level();

            break;
        case NIC_REG_INT_MASK_SET:
            *dest_32 = _int_mask;
            break;
        case NIC_REG_STATUS:
            *dest_32 = _status;
            NIC_LOG("STATUS 0x%x", _status);
            break;
        case NIC_REG_TX_DESCRIPTOR_HEAD_0:
            *dest_32 = _tx_queue[0].get_public_head();
            break;
        case NIC_REG_TX_DESCRIPTOR_HEAD_1:
            *dest_32 = _tx_queue[1].get_public_head();
            break;
        case NIC_REG_TX_DESCRIPTOR_TAIL_0:
            *dest_32 = _tx_queue[0].get_tail();
            break;
        case NIC_REG_TX_DESCRIPTOR_TAIL_1:
            *dest_32 = _tx_queue[1].get_tail();
            break;
        case NIC_REG_SOFT_SEM:
            *dest_32 = _soft_sem;
            _soft_sem |= NIC_SOFT_SEM_LOCK;
            break;
        case NIC_REG_NVM_CTRL:
            *dest_32 = _nvm_ctrl;
            break;
        case NIC_REG_EXT_CONF_CTRL:
            *dest_32 = _ext_conf_ctrl;
            break;
        case NIC_REG_RX_CTRL:
            *dest_32 = _rx_ctrl;
            break;
        case NIC_REG_MDI_CTRL:
            *dest_32 = _mdi_ctrl;
            break;
        case NIC_REG_MNG_CTRL:
            *dest_32 = NIC_MNG_CTRL_DIS_ARP_IP_CHECK;
            break;
        case NIC_REG_CTRL:
        case NIC_REG_CTRL_ALIAS:
            *dest_32 = _ctrl;
            break;
        case NIC_REG_TX_CTRL:
            *dest_32 = _tx_ctrl;
            break;
        case NIC_REG_EEPROM_READ:
            *dest_32 = _rom_read;
            break;
        case NIC_REG_PACKET_BUFF_ALLOC:
            *dest_32 = _packet_buff_alloc;
            break;
        case NIC_REG_PACKET_BUFF_SIZE:
            *dest_32 = _packet_buff_size;
            break;
        case NIC_REG_EXT_CTRL:
            *dest_32 = _ext_ctrl;
            break;
        case NIC_REG_WAKEUP_CTRL:
            *dest_32 = _wakeup_ctrl;
            break;
        case NIC_REG_WAKEUP_FILTER:
            *dest_32 = _wakeup_filter;
            break;
        case NIC_REG_TX_DESCRIPTOR_CTRL_0:
            *dest_32 = _tx_queue[0].get_descriptor_ctrl();
            break;
        case NIC_REG_TX_DESCRIPTOR_CTRL_1:
            *dest_32 = _tx_queue[1].get_descriptor_ctrl();
            break;
        case NIC_REG_TX_ARBITRATION_COUNT_0:
            *dest_32 = _tx_arbitration_count_0;
            break;
        case NIC_REG_TX_ARBITRATION_COUNT_1:
            *dest_32 = _tx_arbitration_count_1;
            break;
        case NIC_REG_3GIO_CTRL_1:
            *dest_32 = _3gio_ctrl_1;
            break;
        case NIC_REG_3GIO_CTRL_2:
            *dest_32 = _3gio_ctrl_2;
            break;
        case NIC_REG_LED_CTRL:
            *dest_32 = _led_ctrl;
            break;
        case NIC_REG_VLAN_FILTER_TABLE_ARRAY ... NIC_REG_VLAN_FILTER_TABLE_ARRAY +
                                                 (VLAN_FILTER_TABLE_SIZE - 1 ) * 4:
            *dest_32 = _vlan_filter_table[(src - NIC_REG_VLAN_FILTER_TABLE_ARRAY) >> 2];
            break;
        case NIC_REG_RECEIVE_ADDR_START ... NIC_REG_RECEIVE_ADDR_END:
            *dest_32 = _receive_addr_table[(src - NIC_REG_RECEIVE_ADDR_START) >> 2];
            break;
        case NIC_REG_MULTICAST_TABLE_START ... NIC_REG_MULTICAST_TABLE_END:
            *dest_32 = _multicast_table[(src - NIC_REG_MULTICAST_TABLE_START) >> 2];
            break;
        case NIC_REG_VLAN_ETHER_TYPE:
            *dest_32 = _vlan_ether_type;
            break;
        case NIC_REG_ADAPT_IFS_THROT:
            *dest_32 = _adaptive_IFS_throttle;
            break;
        case NIC_REG_TIME_SYNC_TX_CTRL:
            *dest_32 = _time_sync_tx_ctrl;
            break;
        case NIC_REG_TIME_SYNC_RX_CTRL:
            *dest_32 = _time_sync_rx_ctrl;
            break;
        case NIC_REG_STAT_START ... NIC_REG_STAT_END:
            *dest_32 = _statistic[(src - NIC_REG_STAT_START) >> 2];
            break;
        case NIC_REG_TIME_SYNC_MESS_TYPE:
            *dest_32 = _time_sync_mess_type;
            break;
        case NIC_REG_TIME_SYNC_UDP_PORT:
            *dest_32 = _time_sync_udp_port;
            break;
        case  NIC_REG_TIME_SYNC_STAMP_LOW:
            D_MESSAGE("NIC_REG_TIME_SYNC_STAMP_LOW");
            *dest_32 = 0;
            break;
        case NIC_REG_TIME_SYNC_STAMP_HIGH:
            D_MESSAGE("NIC_REG_TIME_SYNC_STAMP_HIGH");
            *dest_32 = 0;
            break;
        case NIC_REG_INT_MASK_CLEAR:
            W_MESSAGE("read from write only reg 0x%lx", src);
            debug_attention();
            break;
        case  NIC_REG_TIME_SYNC_TX_STAMP_LOW:
            D_MESSAGE("NIC_REG_TIME_SYNC_TX_STAMP_LOW");
            *dest_32 = 0;
            break;
        case NIC_REG_TIME_SYNC_TX_STAMP_HIGH:
            D_MESSAGE("NIC_REG_TIME_SYNC_TX_STAMP_HIGH");
            *dest_32 = 0;
            break;
        case  NIC_REG_TIME_SYNC_SYSTEM_TIME_LOW: //latch High
            D_MESSAGE("NIC_REG_TIME_SYNC_SYSTEM_TIME_LOW");
            *dest_32 = 0;
            break;
        case NIC_REG_TIME_SYNC_SYSTEM_TIME_HIGH:
            D_MESSAGE("NIC_REG_TIME_SYNC_SYSTEM_TIME_HIGH");
            *dest_32 = 0;
            break;
        case NIC_REG_RX_CHECKSUM_CTRL:
            *dest_32 = _rx_checksum_ctrl;
            break;
        case NIC_REG_RX_FILTER_CTRL:
            *dest_32 = _rx_filter_ctrl;
            break;
        case NIC_REG_INT_THROTTLING:
            *dest_32 = _int_throttling;
            break;
        case NIC_REG_INT_AUTO_MASK:
            *dest_32 = _auto_mask;
            break;
        case NIC_REG_STAT_END + 4 ... 0x4124: /* linux driver read statistic that is not defined
                                                 (acording to spec) for 82574
                                              */
            *dest_32 = 0;
            break;
        default:
            W_MESSAGE("unhandled 0x%lx", src);
            *dest_32 = 0;
            debug_unhandled_attention();
        }
    }
}


inline void NIC::update_interrupt_level()
{
    uint level = _int_mask & _int_cause;

    if (level) {
        _int_cause |= NIC_INT_CAUSE_READ_ASSERTED;
        NIC_LOG("set trigger 0x%x cause 0x%x mask 0x%x", level, _int_cause, _int_mask);
    } else {
        _int_cause &= ~NIC_INT_CAUSE_READ_ASSERTED;
        NIC_LOG("clear cause 0x%x mask 0x%x", _int_cause, _int_mask);
    }

    set_interrupt_level(level);
}


inline void NIC::interrupt(uint32_t cause)
{
    _int_cause |= cause;
    update_interrupt_level();
}


void NIC::phy_reset_interface(const char* interface_name)
{
    // tunctl -p -u <user> -g <group> -t tap0
    // ifconfig tap0 up
    // brctl addif br0 tap0

    try {
        if (_interface_event) {
            _interface_event->destroy();
            _interface_event = NULL;
        }

        _interface.reset(-1);

        if (!is_phy_power_up()) {
            return;
        }

        AutoFD tun(open("/dev/net/tun", O_RDWR | O_NONBLOCK ));

        if (!tun.is_valid()) {
            int err = errno;
            W_MESSAGE("opne tun failed  %d %s", err, strerror(err));
            return;
        }

        struct ifreq ifr;

        memset(&ifr, 0, sizeof(ifr));
        ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
        strncpy(ifr.ifr_ifrn.ifrn_name, interface_name, IFNAMSIZ);

        if (ioctl(tun.get(), TUNSETIFF, &ifr) < 0){
            int err = errno;
            W_MESSAGE("TUNSETIFF failed  %d %s", err, strerror(err));
            return;
        }

        I_MESSAGE("net intrface is %s", ifr.ifr_ifrn.ifrn_name);

        _interface_event = _transceiver.create_fd_event(tun.get(),
                                                     (void_callback_t)&NIC::interface_event_handler,
                                                     this);
        _interface.reset(tun.release());
    } catch (Exception& e) {
        W_MESSAGE("%s", e.what());
    } catch (...) {
        W_MESSAGE("unknown exeption");
    };
}


void NIC::phy_reset_common()
{
    _phy_status = PHY_STATUS_100FD | PHY_STATUS_100HD | PHY_STATUS_10FD |
                  PHY_STATUS_10HD | PHY_STATUS_EXT_STATUS | PHY_STATUS_MF |
                  PHY_STATUS_AUTO_NEGO_CAP | PHY_STATUS_EXT_CAP;
    _phy_partner_ability = 0;
    _phy_1000_status = 0;
}


void NIC::phy_update_link_state()
{
    if (phy_link_is_up()) {
        _phy_status |= PHY_STATUS_LINK_UP;
        _phy_1000_status |= PHY_1000_STATUS_LOCAL_RECEIVER_OPERATIONAL |
                            PHY_1000_STATUS_REMOTE_RECEIVER_OPERATIONAL;
        _phy_0_copper_status_1 |= PHY_P0_COPPER_STATUS_1_LINK_UP |
                               PHY_P0_COPPER_STATUS_1_GLOBAL_LINK;
    } else {
        _phy_status &= ~PHY_STATUS_LINK_UP;
        _phy_1000_status &= ~(PHY_1000_STATUS_LOCAL_RECEIVER_OPERATIONAL |
                              PHY_1000_STATUS_REMOTE_RECEIVER_OPERATIONAL);
        _phy_0_copper_status_1 &= ~(PHY_P0_COPPER_STATUS_1_LINK_UP |
                                  PHY_P0_COPPER_STATUS_1_GLOBAL_LINK);
    }
}


void NIC::phy_reset()
{
    D_MESSAGE("");
    debug_attention();
    phy_reset_common();

    _phy_ctrl = PHY_CTRL_AUTO_NEGOTIATION | PHY_CTRL_DUPLEX_MODE | PHY_CTRL_SPEED_MSB;
    _phy_page = 0;

    _phy_0_copper_ctrl_1 = (0x3 << PHY_P0_COPPER_CTRL_1_DOWNSHIFT_COUNTER_SHIFT) |
                           (0x3 << PHY_P0_COPPER_CTRL_1_CROSSOVER_MODE_SHIFT);
                           // 8-9 After a hardware reset, both bits take on the value of
                           // pd_config_edet_a ???????

    _phy_0_copper_status_1 = (NIC_SPEED_1000 << PHY_P0_COPPER_STATUS_1_SPEED_SHIFT) |
                             PHY_P0_COPPER_STATUS_1_MSI_CROSSOVER;

    _phy_0_bias_1 = 3;
    _phy_0_bias_2 = 0;
    _phy_0_oem_bits = 0,

    _phy_id_2 = (PHY_ID2_MODEL << 4) | (PHY_ID2_OUI_LSB << 10) | PHY_ID2_REVISION;

    _phy_auto_nego = PHY_AUTO_NEGO_100FD_ADVERTIS | PHY_AUTO_NEGO_100HD_ADVERTIS |
                     PHY_AUTO_NEGO_10FD_ADVERTIS | PHY_AUTO_NEGO_10HD_ADVERTIS|
                     PHY_AUTO_NEGO_SELECTOR| PHY_AUTO_NEGO_PAUSE |
                     PHY_AUTO_NEGO_ASYMMETRIC_PAUSE;

    _phy_1000_ctrl = PHY_1000_CTRL_1000FD_ADVERTIS;

    _status |= NIC_STATUS_PHYRA;

    phy_reset_interface("tap0");
    phy_update_link_state();
    auto_negotiation();
}


uint NIC::phy_speed_select()
{
    return ((_phy_ctrl >> PHY_CTRL_SPEED_LSB_SHIFT) & 0x01) |
           ((_phy_ctrl >> (PHY_CTRL_SPEED_MSB_SHIFT - 1)) & 0x02);
}


bool NIC::is_phy_power_up()
{
    return !(_phy_ctrl & PHY_CTRL_POWER_DOWN) &&
           !(_phy_0_copper_ctrl_1 & PHY_P0_COPPER_CTRL_1_POWER_DOWN);
}


bool NIC::is_auto_negotiation()
{
    return is_phy_power_up() &&
           ((_phy_ctrl & PHY_CTRL_AUTO_NEGOTIATION) || phy_speed_select() == NIC_SPEED_1000);
}


void NIC::phy_soft_reset()
{
    D_MESSAGE("");
    debug_attention();
    phy_reset_common();

    _phy_ctrl &= PHY_CTRL_SPEED_LSB | PHY_CTRL_AUTO_NEGOTIATION  | PHY_CTRL_POWER_DOWN |
                 PHY_CTRL_DUPLEX_MODE | PHY_CTRL_SPEED_MSB;
    _phy_id_2 &= PHY_ID2_REVISION_MASK;
    _phy_1000_ctrl &= ~PHY_1000_CTRL_TEST_MASK;
    _phy_0_copper_ctrl_1 &= ~PHY_P0_COPPER_CTRL_1_DISABLE_LINK_PULSES;
    _phy_0_copper_status_1 &= PHY_P0_COPPER_STATUS_1_MSI_CROSSOVER;
    _phy_0_oem_bits &= (PHY_P0_OEM_BITS_GBE_DISABLE | PHY_P0_OEM_BITS_LPLU);

    phy_reset_interface("tap0");
    phy_update_link_state();

    if (is_auto_negotiation()) {
        auto_negotiation();
    } else {
        phy_manual_config();
    }
}


void NIC::mac_update_link_state()
{
    bool duplex;
    uint speed;

    if (!(_ctrl & NIC_CTRL_SET_LINK_UP)) {
        /* CTRL.SLU - Must be set to 1b by software to enable communications between
           MAC and PHY.
        */
        D_MESSAGE("no mac phy comunication");
        return;
    }

    _status &= ~(NIC_STATUS_SPEED_MASK | NIC_STATUS_DUPLEX | NIC_STATUS_LINK_UP);

    if (!(_ctrl & NIC_CTRL_FORCE_SPEED) && !(_ctrl & NIC_CTRL_AUTO_SPPED)) {
        speed = (_phy_0_copper_status_1 & PHY_P0_COPPER_STATUS_1_SPEED_MASK) >>
                                                            PHY_P0_COPPER_STATUS_1_SPEED_SHIFT;
    } else {
        speed = (_ctrl & NIC_CTRL_SPEED_MASK) >> NIC_CTRL_SPEED_SHIFT;
    }

    _status |= (speed << NIC_STATUS_SPEED_SHIFT);

    if (!(_ctrl & NIC_CTRL_FORCE_DUPLEX)) {
        duplex = !!(_phy_0_copper_status_1 & PHY_P0_COPPER_STATUS_1_DUPLEX);
    } else {
        duplex = !!(_ctrl & NIC_CTRL_FULL_DUPLEX);
    }

    _status |= ((duplex) ? NIC_STATUS_DUPLEX: 0);

   _status |= (_phy_0_copper_status_1 & PHY_P0_COPPER_STATUS_1_LINK_UP) ? NIC_STATUS_LINK_UP : 0;

    /* If flow control is enabled in the 82574, the settings for the desired flow control
       behavior must be set by software in the PHY registers and auto-negotiation restarted.
       After auto-negotiation completes, the software device driver must read the PHY
       registers to determine the resolved flow control behavior of the link and reflect these in
       the MAC register settings (CTRL.TFCE and CTRL.RFCE). If no software device driver is
       loaded and auto-negotiation is enabled, then hardware sets these bits in accordance
       with the auto-negotiation results.
    */
}


bool NIC::auto_sens_speed(uint& speed, bool& duplex)
{
    if (!(_phy_0_oem_bits & PHY_P0_OEM_BITS_GBE_DISABLE) &&
        _phy_1000_ctrl & (PHY_1000_CTRL_1000FD_ADVERTIS | PHY_1000_CTRL_1000HD_ADVERTIS)) {
        speed = NIC_SPEED_1000;
        duplex = !!(_phy_1000_ctrl & PHY_1000_CTRL_1000FD_ADVERTIS);
        return true;
    }

    if (_phy_auto_nego & (PHY_AUTO_NEGO_100FD_ADVERTIS | PHY_AUTO_NEGO_100HD_ADVERTIS)) {
        speed = NIC_SPEED_100;
        duplex = !!(_phy_auto_nego & PHY_AUTO_NEGO_100FD_ADVERTIS);
        return true;
    }

    if (_phy_auto_nego & (PHY_AUTO_NEGO_10FD_ADVERTIS | PHY_AUTO_NEGO_10HD_ADVERTIS)) {
        speed = NIC_SPEED_10;
        duplex = !!(_phy_auto_nego & PHY_AUTO_NEGO_10FD_ADVERTIS);
        return true;
    }

    return false;
}


void NIC::auto_negotiation()
{
    uint speed;
    bool duplex;

    D_MESSAGE("");

    _phy_status |= PHY_STATUS_AUTO_NEGO_COMPLETE;

    if (!phy_link_is_up()) {
        _phy_partner_ability = 0;
        _phy_1000_status &= ~(PHY_1000_STATUS_PARTNER_CAPABLE_1000FD |
                              PHY_1000_STATUS_PARTNER_CAPABLE_1000HD);

        _phy_0_copper_status_1 &= ~(PHY_P0_COPPER_STATUS_1_SPEED_MASK |
                                PHY_P0_COPPER_STATUS_1_DUPLEX |
                                PHY_P0_COPPER_STATUS_1_RESOLVED);
        mac_update_link_state();
        return;
    }

    _phy_partner_ability = PHY_PARTNER_ABILITY_100FD | PHY_PARTNER_ABILITY_100HD |
                           PHY_PARTNER_ABILITY_10FD | PHY_PARTNER_ABILITY_10HD |
                           PHY_PARTNER_SELECTOR;
    _phy_1000_status |= PHY_1000_STATUS_PARTNER_CAPABLE_1000FD |
                        PHY_1000_STATUS_PARTNER_CAPABLE_1000HD;

    if (!(_phy_ctrl & PHY_CTRL_AUTO_NEGOTIATION)) {
        ASSERT(phy_speed_select() == NIC_SPEED_1000);

        if ((_phy_0_oem_bits & PHY_P0_OEM_BITS_GBE_DISABLE)) {
            D_MESSAGE("filed: forced GBE while GBE is disabled in OEM bits");
            mac_update_link_state();
            return;
        }

        speed = NIC_SPEED_1000;
        duplex = !!(_phy_ctrl & PHY_CTRL_DUPLEX_MODE);
    } else if (!auto_sens_speed(speed, duplex)) {
        D_MESSAGE("no valid speed");
        mac_update_link_state();
        return;
    }

    _phy_0_copper_status_1 &= ~(PHY_P0_COPPER_STATUS_1_SPEED_MASK | PHY_P0_COPPER_STATUS_1_DUPLEX);
    _phy_0_copper_status_1 |= (speed << PHY_P0_COPPER_STATUS_1_SPEED_SHIFT);
    _phy_0_copper_status_1 |= (duplex ? PHY_P0_COPPER_STATUS_1_DUPLEX : 0) |
                            PHY_P0_COPPER_STATUS_1_RESOLVED;

    mac_update_link_state();
}


void NIC::phy_manual_config()
{
    ASSERT(!(_phy_ctrl & PHY_CTRL_AUTO_NEGOTIATION));

    _phy_status &= ~PHY_STATUS_LINK_UP;

    _phy_partner_ability = 0;
    _phy_1000_status &= ~(PHY_1000_STATUS_PARTNER_CAPABLE_1000FD |
                          PHY_1000_STATUS_PARTNER_CAPABLE_1000HD);
    _phy_0_copper_status_1 &= ~(PHY_P0_COPPER_STATUS_1_SPEED_MASK | PHY_P0_COPPER_STATUS_1_DUPLEX);
    _phy_0_copper_status_1 |= (phy_speed_select() << PHY_P0_COPPER_STATUS_1_SPEED_SHIFT);
    _phy_0_copper_status_1 |= ((_phy_ctrl & PHY_CTRL_DUPLEX_MODE) ?
                                                        PHY_P0_COPPER_STATUS_1_DUPLEX : 0) |
                              PHY_P0_COPPER_STATUS_1_RESOLVED;
    mac_update_link_state();
}


void NIC::mdi_write_common(uint reg)
{
    switch (reg) {
    case PHY_REG_PAGE:
        _phy_page = (_mdi_ctrl & 0xff);
        NIC_LOG("page %u", _phy_page);
        break;
    case PHY_REG_CTRL:
        _phy_ctrl = _mdi_ctrl & ~PHY_CTRL_RESERVED;

        if (_phy_ctrl & PHY_CTRL_RESTART_AUTO_NEGOTIATION) {
            _phy_ctrl &= ~PHY_CTRL_RESTART_AUTO_NEGOTIATION;
            if (is_auto_negotiation()) {
                auto_negotiation();
                D_MESSAGE("PHY_CTRL_RESTART_AUTO_NEGOTIATION");
            } else {
                W_MESSAGE("PHY_CTRL_RESTART_AUTO_NEGOTIATION: nop");
                debug_attention();
            }
        }

        if (_phy_ctrl & PHY_CTRL_RESET) {
            _phy_ctrl &= ~PHY_CTRL_RESET;
            phy_soft_reset();
        }

        if (_phy_ctrl & PHY_CTRL_LOOPBACK) {
            W_MESSAGE("PHY_CTRL_LOOPBACK");
            debug_attention();
        }
        break;
    case PHY_REG_AUTO_NEGO_ADVERTIS:
        _phy_auto_nego = _mdi_ctrl & ~PHY_AUTO_NEGO_RESERVED;
        break;
    case PHY_REG_1000_CTRL:
        _phy_1000_ctrl = _mdi_ctrl & ~PHY_1000_CTRL_RESERVED;
        break;
    default:
        _mdi_ctrl |= NIC_MDI_CTRL_ERROR;
        W_MESSAGE("invalid reg %u page %u (0x%x))", reg, _phy_page, _phy_ctrl);
        debug_unhandled_attention();
    }
}


void NIC::mdi_page0_write(uint reg)
{
    switch (reg) {
    case PHY_REG_P0_COPPER_CTRL_1:
        _phy_0_copper_ctrl_1 = _mdi_ctrl;
        if (_phy_0_copper_ctrl_1 & PHY_P0_COPPER_CTRL_1_DISABLE_LINK_PULSES) {
            W_MESSAGE("Disable link pulse");
            debug_attention();
        }

        if (_phy_0_copper_ctrl_1 & PHY_P0_COPPER_CTRL_1_FORCE_LINK_GOOD) {
            W_MESSAGE("Force link good");
            debug_attention();
        }

        if (_phy_0_copper_ctrl_1 & PHY_P0_COPPER_CTRL_1_TRANSMITTER_DISABLE) {
            W_MESSAGE("Transmitter disable");
            debug_attention();
        }

        if (_phy_0_copper_ctrl_1 & PHY_P0_COPPER_CTRL_1_POWER_DOWN) {
            W_MESSAGE("Power down");
            debug_attention();
        }

        if (_phy_0_copper_ctrl_1 & PHY_P0_COPPER_CTRL_1_DISABLE_JABBER) {
            W_MESSAGE("Disable jabber function");
            debug_attention();
        }
        break;
    case PHY_REG_P0_BIAS_1:
        _phy_0_bias_1 = _mdi_ctrl;
        break;
    case PHY_REG_P0_BIAS_2:
        _phy_0_bias_2 = _mdi_ctrl;
        break;
    case PHY_REG_P0_OEM_BITS:
        _phy_0_oem_bits &= ~(PHY_P0_OEM_BITS_GBE_DISABLE | PHY_P0_OEM_BITS_LPLU);
        _phy_0_oem_bits = _mdi_ctrl & (PHY_P0_OEM_BITS_GBE_DISABLE | PHY_P0_OEM_BITS_LPLU);

        if (_mdi_ctrl & PHY_P0_OEM_BITS_RESTART_NEGO) {
            if (is_auto_negotiation()) {
                D_MESSAGE("PHY_P0_OEM_BITS_RESTART_NEGO");
                auto_negotiation();
            } else {
                W_MESSAGE("PHY_CTRL_RESTART_AUTO_NEGOTIATION: nop");
                debug_attention();
            }
        }
        break;
    default:
        mdi_write_common(reg);
    }
}


void NIC::mdi_write(uint reg)
{
    NIC_LOG("%u", reg);

    switch (_phy_page) {
    case 0:
        mdi_page0_write(reg);
        break;
    default:
        mdi_write_common(reg);
    }

    _mdi_ctrl |= NIC_MDI_CTRL_READY;

    if (_mdi_ctrl & NIC_MDI_CTRL_INTERRUPT) {
        D_MESSAGE("NIC_REG_MDI_CTRL interrupt");
        interrupt(NIC_INT_CAUSE_MDIO_COMPLETE);
    }
}


inline void NIC::mdi_set_data(uint16_t val)
{
    _mdi_ctrl = (_mdi_ctrl & ~NIC_MDI_CTRL_DATA_MASK) | val;
}


void NIC::mdi_read_common(uint reg)
{
    switch (reg) {
    case PHY_REG_CTRL:
        mdi_set_data(_phy_ctrl);
        break;
    case PHY_REG_ID1:
        mdi_set_data(PHY_ID1);
        break;
    case PHY_REG_ID2:
        mdi_set_data(_phy_id_2);
        break;
    case PHY_REG_PAGE:
        mdi_set_data(_phy_page);
        break;
    case PHY_REG_AUTO_NEGO_ADVERTIS:
        D_MESSAGE("AUTO_NEGO_ADVERTIS 0x%x", _phy_auto_nego);
        mdi_set_data(_phy_auto_nego);
        break;
   case PHY_REG_1000_CTRL:
        mdi_set_data(_phy_1000_ctrl);
        break;
    case PHY_REG_1000_STATUS:
        D_MESSAGE("1000_STATUS 0x%x", _phy_1000_status);
        mdi_set_data(_phy_1000_status);
        break;
    case PHY_REG_STATUS:
        D_MESSAGE("STATUS 0x%x", _phy_status);
        mdi_set_data(_phy_status);
        break;
    case PHY_REG_PARTNER_ABILITY:
        D_MESSAGE("PARTNER_ABILITY 0x%x", _phy_partner_ability);
        mdi_set_data(_phy_partner_ability);
        break;
    case PHY_REG_NEGO_EXP:
        mdi_set_data(PHY_NEGO_EXP_LOCAL_NEXT | PHY_NEGO_EXP_PARTNER_NEGO_ABLE);
        break;
    case PHY_REG_EXT_STATUS:
        mdi_set_data(PHY_EXT_STATUS_1000FD_ABLE | PHY_EXT_STATUS_1000HD_ABLE);
        break;
    case PHY_REG_ERR_COUNT:
        mdi_set_data(0);
        break;
    default:
        _mdi_ctrl |= NIC_MDI_CTRL_ERROR;
        W_MESSAGE("invalid reg %u page %u (0x%x))", reg, _phy_page, _phy_ctrl);
        debug_unhandled_attention();
    }
}


void NIC::mdi_page0_read(uint reg)
{
    switch (reg) {
    case PHY_REG_P0_COPPER_CTRL_1:
        mdi_set_data(_phy_0_copper_ctrl_1);
        break;
    case PHY_REG_P0_COPPER_STATUS_1:
        D_MESSAGE("P0_COPPER_STATUS_1 0x%x", _phy_0_copper_status_1);
        mdi_set_data(_phy_0_copper_status_1);
        break;
    case 25:
        mdi_set_data(0);
        break;
    default:
        mdi_read_common(reg);
    }
}


void NIC::mdi_read(uint reg)
{
    NIC_LOG("%u", reg);

    switch (_phy_page) {
    case 0:
        mdi_page0_read(reg);
        break;
    default:
        mdi_read_common(reg);
    }

    _mdi_ctrl |= NIC_MDI_CTRL_READY;

    if (_mdi_ctrl & NIC_MDI_CTRL_INTERRUPT) {
        D_MESSAGE("NIC_REG_MDI_CTRL interrupt");
        interrupt(NIC_INT_CAUSE_MDIO_COMPLETE);
    }
}


void NIC::speed_detection()
{
    uint phy_speed = (_phy_0_copper_status_1 & PHY_P0_COPPER_STATUS_1_SPEED_MASK) >>
                                                            PHY_P0_COPPER_STATUS_1_SPEED_SHIFT;
    _status &= ~NIC_STATUS_AUTO_SPEED_MASK;
    _status |= (phy_speed << NIC_STATUS_AUTO_SPEED_SHIFT);
}


inline void NIC::tx_trigger()
{
    _tx_trigger->trigger();
}


inline void NIC::rx_trigger()
{
    _rx_trigger->trigger();
}


void NIC::csr_write(const uint8_t* src, uint64_t length, uint64_t dest)
{
    if ((dest & 0x03) || (length & 0x03)) {
        W_MESSAGE("not dword 0x%lx 0x%lx", dest, length);
        return;
    }

    const uint32_t* src_32 = (const uint32_t*)src;
    length = length / 4;

    for (; length--; src_32++, dest += 4) {
        NIC_LOG("csr write 0x%x  0x%x", dest, *src_32);

        Lock lock(_mutex);

        switch (dest) {
        case NIC_REG_INT_MASK_CLEAR:
            NIC_LOG("NIC_REG_INT_MASK_CLEAR 0x%x", *src_32);
            _int_mask &= ~*src_32;
            update_interrupt_level();
            break;
        case NIC_REG_INT_MASK_SET:
            NIC_LOG("NIC_REG_INT_MASK_SET 0x%x", *src_32);
            _int_mask = *src_32 & NIC_INT_MASK_SET_MASK;
            update_interrupt_level();
            break;
        case NIC_REG_SOFT_SEM:
            _soft_sem = *src_32 & NIC_SOFT_SEM_MASK;
            break;
        case NIC_REG_EXT_CONF_CTRL:
            if ((_ext_conf_ctrl & (NIC_EXT_CONF_CTRL_MDIO_HW | NIC_EXT_CONF_CTRL_MDIO_MNG))) {
                break;
            }
            if ((*src_32 & ~(NIC_EXT_CONF_CTRL_MDIO_SW | NIC_EXT_CONF_CTRL_RESERVED_SET))) {
                D_MESSAGE("unexpected NIC_REG_EXT_CONF_CTRL 0x%x", *src_32);
            }
            _ext_conf_ctrl = combine32(_ext_conf_ctrl, NIC_EXT_CONF_CTRL_MDIO_SW, *src_32);
            break;
        case NIC_REG_MDI_CTRL: {
            _mdi_ctrl = *src_32;
            _mdi_ctrl &= ~(NIC_MDI_CTRL_ADDRESS_MASK | NIC_MDI_CTRL_READY | NIC_MDI_CTRL_ERROR |
                           NIC_MDI_CTRL_RESSERVED);
            _mdi_ctrl |= NIC_MDI_CTRL_ADDRESS_GIGABIT;

            if ((*src_32 & NIC_MDI_CTRL_ADDRESS_MASK) != NIC_MDI_CTRL_ADDRESS_GIGABIT) {
                W_MESSAGE("bad address 0x%x", (*src_32 & NIC_MDI_CTRL_ADDRESS_MASK) >>
                          NIC_MDI_CTRL_ADDRESS_SHIFT);
                _mdi_ctrl |= NIC_MDI_CTRL_ERROR | NIC_MDI_CTRL_READY;

                if (_mdi_ctrl & NIC_MDI_CTRL_INTERRUPT) {
                    interrupt(NIC_INT_CAUSE_MDIO_COMPLETE);
                }

                break;
            }

            uint reg = (_mdi_ctrl & NIC_MDI_CTRL_REGADDR_MASK) >> NIC_MDI_CTRL_REGADDR_SHIFT;

            switch ((_mdi_ctrl & NIC_MDI_CTRL_OP_MASK)) {
            case NIC_MDI_CTRL_OP_READ:
                mdi_read(reg);
                break;
            case NIC_MDI_CTRL_OP_WRITE:
                mdi_write(reg);
                break;
            default:
                W_MESSAGE("invalid op 0x%x 0x%x",(_mdi_ctrl & NIC_MDI_CTRL_OP_MASK) >>
                                                                  NIC_MDI_CTRL_OP_SHIFT, _mdi_ctrl);
                debug_attention();
            }
            break;
        }
        case NIC_REG_CTRL:
        case NIC_REG_CTRL_ALIAS:
            _ctrl = combine32(_ctrl, ~NIC_CTRL_RESREVED_MASK, *src_32);

            if ((_ctrl & NIC_CTRL_SOFT_RESET)) {
                soft_reset();
                _ctrl &= ~NIC_CTRL_SOFT_RESET;
            }

            if ((_ctrl & NIC_CTRL_PHY_RESET)) {
                D_MESSAGE("NIC_CTRL_PHY_RESET");
                _ctrl &= ~NIC_CTRL_PHY_RESET;
                phy_reset();
            }

            if (_ctrl & NIC_CTRL_SET_LINK_UP) {
                mac_update_link_state();
            }

            if (_ctrl & NIC_CTRL_MASTER_DISABLE) {
                _status &= ~NIC_STATUS_MASTER_ENABLE_STATUS;
            }

            break;
        case NIC_REG_RX_CTRL:
            if ((_rx_ctrl & NIC_RX_CTRL_ENABLE) != (*src_32 & NIC_RX_CTRL_ENABLE)) {
                if ((*src_32 & NIC_RX_CTRL_ENABLE)) {
                    NIC_LOG("start RX");
                    _rx_queue[0].begin();
                    _rx_queue[1].begin();
                } else {
                    NIC_LOG("stop RX");
                    rx_write_back(0);
                }
            }
            _rx_ctrl = *src_32 & ~NIC_RX_CTRL_RESERVED;

            if ((_rx_ctrl & NIC_RX_CTRL_ENABLE)) {
                rx_trigger();
            }
            break;
        case NIC_REG_TX_CTRL:
            if ((_tx_ctrl & NIC_TX_CTRL_ENABLE) != (*src_32 & NIC_TX_CTRL_ENABLE)) {
                if ((*src_32 & NIC_TX_CTRL_ENABLE)) {
                    NIC_LOG("start TX");
                    _tx_queue[0].begin();
                    _tx_queue[1].begin();
                } else {
                    NIC_LOG("stop TX");
                    tx_write_back();
                }
            }
            _tx_ctrl = *src_32 & ~NIC_TX_CTRL_RESERVED;

            if ((_tx_ctrl & NIC_TX_CTRL_ENABLE)) {
                tx_trigger();
            }
            break;
        case NIC_REG_EEPROM_READ:
            _rom_read = *src_32 & ~(NIC_EEPROM_READ_START | NIC_EEPROM_READ_DONE);

            if (*src_32 & NIC_EEPROM_READ_START) {
                uint address = (_rom_read & NIC_EEPROM_READ_ADDR_MASK) >>
                                                                         NIC_EEPROM_READ_ADDR_SHIFT;

                _rom_read &= ~NIC_EEPROM_READ_DATA_MASK;

                if (address < EEPROM_WORDS) {
                    _rom_read |= (_rom[address] << NIC_EEPROM_READ_DATA_SHIFT) |
                                    NIC_EEPROM_READ_DONE;
                } else {
                    _rom_read |= NIC_EEPROM_READ_DONE;
                }

                NIC_LOG("rom read: address 0x%x val 0x%x", address, _rom_read >> 16);
            }
            break;
        case NIC_REG_PACKET_BUFF_ALLOC: {
            uint32_t rx_size = (*src_32 & NIC_PACKET_BUFF_ALLOC_RX_MASK);
            uint32_t tx_size = _packet_buff_size - rx_size;

            if (tx_size < NIC_PACKET_BUFF_ALLOC_TX_MIN || tx_size > NIC_PACKET_BUFF_SIZE_MAX) {
                W_MESSAGE("NIC_REG_PACKET_BUFF_ALLOC: bad rx size (0x%x)", *src_32);
                break;
            }

            _packet_buff_alloc = rx_size | (tx_size << NIC_PACKET_BUFF_ALLOC_TX_SHIFT);
            break;
        }
        case NIC_REG_PACKET_BUFF_SIZE: {
            uint buf_size = (*src_32 & NIC_PACKET_BUFF_SIZE_MASK);
            uint rx_size = _packet_buff_alloc & NIC_PACKET_BUFF_ALLOC_RX_MASK;
            uint tx_size = buf_size - rx_size;

            if (buf_size > NIC_PACKET_BUFF_SIZE_MAX || tx_size < NIC_PACKET_BUFF_ALLOC_TX_MIN ||
                                                            tx_size > NIC_PACKET_BUFF_SIZE_MAX) {
                W_MESSAGE("NIC_REG_PACKET_BUFF_SIZE: bad buf size (0x%x)", *src_32);
                break;
            }

            _packet_buff_size = buf_size;
            _packet_buff_alloc = rx_size | (tx_size << NIC_PACKET_BUFF_ALLOC_TX_SHIFT);
            break;
        }
        case NIC_REG_INT_CAUSE_READ:
            _int_cause &= ~*src_32;
            NIC_LOG("NIC_REG_INT_CAUSE_READ 0x%x", *src_32);
            update_interrupt_level();
            break;
        case NIC_REG_INT_CAUSE_SET:
            _int_cause |= *src_32 & NIC_INT_CAUSE_SET_MASK;
            NIC_LOG("NIC_REG_INT_CAUSE_SET 0x%x", *src_32);
            update_interrupt_level();
            break;
        case NIC_REG_INT_AUTO_MASK:
            _auto_mask = *src_32 & NIC_INT_AUTO_MASK_MASK;
            NIC_LOG("NIC_REG_INT_AUTO_MASK 0x%x 0x%x", *src_32, *src_32 & NIC_INT_AUTO_MASK_MASK);
            break;
        case NIC_REG_EXT_CTRL:
            _ext_ctrl = *src_32 & ~(NIC_EXT_CTRL_RESERVED | NIC_EXT_CTRL_EEPROM_RESET |
                                    NIC_EXT_CTRL_SPEED_DETECTION);

            if (*src_32 & NIC_EXT_CTRL_EEPROM_RESET) {
                eeprom_reset();
            }

            if (*src_32 & NIC_EXT_CTRL_SPEED_DETECTION) {
                speed_detection();
            }

            break;
        case NIC_REG_WAKEUP_CTRL:
            _wakeup_ctrl = *src_32 & ~(NIC_WAKEUP_CTRL_STATUS | NIC_WAKEUP_CTRL_RESERVED);
            break;
        case NIC_REG_WAKEUP_FILTER:
            _wakeup_filter = *src_32 & ~NIC_WAKEUP_FILTER_RESERVED;
            break;
        case NIC_REG_RX_DESCRIPTOR_ADDRESS_LOW_0:
            _rx_queue[0].set_addr_low(*src_32);
            break;
        case NIC_REG_RX_DESCRIPTOR_ADDRESS_LOW_1:
            _rx_queue[1].set_addr_low(*src_32);
            break;
        case NIC_REG_RX_DESCRIPTOR_ADDRESS_HIGH_0:
            _rx_queue[0].set_addr_high(*src_32);
            break;
        case NIC_REG_RX_DESCRIPTOR_ADDRESS_HIGH_1:
            _rx_queue[1].set_addr_high(*src_32);
            break;
        case NIC_REG_RX_DESCRIPTOR_LENGTH_0:
        case NIC_REG_RX_DESCRIPTOR_LENGTH_1:
             _rx_queue[(dest - NIC_REG_RX_DESCRIPTOR_LENGTH_0) >> 8].set_length(
                                                           *src_32 & NIC_RX_DESCRIPTOR_LENGTH_MASK);
             break;
        case NIC_REG_RX_DESCRIPTOR_HEAD_0:
        case NIC_REG_RX_DESCRIPTOR_HEAD_1:
            NIC_LOG("rx[%u] head %u q items is %u",
                      (dest - NIC_REG_RX_DESCRIPTOR_HEAD_0) >> 8,
                      *src_32 & 0xffff,
                      _rx_queue[(dest - NIC_REG_RX_DESCRIPTOR_HEAD_0) >> 8].num_items());
            _rx_queue[(dest - NIC_REG_RX_DESCRIPTOR_HEAD_0) >> 8].set_public_head(*src_32 & 0xffff);
            break;
        case NIC_REG_RX_DESCRIPTOR_TAIL_0:
        case NIC_REG_RX_DESCRIPTOR_TAIL_1:
            NIC_LOG("rx[%u] tail %u q items is %u",
                      (dest - NIC_REG_RX_DESCRIPTOR_TAIL_0) >> 8,
                      *src_32 & 0xffff,
                      _rx_queue[(dest - NIC_REG_RX_DESCRIPTOR_TAIL_0) >> 8].num_items());
            _rx_queue[(dest - NIC_REG_RX_DESCRIPTOR_TAIL_0) >> 8].set_tail(*src_32 & 0xffff);
            rx_trigger();
            break;
        case NIC_REG_RX_INT_DELAY_VAL:
           _rx_timer.set_val(*src_32);

            NIC_LOG("NIC_REG_RX_INT_DELAY_VAL %u", *src_32 & 0xffff);

            if (*src_32 & (1 << 31)) {
                NIC_LOG("NIC_REG_RX_INT_DELAY_TIMER: now");
                rx_write_back(NIC_INT_CAUSE_RX_TIMER);
            }
            break;
        case NIC_REG_RX_INT_ABS_DELAY_VAL:
            NIC_LOG("NIC_REG_RX_INT_ABS_DELAY_VAL");
            _rx_timer.set_abs_val(*src_32);
            break;
        case NIC_REG_TX_DESCRIPTOR_CTRL_0:
            NIC_LOG("NIC_REG_TX_DESCRIPTOR_CTRL_0: %u", *src_32);
            _tx_queue[0].set_descriptor_ctrl(combine32(_tx_queue[0].get_descriptor_ctrl(),
                                              ~NIC_TX_DESCRIPTOR_CTRL_RESERVED,
                                              *src_32));
            break;
        case NIC_REG_TX_DESCRIPTOR_CTRL_1:
            _tx_queue[1].set_descriptor_ctrl(combine32(_tx_queue[1].get_descriptor_ctrl(),
                                              ~NIC_TX_DESCRIPTOR_CTRL_RESERVED,
                                              *src_32));
            break;
        case NIC_REG_TX_ARBITRATION_COUNT_0:
            _tx_arbitration_count_0 = *src_32 & ~NIC_TX_ARBITRATION_COUNT_RESERVED;
            break;
        case NIC_REG_TX_ARBITRATION_COUNT_1:
            _tx_arbitration_count_1 = *src_32 & ~NIC_TX_ARBITRATION_COUNT_RESERVED;
            break;
        case NIC_REG_TX_DESCRIPTOR_ADDRESS_LOW_0:
        case NIC_REG_TX_DESCRIPTOR_ADDRESS_LOW_1:
            _tx_queue[(dest - NIC_REG_TX_DESCRIPTOR_ADDRESS_LOW_0) >> 8].set_addr_low(*src_32);
            break;
        case NIC_REG_TX_DESCRIPTOR_ADDRESS_HIGH_0:
        case NIC_REG_TX_DESCRIPTOR_ADDRESS_HIGH_1:
            _tx_queue[(dest - NIC_REG_TX_DESCRIPTOR_ADDRESS_HIGH_0) >> 8].set_addr_high(*src_32);
            break;
        case NIC_REG_TX_DESCRIPTOR_LENGTH_0:
        case NIC_REG_TX_DESCRIPTOR_LENGTH_1:
             _tx_queue[(dest - NIC_REG_TX_DESCRIPTOR_LENGTH_0) >> 8].set_length(
                                                           *src_32 & NIC_TX_DESCRIPTOR_LENGTH_MASK);
             break;
        case NIC_REG_TX_DESCRIPTOR_HEAD_0:
        case NIC_REG_TX_DESCRIPTOR_HEAD_1:
            NIC_LOG("tx[%u] head is %u q items is %u",
                      (dest - NIC_REG_TX_DESCRIPTOR_HEAD_0) >> 8,
                      *src_32 & 0xffff,
                      _rx_queue[(dest - NIC_REG_TX_DESCRIPTOR_HEAD_0) >> 8].num_items());
            _tx_queue[(dest - NIC_REG_TX_DESCRIPTOR_HEAD_0) >> 8].set_public_head(*src_32 & 0xffff);
            break;
        case NIC_REG_TX_DESCRIPTOR_TAIL_0:
        case NIC_REG_TX_DESCRIPTOR_TAIL_1:
            NIC_LOG("tx[%u] tail is %u q items is %u",
                      (dest - NIC_REG_TX_DESCRIPTOR_TAIL_0) >> 8,
                      *src_32 & 0xffff,
                      _rx_queue[(dest - NIC_REG_TX_DESCRIPTOR_TAIL_0) >> 8].num_items());
            _tx_queue[(dest - NIC_REG_TX_DESCRIPTOR_TAIL_0) >> 8].set_tail(*src_32 & 0xffff);
            tx_trigger();
            break;
        case NIC_REG_TX_INT_DELAY_VAL:
            _tx_timer.set_val(*src_32);

            NIC_LOG("NIC_REG_TX_INT_DELAY_VAL %u", *src_32 & 0xffff)

            if (*src_32 & (1 << 31)) {
                NIC_LOG("NIC_REG_TX_INT_DELAY_VAL: now");
                tx_write_back();
            }
            break;
        case NIC_REG_TX_INT_ABS_DELAY_VAL:
            _tx_timer.set_abs_val(*src_32);
            NIC_LOG("NIC_REG_TX_INT_DELAY_VAL %u", *src_32 & 0xffff);
            break;
        case NIC_REG_RX_CHECKSUM_CTRL:
            _rx_checksum_ctrl = *src_32 & ~NIC_RX_CHECKSUM_RESERVED;
            break;
        case NIC_REG_MULTI_RQ_CMD:
           _multi_rq_command = combine32(_multi_rq_command, ~NIC_MULTI_RQ_CMD_RESERVED, *src_32);
            break;
        case NIC_REG_3GIO_CTRL_1:
            _3gio_ctrl_1 = *src_32 & ~NIC_3GIO_CTRL_1_RESERVED;
            break;
        case NIC_REG_3GIO_CTRL_2:
            _3gio_ctrl_2 = *src_32 & ~NIC_3GIO_CTRL_2_RESERVED;
            break;
        case NIC_REG_LED_CTRL:
            _led_ctrl = *src_32 & ~NIC_LED_CTRL_RESERVED;
            break;
        case NIC_REG_VLAN_FILTER_TABLE_ARRAY ... NIC_REG_VLAN_FILTER_TABLE_ARRAY +
                                                 (VLAN_FILTER_TABLE_SIZE - 1 ) * 4:
            _vlan_filter_table[(dest - NIC_REG_VLAN_FILTER_TABLE_ARRAY) >> 2] = *src_32;
            break;
        case NIC_REG_RECEIVE_ADDR_START ... NIC_REG_RECEIVE_ADDR_END:
            _receive_addr_table[(dest - NIC_REG_RECEIVE_ADDR_START) >> 2] = *src_32;
            break;
        case NIC_REG_MULTICAST_TABLE_START ... NIC_REG_MULTICAST_TABLE_END:
            _multicast_table[(dest - NIC_REG_MULTICAST_TABLE_START) >> 2] = *src_32;
            break;
        case NIC_REG_FLOW_CTRL:
        case NIC_REG_FLOW_CTRL_ADDR_LOW:
        case NIC_REG_FLOW_CTRL_ADDR_HIGH:
        case NIC_REG_FLOW_CTRL_TTV:
        case NIC_REG_FLOW_CTRL_RT_LOW:
        case NIC_REG_FLOW_CTRL_RT_HIGH:
            break;
        case NIC_REG_RX_RSS_KEY_START ... NIC_REG_RX_RSS_KEY_END:
            _rx_rss_key[(dest - NIC_REG_RX_RSS_KEY_START) >> 2] = *src_32;
            break;
        case NIC_REG_RX_FILTER_CTRL:
            _rx_filter_ctrl = *src_32 & ~NIC_RX_FILTER_CTRL_RESERVED;
            break;
        case NIC_REG_RX_DESCRIPTOR_CTRL_0:
        case NIC_REG_RX_DESCRIPTOR_CTRL_1:
            break;
        case NIC_REG_REDIRECTION_START ... NIC_REG_REDIRECTION_END:
            _redirection_table[(dest - NIC_REG_REDIRECTION_START) >> 2] = *src_32 & 0x3f3f3f3f;
            break;
        case NIC_REG_VLAN_ETHER_TYPE:
            _vlan_ether_type = *src_32 & 0xffff;
            break;
        case NIC_REG_ADAPT_IFS_THROT:
            _adaptive_IFS_throttle = *src_32 & 0xffff;
            break;
        case NIC_REG_TIME_SYNC_TX_CTRL:
            _time_sync_tx_ctrl = combine32(_time_sync_tx_ctrl, (1 << 4), *src_32);

            if (_time_sync_tx_ctrl & (1 << 4)) {
                D_MESSAGE("TX Time stamping enabled");
            }

            break;
        case NIC_REG_TIME_SYNC_RX_CTRL:
            _time_sync_rx_ctrl = combine32(_time_sync_rx_ctrl, 0x1e, *src_32);

            if (_time_sync_rx_ctrl & (1 << 4)) {
                D_MESSAGE("RX Time stamping enabled");
            }
            break;
        case NIC_REG_TIME_SYNC_MESS_TYPE:
            _time_sync_mess_type = *src_32;
            break;
        case NIC_REG_TIME_SYNC_UDP_PORT:
            _time_sync_udp_port = *src_32 & 0xffff;
            break;
        case NIC_REG_TIME_SYNC_INC_ATTRIB:
            D_MESSAGE("period 0x%x value 0x%x", *src_32 >> 24, *src_32 & 0x00ffffff);
            break;
        case NIC_REG_INT_THROTTLING:
            _int_throttling = *src_32 & 0xffff;
            break;
        case NIC_REG_STATUS:
            if (!(*src_32 & NIC_STATUS_PHYRA)) {
                D_MESSAGE("NIC_REG_STATUS");
                _status &= ~NIC_STATUS_PHYRA;
            }
            break;
        default:
            W_MESSAGE("unhandled reg 0x%lx val 0x%lx", dest, *src_32);
            debug_unhandled_attention();
        }
    }
}


uint32_t NIC::io_read_dword(uint16_t port)
{
    port -= get_region_address(NIC_IO_BAR);

    if (port == NIC_IO_ADDR_PORT) {
        return _io_address;
    } else if (port == NIC_IO_DATA_PORT) {
        uint32_t data;
        csr_read(_io_address, sizeof (data), (uint8_t*)&data);
        return data;
    } else {
        W_MESSAGE("invalod port 0x%x", port + get_region_address(NIC_IO_BAR));
        return 0;
    }
}


void NIC::io_write_dword(uint16_t port, uint32_t val)
{
    port -= get_region_address(NIC_IO_BAR);

    if (port == NIC_IO_ADDR_PORT) {
        _io_address = val;
    } else if (port == NIC_IO_DATA_PORT) {
        csr_write((uint8_t*)&val, sizeof (val), _io_address);
    } else {
        W_MESSAGE("invalod port 0x%x", port + get_region_address(NIC_IO_BAR));
    }
}

