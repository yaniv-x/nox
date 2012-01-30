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

#ifndef _H_NOX
#define _H_NOX

#ifdef __GNUC__
#define ATTR_PACKED_1
#define ATTR_PACKED_2 __attribute__ ((__packed__))
#else
#define ATTR_PACKED_1 _Packed
#define ATTR_PACKED_2
#endif

#define NOX_PCI_VENDOR_ID 0x1aaa
#define NOX_PCI_DEV_ID_HOST_BRIDGE 0x0001
#define NOX_PCI_DEV_HOST_BRIDGE_REV 1
#define NOX_PCI_DEV_ID_PM_CONTROLLER 0x0010
#define NOX_PCI_DEV_PM_CONTROLLER_REV 1
#define NOX_ADDRESS_LINES 52
#define NOX_PCI_IRQ_LINES_MASK 0xdef8 // exclude: PIT, keyboard, PIC slave, RTC, and DMA
#define NOX_PCI_IRQ_EXCLUSIVE_MASK (NOX_PCI_IRQ_LINES_MASK & ~(0xd000)) // exclude: mouse, and
                                                                            //          lagacy ide

enum {
    HOST_BRIDGE_SLOT = 0,
    ISA_BRIDGE_SLOT = 1,
    PM_CONTROLLER_SLOT = 2,
};


enum {
    PM_IO_STATUS = 0x00,
    PM_IO_ENABLE = PM_IO_STATUS + 2,
    PM_IO_CONTROL = 0x04,
    PM_IO_RESET = 0x06,
    PM_IO_TIMER = 0x08,

    PM_IO_END = 0x0c,

    PM_RESET_MAGIC = 0xbf,
    PM_IRQ_LINE = 9,
};


enum {
    PLATFORM_IO_LOCK = 0x00,
    PLATFORM_IO_SELECT = 0x01,
    PLATFORM_IO_LOG = 0x02,
    PLATFORM_IO_CMD = 0x03,
    PLATFORM_IO_BYTE = 0x04,

    PLATFORM_IO_ERROR = 0x08,
    PLATFORM_IO_REGISTER = 0x0c,
    PLATFORM_IO_END = 0x10,

    PLATFORM_MEM_PAGES = 1,
    PLATFORM_LOG_BUF_START = 0,
    PLATFORM_LOG_BUF_SIZE = 1024,
    PLATFORM_CMD_BUF_START = PLATFORM_LOG_BUF_START,
    PLATFORM_CMD_BUF_SIZE = PLATFORM_LOG_BUF_SIZE,
    PLATFORM_BIOS_DATA_START = PLATFORM_CMD_BUF_START + PLATFORM_CMD_BUF_SIZE,
    PLATFORM_BIOS_DATA_SIZE = 1024,
};

enum {
    PLATFORM_REG_BELOW_1M_USED_PAGES,
    PLATFORM_REG_ABOVE_1M_PAGES,
    PLATFORM_REG_BELOW_4G_PAGES,
    PLATFORM_REG_BELOW_4G_USED_PAGES,
    PLATFORM_REG_ABOVE_4G_PAGES,
    PLATFORM_REG_WRITE_POS,
    PLATFORM_REG_READ_POS,
};

enum {
    PALTFORM_CMD_SET_PCI_IRQ = 1,
};


typedef ATTR_PACKED_1 struct ATTR_PACKED_2 PCmdSetIRQ {
    uint8_t bus;
    uint8_t device;
    uint8_t pin;
    uint8_t irq;
    uint8_t ret_val;
} PCmdSetIRQ;


#define PLATFORM_ERR_TYPE_SHIFT 29
#define PLATFORM_ERR_SUBSYS_SHIFT 16

#define PLATFORM_MK_ERR(type, subsys, code)                     \
    (((uint32_t)(type) << PLATFORM_ERR_TYPE_SHIFT) |            \
    ((uint32_t)(subsys) << PLATFORM_ERR_SUBSYS_SHIFT) |         \
    (code))


enum {
    PLATFORM_ERR_TYPE_INFO = 1,
    PLATFORM_ERR_TYPE_WARN,
    PLATFORM_ERR_TYPE_ERROR,
};


enum {
    PLATFORM_ERR_SUBSYS_BIOS = 1,
};


#endif

