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

#include "dma.h"
#include "nox_vm.h"
#include "io_bus.h"

enum {
    IO_ADDRESS_0 = 0x00,
    IO_COUNTER_0 = 0x01,
    IO_ADDRESS_1 = 0x02,
    IO_COUNTER_1 = 0x03,
    IO_ADDRESS_2 = 0x04,
    IO_COUNTER_2 = 0x05,
    IO_ADDRESS_3 = 0x06,
    IO_COUNTER_3 = 0x07,

    IO_DMA1_STATUS = 0x08,
    IO_DMA1_COMMAND = IO_DMA1_STATUS,

    IO_DMA1_REQUEST = 0x09,
    IO_DMA1_MASK = 0x0a,
    IO_DMA1_MODE = 0x0b,
    IO_DMA1_CLEAR = 0x0c,
    IO_DMA1_TEMP = 0x0d,
    IO_DMA1_DISABLE = IO_DMA1_TEMP,
    IO_DMA1_CLEAR_MASK = 0x0e,
    IO_DMA1_WRITE_MASK = 0x0f,
    IO_REGION_A_END,

    IO_ADDRESS_4 = 0xc0,
    IO_COUNTER_4 = 0xc2,
    IO_ADDRESS_5 = 0xc4,
    IO_COUNTER_5 = 0xc6,
    IO_ADDRESS_6 = 0xc8,
    IO_COUNTER_6 = 0xca,
    IO_ADDRESS_7 = 0xcc,
    IO_COUNTER_7 = 0xce,
    IO_REGION_B_END = IO_COUNTER_7 + 2,

    IO_DMA2_STATUS = 0xd0,
    IO_DMA2_COMMAND = IO_DMA2_STATUS,

    IO_DMA2_REQUEST = 0xd2,
    IO_DMA2_MASK = 0xd4,
    IO_DMA2_MODE = 0xd6,
    IO_DMA2_CLEAR = 0xd8,
    IO_DMA2_TEMP = 0xda,
    IO_DMA2_DISABLE = IO_DMA2_TEMP,
    IO_DMA2_CLEAR_MASK = 0xdc,
    IO_DMA2_WRITE_MASK = 0xde,
    IO_REGION_C_END = IO_DMA2_WRITE_MASK + 2,

    IO_PAGE_REGISTER_2 = 0x81,
    IO_PAGE_REGISTER_3 = 0x82,
    IO_PAGE_REGISTER_1 = 0x83,
    IO_REGION_D_END,

    IO_PAGE_REGISTER_0 = 0x87,
    IO_REGION_E_END,

    IO_PAGE_REGISTER_6 = 0x89,
    IO_PAGE_REGISTER_7 = 0x8a,
    IO_PAGE_REGISTER_5 = 0x8b,
    IO_REGION_F_END,

    IO_PAGE_REGISTER_4 = 0x8f,
    IO_REGION_G_END,
};


DMA::DMA(NoxVM& nox)
    : VMPart("dma", nox)
{
    memset(_chips, 0, sizeof(_chips));

    add_io_region(io_bus->register_region(*this, IO_ADDRESS_0,
                                          IO_REGION_A_END - IO_ADDRESS_0, this,
                                          (io_read_byte_proc_t)&DMA::read_byte,
                                          (io_write_byte_proc_t)&DMA::write_byte));

    add_io_region(io_bus->register_region(*this, IO_ADDRESS_4,
                                          IO_REGION_B_END - IO_ADDRESS_4, this,
                                          (io_read_byte_proc_t)&DMA::read_byte,
                                          (io_write_byte_proc_t)&DMA::write_byte));

    add_io_region(io_bus->register_region(*this, IO_DMA2_STATUS,
                                          IO_REGION_C_END - IO_DMA2_STATUS, this,
                                          (io_read_byte_proc_t)&DMA::read_byte,
                                          (io_write_byte_proc_t)&DMA::write_byte));

    add_io_region(io_bus->register_region(*this, IO_PAGE_REGISTER_2,
                                          IO_REGION_D_END - IO_PAGE_REGISTER_2, this,
                                          (io_read_byte_proc_t)&DMA::read_byte,
                                          (io_write_byte_proc_t)&DMA::write_byte));

    add_io_region(io_bus->register_region(*this, IO_PAGE_REGISTER_0,
                                          IO_REGION_E_END - IO_PAGE_REGISTER_0, this,
                                          (io_read_byte_proc_t)&DMA::read_byte,
                                          (io_write_byte_proc_t)&DMA::write_byte));

    add_io_region(io_bus->register_region(*this, IO_PAGE_REGISTER_6,
                                          IO_REGION_F_END - IO_PAGE_REGISTER_6, this,
                                          (io_read_byte_proc_t)&DMA::read_byte,
                                          (io_write_byte_proc_t)&DMA::write_byte));

    add_io_region(io_bus->register_region(*this, IO_PAGE_REGISTER_4,
                                          IO_REGION_G_END - IO_PAGE_REGISTER_4,
                                          this,
                                          (io_read_byte_proc_t)&DMA::read_byte,
                                          (io_write_byte_proc_t)&DMA::write_byte));
}


DMA::~DMA()
{
}


uint8_t DMA::read_byte(uint16_t port)
{
    D_MESSAGE("");
    return 0xff;
}


void DMA::disable_dma1()
{

}


void DMA::disable_dma2()
{

}


enum {
    DMA1,
    DMA2,
    CHANNEL_MASK = 0x03,
    MASK_SHIFT = 1,
};


void DMA::write_byte(uint16_t port, uint8_t val)
{
    D_MESSAGE("");

    switch (port) {
    case IO_DMA1_DISABLE:
        disable_dma1();
        break;
    case IO_DMA2_DISABLE:
        disable_dma2();
        break;
    case IO_DMA2_MODE:
        _chips[DMA2].chanels[val & CHANNEL_MASK].mode = val;
        break;
    case IO_DMA2_MASK:
        _chips[DMA2].chanels[val & CHANNEL_MASK].mask = (val >> MASK_SHIFT) & 1;
        break;
    }
}

void DMA::reset()
{
    memset(_chips, 0, sizeof(_chips));
    remap_io_regions();
}

