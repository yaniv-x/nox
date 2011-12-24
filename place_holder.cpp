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

#include "place_holder.h"
#include "nox_vm.h"
#include "application.h"
#include "io_bus.h"


#define NUM_MICRO 30

PlaceHolder::PlaceHolder(NoxVM& nox)
    : VMPart ("holder", nox)
{
    add_io_region(io_bus->register_region(*this, 0x80, 1, this, NULL,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    add_io_region(io_bus->register_region(*this, 0x278, 3, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x378, 3, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));


    add_io_region(io_bus->register_region(*this, 0x2e8, 8, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x2f8, 8, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x3e8, 8, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x3f8, 8, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //disk controller 2
    add_io_region(io_bus->register_region(*this, 0x0170, 8, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x0376, 1, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //disk controller 2
    add_io_region(io_bus->register_region(*this, 0x01e8, 8, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x03e6, 1, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //disk controller 2
    add_io_region(io_bus->register_region(*this, 0x0168, 8, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x0366, 1, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));


    // game port
    add_io_region(io_bus->register_region(*this, 0x0200, 0x0210 - 0x0200, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //sound blaster
    add_io_region(io_bus->register_region(*this, 0x0220, 0x0230 - 0x0220, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //Gravis ultra sound
    add_io_region(io_bus->register_region(*this, 0x0240, 0x0250 - 0x0240, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //prototype cards
    add_io_region(io_bus->register_region(*this, 0x0300, 0x0320 - 0x0300, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //SCSI
    add_io_region(io_bus->register_region(*this, 0x0330, 0x0340 - 0x0330, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x0140, 0x0150 - 0x0140, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x0340, 0x0350 - 0x0340, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //ARTEC Handyscanner A400Z
    add_io_region(io_bus->register_region(*this, 0x035f, 1, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x015f, 1, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //Adaptec 154xB/154xC SCSI adapter.
    add_io_region(io_bus->register_region(*this, 0x234, 0x238 - 0x234, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x130, 0x134 - 0x130, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x134, 0x138 - 0x134, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(io_bus->register_region(*this, 0x230, 0x234 - 0x230, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

    // CompaQ tape drive adapter. alternate address at 0300
    add_io_region(io_bus->register_region(*this, 0x100, 0x110 - 0x100, this,
                                          (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                          (io_write_byte_proc_t)&PlaceHolder::write_byte));

}


PlaceHolder::~PlaceHolder()
{

}


void PlaceHolder::reset()
{
    remap_io_regions();
}


uint8_t PlaceHolder::read_byte(uint16_t port)
{
    return 0xff;
}


void PlaceHolder::write_byte(uint16_t port, uint8_t val)
{

}

uint16_t PlaceHolder::read_word(uint16_t port)
{
    return 0xff;
}


void PlaceHolder::write_word(uint16_t port, uint16_t val)
{

}

