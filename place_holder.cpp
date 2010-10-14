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
    , _io (nox.get_io_bus())
{
    _parallel_region_a = _io.register_region(*this, 0x278, 3, this,
                                             (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                             (io_write_byte_proc_t)&PlaceHolder::write_byte);
    _parallel_region_b = _io.register_region(*this, 0x378, 3, this,
                                             (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                             (io_write_byte_proc_t)&PlaceHolder::write_byte);


    _serial_region_a = _io.register_region(*this, 0x2e8, 8, this,
                                           (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                           (io_write_byte_proc_t)&PlaceHolder::write_byte);
    _serial_region_b = _io.register_region(*this, 0x2f8, 8, this,
                                           (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                           (io_write_byte_proc_t)&PlaceHolder::write_byte);
    _serial_region_c = _io.register_region(*this, 0x3e8, 8, this,
                                           (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                           (io_write_byte_proc_t)&PlaceHolder::write_byte);
    _serial_region_d = _io.register_region(*this, 0x3f8, 8, this,
                                           (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                           (io_write_byte_proc_t)&PlaceHolder::write_byte);

    //disk controller 2
    add_io_region(_io.register_region(*this, 0x0170, 8, this,
                                      (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                      (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(_io.register_region(*this, 0x0376, 1, this,
                                      (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                      (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //disk controller 2
    add_io_region(_io.register_region(*this, 0x01e8, 8, this,
                                      (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                      (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(_io.register_region(*this, 0x03e6, 1, this,
                                      (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                      (io_write_byte_proc_t)&PlaceHolder::write_byte));

    //disk controller 2
    add_io_region(_io.register_region(*this, 0x0168, 8, this,
                                      (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                      (io_write_byte_proc_t)&PlaceHolder::write_byte));
    add_io_region(_io.register_region(*this, 0x0366, 1, this,
                                      (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                      (io_write_byte_proc_t)&PlaceHolder::write_byte));

    // VBE
    add_io_region(_io.register_region(*this, 0x01ce, 3, this,
                                      (io_read_byte_proc_t)&PlaceHolder::read_byte,
                                      (io_write_byte_proc_t)&PlaceHolder::write_byte,
                                      (io_read_word_proc_t)&PlaceHolder::read_word,
                                      (io_write_word_proc_t)&PlaceHolder::write_word));


    t2 = application->create_timer((void_callback_t)&PlaceHolder::timer_call_2222222222, this);
    //t2->arm(1000ULL * NUM_MICRO , true);

    t = application->create_timer((void_callback_t)&PlaceHolder::timer_call, this);
    //t->arm(1000ULL * 1000 * 1000 * 1, true);
    event = application->create_event((void_callback_t)&PlaceHolder::trigger_call, this);
}


PlaceHolder::~PlaceHolder()
{
    _io.unregister_region(_parallel_region_a);
    _io.unregister_region(_parallel_region_b);


    _io.unregister_region(_serial_region_a);
    _io.unregister_region(_serial_region_b);
    _io.unregister_region(_serial_region_c);
    _io.unregister_region(_serial_region_d);
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

void PlaceHolder::trigger_call()
{
   // D_MESSAGE("******************************************8");
}


void PlaceHolder::timer_call()
{
   // D_MESSAGE("");
    static int count = 0;
    count++;

    if ((count % 10) == 0) {
        event->trigger();
        //t->disarm();
        //t->arm(1000ULL * 1000 * 1000 * 10, true);
    }
}

void PlaceHolder::timer_call_2222222222()
{
    static uint64_t counter = 0;
    static nox_time_t start = 0;

    if (counter == 0) {
        start = get_monolitic_time();
    }

    ++counter;

    if ((counter % (1000 * 1000 / NUM_MICRO)) == 0) {
        uint64_t now = get_monolitic_time();
        D_MESSAGE("%llu", now - start);
        start = now;
    }
}

