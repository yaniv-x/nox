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

#ifndef _H_KEYBOARD
#define _H_KEYBOARD

#include "vm_part.h"
#include "threads.h"
#include "nox_key.h"

class NoxVM;
class IORegion;
class PICWire;

class CyclicBuffer {
public:
    CyclicBuffer()
    {
        reset();
    }

    enum {
        BUF_SIZE = 256,
    };

    void reset() { _head = _tail = 0;}
    bool is_empty() { return _head == _tail; }
    bool is_full() { return num_items() == capacity(); }
    uint num_items() { return _tail - _head; }
    uint capacity() { return BUF_SIZE;}

    void insert(uint8_t new_item) { buf[_tail++ % BUF_SIZE] = new_item; }
    uint8_t pop() { return buf[_head++ % BUF_SIZE];}
    uint8_t& item_at(uint index);

private:
    uint8_t buf[BUF_SIZE];
    uint _head;
    uint _tail;
};

class KbdController: public VMPart {
public:
    KbdController(NoxVM& vm);
    virtual ~KbdController();

    virtual void reset();
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void key_down(NoxKey code);
    void key_up(NoxKey code);

private:
    uint8_t io_read_port_a(uint16_t port);
    void io_write_port_a(uint16_t port, uint8_t val);
    uint8_t io_read_status(uint16_t port);
    void io_write_command(uint16_t port, uint8_t val);
    void put_mouse_data(uint data);
    void put_data(uint data);
    void restore_keyboard_defaults();
    void reset_keyboard();
    void restore_mouse_defaults();
    void reset_mouse();
    void set_command_byte(uint8_t command_byte);
    void write_output_port(uint8_t val);
    void write_to_mouse(uint8_t val);
    void refill_outgoing();
    bool mouse_is_active();
    bool keyboard_is_active();
    void key_common(NoxKey code, uint scan_index);

private:
    Mutex _mutex;
    IORegion *_io_region_a;
    IORegion *_io_region_b;

    struct KBCOutput {
        PICWire* irq;
        CyclicBuffer buf;
    };

    uint8_t _outgoing;

    KBCOutput _keyboard_output;
    bool _kbd_enabled;
    uint8_t _kbd_leds;
    uint8_t _kbd_rate;

    KBCOutput _mouse_output;
    bool _mouse_scaling;
    bool _mouse_reporting;
    bool _mouse_reomte_mode;
    bool _mouse_warp_mode;
    uint8_t _mouse_resolution;
    uint8_t _mouse_sample_rate;
    uint8_t _mouse_button;
    int _mouse_write_state;

    uint8_t _state;
    uint8_t _command_byte;
    int _write_state;
};

#endif

