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
#include "io_bus.h"
#include "threads.h"

class NoxVM;
class IORegion;

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
    bool is_full() { return size() == BUF_SIZE; }
    uint size() { return _tail - _head; }

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

    NoxVM& nox() { return *(NoxVM*)get_container();}

    virtual void reset();
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

private:
    uint8_t io_read_port_a(uint16_t port);
    void io_write_port_a(uint16_t port, uint8_t val);
    uint8_t io_read_status(uint16_t port);
    void io_write_command(uint16_t port, uint8_t val);
    void put_data(uint data);
    void enable_keyboard();
    void disable_keyboard();
    void restore_keyboard_defaults();
    void clear_outputs();
    void reset_keybord();
    void set_command_byte(uint command_byte);

public:
    Mutex _mutex;
    IORegion *_io_region_a;
    IORegion *_io_region_b;
    uint8_t _state;
    CyclicBuffer _output;
    bool _keyboard_line_enabled;
    bool _mouse_line_enabled;
    bool _expect_command_byte;
};

#endif

