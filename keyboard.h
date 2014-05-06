/*
    Copyright (c) 2013-2014 Yaniv Kamay,
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
#include "wire.h"

class NoxVM;

class CyclicBuffer {
public:
    CyclicBuffer()
    {
        reset();
    }

    enum {
        BUF_SIZE = 64,
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
    virtual bool start() { return true;}
    virtual bool stop() { return true;}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void key_down(NoxKey code);
    void key_up(NoxKey code);
    void mouse_motion(int dx, int dy);
    void mouse_z_motion(int dz);

    enum MouseButton {
        MOUSE_LEFT_BUTTON,
        MOUSE_RIGHT_BUTTON,
        MOUSE_MIDDLE_BUTTON,
    };

    void mouse_button_press(MouseButton button);
    void mouse_button_release(MouseButton button);

private:
    struct KBCOutput {
        KBCOutput(KbdController& controller)
            : irq_wire(controller)
        {
        }

        Wire irq_wire;
        CyclicBuffer buf;
        uint reply_count;
        uint32_t reply;
    };

    uint8_t io_read_port_a(uint16_t port);
    void io_write_port_a(uint16_t port, uint8_t val);
    uint8_t io_read_status(uint16_t port);
    void io_write_command(uint16_t port, uint8_t val);
    void write_to_keyboard(uint8_t val);
    void put_mouse_data(uint8_t data);
    void put_keyboard_data(uint8_t data);
    void put_reply(KBCOutput& output, uint8_t data);
    void keyboard_put_reply(uint8_t data);
    void put_controller_data(uint8_t data);
    void restore_keyboard_defaults();
    void reset_keyboard(bool cold);
    void restore_mouse_defaults();
    void reset_mouse(bool cold);
    void reset(bool cold);
    void set_command_byte(uint8_t command_byte);
    void write_output_port(uint8_t val);
    void write_to_mouse(uint8_t val);
    void prepare_outgoing();
    bool mouse_is_active();
    bool keyboard_is_active();
    void key_common(NoxKey code, uint scan_index);
    bool mouse_stream_test();
    void mouse_put_reply(uint8_t val);
    void push_mouse_packet();
    void compile_mouse_packet();
    static uint8_t get_output(KBCOutput& output);
    uint8_t mouse_get_output();
    bool mouse_has_output();
    uint8_t keyboard_get_output();
    bool keyboard_has_output();
    void keyboard_set_leds(uint8_t val);

private:
    Mutex _mutex;

    uint _output_source;
    uint8_t _self_output;

    KBCOutput _keyboard_output;
    bool _kbd_scan_enabled;
    uint8_t _kbd_leds;
    uint8_t _kbd_rate;
    int _kbd_write_state;

    KBCOutput _mouse_output;
    bool _mouse_scaling;
    bool _mouse_reporting;
    bool _mouse_reomte_mode;
    bool _mouse_warp_mode;
    uint _sens;
    bool _intelli_mouse;
    uint8_t _mouse_resolution;
    uint8_t _mouse_sample_rate;
    uint8_t _mouse_buttons;
    int _mouse_write_state;
    bool _mouse_packet_pending;
    int32_t _mouse_dx;
    int32_t _mouse_dy;
    int32_t _mouse_dz;

    uint8_t _state;
    uint8_t _command_byte;
    int _write_state;
};

#endif

