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

#include "keyboard.h"
#include "nox_vm.h"
#include "pic.h"
#include "io_bus.h"
#include "memory_bus.h"

enum {
    IO_PORT_KBD_DATA = 0x60,
    IO_PORT_KBD_COMMAND = 0x64,

    KBD_IRQ = 1,
    MOUSE_IRQ = 12,
};

enum {
    CTRL_STATUS_DATA_READY_MASK = (1 << 0),
    CTRL_STATUS_SELF_TEST_MASK = (1 << 2),
    CTRL_STATUS_LAST_INPUT_COMMAND_MASK = (1 << 3),
    CTRL_STATUS_KEYBORD_NOT_INHIBIT_MASK = (1 << 4),
    CTRL_STATUS_MOUSE_DATA_READY_MASK = (1 << 5),

    CTRL_CMD_READ_COMMAND_BYTE = 0x20,
    CTRL_CMD_WRITE_COMMAND_BYTE = 0x60,
    CTRL_CMD_DISABLE_MOUSE = 0xa7,
    CTRL_CMD_ENABLE_MOUSE,
    CTRL_CMD_MOUSE_INTERFACE_TEST,
    CTRL_CMD_SELF_TEST = 0xaa,
    CTRL_CMD_KEYBOARD_INTERFACE_TEST,
    CTRL_CMD_DIAGNOSTIC_DUMP,
    CTRL_CMD_DISABLE_KEYBOARD,
    CTRL_CMD_ENABLE_KEYBOARD,
    CTRL_CMD_READ_INPUT_PORT = 0xc0,
    CTRL_CMD_READ_OUTPUT_PORT = 0xd0,
    CTRL_CMD_WRITE_OUTPUT_PORT = 0xd1,
    CTRL_CMD_WRITE_KBD_OUTPUT = 0xd2,
    CTRL_CMD_WRITE_MOUSE_OUTPUT = 0xd3,
    CTRL_CMD_WRITE_TO_MOUSE = 0xd4,
    CTRL_CMD_DISABLE_A20 = 0xdd,
    CTRL_CMD_ENABLE_A20 = 0xdf,
    CTRL_CMD_READ_TEST_INPUTS = 0xe0,
    CTRL_CMD_PULSE_0UTPUT_FIRST = 0xf0,
    CTRL_CMD_PULSE_0UTPUT_RESET = 0xfe,
    CTRL_CMD_PULSE_0UTPUT_LAST = 0xff,

    CTRL_SELF_TEST_REPLAY = 0x55,
    CTRL_KEYBOARD_INTERFACE_TEST_REPLY_NO_ERROR = 0,
    CTRL_MOUSE_INTERFACE_TEST_REPLY_NO_ERROR = 0,

    KBD_CMD_LED = 0xed,
    KBD_CMD_ECHO = 0xee,
    KBD_CMD_GET_ID = 0xf2,
    KBD_CMD_REPEAT_RATE = 0xf3,
    KBD_CMD_ENABLE_SCANNING = 0xf4,
    KBD_CMD_DISABLE_SCANNING_AND_SET_DEFAULTS = 0xf5,
    KBD_CMD_SET_DEFAUL = 0xf6,
    KBD_CMD_RESEND = 0xfe,
    KBD_CMD_RESET = 0xff,

    KBD_SELF_TEST_REPLAY = 0xaa,
    KBD_ACK = 0xfa,
    KBD_NAK = 0xfe,
    KBD_ECHO = 0xee,

    KBD_LEDS_SCROLL_MASK = (1 << 0),
    KBD_LEDS_NUM_MASK = (1 << 1),
    KBD_LEDS_CAPS_MASK = (1 << 2),

    MOUSE_CMD_SCALING_1_1 = 0xe6,
    MOUSE_CMD_SCALING_2_1 = 0xe7,
    MOUSE_CMD_RESOLUTION = 0xe8,
    MOUSE_CMD_STATUS = 0xe9,
    MOUSE_CMD_STREAM_MODE = 0xea,
    MOUSE_CMD_READ_DATA = 0xeb,
    MOUSE_CMD_RESET_WARP_MOD = 0xec,
    MOUSE_CMD_SET_WARP_MODE = 0xee,
    MOUSE_CMD_REMOTE_MODE = 0xf0,
    MOUSE_CMD_READ_ID = 0xf2,
    MOUSE_CMD_SAMPLE_RATE = 0xf3,
    MOUSE_CMD_ENABLE_DATA_REPORTING = 0xf4,
    MOUSE_CMD_DISABLE_DATA_REPORTING = 0xf5,
    MOUSE_CMD_SET_DEFAULT = 0xf6,
    MOUSE_CMD_RESEND = 0xfe,
    MOUSE_CMD_RESET = 0xff,

    MOUSE_STATE_BUTTON_MASK = 0x7,
    MOUSE_STATE_SCALING_MASK = (1 << 4),
    MOUSE_STATE_DATA_REPORTING_MASK = (1 << 5),
    MOUSE_STATE_REMOTE_MODE_MASK = (1 << 6),

    OUTPUT_PORT_RESET_MASK = (1 << 1),
    OUTPUT_PORT_A20_MASK = (1 << 2),
    OUTPUT_PORT_IRQ1_MASK = (1 << 4),
    OUTPUT_PORT_IRQ12_MASK = (1 << 5),

    COMMAND_BYTE_TRANSLATE_MASK = (1 << 6),
    COMMAND_BYTE_DISABLE_MOUSE_MASK = (1 << 5),
    COMMAND_BYTE_DISABLE_KYBD_MASK = (1 << 4),
    COMMAND_BYTE_SYS_MASK = (1 << 2),
    COMMAND_BYTE_IRQ12_MASK = (1 << 1),
    COMMAND_BYTE_IRQ1_MASK = (1 << 0),

    MOUSE_PACKET_SIZE = 3,
};


enum {
    WRITE_STATE_KBD_CMD,
    WRITE_STATE_MOUSE,
    WRITE_STATE_KBD_OUTPUT,
    WRITE_STATE_MOUSE_OUTPUT,
    WRITE_STATE_COMMAND_BYTE,
    WRITE_STATE_OUTPUT_PORT,
};


enum {
    KBD_WRITE_STATE_CMD,
    KBD_WRITE_STATE_LEDS,
    KBD_WRITE_STATE_RATE,
};


enum {
    MOUSE_WRITE_STATE_CMD,
    MOUSE_WRITE_STATE_RESOLUTION,
    MOUSE_WRITE_STATE_SAMPLE_RATE,
};


enum {
    OUTPUT_SOURCE_SELF,
    OUTPUT_SOURCE_KEYBOARD,
    OUTPUT_SOURCE_MOUSE,
};


KbdController::KbdController(NoxVM& nox)
    : VMPart("kbd", nox)
    , _keyboard_output(*this)
    , _mouse_output(*this)
{
    add_io_region(io_bus->register_region(*this, IO_PORT_KBD_DATA, 1, this,
                                          (io_read_byte_proc_t)&KbdController::io_read_port_a,
                                          (io_write_byte_proc_t)&KbdController::io_write_port_a));
    add_io_region(io_bus->register_region(*this, IO_PORT_KBD_COMMAND, 1, this,
                                          (io_read_byte_proc_t)&KbdController::io_read_status,
                                          (io_write_byte_proc_t)&KbdController::io_write_command));

    pic->wire(_keyboard_output.irq_wire, KBD_IRQ);
    pic->wire(_mouse_output.irq_wire, MOUSE_IRQ);
}


KbdController::~KbdController()
{
}


uint8_t KbdController::get_output(KBCOutput& output)
{
    ASSERT(output.reply_count || !output.buf.is_empty());

    if (output.reply_count) {
        output.reply_count--;
        uint8_t ret = output.reply;
        output.reply >>= 8;
        return ret;
    }

    return output.buf.pop();
}


bool KbdController::keyboard_has_output()
{
    return keyboard_is_active() &&
           (_keyboard_output.reply_count || !_keyboard_output.buf.is_empty());
}


uint8_t KbdController::keyboard_get_output()
{
    return get_output(_keyboard_output);
}


void KbdController::compile_mouse_packet()
{
    ASSERT(_keyboard_output.buf.is_empty());

    uint8_t state = (1 << 3) | (_mouse_buttons & 0x7) |
                    ((uint32_t)(_mouse_dx & (1 << 31)) >> (31 - 4)) |
                    ((uint32_t)(_mouse_dy & (1 << 31)) >> (31 - 5));

    _mouse_output.buf.insert(state);
    _mouse_output.buf.insert(_mouse_dx);
    _mouse_output.buf.insert(_mouse_dy);

    _mouse_dx = _mouse_dy = 0;
    _mouse_packet_pending = false;
}


void KbdController::push_mouse_packet()
{
    if (!_keyboard_output.buf.is_empty()) {
        _mouse_packet_pending = true;
        return;
    }

    compile_mouse_packet();
    prepare_outgoing();
}


uint8_t KbdController::mouse_get_output()
{
    uint8_t ret = get_output(_mouse_output);;

    if (_mouse_output.buf.is_empty() && _mouse_packet_pending) {
        compile_mouse_packet();
    }

    return ret;
}


bool KbdController::mouse_has_output()
{
    return mouse_is_active() && (_mouse_output.reply_count || !_mouse_output.buf.is_empty());
}


void KbdController::prepare_outgoing()
{
    if ((_state & CTRL_STATUS_DATA_READY_MASK)) {
        return;
    }

    if (keyboard_has_output()) {
        _output_source = OUTPUT_SOURCE_KEYBOARD;
        _state |= CTRL_STATUS_DATA_READY_MASK;

        if (_command_byte & COMMAND_BYTE_IRQ1_MASK) {
            _keyboard_output.irq_wire.raise();
        }

        return;
    }

    if (mouse_has_output()) {
        _output_source = OUTPUT_SOURCE_MOUSE;
        _state |= CTRL_STATUS_DATA_READY_MASK | CTRL_STATUS_MOUSE_DATA_READY_MASK;

        if (_command_byte & COMMAND_BYTE_IRQ12_MASK) {
            _mouse_output.irq_wire.raise();
        }

        return;
    }

    _output_source = OUTPUT_SOURCE_SELF;
}


uint8_t KbdController::io_read_port_a(uint16_t port)
{
    Lock lock(_mutex);

    if (!(_state & CTRL_STATUS_DATA_READY_MASK)) {
        D_MESSAGE("unexpected read");
    }

    switch (_output_source) {
    case OUTPUT_SOURCE_MOUSE:
        _self_output = mouse_get_output();
        break;
    case OUTPUT_SOURCE_KEYBOARD:
        _self_output = keyboard_get_output();
        break;
    }

    _keyboard_output.irq_wire.drop();
    _mouse_output.irq_wire.drop();
    _state &= ~(CTRL_STATUS_DATA_READY_MASK | CTRL_STATUS_MOUSE_DATA_READY_MASK);

    prepare_outgoing();

    return _self_output;
}


void KbdController::restore_keyboard_defaults()
{
    keyboard_set_leds(0);

    //delay formula (1 + a) * 250 mili sec
    uint a = 1; // bits 5-6

    // period formula (2 ^ b) * (c + 8) / 240 sec
    // typematic rate 1/period
    uint b = 1; // bits 3-4
    uint c = 3; // bits 0-2

    _kbd_rate = (a << 5) /* 500 ms delay */ | (( b << 3) | c) /* 10.9 characters per second */;
}


void KbdController::reset_keyboard(bool cold)
{
    _kbd_scan_enabled = true;
    _kbd_write_state = KBD_WRITE_STATE_CMD;
    restore_keyboard_defaults();
    _keyboard_output.buf.reset();
    _keyboard_output.reply_count = 0;
    _keyboard_output.reply = 0;

    if (cold) {
        _keyboard_output.irq_wire.reset();
    } else {
        _keyboard_output.irq_wire.drop();
    }
}


void KbdController::restore_mouse_defaults()
{
    _mouse_scaling = false;
    _mouse_resolution = 2;
    _mouse_sample_rate = 100;
    _mouse_reporting = false;
    _mouse_reomte_mode = false;
    _mouse_warp_mode = false;
}


void KbdController::reset_mouse(bool cold)
{
    _mouse_packet_pending = false;
    _mouse_dx = 0;
    _mouse_dy = 0;
    _mouse_buttons = 0;
    _mouse_write_state = MOUSE_WRITE_STATE_CMD,
    restore_mouse_defaults();
    _mouse_output.buf.reset();
    _mouse_output.reply_count = 0;
    _mouse_output.reply = 0;

    if (cold) {
        _mouse_output.irq_wire.reset();
    } else {
        _mouse_output.irq_wire.drop();
    }
}


void KbdController::set_command_byte(uint8_t command_byte)
{
    if (command_byte & COMMAND_BYTE_SYS_MASK) {
        _state |= CTRL_STATUS_SELF_TEST_MASK;
    } else {
        _state &= ~CTRL_STATUS_SELF_TEST_MASK;
    }

    _command_byte = command_byte;

    // in case mouse or keyboard state changed to enabled
    prepare_outgoing();
}


void KbdController::write_output_port(uint8_t output_port)
{
    if (!(output_port & OUTPUT_PORT_RESET_MASK)) {
        throw ResetException();
    }

    if (output_port & OUTPUT_PORT_A20_MASK) {
        memory_bus->enable_address_line_20();
    } else {
        memory_bus->disable_address_line_20();
    }

    // OUTPUT_PORT_IRQ1_MASK and OUTPUT_PORT_IRQ12_MASK have no efect in my physical test machine
}


bool KbdController::mouse_is_active()
{
    return !(_command_byte & COMMAND_BYTE_DISABLE_MOUSE_MASK);
}


bool KbdController::keyboard_is_active()
{
    return !(_command_byte & COMMAND_BYTE_DISABLE_KYBD_MASK);
}


void KbdController::write_to_mouse(uint8_t val)
{
    _command_byte &= ~COMMAND_BYTE_DISABLE_MOUSE_MASK; // according to real world test the keyboard
                                                       // link is automaticlly enabled on writing
                                                       // to the keybord. although real word show
                                                       // otherwise, applying the same rule on the
                                                       // mouse interface.

    if (_mouse_warp_mode && val != MOUSE_CMD_RESET_WARP_MOD && val != MOUSE_CMD_RESET) {
        put_mouse_data(val);
        return;
    }

    int write_state = _mouse_write_state;
    _mouse_write_state = MOUSE_WRITE_STATE_CMD;

    bool resend = !_mouse_output.buf.is_empty();
    _mouse_output.buf.reset();
    _mouse_dx = _mouse_dy = 0;

    switch (write_state) {
    case MOUSE_WRITE_STATE_CMD:
        switch (val) {
        case MOUSE_CMD_READ_DATA:
            mouse_put_reply(KBD_ACK);
            push_mouse_packet();
            break;
        case MOUSE_CMD_RESET_WARP_MOD:
            mouse_put_reply(KBD_ACK);
            _mouse_warp_mode = false;
            break;
        case MOUSE_CMD_SET_WARP_MODE:
            mouse_put_reply(KBD_ACK);
            _mouse_warp_mode = true;
            break;
        case MOUSE_CMD_SCALING_1_1:
            mouse_put_reply(KBD_ACK);
            _mouse_scaling = false;
            break;
        case MOUSE_CMD_SCALING_2_1:
            mouse_put_reply(KBD_ACK);
            _mouse_scaling = true;
            break;
        case MOUSE_CMD_RESOLUTION:
            mouse_put_reply(KBD_ACK);
            // temporarily disabel streaming ?
            _mouse_write_state = MOUSE_WRITE_STATE_RESOLUTION;
            break;
        case MOUSE_CMD_SAMPLE_RATE:
            mouse_put_reply(KBD_ACK);
            // temporarily disabel streaming ?
            _mouse_write_state = MOUSE_WRITE_STATE_SAMPLE_RATE;
            break;
        case MOUSE_CMD_ENABLE_DATA_REPORTING:
            mouse_put_reply(KBD_ACK);
            _mouse_reporting = true;
            break;
        case MOUSE_CMD_RESET:
            reset_mouse(false);
            mouse_put_reply(KBD_ACK);
            mouse_put_reply(KBD_SELF_TEST_REPLAY);
            mouse_put_reply(0); // id
            break;
        case MOUSE_CMD_STATUS:
            mouse_put_reply(KBD_ACK);
            mouse_put_reply((_mouse_buttons & MOUSE_STATE_BUTTON_MASK) |
                            (_mouse_scaling ? MOUSE_STATE_SCALING_MASK : 0) |
                            (_mouse_reomte_mode ? MOUSE_STATE_REMOTE_MODE_MASK : 0) |
                            (_mouse_reporting ? MOUSE_STATE_DATA_REPORTING_MASK : 0));
            mouse_put_reply(_mouse_resolution);
            mouse_put_reply(_mouse_sample_rate);
            break;
        case MOUSE_CMD_READ_ID:
            mouse_put_reply(KBD_ACK);
            mouse_put_reply(0);
            break;
        case MOUSE_CMD_DISABLE_DATA_REPORTING:
            mouse_put_reply(KBD_ACK);
            _mouse_reporting = false;
            break;
        case MOUSE_CMD_STREAM_MODE:
            mouse_put_reply(KBD_ACK);
            _mouse_reomte_mode = false;
            break;
        case MOUSE_CMD_REMOTE_MODE:
            mouse_put_reply(KBD_ACK);
            _mouse_reomte_mode = true;
            break;
        case MOUSE_CMD_SET_DEFAULT:
            mouse_put_reply(KBD_ACK);
            restore_mouse_defaults();
            break;
        case MOUSE_CMD_RESEND:
            mouse_put_reply(KBD_ACK);
            D_MESSAGE("todo: resend last mouse packet");
            break;
        default:
            D_MESSAGE("unhandled command 0x%x", val);
            mouse_put_reply(KBD_NAK);
        }
        break;
    case MOUSE_WRITE_STATE_RESOLUTION:
        if (val > 4) {
            mouse_put_reply(KBD_NAK);
        } else {
            _mouse_resolution = val;
            mouse_put_reply(KBD_ACK);
        }
        break;
    case MOUSE_WRITE_STATE_SAMPLE_RATE:
        uint8_t rates[]= {10, 20, 40, 60, 80, 100, 200};
        std::set<uint8_t> rates_set(rates, rates + sizeof(rates));

        if (rates_set.find(val) == rates_set.end()) {
            mouse_put_reply(KBD_NAK);
        } else {
            _mouse_sample_rate = val;
            mouse_put_reply(KBD_ACK);
        }
        break;
    }

    if (resend && mouse_stream_test()) {
        push_mouse_packet();
    }
}


void KbdController::keyboard_set_leds(uint8_t val)
{
    _kbd_leds = val;
    D_MESSAGE("SCROLL-%s NUM-%s CAPS-%s",
              (_kbd_leds & KBD_LEDS_SCROLL_MASK) ? "ON " : "OFF",
              (_kbd_leds & KBD_LEDS_NUM_MASK) ? "ON " : "OFF",
              (_kbd_leds & KBD_LEDS_CAPS_MASK) ? "ON " : "OFF");
}


void KbdController::write_to_keyboard(uint8_t val)
{
    _command_byte &= ~COMMAND_BYTE_DISABLE_KYBD_MASK;  // according to real world test test the
                                                       // keyboard link is automaticlly enabled
                                                       // on writing to the keybord

    int write_state = _kbd_write_state;
    _kbd_write_state = KBD_WRITE_STATE_CMD;

    switch (write_state) {
    case KBD_WRITE_STATE_CMD:
        switch (val) {
        case KBD_CMD_LED:
            keyboard_put_reply(KBD_ACK);
            _kbd_write_state = KBD_WRITE_STATE_LEDS;
            break;
        case KBD_CMD_DISABLE_SCANNING_AND_SET_DEFAULTS:
            _kbd_scan_enabled = false;
            restore_keyboard_defaults();
            _keyboard_output.buf.reset();
            keyboard_put_reply(KBD_ACK);
            break;
        case KBD_CMD_SET_DEFAUL:
            restore_keyboard_defaults();
            _keyboard_output.buf.reset();
            keyboard_put_reply(KBD_ACK);
            break;
        case KBD_CMD_RESET:
            reset_keyboard(false);
            keyboard_put_reply(KBD_ACK);
            // flush the leds
            keyboard_set_leds(KBD_LEDS_SCROLL_MASK | KBD_LEDS_NUM_MASK | KBD_LEDS_CAPS_MASK);
            // todo: use timer to do the rest in delay of 1/4sec
            keyboard_put_reply(KBD_SELF_TEST_REPLAY);
            keyboard_set_leds(0);
            break;
        case KBD_CMD_ENABLE_SCANNING:
            _kbd_scan_enabled = true;
            keyboard_put_reply(KBD_ACK);
            break;
        case KBD_CMD_ECHO:
            keyboard_put_reply(KBD_ECHO);
            break;
        case KBD_CMD_REPEAT_RATE:
            keyboard_put_reply(KBD_ACK);
            _kbd_write_state = KBD_WRITE_STATE_RATE;
            break;
        case KBD_CMD_GET_ID:
            keyboard_put_reply(KBD_ACK);
            keyboard_put_reply(0xab); // what is the correct id ?
            keyboard_put_reply(0x41);
            break;
        case KBD_CMD_RESEND:
            D_MESSAGE("todo: resend valid data");
            keyboard_put_reply(0);
            break;
        default:
            D_MESSAGE("unhandled command 0x%x", val);
            keyboard_put_reply(KBD_NAK);
        }
        break;
    case KBD_WRITE_STATE_LEDS:
        keyboard_put_reply(KBD_ACK);
        keyboard_set_leds(val & 0x7);
        break;
    case KBD_WRITE_STATE_RATE:
        if (val & (1 << 7)) {
            keyboard_put_reply(KBD_NAK);
        } else {
            keyboard_put_reply(KBD_ACK);
            _kbd_rate = val;
        }
        break;
    }
}


void KbdController::io_write_port_a(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    _state &= ~CTRL_STATUS_LAST_INPUT_COMMAND_MASK;
    int write_state = _write_state;
    _write_state = WRITE_STATE_KBD_CMD;

    switch (write_state) {
    case WRITE_STATE_KBD_CMD:
        write_to_keyboard(val);
        break;
    case WRITE_STATE_KBD_OUTPUT:
        // brute force
        put_controller_data(val); // Write the keyboard controllers output buffer with the byte next
                                  // written to port 0x60, and act as if this was keyboard data.
        _state &= ~CTRL_STATUS_MOUSE_DATA_READY_MASK;
        if (_command_byte & COMMAND_BYTE_IRQ1_MASK) {
            _keyboard_output.irq_wire.raise();
        }
        break;
    case WRITE_STATE_MOUSE_OUTPUT:
        // brute force
        put_controller_data(val); // Write the keyboard controllers output buffer with the byte next
                                  // written to port 0x60, and act as if this was mouse data.
        _state |= CTRL_STATUS_MOUSE_DATA_READY_MASK;
        if (_command_byte & COMMAND_BYTE_IRQ12_MASK) {
            _mouse_output.irq_wire.raise();
        }
        break;
    case WRITE_STATE_MOUSE:
        write_to_mouse(val);
        break;
    case WRITE_STATE_COMMAND_BYTE:
        set_command_byte(val);
        break;
    case WRITE_STATE_OUTPUT_PORT:
        write_output_port(val);
        break;
    }
}


uint8_t KbdController::io_read_status(uint16_t port)
{
    return _state;
}


void KbdController::reset(bool cold)
{
    reset_mouse(cold);
    reset_keyboard(cold);
    _state = 0;
    _output_source = OUTPUT_SOURCE_SELF;
    _self_output = 0;
    _command_byte = COMMAND_BYTE_DISABLE_MOUSE_MASK | COMMAND_BYTE_DISABLE_KYBD_MASK;
    _write_state = WRITE_STATE_KBD_CMD;
    remap_io_regions();
}


void KbdController::reset()
{
    reset(true);
}


void KbdController::put_mouse_data(uint8_t data)
{
    if (_mouse_output.buf.is_full()) {
        D_MESSAGE("full");
        return;
    }

    _mouse_output.buf.insert(data);

    prepare_outgoing();
}


void KbdController::put_keyboard_data(uint8_t data)
{
    if (_keyboard_output.buf.is_full()) {
        D_MESSAGE("full");
        return;
    }

    _keyboard_output.buf.insert(data);

    prepare_outgoing();
}


void KbdController::put_reply(KBCOutput& output, uint8_t data)
{
    if (output.reply_count == sizeof(output.reply)) {
        D_MESSAGE("no space");
        return;
    }

    typeof(output.reply) next = data;
    next <<= (output.reply_count++ * 8);
    output.reply |= next;

    prepare_outgoing();
}


void KbdController::keyboard_put_reply(uint8_t data)
{
    put_reply(_keyboard_output, data);
}


void KbdController::mouse_put_reply(uint8_t data)
{
    put_reply(_mouse_output, data);
}


void KbdController::put_controller_data(uint8_t data)
{
    // according to real world test:
    //  - no irq
    //  - CTRL_STATUS_MOUSE_DATA_READY_MASK is ignored.
    //  - any pending data will be lost

    if ((_state & CTRL_STATUS_DATA_READY_MASK)) {
        D_MESSAGE("outgoing data lost");
    }

    _output_source = OUTPUT_SOURCE_SELF;
    _self_output = data;
    _state |= CTRL_STATUS_DATA_READY_MASK;
}


void KbdController::io_write_command(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    _state |= CTRL_STATUS_LAST_INPUT_COMMAND_MASK;

    switch (val) {
    case CTRL_CMD_READ_COMMAND_BYTE:
        put_controller_data(_command_byte);
        break;
    case CTRL_CMD_SELF_TEST:
        reset(false);
        _state |= CTRL_STATUS_SELF_TEST_MASK | CTRL_STATUS_KEYBORD_NOT_INHIBIT_MASK;
        _command_byte |= COMMAND_BYTE_SYS_MASK;
        put_controller_data(CTRL_SELF_TEST_REPLAY);
        break;
    case CTRL_CMD_KEYBOARD_INTERFACE_TEST:
        put_controller_data(CTRL_KEYBOARD_INTERFACE_TEST_REPLY_NO_ERROR);
        break;
    case CTRL_CMD_MOUSE_INTERFACE_TEST:
        put_controller_data(CTRL_MOUSE_INTERFACE_TEST_REPLY_NO_ERROR);
        break;
    case CTRL_CMD_DISABLE_MOUSE:
        _command_byte |= COMMAND_BYTE_DISABLE_MOUSE_MASK;
        break;
    case CTRL_CMD_ENABLE_MOUSE:
        _command_byte &= ~COMMAND_BYTE_DISABLE_MOUSE_MASK;
        prepare_outgoing();
        break;
    case CTRL_CMD_DISABLE_KEYBOARD:
         _command_byte |= COMMAND_BYTE_DISABLE_KYBD_MASK;
        break;
    case CTRL_CMD_ENABLE_KEYBOARD:
        _command_byte &= ~COMMAND_BYTE_DISABLE_KYBD_MASK;
        prepare_outgoing();
        break;
    case CTRL_CMD_WRITE_COMMAND_BYTE:
        _write_state = WRITE_STATE_COMMAND_BYTE;
        break;
    case CTRL_CMD_WRITE_TO_MOUSE:
         _write_state = WRITE_STATE_MOUSE;
        break;
    case CTRL_CMD_WRITE_KBD_OUTPUT:
         _write_state = WRITE_STATE_KBD_OUTPUT;
         break;
    case CTRL_CMD_WRITE_MOUSE_OUTPUT:
         _write_state = WRITE_STATE_MOUSE_OUTPUT;
        break;
    case CTRL_CMD_WRITE_OUTPUT_PORT:
        _write_state = WRITE_STATE_OUTPUT_PORT;
        break;
    case CTRL_CMD_READ_INPUT_PORT:
        // not sure about the following.
        put_controller_data(0xa7); // the value I got on physical machine
        break;
    case CTRL_CMD_READ_OUTPUT_PORT: {
        uint8_t output_port = OUTPUT_PORT_RESET_MASK;
        output_port |= memory_bus->line_20_is_set() ? OUTPUT_PORT_A20_MASK : 0;

        // not sure about the following
        output_port |= _keyboard_output.irq_wire.output() ? OUTPUT_PORT_IRQ1_MASK : 0;
        output_port |= _mouse_output.irq_wire.output() ? OUTPUT_PORT_IRQ12_MASK : 0;

        // do we need to use COMMAND_BYTE_DISABLE_KYBD_MASK and
        // COMMAND_BYTE_DISABLE_MOUSE_MASK here?

        put_controller_data(output_port);
        break;
    }
    case CTRL_CMD_PULSE_0UTPUT_RESET:
        throw ResetException();
    case CTRL_CMD_DISABLE_A20:
        memory_bus->disable_address_line_20();
        break;
    case CTRL_CMD_ENABLE_A20:
        memory_bus->enable_address_line_20();
        break;
    case CTRL_CMD_READ_TEST_INPUTS:
        put_controller_data(!!(_command_byte & COMMAND_BYTE_DISABLE_KYBD_MASK) |
                            (!!(_command_byte & COMMAND_BYTE_DISABLE_MOUSE_MASK) << 1));
        break;
    default:
        if (val < CTRL_CMD_PULSE_0UTPUT_FIRST) {
            D_MESSAGE("unhandled command 0x%x", val);
        }
    }
}

enum {
    MAKE,
    BREAK,
};

struct KeyScanCode {
    uint64_t make_break[2];
};

static KeyScanCode set_1_map[256];
static KeyScanCode set_2_map[256];


#define SET1_SIMPLE(name, key_val) {                                         \
    set_1_map[NOX_KEY_##name].make_break[MAKE] = key_val;                  \
    set_1_map[NOX_KEY_##name].make_break[BREAK] = key_val | 0x80;          \
}


#define SET2_SIMPLE(name, key_val) {                                         \
    set_2_map[NOX_KEY_##name].make_break[MAKE] = key_val;                  \
    set_2_map[NOX_KEY_##name].make_break[BREAK] = 0x0f | (key_val << 8);   \
}


#define SET_SIMPLE(name, set1_val, set2_val) {  \
    SET1_SIMPLE(name, set1_val);                \
    SET2_SIMPLE(name, set2_val);                \
}


#define SET1_ESCAPE(name, key_val) {                                             \
    set_1_map[NOX_KEY_##name].make_break[MAKE] = 0xe0 | (key_val << 8);        \
    set_1_map[NOX_KEY_##name].make_break[BREAK] = 0x80e0 | (key_val << 8);     \
}


#define SET2_ESCAPE(name, key_val) {                                             \
    set_2_map[NOX_KEY_##name].make_break[MAKE] = 0x00e0 | (key_val << 8);      \
    set_2_map[NOX_KEY_##name].make_break[BREAK] = 0xf0e0 | (key_val << 16);    \
}


#define SET_ESCAPE(name, set1_val, set2_val) {  \
    SET1_ESCAPE(name, set1_val);                \
    SET2_ESCAPE(name, set2_val);                \
}


static void __attribute__ ((constructor)) init_scan_codes()
{
    memset(set_1_map, 0, sizeof(set_1_map));
    memset(set_2_map, 0, sizeof(set_2_map));

    SET_SIMPLE(ESCAPE, 0x01, 0x76);
    SET_SIMPLE(1, 0x02, 0x16);
    SET_SIMPLE(2, 0x03, 0x1e);
    SET_SIMPLE(3, 0x04, 0x26);
    SET_SIMPLE(4, 0x05, 0x25);
    SET_SIMPLE(5, 0x06, 0x2e);
    SET_SIMPLE(6, 0x07, 0x36);
    SET_SIMPLE(7, 0x08, 0x3d);
    SET_SIMPLE(8, 0x09, 0x3e);
    SET_SIMPLE(9, 0x0a, 0x46);
    SET_SIMPLE(0, 0x0b, 0x45);
    SET_SIMPLE(MINUS, 0x0c, 0x4e);
    SET_SIMPLE(EQUAL, 0x0d, 0x55);
    SET_SIMPLE(BACKSPACE, 0x0e, 0x66);
    SET_SIMPLE(TAB, 0x0f, 0x0d);
    SET_SIMPLE(Q, 0x10, 0x15);
    SET_SIMPLE(W, 0x11, 0x1d);
    SET_SIMPLE(E, 0x12, 0x24);
    SET_SIMPLE(R, 0x13, 0x2d);
    SET_SIMPLE(T, 0x14, 0x2c);
    SET_SIMPLE(Y, 0x15, 0x35);
    SET_SIMPLE(U, 0x16, 0x3c);
    SET_SIMPLE(I, 0x17, 0x43);
    SET_SIMPLE(O, 0x18, 0x44);
    SET_SIMPLE(P, 0x19, 0x4d);
    SET_SIMPLE(LEFT_BRACKET, 0x1a, 0x54);
    SET_SIMPLE(RIGHT_BRACKET, 0x1b, 0x5b);
    SET_SIMPLE(RETURN, 0x1c, 0x5a);
    SET_SIMPLE(LEFT_CONTROL, 0x1d, 0x14);
    SET_SIMPLE(A, 0x1e, 0x1c);
    SET_SIMPLE(S, 0x1f, 0x1b);
    SET_SIMPLE(D, 0x20, 0x23);
    SET_SIMPLE(F, 0x21, 0x2b);
    SET_SIMPLE(G, 0x22, 0x34);
    SET_SIMPLE(H, 0x23, 0x33);
    SET_SIMPLE(J, 0x24, 0x3b);
    SET_SIMPLE(K, 0x25, 0x42);
    SET_SIMPLE(L, 0x26, 0x4b);
    SET_SIMPLE(SEMICOLON, 0x27, 0x4c);
    SET_SIMPLE(APOSTROPHE, 0x28, 0x52);
    SET_SIMPLE(BACKQUAT, 0x29, 0x0e);
    SET_SIMPLE(LEFT_SHIFT, 0x2a, 0x12);
    SET_SIMPLE(BACKSLASH, 0x2b, 0x5d);
    SET_SIMPLE(Z, 0x2c, 0x1a);
    SET_SIMPLE(X, 0x2d, 0x22);
    SET_SIMPLE(C, 0x2e, 0x21);
    SET_SIMPLE(V, 0x2f, 0x2a);
    SET_SIMPLE(B, 0x30, 0x32);
    SET_SIMPLE(N, 0x31, 0x31);
    SET_SIMPLE(M, 0x32, 0x3a);
    SET_SIMPLE(COMMA, 0x33, 0x41);
    SET_SIMPLE(DOT, 0x34, 0x49);
    SET_SIMPLE(SLASH, 0x35, 0x4a);
    SET_SIMPLE(RIGHT_SHIFT, 0x36, 0x59);
    SET_SIMPLE(PAD_MUL, 0x37, 0x7c);
    SET_SIMPLE(LEFT_ALT, 0x38, 0x11);
    SET_SIMPLE(SPACE, 0x39, 0x29);
    SET_SIMPLE(CAPSLOCK, 0x3a, 0x58);
    SET_SIMPLE(F1, 0x3b, 0x05);
    SET_SIMPLE(F2, 0x3c, 0x06);
    SET_SIMPLE(F3, 0x3d, 0x04);
    SET_SIMPLE(F4, 0x3e, 0x0c);
    SET_SIMPLE(F5, 0x3f, 0x03);
    SET_SIMPLE(F6, 0x40, 0x0b);
    SET_SIMPLE(F7, 0x41, 0x83);
    SET_SIMPLE(F8, 0x42, 0x0a);
    SET_SIMPLE(F9, 0x43, 0x01);
    SET_SIMPLE(F10, 0x44, 0x09);
    SET_SIMPLE(NUMLOCK, 0x45, 0x77);
    SET_SIMPLE(SCROLLLOCK, 0x46, 0x7e);
    SET_SIMPLE(PAD_7, 0x47, 0x6c);
    SET_SIMPLE(PAD_8, 0x48, 0x75);
    SET_SIMPLE(PAD_9, 0x49, 0x7d);
    SET_SIMPLE(PAD_MINUS, 0x4a, 0x7b);
    SET_SIMPLE(PAD_4, 0x4b, 0x6b);
    SET_SIMPLE(PAD_5, 0x4c, 0x73);
    SET_SIMPLE(PAD_6, 0x4d, 0x74);
    SET_SIMPLE(PAD_PLUSE, 0x4e, 0x79);
    SET_SIMPLE(PAD_1, 0x4f, 0x69);
    SET_SIMPLE(PAD_2, 0x50, 0x72);
    SET_SIMPLE(PAD_3, 0x51, 0x7a);
    SET_SIMPLE(PAD_0, 0x52, 0x70);
    SET_SIMPLE(PAD_DEL, 0x53, 0x71);
    SET_SIMPLE(F11, 0x57, 0x78);
    SET_SIMPLE(F12, 0x58, 0x07);

    SET_ESCAPE(PAD_ENTER, 0x1c, 0x5a);
    SET_ESCAPE(RIGHT_CONTROL, 0x1d, 0x14);
    SET_ESCAPE(PAD_DIV, 0x35, 0x4a);
    SET_ESCAPE(PRINT, 0x37, 0x7c);
    SET_ESCAPE(RIGHT_ALT, 0x38, 0x11);
    SET_ESCAPE(HOME, 0x47, 0x6c);
    SET_ESCAPE(UP, 0x48, 0x75);
    SET_ESCAPE(PAGE_UP, 0x49, 0x7d);
    SET_ESCAPE(LEFT, 0x4b, 0x6b);
    SET_ESCAPE(RIGHT, 0x4d, 0x74);
    SET_ESCAPE(END, 0x4f, 0x69);
    SET_ESCAPE(DOWN, 0x50, 0x72);
    SET_ESCAPE(PAGEDOWN, 0x51, 0x7a);
    SET_ESCAPE(INSERT, 0x52, 0x70);
    SET_ESCAPE(DELETE, 0x53, 0x71);
    SET_ESCAPE(LEFT_META, 0x5b, 0x1f);
    SET_ESCAPE(RIGHT_META, 0x5c, 0x27);
    SET_ESCAPE(MENU, 0x5d, 0x2f);

    set_1_map[NOX_KEY_PAUSE].make_break[MAKE] = 0xc59de1451de1;
    set_1_map[NOX_KEY_PAUSE].make_break[BREAK] = 0;

    set_2_map[NOX_KEY_PAUSE].make_break[MAKE] = 0x77f014f0e17714e1;
    set_2_map[NOX_KEY_PAUSE].make_break[BREAK] = 0;
}


void KbdController::key_common(NoxKey code, uint scan_index)
{
    ASSERT(scan_index < 2);

    Lock lock(_mutex);

    if (!_kbd_scan_enabled) {
        return;
    }

    if (code > 255 || code < 0) {
        D_MESSAGE("inavlid code");
        return;
    }

    if (_keyboard_output.buf.capacity() - _keyboard_output.buf.num_items() < sizeof(KeyScanCode)) {
        D_MESSAGE("no space");
        return;
    }

    bool set1 = !!(_command_byte & COMMAND_BYTE_TRANSLATE_MASK);
    KeyScanCode* map = (set1) ? set_1_map : set_2_map;
    uint8_t* scan =  (uint8_t*)&map[code].make_break[scan_index];
    uint8_t* scan_end  =  scan + sizeof(map[0].make_break[0]);

    for (; scan < scan_end && *scan; scan++) {
        put_keyboard_data(*scan);
    }
}


void KbdController::key_down(NoxKey code)
{
    if (get_state() != VMPart::RUNNING) {
        return;
    }

    key_common(code, MAKE);
}

static int flipflop = 0;

void KbdController::key_up(NoxKey code)
{
    if (get_state() != VMPart::RUNNING) {
        return;
    }

    // temporary: sticky alt
    if (code == NOX_KEY_LEFT_ALT) {
        flipflop ^= 1;
        if (flipflop) {
            D_MESSAGE("NOX_KEY_LEFT_ALT");
            return;
        }
    }

    key_common(code, BREAK);
}


bool KbdController::mouse_stream_test()
{
    return !_mouse_warp_mode && !_mouse_reomte_mode && _mouse_reporting && mouse_is_active();
}


void KbdController::mouse_motion(int dx, int dy)
{
    Lock lock(_mutex);

    if (get_state() != VMPart::RUNNING) {
        return;
    }

    _mouse_dx += dx;
    _mouse_dy -= dy;

    if (!mouse_stream_test()) {
        return;
    }

    push_mouse_packet();
}


void KbdController::mouse_button_press(MouseButton button)
{
    Lock lock(_mutex);

    if (get_state() != VMPart::RUNNING) {
        return;
    }

    _mouse_buttons |= (1 << button);

    if (!mouse_stream_test()) {
        return;
    }

    push_mouse_packet();
}


void KbdController::mouse_button_release(MouseButton button)
{
    Lock lock(_mutex);

    if (get_state() != VMPart::RUNNING) {
        return;
    }

    _mouse_buttons &= ~(1 << button);

    if (!mouse_stream_test()) {
        return;
    }

    push_mouse_packet();
}

