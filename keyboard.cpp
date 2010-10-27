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
    CTRL_CMD_WRITE_TO_MOUSE_OUTPUT = 0xd3,
    CTRL_CMD_WRITE_TO_MOUSE = 0xd4,
    CTRL_CMD_DISABLE_A20 = 0xdd,
    CTRL_CMD_ENABLE_A20 = 0xdf,
    CTRL_CMD_READ_TEST_INPUTS = 0xe0,
    CTRL_CMD_PULSE_0UTPUT_FIRST = 0xf0,
    CTRL_CMD_PULSE_0UTPUT_RESET = 0xfe,
    CTRL_CMD_PULSE_0UTPUT_LAST = 0xff,

    CTRL_SELF_TEST_REPLAY = 0x55,
    CTRL_KEYBOARD_INTERFACE_TEST_REPLAY = 0,
    CTRL_MOUSE_INTERFACE_TEST_REPLAY = 0,

    KBD_CMD_LED = 0xed,
    KBD_CMD_ECHO = 0xee,
    KBD_CMD_NOP2_FIRST = 0xef,
    KBD_CMD_NOP2_LAST = 0xf2,
    KBD_CMD_REPEAT_RATE = 0xf3,
    KBD_CMD_ENABLE_SCANNING = 0xf4,
    KBD_CMD_DISABLE_SCANNING_AND_SET_DEFAULTS = 0xf5,
    KBD_CMD_SET_DEFAUL = 0xf6,
    KBD_CMD_NOP1_FIRST = 0xf7,
    KBD_CMD_NOP1_LAST = 0xfd,
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
    MOUSE_CMD_RESOLUTUIN = 0xe8,
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
    MOUSE_STATE_SCALING_MASK = (1 << 5),
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
};


enum {
    WRITE_STATE_KBD_CMD,
    WRITE_STATE_MOUSE,
    WRITE_STATE_MOUSE_OUTPUT,
    WRITE_STATE_COMMAND_BYTE,
    WRITE_STATE_COMMAND_OUTPUT_PORT,
    WRITE_STATE_KBD_LEDS,
    WRITE_STATE_KBD_RATE,
};

enum {
    MOUSE_WRITE_STATE_CMD,
    MOUSE_WRITE_STATE_RESOLUTUIN,
    MOUSE_WRITE_STATE_SAMPLE_RATE,
};


KbdController::KbdController(NoxVM& nox)
    : VMPart("kbd", nox)
{
    IOBus& _bus = nox.get_io_bus();

    _io_region_a = _bus.register_region(*this, IO_PORT_KBD_DATA, 1, this,
                                        (io_read_byte_proc_t)&KbdController::io_read_port_a,
                                        (io_write_byte_proc_t)&KbdController::io_write_port_a);
    _io_region_b = _bus.register_region(*this, IO_PORT_KBD_COMMAND, 1, this,
                                        (io_read_byte_proc_t)&KbdController::io_read_status,
                                        (io_write_byte_proc_t)&KbdController::io_write_command);

    _keyboard_output.irq = pic->wire(*this, KBD_IRQ);
    _mouse_output.irq = pic->wire(*this, MOUSE_IRQ);
}

KbdController::~KbdController()
{
    get_nox().get_io_bus().unregister_region(_io_region_a);
    get_nox().get_io_bus().unregister_region(_io_region_b);
}

void KbdController::refill_outgoing()
{
    if (!_keyboard_output.buf.is_empty()) {
        _state |= CTRL_STATUS_DATA_READY_MASK;

        if (_command_byte & COMMAND_BYTE_IRQ1_MASK) {
            _keyboard_output.irq->raise();
        }

        _outgoing = _keyboard_output.buf.pop();

        return;
    }

    if (!_mouse_output.buf.is_empty()) {
        _state |= CTRL_STATUS_DATA_READY_MASK | CTRL_STATUS_MOUSE_DATA_READY_MASK;

        if (_command_byte & COMMAND_BYTE_IRQ12_MASK) {
            _mouse_output.irq->raise();
        }

        _outgoing = _mouse_output.buf.pop();

        return;
    }
}

uint8_t KbdController::io_read_port_a(uint16_t port)
{
    Lock lock(_mutex);

    if (!(_state & CTRL_STATUS_DATA_READY_MASK)) {
        D_MESSAGE("unexpected read");
    }

    uint8_t ret = _outgoing;
    _keyboard_output.irq->drop();
    _mouse_output.irq->drop();
    _state &= ~(CTRL_STATUS_DATA_READY_MASK | CTRL_STATUS_MOUSE_DATA_READY_MASK);

    refill_outgoing();

    return ret;
}


void KbdController::restore_keyboard_defaults()
{
    _kbd_enabled = true;
    _kbd_leds = 0;
    _kbd_rate = 0;
}


void KbdController::reset_keyboard()
{
    restore_keyboard_defaults();
    _keyboard_output.buf.reset();
    _keyboard_output.irq->drop();

    /*if (!(_state & CTRL_STATUS_MOUSE_DATA_READY_MASK)) {
        _state &= ~CTRL_STATUS_DATA_READY_MASK;
        refill_outgoing();
    }*/
}


void KbdController::restore_mouse_defaults()
{
    _mouse_scaling = false;
    _mouse_resolution = 100;
    _mouse_sample_rate = 30;
    _mouse_reporting = false;
    _mouse_reomte_mode = false;
}


void KbdController::reset_mouse()
{
    _mouse_button = 0;
    _mouse_write_state = MOUSE_WRITE_STATE_CMD,
    restore_mouse_defaults();
    _mouse_output.buf.reset();
    _mouse_output.irq->drop();
}


void KbdController::set_command_byte(uint8_t command_byte)
{
    if (_command_byte & COMMAND_BYTE_SYS_MASK) {
        _state |= CTRL_STATUS_SELF_TEST_MASK;
    } else {
        _state &= ~CTRL_STATUS_SELF_TEST_MASK;
    }

    if (_command_byte & COMMAND_BYTE_TRANSLATE_MASK) {
        D_MESSAGE("translate");
    }

    _command_byte = command_byte;
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

    // not sure about the following

    if (output_port & OUTPUT_PORT_IRQ1_MASK) {
        D_MESSAGE("raising keyboard irq");
        _keyboard_output.irq->raise();
    } else {
        D_MESSAGE("dropping keyboard irq");
        _keyboard_output.irq->drop();
    }

    if (output_port & OUTPUT_PORT_IRQ12_MASK) {
        D_MESSAGE("raising mouse irq");
        _mouse_output.irq->raise();
    } else {
        D_MESSAGE("dropping mouse irq");
        _mouse_output.irq->drop();
    }
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
    if (!mouse_is_active()) {
        return;
    }

    if (_mouse_warp_mode && val != MOUSE_CMD_RESET_WARP_MOD && val != MOUSE_CMD_RESET) {
        // send ack ?
        D_MESSAGE("send ack ?");
        put_mouse_data(val);
        return;
    }

    int write_state = _mouse_write_state;
    _mouse_write_state = MOUSE_WRITE_STATE_CMD;

    switch (write_state) {
    case MOUSE_WRITE_STATE_CMD:
        switch (val) {
        case MOUSE_CMD_READ_DATA:
            put_mouse_data(KBD_ACK);
            if (_mouse_reomte_mode) {
                D_MESSAGE("push current mouse position to mouse output buf ...");
            }
            break;
        case MOUSE_CMD_RESET_WARP_MOD:
            put_mouse_data(KBD_ACK);
            _mouse_warp_mode = true;
            break;
        case MOUSE_CMD_SET_WARP_MODE:
            put_mouse_data(KBD_ACK);
            _mouse_warp_mode = true;
            break;
        case MOUSE_CMD_SCALING_1_1:
            put_mouse_data(KBD_ACK);
            _mouse_scaling = false;
            break;
        case MOUSE_CMD_SCALING_2_1:
            put_mouse_data(KBD_ACK);
            _mouse_scaling = true;
            break;
        case MOUSE_CMD_RESOLUTUIN:
            put_mouse_data(KBD_ACK);
            _mouse_write_state = MOUSE_WRITE_STATE_RESOLUTUIN;
            break;
        case MOUSE_CMD_SAMPLE_RATE:
            put_mouse_data(KBD_ACK);
             _mouse_write_state = MOUSE_WRITE_STATE_SAMPLE_RATE;
            break;
        case MOUSE_CMD_ENABLE_DATA_REPORTING:
            put_mouse_data(KBD_ACK);
            _mouse_reporting = true;
            break;
        case MOUSE_CMD_RESET:
            reset_mouse();
            put_mouse_data(KBD_ACK);
            put_mouse_data(KBD_SELF_TEST_REPLAY);
            break;
        case MOUSE_CMD_STATUS:
            put_mouse_data(KBD_ACK);
            put_mouse_data((_mouse_button & MOUSE_STATE_BUTTON_MASK) |
                           (_mouse_scaling ? MOUSE_STATE_SCALING_MASK : 0) |
                           (_mouse_reomte_mode ? MOUSE_STATE_REMOTE_MODE_MASK : 0) |
                           (_mouse_reporting ? MOUSE_STATE_DATA_REPORTING_MASK : 0));
            put_mouse_data(_mouse_resolution);
            put_mouse_data(_mouse_sample_rate);
            break;
        case MOUSE_CMD_READ_ID:
            put_mouse_data(KBD_ACK);
            put_mouse_data(0);
            break;
        case MOUSE_CMD_DISABLE_DATA_REPORTING:
            put_mouse_data(KBD_ACK);
            _mouse_reporting = false;
            break;
        case MOUSE_CMD_STREAM_MODE:
            put_mouse_data(KBD_ACK);
            _mouse_reomte_mode = false;
            break;
        case MOUSE_CMD_REMOTE_MODE:
            put_mouse_data(KBD_ACK);
            _mouse_reomte_mode = true;
            break;
        case MOUSE_CMD_SET_DEFAULT:
            put_mouse_data(KBD_ACK);
            restore_mouse_defaults();
            break;
        case MOUSE_CMD_RESEND:
            put_mouse_data(KBD_ACK);
            D_MESSAGE("todo: resend last mouse packet");
            break;
        default:
            D_MESSAGE("unhandled command 0x%x", val);
            put_mouse_data(KBD_ACK);
        }
        break;
    case MOUSE_WRITE_STATE_RESOLUTUIN:
        _mouse_resolution = val;
        put_mouse_data(KBD_ACK);
        break;
    case MOUSE_WRITE_STATE_SAMPLE_RATE:
        _mouse_sample_rate = val;
        put_mouse_data(KBD_ACK);
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
        if (!keyboard_is_active()) {
            return;
        }
        switch (val) {
        case KBD_CMD_LED:
            put_data(KBD_ACK);
            _write_state = WRITE_STATE_KBD_LEDS;
            break;
        case KBD_CMD_DISABLE_SCANNING_AND_SET_DEFAULTS:
            _kbd_enabled = false;
            restore_keyboard_defaults();
            put_data(KBD_ACK);
            break;
        case KBD_CMD_SET_DEFAUL:
            restore_keyboard_defaults();
            put_data(KBD_ACK);
            break;
        case KBD_CMD_RESET:
            reset_keyboard();
            put_data(KBD_ACK);
            put_data(KBD_SELF_TEST_REPLAY);
            break;
        case KBD_CMD_ENABLE_SCANNING:
            _kbd_enabled = true;
            put_data(KBD_ACK);
            break;
        case KBD_CMD_ECHO:
            put_data(KBD_ECHO);
            break;
        case KBD_CMD_REPEAT_RATE:
            put_data(KBD_ACK);
            _write_state = WRITE_STATE_KBD_RATE;
            break;
        case KBD_CMD_RESEND:
            put_data(KBD_ACK);
            D_MESSAGE("todo: resend last mouse packet");
            break;
        default:
            if (!(val >= KBD_CMD_NOP2_FIRST && val <= KBD_CMD_NOP2_LAST) &&
               !(val >= KBD_CMD_NOP1_FIRST && val <= KBD_CMD_NOP1_LAST)) {
                D_MESSAGE("unhandled command 0x%x", val);
            }
            put_data(KBD_ACK);
        }
        break;
    case WRITE_STATE_MOUSE_OUTPUT:
        put_mouse_data(val);
        break;
    case WRITE_STATE_MOUSE:
        write_to_mouse(val);
        break;
    case WRITE_STATE_COMMAND_BYTE:
        set_command_byte(val);
        break;
    case WRITE_STATE_COMMAND_OUTPUT_PORT:
        write_output_port(val);
        break;
    case WRITE_STATE_KBD_LEDS:
        if (!keyboard_is_active()) {
            return;
        }
        put_data(KBD_ACK);
        _kbd_leds = val;
        break;
    case WRITE_STATE_KBD_RATE:
        if (!keyboard_is_active()) {
            return;
        }
        put_data(KBD_ACK);
        _kbd_rate = val;
        break;
    }
}


uint8_t KbdController::io_read_status(uint16_t port)
{
    return _state;
}


void KbdController::reset()
{
    get_nox().get_io_bus().remap_region(_io_region_a);
    get_nox().get_io_bus().remap_region(_io_region_b);

    reset_mouse();
    reset_keyboard();
    _state = 0;
    _command_byte = 0;
    _write_state = WRITE_STATE_KBD_CMD;
}


void KbdController::put_mouse_data(uint data)
{
    if (_mouse_output.buf.is_full()) {
        D_MESSAGE("full");
        return;
    }

    _mouse_output.buf.insert(data);

    if (!(_state & CTRL_STATUS_DATA_READY_MASK)) {
        refill_outgoing();
    }
}


void KbdController::put_data(uint data)
{
    if (_keyboard_output.buf.is_full()) {
        D_MESSAGE("full");
        return;
    }

    _keyboard_output.buf.insert(data);

    if (!(_state & CTRL_STATUS_DATA_READY_MASK)) {
        refill_outgoing();
    }
}


void KbdController::io_write_command(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    _state |= CTRL_STATUS_LAST_INPUT_COMMAND_MASK;

    switch (val) {
    case CTRL_CMD_READ_COMMAND_BYTE:
        if (!_keyboard_output.buf.is_empty()) {
            D_MESSAGE("CTRL_CMD_READ_COMMAND_BYTE while output is not ready")
        }
        put_data(_command_byte);
        break;
    case CTRL_CMD_SELF_TEST:
        reset();
        _state |= CTRL_STATUS_SELF_TEST_MASK;
        put_data(CTRL_SELF_TEST_REPLAY);
        break;
    case CTRL_CMD_KEYBOARD_INTERFACE_TEST:
        if (!_keyboard_output.buf.is_empty()) {
            D_MESSAGE("CTRL_INTERFACE_TEST_REPLAY while output is not ready")
        }
        put_data(CTRL_KEYBOARD_INTERFACE_TEST_REPLAY); // use COMMAND_BYTE_DISABLE_KYBD_MASK here?
        break;
    case CTRL_CMD_MOUSE_INTERFACE_TEST:
        if (!_keyboard_output.buf.is_empty()) {
            D_MESSAGE("CTRL_CMD_MOUSE_INTERFACE_TEST while output is not ready")
        }
        put_data(CTRL_MOUSE_INTERFACE_TEST_REPLAY); // use COMMAND_BYTE_DISABLE_MOUSE_MASK here?
        break;
    case CTRL_CMD_DIAGNOSTIC_DUMP:
        PANIC("CTRL_CMD_DIAGNOSTIC_DUMP: what?");
        break;
    case CTRL_CMD_DISABLE_MOUSE:
        _command_byte |= COMMAND_BYTE_DISABLE_MOUSE_MASK;
        break;
    case CTRL_CMD_ENABLE_MOUSE:
        _command_byte &= ~COMMAND_BYTE_DISABLE_MOUSE_MASK;
        break;
    case CTRL_CMD_DISABLE_KEYBOARD:
         _command_byte |= COMMAND_BYTE_DISABLE_KYBD_MASK;
        break;
    case CTRL_CMD_ENABLE_KEYBOARD:
        _command_byte &= ~COMMAND_BYTE_DISABLE_KYBD_MASK;
        break;
    case CTRL_CMD_WRITE_COMMAND_BYTE:
        _write_state = WRITE_STATE_COMMAND_BYTE;
        break;
    case CTRL_CMD_WRITE_TO_MOUSE:
         _write_state = WRITE_STATE_MOUSE;
        break;
    case CTRL_CMD_WRITE_TO_MOUSE_OUTPUT:
         _write_state = WRITE_STATE_MOUSE_OUTPUT;
        break;
    case CTRL_CMD_WRITE_OUTPUT_PORT:
        _write_state = WRITE_STATE_COMMAND_OUTPUT_PORT;
        break;
    case CTRL_CMD_READ_INPUT_PORT:
        if (!_keyboard_output.buf.is_empty()) {
            D_MESSAGE("CTRL_CMD_READ_INPUT_PORT while output is not ready")
        }
        put_data(0);
        break;
    case CTRL_CMD_READ_OUTPUT_PORT: {
        if (!_keyboard_output.buf.is_empty()) {
            D_MESSAGE("CTRL_CMD_READ_OUTPUT_PORT while output is not ready")
        }

        uint8_t output_port = OUTPUT_PORT_RESET_MASK;
        output_port |= memory_bus->line_20_is_set() ? OUTPUT_PORT_A20_MASK : 0;

        // not sure about the following
        output_port |= _keyboard_output.irq->is_high() ? OUTPUT_PORT_IRQ1_MASK : 0;
        output_port |= _mouse_output.irq->is_high() ? OUTPUT_PORT_IRQ12_MASK : 0;

        // do we need to use COMMAND_BYTE_DISABLE_KYBD_MASK and
        // COMMAND_BYTE_DISABLE_MOUSE_MASK here?

        put_data(output_port);
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
        put_data(0x3);  // keybaord and mouse input clock is up.
                        // do we need to use COMMAND_BYTE_DISABLE_KYBD_MASK and
                        // COMMAND_BYTE_DISABLE_MOUSE_MASK here?
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
    SET_ESCAPE(HOME, 0x6c, 0x47);
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

    if ((_command_byte & COMMAND_BYTE_DISABLE_KYBD_MASK) || !_kbd_enabled) {
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
        put_data(*scan);
    }
}


void KbdController::key_down(NoxKey code)
{
    // todo: test running state
    key_common(code, MAKE);
}


void KbdController::key_up(NoxKey code)
{
    // todo: test running state
    key_common(code, BREAK);
}

