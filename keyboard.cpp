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

enum {
    IO_PORT_KBD_DATA = 0x60,
    IO_PORT_KBD_COMMAND = 0x64,
};

enum {
    CTRL_STATUS_DATA_READY_MASK = (1 << 0),
    CTRL_STATUS_INPUT_FULL_MASK = (1 << 1),
    CTRL_STATUS_SELF_TEST_MASK = (1 << 2),
    CTRL_STATUS_LAST_INPUT_COMMAND_MASK = (1 << 3),
    CTRL_STATUS_INHIBIT_MASK = (1 << 4),
    CTRL_STATUS_TRANSMIT_TIMOUT_MASK = (1 << 5),
    CTRL_STATUS_RECIVE_TIMOUT_MASK = (1 << 6),
    CTRL_STATUS_PARITY_MASK = (1 << 7),

    CTRL_CMD_COMMAND_BYTE = 0x60,
    CTRL_CMD_ENABLE_MOUSE = 0xa8,
    CTRL_CMD_SELF_TEST = 0xaa,
    CTRL_CMD_INTERFACE_TEST,
    CTRL_CMD_ENABLE_KEYBOARD = 0xae,

    CTRL_SELF_TEST_REPLAY = 0x55,
    CTRL_INTERFACE_TEST_REPLAY = 0,

    KBD_CMD_RESET = 0xff,
    KBD_ENABLE = 0xf4,
    KBD_CMD_SET_DEFAULT_AND_DISABLE = 0xf5,

    KBD_SELF_TEST_REPLAY = 0xaa,
    KBD_ACK = 0xfa,
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
}

KbdController::~KbdController()
{
    nox().get_io_bus().unregister_region(_io_region_a);
    nox().get_io_bus().unregister_region(_io_region_b);
}

uint8_t KbdController::io_read_port_a(uint16_t port)
{
    Lock lock(_mutex);

    uint items = _output.size();

    if (!items) {
        D_MESSAGE("no output")
        return 0xff;
    }

    if (items == 1) {
        _state &= ~CTRL_STATUS_DATA_READY_MASK;
    }

    return _output.pop();
}

void KbdController::disable_keyboard()
{

}

void KbdController::enable_keyboard()
{

}

void KbdController::restore_keyboard_defaults()
{

}


void KbdController::clear_outputs()
{
    _output.reset();
    _state &= ~CTRL_STATUS_DATA_READY_MASK;
}

void KbdController::reset_keybord()
{

}

void KbdController::set_command_byte(uint command_byte)
{

}

void KbdController::io_write_port_a(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    _state &= ~CTRL_STATUS_LAST_INPUT_COMMAND_MASK;

    if (_expect_command_byte) {
        _expect_command_byte = false;
        set_command_byte(val);
        return;
    }

    switch (val) {
    case KBD_CMD_SET_DEFAULT_AND_DISABLE:
        disable_keyboard();
        restore_keyboard_defaults();
        clear_outputs();
        put_data(KBD_ACK);
        break;
    case KBD_CMD_RESET:
        reset_keybord();
        put_data(KBD_ACK);
        put_data(KBD_SELF_TEST_REPLAY);
        break;
    case KBD_ENABLE:
        enable_keyboard();
        put_data(KBD_ACK);
        break;
    default:
         D_MESSAGE("0x%x", val);
         for (;;) { sleep(1);}
    }
}


uint8_t KbdController::io_read_status(uint16_t port)
{
    return _state;
}


void KbdController::reset()
{
    nox().get_io_bus().remap_region(_io_region_a);
    nox().get_io_bus().remap_region(_io_region_b);

    _state = 0;
    _keyboard_line_enabled = false;
    _mouse_line_enabled = false,
    _expect_command_byte = false;
    _output.reset();
}

void KbdController::put_data(uint data)
{
    if (_output.is_full()) {
        D_MESSAGE("full");
        return;
    }
    _output.insert(data);
    _state |= CTRL_STATUS_DATA_READY_MASK;
}

void KbdController::io_write_command(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    _state |= CTRL_STATUS_LAST_INPUT_COMMAND_MASK;

    switch (val) {
    case CTRL_CMD_SELF_TEST:
        reset();
        _state |= CTRL_STATUS_SELF_TEST_MASK;
        put_data(CTRL_SELF_TEST_REPLAY);
        break;
    case CTRL_CMD_INTERFACE_TEST:
        put_data(CTRL_INTERFACE_TEST_REPLAY);
        break;
    case CTRL_CMD_ENABLE_KEYBOARD:
        _keyboard_line_enabled = true;
        break;
    case CTRL_CMD_ENABLE_MOUSE:
        _mouse_line_enabled = true;
        break;
    case CTRL_CMD_COMMAND_BYTE:
        _expect_command_byte = true;
        break;
    default:
        D_MESSAGE("0x%x", val);
        for( ;;) sleep(1);
    }
}

