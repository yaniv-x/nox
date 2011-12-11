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

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/fcntl.h>
#include "common.h"
#include "gdb_target.h"
#include "run_loop.h"
#include "nox_vm.h"
#include "memory_bus.h"
#include "cpu.h"


/*
    todo:
        support target descriptions via qXfer where annex is target.xml
        support multi thread including qXfer:threads:read
        support G command
        support z0 and Z0
        discconect on mode change
        translet ip in case of 16bit
        support 64bit
*/


#define INTERRUPT 0x03


enum {
    DISCONNECTED,
    ATTACHING,
    ATTACHED,
    DETACHING,
    DETACHED,
    TERMINATING,
};


enum {
    START,
    DATA,
    SUM0,
    SUM1,
};


GDBTarget::GDBTarget(NoxVM& vm, RunLoop& loop)
    : _vm (vm)
    , _loop (loop)
    , _listenr (-1)
    , _accept_event (NULL)
    , _connection (-1)
    , _io_event (NULL)
    , _state (DISCONNECTED)
{
    struct sockaddr_in address;

    int listner = socket(PF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);

    if (listner == -1) {
        THROW_SYS_ERROR("socket failed");
    }

    int opt_val = 1;
    if (setsockopt(listner, SOL_SOCKET, SO_REUSEADDR, &opt_val, sizeof(opt_val)) == -1) {
        D_MESSAGE("setsockopt failed");
    }

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(3132);
    address.sin_addr.s_addr = INADDR_ANY;

    if (bind(listner, (struct sockaddr*) &address, sizeof(address)) == -1) {
        THROW_SYS_ERROR("bind failed");
    }

    if (listen(listner, 1) == -1) {
        THROW_SYS_ERROR("listen failed");
    }

    _listenr = listner;
    _accept_event = loop.create_fd_event(_listenr, (void_callback_t)&GDBTarget::accept, this);
    _sum[2] = 0;
}


GDBTarget::~GDBTarget()
{
    disconnect();
    _accept_event->destroy();
    close(_listenr);
}


void GDBTarget::cpu_interrupt()
{
    Lock lock(_detach_mutex);

    if (_state != ATTACHED) {
        return;
    }

    ASSERT(vcpu);

    vcpu->trigger_debug_trap();
}


void GDBTarget::attach(bool ok)
{
    ASSERT(_loop.is_self_thread_equal());

    if (_state != ATTACHING) {
        return;
    }

    if (!ok) {
        E_MESSAGE("stop failed");
        disconnect();
        return;
    }

    _in_state = START;
    _output.clear();
    _data.clear();

    CPU* cpu = _vm.get_cpu(0);

    if (!cpu) {
        THROW("no cpu 0");
    }

    cpu->enter_debug_mode((void_callback_t)&GDBTarget::debug_condition, this);

    _state = ATTACHED;

    _io_event = _loop.create_fd_event(_connection, (void_callback_t)&GDBTarget::handle_io, this);
    I_MESSAGE("debugger attached");
}


void GDBTarget::interrupt(bool ok)
{
    ASSERT(_loop.is_self_thread_equal());

    if (_state != ATTACHED) {
        return;
    }

    if (!ok) {
        E_MESSAGE("stop failed");
        return;
    }

    put_packet("T02"); //SIGINT
    transmit();
}


void GDBTarget::trap(bool ok)
{
    ASSERT(_loop.is_self_thread_equal());

    if (_state != ATTACHED) {
        return;
    }

    if (!ok) {
        E_MESSAGE("stop failed");
        return;
    }

    put_packet("T05"); //SIGTRAP
    transmit();
}


void GDBTarget::debug_condition()
{
    _vm.vm_debug((NoxVM::compleation_routin_t)&GDBTarget::trap, this);
}


void GDBTarget::detach(bool ok)
{
    ASSERT(_loop.is_self_thread_equal());

    if (_state != DETACHING) {
        return;
    }

   if (!ok) {
        E_MESSAGE("stop failed");
        return;
    }

    Lock lock(_detach_mutex);

    CPU* cpu = _vm.get_cpu(0);

    if (!cpu) {
        THROW("no cpu 0");
    }

    cpu->exit_debug_mode();
    _vm.vm_debug_cont(NULL, NULL);
    _state = DETACHED;
    put_packet("OK");
    transmit();
    I_MESSAGE("debugger detached");
}


void GDBTarget::terminate_cb(bool ok)
{
    if (!ok) {
        E_MESSAGE("stop failed");
        return;
    }

    Lock lock(_detach_mutex);

    CPU* cpu = _vm.get_cpu(0);

    if (!cpu) {
        THROW("no cpu 0");
    }

    cpu->exit_debug_mode();
    _vm.vm_debug_cont(NULL, NULL);
    _state = DETACHED;
    disconnect();
}


void GDBTarget::accept()
{
    int new_connection = ::accept(_listenr, NULL, NULL);
    int flags;

    if (new_connection == -1) {
        return;
    }

    if (_state != DISCONNECTED) {
        W_MESSAGE("active gdb session present. rejecting");
        close(new_connection);
        return;
    }


    if ((flags = fcntl(new_connection, F_GETFL)) == -1 ||
        fcntl(new_connection, F_SETFL, flags | O_NONBLOCK) == -1) {
        W_MESSAGE("accept failed, %s", strerror(errno));
        close(new_connection);
        return;
    }

    _state = ATTACHING;
    _connection = new_connection;
    _vm.vm_debug((NoxVM::compleation_routin_t)&GDBTarget::attach, this);
}


void GDBTarget::handle_io()
{
    recive();
    transmit();
}


void GDBTarget::disconnect()
{
    if (_state == DISCONNECTED) {
        return;
    }

    close(_connection);
    _connection = -1;

    if (_io_event) {
        _io_event->destroy();
        _io_event = NULL;
    }

    _state = DISCONNECTED;
}


void GDBTarget::ack_recived()
{
}


void GDBTarget::nak_recived()
{
    D_MESSAGE("impement me");
    THROW("nak is not implemented yet");
}


void GDBTarget::test_sum()
{
    char *endptr;
    long sum = strtol(_sum, &endptr, 16);

    if (endptr != _sum + 2) {
        THROW("parse failed");
    }

    uint8_t my_sum = 0;

    for (int i = 0; i < _data.length(); i++) {
        my_sum += _data[i];
    }

    if (sum != my_sum) {
        THROW("bad sum");
    }
}


void GDBTarget::ack()
{
    _output.push_back('+');
}


void GDBTarget::put_packet(const char* data)
{
    uint8_t sum = 0;

    for (int i = 0; i < strlen(data); i++) {
        sum += data[i];
    }

    std::string packet;
    sprintf(packet, "$%s#%.2x", data, sum);
    _output += packet;
}


static void put_byte(std::string& str, uint8_t val)
{
    static char hex_conv[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'a', 'b', 'c', 'd', 'e', 'f'
    };

    str.push_back(hex_conv[val >> 4]);
    str.push_back(hex_conv[val & 0x0f]);
}


static void put_word(std::string& str, uint16_t val)
{
    put_byte(str, val);
    put_byte(str, val >> 8);
}


static void put_dword(std::string& str, uint32_t val)
{
    put_word(str, val);
    put_word(str, val >> 16);
}


void GDBTarget::handle_v()
{
    AutoArray<char> v(copy_cstr(_data.c_str() + 1));
    char* str = v.get();

    if (!strncmp(str, "Cont", strlen("Cont"))) {
        str += strlen("Cont");

        if (*str == '?') {
            put_packet("vCont;c;C;s;S");
            return;
        }

        if (*str++ != ';') {
            THROW("parse failed %s", _data.c_str());
        }

        for (;;) {
            char* next = strchr(str, ';');

            if (next) {
                *next++ = 0;
            }

            uint __attribute__ ((unused)) signal = 0;
            uint __attribute__ ((unused)) tid;
            bool step;

            switch (*str++) {
            case 'C': {
                char* end_ptr;
                signal = strtol(str, &end_ptr, 16);
                if (end_ptr != str + 2) {
                    THROW("parse failed %s", _data.c_str());
                }
                str += 2;
            }
            case 'c':
                step = false;
                break;
            case 'S': {
                char* end_ptr;
                signal = strtol(str, &end_ptr, 16);
                if (end_ptr != str + 2) {
                    THROW("parse failed %s", _data.c_str());
                }
                str += 2;
            }
            case 's':
                step = true;
                break;
            default:
                THROW("parse failed %s", _data.c_str());
            }

            if (*str == ':') {
                str += 1;
                char* end_ptr;
                tid = strtol(str, &end_ptr, 16);
                if (end_ptr == str) {
                    THROW("parse failed %s", _data.c_str());
                }
                str = end_ptr;
            } else {
                tid = 0;
            }

            if (*str) {
                THROW("parse failed %s", _data.c_str());
            }

            CPU* cpu = _vm.get_cpu(0);

            if (!cpu) {
                THROW("no cpu 0");
            }

            if (step) {
                cpu->set_single_step();
            } else {
                cpu->cancle_single_step();
            }

            cpu->debug_untrap();

            _vm.vm_debug_cont(NULL, NULL);

            if (!next) {
                return;
            }

            str = next;
        }
    }

    D_MESSAGE("unhandles: %s", _data.c_str());
    put_packet("");
    return;
}


uint hex_to_d(char val)
{
    if (val >= '0' && val <= '9') {
        return val - '0';
    }

    if (val >= 'a' && val <= 'f') {
        return val - ('a' - 10);
    }

    if (val >= 'A' && val <= 'F') {
        return val - ('A' - 10);
    }

    THROW("invalid");
}


void hex_to_bin(uint8_t* dest, const char* src, uint len)
{
    for (uint i = 0; i < len; i++) {
        dest[i] = (hex_to_d(src[i * 2]) << 4) | hex_to_d(src[i * 2 + 1]);
    }
}


void GDBTarget::handle_write_mem()
{
    uint64_t adderss;
    uint64_t pysical;
    uint size;
    uint8_t buf[GUEST_PAGE_SIZE];

    CPU* cpu = _vm.get_cpu(0);

    if (!cpu) {
        THROW("no cpu 0");
    }

    char dumy;
    int n = sscanf(_data.c_str(), "M%lx,%x:%c", &adderss, &size, &dumy);
    if (n != 3 || size * 2 > GUEST_PAGE_SIZE) {
        THROW("parse failed %s", _data.c_str());
    }

    if (size * 2 > GUEST_PAGE_SIZE) {
        THROW("too big %s", _data.c_str());
    }

    if (!cpu->translate(adderss, pysical)) {
        THROW("translate failed %s", _data.c_str());
    }

    const char* data_start = strchr(_data.c_str(), ':');

    if (!data_start) {
        THROW("':' is expected %s", _data.c_str());
    }

    if (strlen(++data_start) != size * 2 ) {
        THROW("size mismatche %s", _data.c_str());
    }

    hex_to_bin(buf, data_start, size);
    memory_bus->write(buf, size, pysical);

    put_packet("OK");
}


void GDBTarget::handle_regs()
{
    CPU* cpu = _vm.get_cpu(0);

    if (!cpu) {
        THROW("no cpu 0");
    }

    CPURegs regs;
    cpu->get_regs(regs);

    std::string regs_str;

    put_dword(regs_str, regs.r[CPU_REG_A_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_C_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_D_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_B_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_SP_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_BP_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_SI_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_DI_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_IP_INDEX]);
    put_dword(regs_str, regs.r[CPU_REG_FLAGS_INDEX]);
    put_dword(regs_str, regs.seg[CPU_SEG_CS]);
    put_dword(regs_str, regs.seg[CPU_SEG_SS]);
    put_dword(regs_str, regs.seg[CPU_SEG_DS]);
    put_dword(regs_str, regs.seg[CPU_SEG_ES]);
    put_dword(regs_str, regs.seg[CPU_SEG_FS]);
    put_dword(regs_str, regs.seg[CPU_SEG_GS]);

    put_packet(regs_str.c_str());
}


void GDBTarget::handle_read_mem()
{
    uint64_t adderss;
    uint64_t pysical;
    unsigned size;
    uint8_t buf[GUEST_PAGE_SIZE];

    CPU* cpu = _vm.get_cpu(0);

    if (!cpu) {
        THROW("no cpu 0");
    }

    if (sscanf(_data.c_str(), "m%lx,%x", &adderss, &size) != 2 || size > GUEST_PAGE_SIZE) {
        THROW("parse failed %s", _data.c_str());
    }

    if (!cpu->translate(adderss, pysical)) {
        THROW("translate failed", _data.c_str());
    }

    std::string mem;

    memory_bus->read(pysical, size, buf);

    for (int i = 0; i < size; i++) {
        put_byte(mem, buf[i]);
    }

    put_packet(mem.c_str());
}


void GDBTarget::handle_q()
{
    if (strncmp(_data.c_str(), "qSupported", strlen("qSupported")) == 0) {
        put_packet("");
    } else  if (strncmp(_data.c_str(), "qAttached", strlen("qAttached")) == 0) {
        put_packet("1");
    } else if (strcmp(_data.c_str(), "qC") == 0) {
        put_packet("1");
    } else {
        put_packet("");
    }
}


void GDBTarget::handle_H()
{
    if (strcmp(_data.c_str(), "Hg0") == 0) {
        put_packet("OK");
    } else if (strstr(_data.c_str(), "Hc") == _data.c_str()) {
        put_packet("OK");
    } else {
        put_packet("");
    }
}


void GDBTarget::process_packet()
{
    if (_state != ATTACHED) {
        D_MESSAGE("command \"%s\" while not attached", _data.c_str());
        return;
    }

    ack();

    switch (_data.c_str()[0]) {
    case 'm':
        handle_read_mem();
        break;
    case 'M':
        handle_write_mem();
        break;
    case 'g':
        handle_regs();
        break;
    case 'D':
        if (strcmp(_data.c_str(), "D")) {
            D_MESSAGE("unexpected pid in D packet? \"%s\"", _data.c_str());
        }
        _state = DETACHING;
        _vm.vm_debug((NoxVM::compleation_routin_t)&GDBTarget::detach, this);
        break;
    case 'v':
        handle_v();
        break;
    case 'q':
        handle_q();
        break;
    case '?':
        put_packet("S11");
        break;
    case 'H':
        handle_H();
        break;
    default:
        D_MESSAGE("unhandles: %s", _data.c_str());
        put_packet("");
    }
}


void GDBTarget::process(uint8_t* data, int len)
{
    for ( ; len--; data++) {
        switch (_in_state) {
        case START:
            if (*data == '+') {
                ack_recived();
            } else if (*data == '-') {
                nak_recived();
            } else if (*data == '$') {
                _data.clear();
                _in_state = DATA;
            } else if (*data == INTERRUPT) {
                _vm.vm_debug((NoxVM::compleation_routin_t)&GDBTarget::interrupt, this);
            } else {
                THROW("unexpected: 0x%x", *data);
            }
            break;
        case DATA:
            if (*data == '#') {
                _data.push_back(0);
                _in_state = SUM0;
                break;
            }

            if (_data.length() == 1024) {
                THROW("packet is too big %u", _data.size() + 1);
            }

            _data.push_back(*data);
            break;
        case SUM0:
            _sum[0] = *data;
            _in_state = SUM1;
            break;
        case SUM1:
            _sum[1] = *data;
            _in_state = START;
            test_sum();
            process_packet();
            break;
        }
    }
}


void GDBTarget::terminate()
{
    _state = TERMINATING;
    _io_event->destroy();
    _io_event = NULL;
    _vm.vm_debug((NoxVM::compleation_routin_t)&GDBTarget::terminate_cb, this);
}


void GDBTarget::recive()
{
    for (;;) {
        uint8_t buf[1024];
        int n = read(_connection, buf, sizeof(buf));

        if (n <= 0) {
            if (n == -1) {
                if (errno == EAGAIN) {
                    return;
                }

                if (errno == EINTR) {
                    continue;
                }
            }
            break;
        }

        try {
            process(buf, n);
        } catch (Exception& e) {
            D_MESSAGE("%s", e.what());
            break;
        }
    }

    terminate();
}


void GDBTarget::transmit()
{
    if (_connection == -1) {
        return;
    }

    for (; _output.length();) {
        int n = write(_connection, _output.c_str(), _output.length());

        if (n <= 0) {
            if (errno == EAGAIN) {
                return;
            }

            if (errno == EINTR) {
                continue;
            }

            terminate();
            break;
        }

        _output.erase(0, n);
    }
}

