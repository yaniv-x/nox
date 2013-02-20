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
        support G command
        support z0 and Z0
        discconect on mode change?
        translet ip in case of 16bit
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


const char* amd64_arc = "<?xml version=\"1.0\"?>\n"
                        "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
                        "<target version=\"1.0\">\n"
                            "\t<architecture>i386:x86-64</architecture>\n"
                        "</target>\n";


const char* i386_arc =  "<?xml version=\"1.0\"?>\n"
                        "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
                        "<target version=\"1.0\">\n"
                            "\t<architecture>i386</architecture>\n"
                        "</target>\n";


static void push_hex_byte(std::string& str, uint8_t val)
{
    static char hex_conv[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'a', 'b', 'c', 'd', 'e', 'f'
    };

    str.push_back(hex_conv[val >> 4]);
    str.push_back(hex_conv[val & 0x0f]);
}


static void push_hex_word(std::string& str, uint16_t val)
{
    push_hex_byte(str, val);
    push_hex_byte(str, val >> 8);
}


static void push_hex_dword(std::string& str, uint32_t val)
{
    push_hex_word(str, val);
    push_hex_word(str, val >> 16);
}


static void push_hex_qword(std::string& str, uint64_t val)
{
    push_hex_dword(str, val);
    push_hex_dword(str, val >> 32);
}


static uint hex_to_d(char val)
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


static void hex_to_bin(uint8_t* dest, const char* src, uint len)
{
    for (uint i = 0; i < len; i++) {
        dest[i] = (hex_to_d(src[i * 2]) << 4) | hex_to_d(src[i * 2 + 1]);
    }
}


static void to_binary_data(std::string& str, const char *src, int n)
{
    const char *end =  src + n;

    #define ESCAPE '}'

    for (; src < end; ++src) {
        switch (*src) {
        case ESCAPE:
        case '#':
        case '$':
        case '*':
            str.push_back(ESCAPE);
            str.push_back(*src ^ 0x20);
        default:
            str.push_back(*src);
        }
    }
}


GDBTarget::GDBTarget(NoxVM& vm, RunLoop& loop, uint16_t port)
    : _vm (vm)
    , _loop (loop)
    , _port (port)
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
    address.sin_port = htons(_port);
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


bool GDBTarget::is_valid_thread_id(uint id)
{
    return id > 0 && id <= _vm.get_cpu_count();
}


CPU& GDBTarget::get_cpu(uint cpu_id)
{
    CPU* cpu = _vm.get_cpu(cpu_id);

    if (!cpu) {
        THROW("no cpu %u", cpu_id);
    }

    return *cpu;
}


CPU& GDBTarget::target_cpu()
{
    return get_cpu(_target - 1);
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
    _target = 1;
    _current_thread = 1;
    _break_initator = ~0;
    _halt_reason = SIGSTOP;
    _long_mode = get_cpu(0).is_long_mode();

    for (uint i = 0; i < _vm.get_cpu_count(); i++) {
        get_cpu(i).enter_debug_mode((void_callback_t)&GDBTarget::debug_condition, this);
    }

    _state = ATTACHED;

    _io_event = _loop.create_fd_event(_connection, (void_callback_t)&GDBTarget::handle_io, this);
    I_MESSAGE("debugger attached");
}


void GDBTarget::clear_debugger_traps()
{
    for (uint i = 0; i < _vm.get_cpu_count(); i++) {
        get_cpu(i).remove_debugger_trap();
    }
}


void GDBTarget::set_debugger_traps()
{
    for (uint i = 0; i < _vm.get_cpu_count(); i++) {
        get_cpu(i).set_debugger_trap();
    }
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

    set_debugger_traps();
    _halt_reason = SIGINT;
    _current_thread = 1;
    push_packet("T02");
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

    set_debugger_traps();
    _halt_reason = SIGTRAP;
    std::string  notifiction("T05thread:");
    push_hex_byte(notifiction, _break_initator);
    _current_thread = _break_initator;
    _break_initator = ~0;
    notifiction += ";";
    push_packet(notifiction.c_str());
    transmit();
}


void GDBTarget::debug_condition()
{
    if (_break_initator != ~0) {
        W_MESSAGE("initator exist");
    }

    _break_initator = vcpu->get_id() + 1;
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

    for (uint i = 0; i < _vm.get_cpu_count(); i++) {
        get_cpu(i).exit_debug_mode();
    }

    _vm.vm_debug_cont(NULL, NULL);
    _state = DETACHED;
    push_packet("OK");
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

    for (uint i = 0; i < _vm.get_cpu_count(); i++) {
        get_cpu(i).exit_debug_mode();
    }

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


void GDBTarget::push_packet(const char* data)
{
    uint8_t sum = 0;

    for (int i = 0; i < strlen(data); i++) {
        sum += data[i];
    }

    std::string packet;
    sprintf(packet, "$%s#%.2x", data, sum);
    _output += packet;
}


enum {
   NOP,
   STEP,
   CONT,
};


void GDBTarget::resume(std::vector<uint>& cpu_actions)
{
    for (uint i = 0; i < cpu_actions.size(); i++) {
        CPU& cpu = get_cpu(i);

        switch (cpu_actions[i]) {
        case STEP:
            cpu.remove_debugger_trap();
            cpu.set_single_step();
            break;
        case CONT:
            cpu.remove_debugger_trap();
            cpu.cancle_single_step();
            break;
        case NOP:
            D_MESSAGE("no action for cpu %u", i);
        }
    }

    _halt_reason = 0;
    _vm.vm_debug_cont(NULL, NULL);
}


void GDBTarget::handle_v()
{
    AutoArray<char> v(copy_cstr(_data.c_str() + 1));
    char* str = v.get();

    if (!strncmp(str, "Cont", strlen("Cont"))) {
        str += strlen("Cont");

        if (*str == '?') {
            push_packet("vCont;c;C;s;S"); // todo: try to remove C and S
            return;
        }

        if (!_halt_reason) {
            D_MESSAGE("ignoring %s while not in halt state", _data.c_str());
            return;
        }

        if (*str++ != ';') {
            THROW("parse failed %s", _data.c_str());
        }

        std::vector<uint> cpu_actions(_vm.get_cpu_count(), NOP);

        for (;;) {
            char* next = strchr(str, ';');

            if (next) {
                *next++ = 0;
            }

            uint __attribute__ ((unused)) signal = 0;
            bool step;
            bool all = true;

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
                uint tid = strtol(str, &end_ptr, 16);

                if (end_ptr == str) {
                    THROW("parse failed %s", _data.c_str());
                }

                str = end_ptr;

                if (tid != -1) {

                    all = false;

                    if (!is_valid_thread_id(tid)) {
                        THROW("invalid thread id %s", _data.c_str());
                    }

                    cpu_actions[tid - 1] = (step) ? STEP : CONT;
                }
            }

            if (all) {
                for (uint i = 0; i < cpu_actions.size(); i++) {
                    if (cpu_actions[i] == NOP) {
                        cpu_actions[i] = (step) ? STEP : CONT;
                    }
                }
            }

            if (*str) {
                THROW("parse failed %s", _data.c_str());
            }

            if (!next) {
                resume(cpu_actions);
                return;
            }

            str = next;
        }
    }

    D_MESSAGE("unhandles: %s", _data.c_str());
    push_packet("");
    return;
}


void GDBTarget::handle_write_mem()
{
    uint8_t buf[GUEST_PAGE_SIZE];
    uint64_t address;
    uint size;
    char dumy;

    CPU& cpu = target_cpu();

    if (sscanf(_data.c_str(), "M%lx,%x:%c", &address, &size, &dumy) != 3) {
        THROW("parse failed %s", _data.c_str());
    }

    if (size > GUEST_PAGE_SIZE) {
        THROW("too big %s", _data.c_str());
    }

    const char* data_start = strchr(_data.c_str(), ':');

    if (!data_start) {
        THROW("':' is expected %s", _data.c_str());
    }

    if (strlen(++data_start) != size * 2 ) {
        THROW("size mismatche %s", _data.c_str());
    }

    while (size) {
        uint now = MIN(size, GUEST_PAGE_SIZE - address % GUEST_PAGE_SIZE);
        uint64_t pysical;

        if (!cpu.translate(address, pysical)) {
            push_packet("E01");
            return;
        }

        hex_to_bin(buf, data_start, now);
        memory_bus->write(buf, now, pysical);

        address += now;
        size -= now;
        data_start += now * 2;
    }

    push_packet("OK");
}


void GDBTarget::handle_regs()
{
    CPU& cpu = target_cpu();

    CPURegs regs;
    cpu.get_regs(regs);

    std::string regs_str;

    if (_long_mode) {
        push_hex_qword(regs_str, regs.r[CPU_REG_A_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_B_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_C_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_D_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_SI_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_DI_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_BP_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_SP_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_8_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_9_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_10_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_11_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_12_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_13_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_14_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_15_INDEX]);
        push_hex_qword(regs_str, regs.r[CPU_REG_IP_INDEX]);
    } else {
        push_hex_dword(regs_str, regs.r[CPU_REG_A_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_C_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_D_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_B_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_SP_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_BP_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_SI_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_DI_INDEX]);
        push_hex_dword(regs_str, regs.r[CPU_REG_IP_INDEX]);
    }

    push_hex_dword(regs_str, regs.r[CPU_REG_FLAGS_INDEX]);
    push_hex_dword(regs_str, regs.seg[CPU_SEG_CS]);
    push_hex_dword(regs_str, regs.seg[CPU_SEG_SS]);
    push_hex_dword(regs_str, regs.seg[CPU_SEG_DS]);
    push_hex_dword(regs_str, regs.seg[CPU_SEG_ES]);
    push_hex_dword(regs_str, regs.seg[CPU_SEG_FS]);
    push_hex_dword(regs_str, regs.seg[CPU_SEG_GS]);

    push_packet(regs_str.c_str());
}


void GDBTarget::handle_read_mem()
{
    uint8_t buf[GUEST_PAGE_SIZE];
    uint64_t address;
    uint size;

    CPU& cpu = target_cpu();

    if (sscanf(_data.c_str(), "m%lx,%x", &address, &size) != 2 || size > GUEST_PAGE_SIZE) {
        THROW("parse failed %s", _data.c_str());
    }

    std::string mem;

    while (size) {
        uint now = MIN(size, GUEST_PAGE_SIZE - address % GUEST_PAGE_SIZE);
        uint64_t pysical;

        if (!cpu.translate(address, pysical)) {
            push_packet("E01");
            return;
        }

        memory_bus->read(pysical, now, buf);

        for (int i = 0; i < now; i++) {
            push_hex_byte(mem, buf[i]);
        }

        address += now;
        size -= now;
    }

    push_packet(mem.c_str());
}


void GDBTarget::handle_thread_ext_info()
{
    ulong id;

    bool num_ok = str_to_ulong(_data.c_str() + strlen("qThreadExtraInfo,"), id, 16);

    if (!num_ok || !is_valid_thread_id(id)) {
        push_packet("E01");
        return;
    }

    std::string description;
    sprintf(description, "cpu-%u %u-bits", id - 1, get_cpu(id - 1).get_cs_bits());
    std::string hex;

    for (int i = 0; i < description.size(); i++) {
        push_hex_byte(hex, description[i]);
    }
    push_packet(hex.c_str());
}


void GDBTarget::handle_thread_info()
{
    uint num_cpus = _vm.get_cpu_count();
    std::string reply("m");
    push_hex_byte(reply, 1);

    for (uint i = 2; i <= num_cpus; i++) {
        reply += ',';
        push_hex_byte(reply, i);
    }

    push_packet(reply.c_str());
}


void GDBTarget::handle_features_read(const char* features)
{
    uint offset;
    uint size;
    uint name_len = strlen("target.xml:");

    if (strncmp(features, "target.xml:", name_len)) {
        D_MESSAGE("not supported %s", features);
        push_packet("E01");
        return;
    }

    if (sscanf(features + name_len, "%x,%x", &offset, &size) != 2) {
        D_MESSAGE("invalid format %s", features);
        push_packet("E01");
    }

    const char* target = (_long_mode) ? amd64_arc : i386_arc;
    uint n = strlen(target);

    if (offset == n) {
        push_packet("l");
        return;
    }

    if (offset > n) {
        push_packet("E01");
        return;
    }

    n -= offset;

    std::string str;

    if (size < n) {
        str = "m";
        n = size;
    } else {
        str = "l";
    }

    const char* pos = target + offset;

    to_binary_data(str, pos, n);
    push_packet(str.c_str());
}


void GDBTarget::handle_q()
{
    if (strncmp(_data.c_str(), "qSupported", strlen("qSupported")) == 0) {
        push_packet("qXfer:features:read+");
    } else  if (strncmp(_data.c_str(), "qAttached", strlen("qAttached")) == 0) {
        push_packet("1");
    } else if (strcmp(_data.c_str(), "qC") == 0) {
        std::string reply("QC");
        push_hex_byte(reply, _current_thread);
        push_packet(reply.c_str());
    } else if (strcmp(_data.c_str(), "qfThreadInfo") == 0) {
        handle_thread_info();
    } else if (strcmp(_data.c_str(), "qsThreadInfo") == 0) {
        push_packet("l");
    } else if (strncmp(_data.c_str(), "qThreadExtraInfo,", strlen("qThreadExtraInfo,")) == 0) {
        handle_thread_ext_info();
    } else if (strncmp(_data.c_str(), "qXfer:features:read:",
                       strlen("qXfer:features:read:")) == 0) {
        handle_features_read(_data.c_str() + strlen("qXfer:features:read:"));
    } else {
        push_packet("");
    }
}


void GDBTarget::handle_H(const char* cmd)
{
    long id;

    if (strlen(cmd) < 2 || !str_to_long(cmd + 1, id, 16)) {
        push_packet("E01");
        return;
    }

    if (id == 0 || id == -1) { // `-1' to indicate all threads, or `0' to pick any thread
        id = 1;
    }

    if (!is_valid_thread_id(id)) {
        push_packet("E01");
        return;
    }

    if (cmd[0] == 'g') {
        _target = id;
    } else if (cmd[0] != 'c') {
        push_packet("E01");
        return;
    }

    push_packet("OK");
}


void GDBTarget::handle_T()
{
    ulong id;

    if (!str_to_ulong(_data.c_str() + 1, id, 16) || !is_valid_thread_id(id)) {
        push_packet("E01");
        return;
    }

    push_packet("OK");
}


void GDBTarget::handle_halt_reason()
{
    std::string reply;

    sprintf(reply, "S%x", _halt_reason);
    push_packet(reply.c_str());
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
        handle_halt_reason();
        break;
    case 'H':
        handle_H(_data.c_str() + 1);
        break;
    case 'T':
        handle_T();
        break;
    default:
        D_MESSAGE("unhandles: %s", _data.c_str());
        push_packet("");
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
        } catch (...) {
            D_MESSAGE("unknown exception");
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

