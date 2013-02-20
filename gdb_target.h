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

#ifndef _H_GDB_TARGET
#define _H_GDB_TARGET

#include "non_copyable.h"
#include "threads.h"

class NoxVM;
class RunLoop;
class FDEvent;
class CPU;

class GDBTarget: public NonCopyable {
public:
    GDBTarget(NoxVM& vm, RunLoop& loop, uint16_t port);
    virtual ~GDBTarget();

    void cpu_interrupt();
    uint16_t get_port() { return _port;}

private:
    void accept();
    void handle_io();
    void recive();
    void transmit();
    void disconnect();
    void terminate();

    void process(uint8_t* data, int len);
    void ack_recived();
    void nak_recived();
    void test_sum();
    void resume(std::vector<uint>& actions);
    void handle_v();
    void handle_features_read(const char* features);
    void handle_thread_ext_info();
    void handle_thread_info();
    void handle_q();
    void handle_H(const char* str);
    void handle_T();
    void handle_read_mem();
    void handle_write_mem();
    void handle_regs();
    void handle_halt_reason();
    void process_packet();
    void ack();
    void push_packet(const char* data);
    void attach(bool ok);
    void set_debugger_traps();
    void clear_debugger_traps();
    void trap(bool ok);
    void interrupt(bool ok);
    void detach(bool ok);
    void terminate_cb(bool ok);
    void debug_condition();

    bool is_valid_thread_id(uint id);
    CPU& get_cpu(uint cpu_id);
    CPU& target_cpu();

private:
    Mutex _detach_mutex;
    NoxVM& _vm;
    RunLoop& _loop;
    uint16_t _port;
    int _listenr;
    FDEvent* _accept_event;
    int _connection;
    FDEvent* _io_event;

    int _state;
    int _in_state;
    std::string _data;
    char _sum[3];
    std::string _output;
    uint _current_thread;
    uint _break_initator;
    uint _halt_reason;
    uint _target;
    bool _long_mode;
};


#endif

