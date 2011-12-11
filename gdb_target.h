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

class GDBTarget: public NonCopyable {
public:
    GDBTarget(NoxVM& vm, RunLoop& loop);
    virtual ~GDBTarget();

    void cpu_interrupt();

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
    void handle_v();
    void handle_q();
    void handle_H();
    void handle_read_mem();
    void handle_write_mem();
    void handle_regs();
    void process_packet();
    void ack();
    void put_packet(const char* data);
    void attach(bool ok);
    void trap(bool ok);
    void interrupt(bool ok);
    void detach(bool ok);
    void terminate_cb(bool ok);
    void debug_condition();

private:
    Mutex _detach_mutex;
    NoxVM& _vm;
    RunLoop& _loop;
    int _listenr;
    FDEvent* _accept_event;
    int _connection;
    FDEvent* _io_event;

    int _state;
    int _in_state;
    std::string _data;
    char _sum[3];
    std::string _output;
};


#endif

