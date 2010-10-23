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

#ifndef _H_CPU
#define _H_CPU

#include "vm_part.h"
#include "threads.h"

class NoxVM;
class IOBus;

class CPU: public VMPart {
public:
    CPU(NoxVM& vm, uint id);
    ~CPU();

    NoxVM& get_vm() { return *(NoxVM*)get_container();}

    virtual void load(InStream &stream) {}
    virtual void power() {}
    virtual void reset();
    virtual void save(OutStream &stream) {}
    virtual void start();
    virtual void stop() {}

    enum Command {
        WAIT,
        RUN,
        TERMINATE,
    };

    enum State {
        INITIALIZING,
        WAITING,
        RUNNING,
        TERMINATING,
        TERMINATED,
    };

private:
    void run_loop();
    void run();
    void reset_regs();
    void reset_sys_regs();
    void handle_io();
    void set_state(State state);
    void setup_cpuid();
    void create();
    void output_trigger();
    void halt();
    void handle_mmio();

    static void* thread_main(void *);

private:
    uint _id;
    State _state;
    Command _command;
    Mutex _command_mutex;
    Condition _command_condition;
    Mutex _state_mutex;
    Condition _state_condition;
    AutoFD _vcpu_fd;
    struct kvm_run* _kvm_run;
    int _vcpu_mmap_size;
    bool _execution_break;
    IOBus& _io_bus;
    Thread _thread;
    bool _executing;
    bool _test_interrupts;
    uint _interrupt_mark_set;
    uint _interrupt_mark_get;
    bool _halt;
    Mutex _halt_mutex;
    Condition _halt_condition;
    uint32_t _version_information;
};

#endif

