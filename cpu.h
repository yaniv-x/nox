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
class Timer;

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

    void backtrace_64();

    enum Command {
        WAIT,
        RUN,
        TERMINATE,
    };

    enum CPUState {
        INITIALIZING,
        WAITING,
        RUNNING,
        TERMINATING,
        TERMINATED,
        ERROR,
    };

    enum {
        INVALID_INTERRUPT,
    };

private:
    void run_loop();
    void run();
    void reset_regs();
    void reset_sys_regs();
    void reset_fpu();
    void handle_io();
    void set_cpu_state(CPUState state);
    void setup_cpuid();
    void create();
    void output_trigger();
    void halt();
    void apic_write(uint32_t offset, uint32_t n, uint8_t* src);
    void apic_read(uint32_t offset, uint32_t n, uint8_t* dest);
    void handle_mmio();

    void apic_eoi();
    void apic_update_error();
    void apic_set_spurious(uint32_t val);
    void apic_command(uint32_t val);
    void apic_set_timer(uint32_t val);
    void apic_update_timer();
    void apic_reset();
    bool is_apic_enabled();
    bool is_apic_soft_enabled();
    uint get_interrupt();
    bool interrupt_test();
    uint get_direct_interrupt();
    bool is_dirct_interrupt_pending();
    void set_apic_address(address_t address);
    void apic_put_irr(int irr);
    void sync_tpr();
    void apic_timer_cb();

    void apic_update_priority_isr(int isr);
    void apic_update_priority_irr(int irr);
    void apic_update_priority();

    void back_trace_64(address_t rip, address_t frame_pointer, int depth);

    void* thread_main();

private:
    uint _id;
    CPUState _cpu_state;
    Command _command;
    Mutex _command_mutex;
    Condition _command_condition;
    Mutex _cpu_state_mutex;
    Condition _cpu_state_condition;
    AutoFD _vcpu_fd;
    struct kvm_run* _kvm_run;
    int _vcpu_mmap_size;
    bool _execution_break;
    IOBus& _io_bus;
    Thread _thread;
    bool _executing;
    bool _test_interrupts;
    bool _need_timer_update;
    uint _interrupt_mark_set;
    uint _interrupt_mark_get;
    bool _halt;
    Mutex _halt_mutex;
    Condition _halt_condition;
    uint32_t _version_information;
    address_t _apic_address;
    address_t _apic_start;
    address_t _apic_end;
    uint32_t _apic_regs[GUEST_PAGE_SIZE / 16];
    uint64_t _apic_timer_start_tsc;
    uint64_t _apic_timer_div;
    Timer* _apic_timer;
    int _current_interrupt;
    //Mutex _apic_timer_mutex;
};



extern __thread CPU* vcpu;

#endif

