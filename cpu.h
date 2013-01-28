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

#define MAX_CPUS 255

class NoxVM;
class IOBus;
class Timer;


enum {
    CPU_REG_A_INDEX,
    CPU_REG_B_INDEX,
    CPU_REG_C_INDEX,
    CPU_REG_D_INDEX,
    CPU_REG_SI_INDEX,
    CPU_REG_DI_INDEX,
    CPU_REG_SP_INDEX,
    CPU_REG_BP_INDEX,
    CPU_REG_8_INDEX,
    CPU_REG_9_INDEX,
    CPU_REG_10_INDEX,
    CPU_REG_11_INDEX,
    CPU_REG_12_INDEX,
    CPU_REG_13_INDEX,
    CPU_REG_14_INDEX,
    CPU_REG_15_INDEX,
    CPU_REG_IP_INDEX,
    CPU_REG_FLAGS_INDEX,

    CPU_REGS_COUNT
};


enum {
    CPU_SEG_CS,
    CPU_SEG_DS,
    CPU_SEG_ES,
    CPU_SEG_FS,
    CPU_SEG_GS,
    CPU_SEG_SS,

    CPU_SEG_COUNT
};


struct CPURegs {
    uint64_t r[CPU_REGS_COUNT];
    uint16_t seg[CPU_SEG_COUNT];
};


class CPU: public VMPart {
public:
    CPU(NoxVM& vm);
    virtual ~CPU();

    virtual void load(InStream &stream) {}
    virtual void reset();
    virtual void save(OutStream &stream) {}
    virtual bool start();
    virtual bool stop();

    uint get_id() { return _id;}
    void get_regs(CPURegs& regs);
    bool translate(uint64_t address, uint64_t& pysical);
    void backtrace_64();
    void set_single_step();
    void cancle_single_step();
    void debug_untrap();
    void enter_debug_mode(void_callback_t cb, void* opaque);
    void exit_debug_mode();
    void trigger_debug_trap();
    bool pending_sleep_request();
    void clear_sleep_request();

    void __apic_deliver_interrupt_logical(uint vector, uint dest, bool level);
    void __apic_deliver_interrupt_all(uint vector, bool level);
    void __apic_deliver_interrupt_exclude(uint vector, bool level);

    static void apic_deliver_interrupt_physical(uint vector, uint dest, bool level);
    static void apic_deliver_interrupt_logical(uint vector, uint dest, bool level);
    static void apic_deliver_interrupt_lowest(uint vector, uint dest, bool level);
    static void apic_deliver_nmi_physical(uint dest);
    static void apic_deliver_nmi_logical(uint dest);

    enum Command {
        WAIT,
        RUN,
        TERMINATE,
    };

    enum CPUState {
        INVALID,
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
    void reset_msrs();
    void handle_io();
    void set_cpu_state(CPUState state);
    void setup_cpuid();
    void save_init_msrs();
    void create();
    void force_exec_loop();
    void output_trigger();
    void trap_wait();
    bool halt_trap();
    void set_halt_trap();
    bool debug_trap();
    void set_debug_trap();
    void apic_write(uint32_t offset, uint32_t n, uint8_t* src);
    void apic_read(uint32_t offset, uint32_t n, uint8_t* dest);
    void handle_mmio();
    void nmi();
    void __reset();

    int apic_eoi();
    void apic_update_error();
    void apic_set_spurious(uint32_t val);

    void populate_dest_mask(uint dest, bool logical);
    void apic_command_init(uint32_t cmd_low);
    void apic_command_fixed(uint32_t cmd_low);
    void apic_command(uint32_t cmd_low);
    void apic_set_timer(uint32_t val);
    void apic_update_timer();
    void apic_rearm_timer();
    void apic_reset();
    bool apic_in_logical_dest(uint dest);
    bool is_apic_enabled();
    bool is_apic_soft_enabled();
    uint get_interrupt();
    bool interrupt_test();
    uint get_direct_interrupt();
    bool is_dirct_interrupt_pending();
    void set_apic_address(address_t address);
    void apic_put_irr(int irr, bool level);
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
    CPUState _state_change_target;
    Command _command;
    Mutex _command_mutex;
    Condition _command_condition;
    Mutex _cpu_state_mutex;
    Condition _cpu_state_condition;
    AutoFD _vcpu_fd;
    struct kvm_run* _kvm_run;
    int _vcpu_mmap_size;
    AutoArray<uint8_t> _init_msrs;
    uint _execution_break;
    IOBus& _io_bus;
    Thread _thread;
    bool _executing;
    bool _test_interrupts;
    bool _need_timer_update;
    uint _interrupt_mark_set;
    uint _interrupt_mark_get;
    Mutex _trap_mutex;
    Condition _trap_condition;
    bool (CPU::*_trap)();
    uint32_t _version_information;
    Mutex _apic_mutex;
    address_t _apic_address;
    address_t _apic_start;
    address_t _apic_end;
    uint32_t _apic_regs[GUEST_PAGE_SIZE / 16];
    uint64_t _apic_timer_start_tsc;
    uint64_t _apic_timer_div;
    Timer* _apic_timer;
    int _current_interrupt;
    //Mutex _apic_timer_mutex;
    void_callback_t _debug_cb;
    void* _debug_opaque;
    bool _debug_trap;
    uint32_t _cpu_dest_mask[ALIGN(MAX_CPUS, 32) / 32];
};


extern __thread CPU* vcpu;

#endif

