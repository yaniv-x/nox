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

#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>

#include "cpu.h"
#include "nox_vm.h"
#include "kvm.h"
#include "pic.h"
#include "io_bus.h"
#include "memory_bus.h"
#include "application.h"
#include "io_apic.h"


//#define APIC_DEBUG

#ifdef APIC_DEBUG
#define APIC_DEBUG(format, ...) D_MESSAGE(format, ## __VA_ARGS__)
#else
#define APIC_DEBUG(format, ...)
#endif


enum {
    APIC_ENABLE_MASK = (1 << 11),
    APIC_BOOTSTRAP_PROCESSOR_MASK = (1 << 8),
    APIC_DEFAULT_ADDRESS = 0xfee00000,
    APIC_LVT_COUNT = 6,

    APIC_OFFSET_ID = 0x020 / 16,
    APIC_OFFSET_VERSION = 0x030 / 16,
    APIC_OFFSET_SPURIOUS = 0x0f0 / 16,
    APIC_OFFSET_LVT_TIMER = 0x320 / 16,
    APIC_OFFSET_LVT_THERMAL = 0x330 / 16,
    APIC_OFFSET_LVT_PERFORMANCE = 0x340 / 16,
    APIC_OFFSET_LVT_INT_0 = 0x350 / 16,
    APIC_OFFSET_LVT_INT_1 = 0x360 / 16,
    APIC_OFFSET_LVT_ERROR = 0x370 / 16,
    APIC_OFFSET_EOI = 0x0b0 / 16,
    APIC_OFFSET_TPR = 0x080 / 16,
    APIC_OFFSET_DIV_CONF = 0x3e0 / 16,
    APIC_OFFSET_TIMER_INIT_COUNT = 0x380 / 16,
    APIC_OFFSET_TIMER_CURRENT_COUNT = 0x390 / 16,
    APIC_OFFSET_ERROR = 0x280 / 16,
    APIC_OFFSET_CMD_LOW = 0x300 / 16,
    APIC_OFFSET_CMD_HIGH = 0x310 / 16,
    APIC_OFFSET_LOGICAL_DEST = 0x0d0 / 16,
    APIC_OFFSET_DEST_FORMAT = 0x0e0 / 16,
    APIC_OFFSET_ISR = 0x100 / 16,
    APIC_OFFSET_TMR = 0x180 / 16,
    APIC_OFFSET_IRR = 0x200 / 16,
    APIC_OFFSET_PROCESSOR_PRIORITY = 0x0a0 / 16,
    APIC_OFFSET_ARBITRATION_PRIORITY = 0x090 / 16,


    APIC_ID_SHIFT = 24,

    APIC_VERSION_AMD = 0x10,
    APIC_VERSION_LVT_SHIFT = 16,


    APIC_LVT_TIMER_MODE_BIT = 17,
    APIC_LVT_MASK_BIT = 16,
    APIC_LVT_TRIGGER_MODE_BIT = 15,
    APIC_LVT_VECTOR_MASK = 0xff,
    APIC_LVT_TYPE_MASK = 0x300,


    APIC_LVT_TIMER_MASK = APIC_LVT_VECTOR_MASK | (1 << APIC_LVT_MASK_BIT) |
                          (1 << APIC_LVT_TIMER_MODE_BIT),

    APIC_LVT_LINT_MASK = APIC_LVT_VECTOR_MASK | APIC_LVT_TYPE_MASK | (1 << APIC_LVT_MASK_BIT) |
                         (1 << APIC_LVT_TRIGGER_MODE_BIT),

    APIC_LVT_PERFORMANCE_MASK = APIC_LVT_VECTOR_MASK | APIC_LVT_TYPE_MASK |
                                (1 << APIC_LVT_MASK_BIT),
    APIC_LVT_THERMAL_MASK = APIC_LVT_PERFORMANCE_MASK,
    APIC_LVT_ERROR_MASK = APIC_LVT_PERFORMANCE_MASK,

    APIC_DIV_CONF_MASK = 0x0b,

    APIC_ERROR_SEND_ACCEPT_BIT = 2,
    APIC_ERROR_RECEIVE_ACCEPT_BIT = 3,
    APIC_ERROR_SEND_ILLEGAL_VECTOR_BIT = 5,
    APIC_ERROR_RECIVE_ILLEGAL_VECTOR_BIT = 6,
    APIC_ERROR_ILLEGAL_REGISTER_ADDRESS = 7,

    APIC_LOGICAL_DEST_MASK = (0xff << 24),

    APIC_DEST_FORMAT_MASK = (0x0f << 28),

    APIC_SPURIOUS_APIC_ENABLED_MASK = (1 << 8),
    APIC_SPURIOUS_MASK = (1 << 10) - 1,

    APIC_CMD_LOW_MBZ_MASK = ~((1 << 20) - 1) | (1 << 13),
    APIC_CMD_LOW_RO_MASK = (3 << 16) | (1 << 12),
    APIC_CMD_LOGICAL_MASK = 1 << 11,
    APIC_CMD_MODE_MASK = 1 << 15,
    APIC_CMD_LEVEL_MASK = 1 << 14,
    APIC_CMD_VECTOR_MASK = 0xff,

    APIC_CMD_HIGH_DEST_SHIFT = 24,
    APIC_CMD_HIGH_MASK = (0xff << APIC_CMD_HIGH_DEST_SHIFT),
};

enum {
    APIC_CMD_MT_SHIFT = 8,
    APIC_CMD_MT_MASK = 7 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_FIXED = 0 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_LOWEST = 1 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_SMI = 2 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_READ = 3 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_NMI = 4 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_INIT = 5 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_STARTUP = 6 << APIC_CMD_MT_SHIFT,
    APIC_CMD_MT_EXTERNAL = 7 << APIC_CMD_MT_SHIFT,
};

enum {
    APIC_CMD_SHORTHAND_SHIFT = 18,
    APIC_CMD_SHORTHAND_MASK = 0x03 << APIC_CMD_SHORTHAND_SHIFT,

    APIC_CMD_SHORTHAND_DEST = 0 << APIC_CMD_SHORTHAND_SHIFT,
    APIC_CMD_SHORTHAND_SELF = 1 << APIC_CMD_SHORTHAND_SHIFT,
    APIC_CMD_SHORTHAND_ALL = 2 << APIC_CMD_SHORTHAND_SHIFT,
    APIC_CMD_SHORTHAND_EXCLUD = 3 << APIC_CMD_SHORTHAND_SHIFT,
};


enum {
    EXECUTION_BREAK_ADMIN_BIT = 1,
    EXECUTION_BREAK_INIT_BIT,
    EXECUTION_BREAK_SLEEP_BIT,
};


enum {
    CR0_CD = (1 << 30),
    CR0_NW = (1 << 29),
};


__thread CPU* vcpu = NULL;
static CPU* cpus[MAX_CPUS] = {0};
static uint num_cpus = 0;


static inline void set_bit(uint32_t* reg, uint index)
{
    uint offset = index & 0x1f;
    reg[index >> 5] |= (1 << offset);
}


static inline void clear_bit(uint32_t* reg, uint index)
{
    uint offset = index & 0x1f;
    reg[index >> 5] &= ~(1 << offset);
}


static inline bool test_bit(uint32_t* reg, uint index)
{
    uint offset = index & 0x1f;
    return !!(reg[index >> 5] & (1 << offset));
}


static inline int find_high_interrupt(uint32_t* reg)
{
    for (int i = 7; i >= 0; i--) {
        if (!reg[i]) {
            continue;
        }

        //todo: use bit search
        for (int j = 31; j >= 0; j--) {
            if (reg[i] & (1 << j)) {
                return (i << 5) + j;
            }
        }
    }

    return -1;
}


static void sig_usr1_handler(int sig_num)
{

}


static void init_sig_usr1()
{
    static bool is_set = false;

    if (is_set) {
        return;
    }

    struct sigaction act;

    memset(&act, 0, sizeof(act));
    sigfillset(&act.sa_mask);

    act.sa_handler = sig_usr1_handler;


    if (sigaction(SIGUSR1, &act, NULL) == -1) {
        THROW("sigaction failed %d", errno);
    }

    is_set = true;
}


CPU::CPU(NoxVM& vm)
    : VMPart ("cpu", vm)
    , _id (num_cpus++)
    , _cpu_state (INITIALIZING)
    , _state_change_target (INVALID)
    , _command (WAIT)
    , _kvm_run (NULL)
    , _execution_break (0)
    , _io_bus (vm.get_io_bus())
    , _thread ((Thread::start_proc_t)&CPU::thread_main, this)
    , _executing (false)
    , _nmi (false)
    , _test_interrupts(false)
    , _interrupt_mark_set(0)
    , _interrupt_mark_get (0)
    , _trap (NULL)
    , _version_information (0)
    , _debug_cb (NULL)
    , _debug_opaque (NULL)
    , _debug_trap (false)
    , _init_trap (false)
{
    ASSERT(_id < MAX_CPUS);
    cpus[_id] = this;
    init_sig_usr1();

    if (_id == 0) {
        pic->attach_notify_target((void_callback_t)&CPU::output_trigger, this);
    }

    _apic_timer = application->create_timer((void_callback_t)&CPU::apic_timer_cb, this);

    Lock lock(_cpu_state_mutex);

    for (;;) {
        switch (_cpu_state) {
        case WAITING:
            return;
        case INITIALIZING:
            _cpu_state_condition.wait(_cpu_state_mutex);
            break;
        default:
            THROW("cpu init failed");
        }
    }
}


CPU::~CPU()
{
    Lock lock(_command_mutex);
    ASSERT(_cpu_state == WAITING);
    _command = TERMINATE;
    _command_condition.signal();
    lock.unlock();

    _thread.join();

    if (_kvm_run) {
        munmap(_kvm_run, _vcpu_mmap_size);
    }
}


void CPU::setup_cpuid()
{
    //todo: disable x2APIC CPUID.01H:ECX[21]

    //todo: for now test if we are on AMD. (apic imp follow AMD spec)

    //todo: review cpuid result and stip it down according to vm benefits

    const int NUM_CPUID_ENTS = 100;

    struct {
        kvm_cpuid2 cpuid2;
        kvm_cpuid_entry2 ents[NUM_CPUID_ENTS];
    } cpuid_info;

    cpuid_info.cpuid2.nent = NUM_CPUID_ENTS;

    if (ioctl(get_nox().get_kvm().get_dev_fd(), KVM_GET_SUPPORTED_CPUID, &cpuid_info) == -1) {
        THROW("get supported cpuid failed");
    }

    for (int i = 0;  i < cpuid_info.cpuid2.nent; i++) {
        struct kvm_cpuid_entry2* entries = cpuid_info.cpuid2.entries;
        if (entries[i].function == 1) {
            _version_information = entries[i].eax;
            entries[i].ecx &= ~(1 << 21);
            entries[i].ebx &= 0xffff; // clera apic id and cpu count
            entries[i].ebx |= (_id << 24);
            entries[i].edx &= ~(1 << 28); // clear HTT
            entries[i].edx |= (1 << 9); // indicates APIC exists and is enabled
            D_MESSAGE("cpuid.1 0x%x 0x%x 0x%x 0x%x",
                      entries[i].eax, entries[i].ebx, entries[i].ecx,entries[i]. edx);
        }

        if (entries[i].function == 0x80000001) {
            entries[i].ecx &= ~(1 << 3); // no extended APIC
            entries[i].edx |= (1 << 9);  // indicates APIC exists and is enabled
            D_MESSAGE("cpuid.0x80000001 0x%x 0x%x 0x%x 0x%x",
                      entries[i].eax, entries[i].ebx, entries[i].ecx,entries[i]. edx);
        }

        if (entries[i].function == 0x80000008) {
            entries[i].ecx &= ~0xff; // set NC = 0 => num cores == 1
            entries[i].ecx &= ~(0x0f << 12); // set ApicIdCoreIdSize = 0
            D_MESSAGE("cpuid.0x80000008 0x%x 0x%x 0x%x 0x%x",
                      entries[i].eax, entries[i].ebx, entries[i].ecx,entries[i]. edx);
        }

        if (entries[i].function == 0x8000001e) {
            entries[i].eax = 0; // clear ExtendedApicId
            D_MESSAGE("cpuid.0x8000001e 0x%x 0x%x 0x%x 0x%x",
                      entries[i].eax, entries[i].ebx, entries[i].ecx,entries[i]. edx);
        }
    }

    std::string id_string;
    char str[5];
    uint32_t* str_ptr = (uint32_t*)str;
    str[4] = 0;
    *str_ptr = cpuid_info.ents[0].ebx;
    id_string += str;
    *str_ptr = cpuid_info.ents[0].edx;
    id_string += str;
    *str_ptr = cpuid_info.ents[0].ecx;
    id_string += str;

    D_MESSAGE("%s version 0x%x", id_string.c_str(), _version_information);

    if (ioctl(_vcpu_fd.get(), KVM_SET_CPUID2, &cpuid_info) == -1) {
         THROW("set cpuid failed");
    }
}


void CPU::save_init_msrs()
{
    KVM& kvm = get_nox().get_kvm();
    const struct kvm_msr_list& list = kvm.get_msrs_list();

    uint size = sizeof(struct kvm_msrs) + list.nmsrs * sizeof(struct kvm_msr_entry);
    AutoArray<uint8_t> msrs_list(new uint8_t[size]);

    struct kvm_msrs* msrs_ptr = (struct kvm_msrs*)msrs_list.get();
    memset(msrs_ptr, 0, size);

    msrs_ptr->nmsrs = list.nmsrs;

    for (int i = 0; i < list.nmsrs; i++) {
        msrs_ptr->entries[i].index = list.indices[i];
    }

    for (;;) {
        int r = ioctl(_vcpu_fd.get(), KVM_GET_MSRS, msrs_ptr);

        if (r < 0) {
            THROW("get init msrs failed");
        }

        if (!msrs_ptr->nmsrs || r == msrs_ptr->nmsrs) {
            break;
        }

        D_MESSAGE("skiping msr 0x%x", msrs_ptr->entries[r].index);
        memcpy(&msrs_ptr->entries[r], &msrs_ptr->entries[r + 1], msrs_ptr->nmsrs - r - 1);
        msrs_ptr->nmsrs--;
    }

    _init_msrs.set(msrs_list.release());
}


void CPU::create()
{
    KVM& kvm = get_nox().get_kvm();

    _vcpu_fd.reset(ioctl(kvm.get_vm_fd(), KVM_CREATE_VCPU, _id));

    if (!_vcpu_fd.is_valid()) {
        THROW("create failed %d", errno);
    }

    ASSERT(_vcpu_fd.get() >= 0);

    _vcpu_mmap_size = kvm.get_vcpu_mmap_size();
    _kvm_run = (kvm_run*)mmap(NULL, _vcpu_mmap_size, PROT_READ | PROT_WRITE,
                              MAP_SHARED, _vcpu_fd.get(), 0);
    if (_kvm_run == MAP_FAILED) {
        THROW("mmap failed %d", errno);
    }

    setup_cpuid();
    save_init_msrs();
}


void CPU::reset_regs()
{
    struct kvm_regs regs;

    memset(&regs, 0, sizeof(regs));
    regs.rip = 0x000000000000fff0;
    regs.rflags = 0x0000000000000002;
    regs.rdx = _version_information;
    if (ioctl(_vcpu_fd.get(), KVM_SET_REGS, &regs) == -1) {
        THROW("failed %d", errno);
    }
}


void CPU::reset_sys_regs(struct kvm_sregs& sys_regs)
{
    memset(&sys_regs, 0, sizeof(sys_regs));

    sys_regs.cr0 = 0x0000000060000010;
    sys_regs.gdt.limit = 0xffff;
    sys_regs.idt.limit = 0xffff;
    sys_regs.tr.limit = 0xffff;
    sys_regs.tr.present = 1;
    sys_regs.tr.type = 3;
    sys_regs.ldt.limit = 0xffff;
    sys_regs.ldt.present = 1;
    sys_regs.ldt.type = 2;
    sys_regs.cs.selector = 0xf000;
    sys_regs.cs.limit = 0xffff;
    sys_regs.cs.base = 0x00000000ffff0000;
    sys_regs.cs.present = 1;
    sys_regs.cs.type = 10;
    sys_regs.cs.s = 1;
    sys_regs.ds.limit = 0xffff;
    sys_regs.ds.present = 1;
    sys_regs.ds.type = 2;
    sys_regs.ds.s = 1;
    sys_regs.es.limit = 0xffff;
    sys_regs.es.present = 1;
    sys_regs.es.type = 2;
    sys_regs.es.s = 1;
    sys_regs.fs.limit = 0xffff;
    sys_regs.fs.present = 1;
    sys_regs.fs.type = 2;
    sys_regs.fs.s = 1;
    sys_regs.gs.limit = 0xffff;
    sys_regs.gs.present = 1;
    sys_regs.gs.type = 2;
    sys_regs.gs.s = 1;
    sys_regs.ss.limit = 0xffff;
    sys_regs.ss.present = 1;
    sys_regs.ss.type = 2;
    sys_regs.ss.s = 1;

    sys_regs.apic_base = APIC_DEFAULT_ADDRESS | APIC_ENABLE_MASK;

    if (_id == 0) {
        sys_regs.apic_base |= APIC_BOOTSTRAP_PROCESSOR_MASK;
    }
}


void CPU::reset_sys_regs()
{
    struct kvm_sregs sys_regs;

    reset_sys_regs(sys_regs);

    _apic_address = sys_regs.apic_base;

    if (ioctl(_vcpu_fd.get(), KVM_SET_SREGS, &sys_regs) == -1) {
        THROW("failed %d", errno);
    }
}


void CPU::reset_dbg_regs()
{
    struct kvm_debugregs debugregs;

    memset(&debugregs, 0, sizeof(debugregs));
    debugregs.dr6 = 0x00000000ffff0ff0;
    debugregs.dr7 = 0x0000000000000400;

    if (ioctl(_vcpu_fd.get(), KVM_SET_DEBUGREGS, &debugregs) == -1) {
        THROW("set debugregs failed %d", errno);
    }
}


void CPU::apic_reset()
{
    // AMD local apic

    memset(_apic_regs, 0, sizeof(_apic_regs));
    _apic_start = APIC_DEFAULT_ADDRESS;
    _apic_end = _apic_start + GUEST_PAGE_SIZE;

    _apic_regs[APIC_OFFSET_ID] = _id << APIC_ID_SHIFT;
    _apic_regs[APIC_OFFSET_VERSION] = APIC_VERSION_AMD |
                                 ((APIC_LVT_COUNT - 1) << APIC_VERSION_LVT_SHIFT) /*| (1 << 31)*/;
                                                                                // bit 31 is for
                                                                                // AMD Extended regs
    _apic_regs[APIC_OFFSET_DEST_FORMAT] = ~0;
    _apic_regs[APIC_OFFSET_SPURIOUS] = 0xff;

    _apic_regs[APIC_OFFSET_LVT_TIMER] = (1 << APIC_LVT_MASK_BIT);
    _apic_regs[APIC_OFFSET_LVT_THERMAL] = (1 << APIC_LVT_MASK_BIT);
    _apic_regs[APIC_OFFSET_LVT_PERFORMANCE] = (1 << APIC_LVT_MASK_BIT);
    _apic_regs[APIC_OFFSET_LVT_INT_0] = (1 << APIC_LVT_MASK_BIT);
    _apic_regs[APIC_OFFSET_LVT_INT_1] = (1 << APIC_LVT_MASK_BIT);
    _apic_regs[APIC_OFFSET_LVT_ERROR] = (1 << APIC_LVT_MASK_BIT);


    _current_interrupt = -1;
    _apic_timer_div = 2;
    _apic_errors = 0;
}


void CPU::reset_fpu()
{
    kvm_fpu fpu_state;

    memset(&fpu_state, 0, sizeof(fpu_state));

    fpu_state.fcw = 0x0040;
    fpu_state.ftwx = 0xff; // equivalnet to 0x5555
    fpu_state.mxcsr = 0x1F80;

    if (ioctl(_vcpu_fd.get(), KVM_SET_FPU, &fpu_state) == -1) {
        THROW("failed %d", errno);
    }
}


void CPU::reset_msrs()
{
    struct kvm_msrs* msrs_ptr = (struct kvm_msrs*)_init_msrs.get();

    int r = ioctl(_vcpu_fd.get(), KVM_SET_MSRS, msrs_ptr);

    if (r < 0) {
         THROW("failed");
    }

    if (r != msrs_ptr->nmsrs) {
        D_MESSAGE("partial set %u %u", r, msrs_ptr->nmsrs);
    }
}


bool CPU::init_trap()
{
    if (_init_trap) {
        return true;
    }

    uint32_t address = _startup_address << GUEST_PAGE_SHIFT;

    struct kvm_sregs sys_regs;

    if (ioctl(_vcpu_fd.get(), KVM_GET_SREGS, &sys_regs) == -1) {
        THROW("get sregs failed %d", errno);
    }

    sys_regs.cs.base = address;
    sys_regs.cs.selector = sys_regs.cs.base >> 4;

    if (ioctl(_vcpu_fd.get(), KVM_SET_SREGS, &sys_regs) == -1) {
        THROW("failed %d", errno);
    }

    struct kvm_regs regs;

    if (ioctl(_vcpu_fd.get(), KVM_GET_REGS, &regs) == -1) {
        THROW("get regs failed %d", errno);
    }

    regs.rip = 0;

    if (ioctl(_vcpu_fd.get(), KVM_SET_REGS, &regs) == -1) {
        THROW("set regs failed %d", errno);
    }

    return false;
}


void CPU::set_init_trap()
{
    ASSERT(_trap == NULL);
    _init_trap = true;
    _trap = &CPU::init_trap;
    trap_wait();
}


void CPU::init()
{
    D_MESSAGE("0x%x", _id);

    __sync_and_and_fetch(&_execution_break,
                         ~((1 << EXECUTION_BREAK_SLEEP_BIT) | (1 << EXECUTION_BREAK_INIT_BIT)));

    if (_trap == &CPU::init_trap) {
        trap_wait();
        return;
    }

    if (_trap == &CPU::halt_trap) {
        _trap = NULL;
    }

    reset_regs();
    reset_dbg_regs();

    struct kvm_sregs curr_sys_regs;

    if (ioctl(_vcpu_fd.get(), KVM_GET_SREGS, &curr_sys_regs) == -1) {
        THROW("get sregs failed %d", errno);
    }

    struct kvm_sregs sys_regs;

    reset_sys_regs(sys_regs);

    sys_regs.cr0 = curr_sys_regs.cr0 & (CR0_CD | CR0_NW);
    sys_regs.cr0 |= (1 << 4);
    sys_regs.cr8 = curr_sys_regs.cr8;
    sys_regs.apic_base = curr_sys_regs.apic_base;

    if (ioctl(_vcpu_fd.get(), KVM_SET_SREGS, &sys_regs) == -1) {
        THROW("failed %d", errno);
    }

    set_init_trap();
}


void CPU::startup(uint32_t address)
{
    D_MESSAGE("0x%x", _id);
    _startup_address = address;
    __sync_synchronize();
    _init_trap = false;
    force_exec_loop();
}


void CPU::reset()
{
    Lock lock(_cpu_state_mutex);

    if (_cpu_state != WAITING) {
        THROW("invalid cpu state");
    }

    apic_reset();
    reset_regs();
    reset_sys_regs();
    reset_dbg_regs();
    reset_fpu();
    reset_msrs();

    _trap = NULL;
    _nmi = false;
    _debug_trap = false;
    _init_trap = false;
    _startup_address = 0;

    __sync_and_and_fetch(&_execution_break,
                         ~((1 << EXECUTION_BREAK_SLEEP_BIT) | (1 << EXECUTION_BREAK_INIT_BIT)));

    if (_id != 0) {
        __sync_or_and_fetch(&_execution_break, (1 << EXECUTION_BREAK_INIT_BIT));
    }
}


void CPU::handle_io()
{
    uint8_t* data = (uint8_t*)_kvm_run + _kvm_run->io.data_offset;

    if (_kvm_run->io.direction == KVM_EXIT_IO_IN) {
        switch (_kvm_run->io.size) {
        case 1:
            _io_bus.read_byte(_kvm_run->io.port, data, _kvm_run->io.count);
            break;
        case 2:
            _io_bus.read_word(_kvm_run->io.port, (uint16_t*)data, _kvm_run->io.count);
            break;
        case 4:
            _io_bus.read_dword(_kvm_run->io.port, (uint32_t*)data, _kvm_run->io.count);
            break;
        }
    } else {
        switch (_kvm_run->io.size) {
        case 1:
            _io_bus.write_byte(_kvm_run->io.port, data, _kvm_run->io.count);
            break;
        case 2:
            _io_bus.write_word(_kvm_run->io.port, (uint16_t*)data, _kvm_run->io.count);
            break;
        case 4:
            _io_bus.write_dword(_kvm_run->io.port, (uint32_t*)data, _kvm_run->io.count);
            break;
        }
    }
}


void CPU::trap_wait()
{
    Lock lock(_trap_mutex);

    if (!_trap) {
        return;
    }

    for (;;) {

        if (!(this->*_trap)()) {
            _trap = NULL;
            break;
        }

        if (_execution_break) {
            break;
        }

        _trap_condition.wait(_trap_mutex);
    }
}


bool CPU::debug_trap()
{
    return _debug_trap;
}


void CPU::set_debug_trap()
{
    ASSERT(_trap == NULL);
    _debug_trap = true;
    _trap = &CPU::debug_trap;
    _debug_cb(_debug_opaque);
    trap_wait();
}


void CPU::debug_untrap()
{
    Lock lock(_trap_mutex);
    _debug_trap = false;
    _trap_condition.signal();
}


bool CPU::halt_trap()
{
    if (_need_timer_update) {
        _need_timer_update = false;
        apic_update_timer();
    }

    return !(_nmi || (_kvm_run->if_flag && interrupt_test()));
}


void CPU::set_halt_trap()
{
    ASSERT(_trap == NULL);
    _trap = &CPU::halt_trap;
    trap_wait();
}


inline int CPU::apic_eoi()
{
    // (A write to the EOI register must not be included in the handler routine for
    // an NMI, SMI, INIT, ExtINT, or SIPI.)

    int current = _current_interrupt;

    ASSERT(current == find_high_interrupt(_apic_regs + APIC_OFFSET_ISR));

    if (current == -1) {
        D_MESSAGE("nothing to clean");
    } else {
        clear_bit(_apic_regs + APIC_OFFSET_ISR, current);

        if (test_bit(_apic_regs + APIC_OFFSET_TMR, current)) {
            clear_bit(_apic_regs + APIC_OFFSET_TMR, current);
        } else {
            current = -1;
        }
    }

    _current_interrupt = find_high_interrupt(_apic_regs + APIC_OFFSET_ISR);

    apic_update_priority();

    return current;
}


void CPU::apic_update_error()
{
    _apic_regs[APIC_OFFSET_ERROR] = _apic_errors;
    _apic_errors = 0;
}


void CPU::apic_set_spurious(uint32_t val)
{
    if ((val & APIC_SPURIOUS_APIC_ENABLED_MASK) !=
        (_apic_regs[APIC_OFFSET_SPURIOUS] & APIC_SPURIOUS_APIC_ENABLED_MASK)) {

        if (!(val & APIC_SPURIOUS_APIC_ENABLED_MASK)) {

            uint32_t* lvt = &_apic_regs[APIC_OFFSET_LVT_TIMER];
            uint32_t* lvt_end = lvt + APIC_LVT_COUNT;

            for (; lvt < lvt_end; lvt++) {
                *lvt |= (1 << APIC_LVT_MASK_BIT);
            }

        } else {
            _test_interrupts = true;
        }
    }

    _apic_regs[APIC_OFFSET_SPURIOUS] = val & APIC_SPURIOUS_MASK;
}


inline void CPU::apic_command_fixed(uint32_t cmd_low)
{
    uint vector = cmd_low & APIC_CMD_VECTOR_MASK;

    if (vector < 16) {
        D_MESSAGE("illegal vector 0x%x (0x%x)", vector, cmd_low);
        //todo: set apic error
        return;
    }

    bool level_mode = !!(cmd_low & APIC_CMD_MODE_MASK);
    bool level = !!(cmd_low & APIC_CMD_LEVEL_MASK);

    if (level_mode && !level) {
        D_MESSAGE("illegal combination: level-sensitive and level is clear (0x%x)", cmd_low);
        //todo: sent accept error
        return;
    }

    switch (cmd_low & APIC_CMD_SHORTHAND_MASK) {
    case APIC_CMD_SHORTHAND_DEST: {
        uint dest = _apic_regs[APIC_OFFSET_CMD_HIGH] >> APIC_CMD_HIGH_DEST_SHIFT;

        if ((cmd_low & APIC_CMD_LOGICAL_MASK)) {
            APIC_DEBUG("APIC_CMD_SHORTHAND_DEST logical");
            __apic_deliver_interrupt_logical(vector, dest, level_mode);
        } else {
            APIC_DEBUG("APIC_CMD_SHORTHAND_DEST physical");
            if (dest < num_cpus) {
                if (dest == _id) {
                    Lock lock(_apic_mutex);
                    apic_put_irr(vector, level_mode);
                } else {
                    Lock lock(cpus[dest]->_apic_mutex);
                    cpus[dest]->apic_put_irr(vector, level);
                    lock.unlock();
                    cpus[dest]->output_trigger();
                }
                break;
            }

            if (dest == 0xff) {
                __apic_deliver_interrupt_all(vector, level_mode);
                break;
            }

            D_MESSAGE("invalid apic id");
        }
        break;
    }
    case APIC_CMD_SHORTHAND_SELF: {
        APIC_DEBUG("APIC_CMD_SHORTHAND_SELF");
        Lock lock(_apic_mutex);
        apic_put_irr(vector, level_mode);
        break;
    }
    case APIC_CMD_SHORTHAND_ALL:
        APIC_DEBUG("APIC_CMD_SHORTHAND_ALL");
        __apic_deliver_interrupt_all(vector, level_mode);
        break;
    case APIC_CMD_SHORTHAND_EXCLUD:
        APIC_DEBUG("APIC_CMD_SHORTHAND_EXCLUD");
        __apic_deliver_interrupt_exclude(vector, level_mode);
        break;
    }
}


void CPU::populate_dest_mask(uint dest, bool logical)
{
    if (logical) {
        memset(_cpu_dest_mask, 0, ALIGN(num_cpus, 8) / 8);
        for (uint i = 0; i < num_cpus; i++) {
            if (cpus[i]->apic_in_logical_dest(dest)) {
                set_bit(_cpu_dest_mask, i);
            }
        }
    } else {
        if (dest < num_cpus) {
            memset(_cpu_dest_mask, 0, ALIGN(num_cpus, 8) / 8);
            set_bit(_cpu_dest_mask, dest);
        } else if (dest == 0xff) {
            memset(_cpu_dest_mask, 0xff, ALIGN(num_cpus, 8) / 8);
        } else {
            memset(_cpu_dest_mask, 0, ALIGN(num_cpus, 8) / 8);
            D_MESSAGE("invalid apic_id");
        }
    }
}


void CPU::apic_command_init(uint32_t cmd_low)
{
    bool level_mode = !!(cmd_low & APIC_CMD_MODE_MASK);
    bool level = !!(cmd_low & APIC_CMD_LEVEL_MASK);

    if (level_mode && !level) {
        W_MESSAGE("illegal combination: level-sensitive and level is clear 0x%x", cmd_low);
        return;
    }

    switch (cmd_low & APIC_CMD_SHORTHAND_MASK) {
    case APIC_CMD_SHORTHAND_DEST: {
        APIC_DEBUG("APIC_CMD_SHORTHAND_DEST");
        uint dest = _apic_regs[APIC_OFFSET_CMD_HIGH] >> APIC_CMD_HIGH_DEST_SHIFT;
        populate_dest_mask(dest, !!(cmd_low & APIC_CMD_LOGICAL_MASK));

        for (uint i = 0; i < num_cpus; i++) {
            if (test_bit(_cpu_dest_mask, i)) {
                __sync_or_and_fetch(&cpus[i]->_execution_break, 1 << EXECUTION_BREAK_INIT_BIT);

                if (cpus[i] == this) {
                    continue;
                }

                cpus[i]->force_exec_loop();
            }
        }
        break;
    }
    case APIC_CMD_SHORTHAND_EXCLUD: {
        APIC_DEBUG("APIC_CMD_SHORTHAND_EXCLUD");

        for (uint i = 0; i < num_cpus; i++) {
            if (cpus[i] != this) {
                __sync_or_and_fetch(&cpus[i]->_execution_break, 1 << EXECUTION_BREAK_INIT_BIT);
                cpus[i]->force_exec_loop();
            }
        }
        break;
    }
    default:
        W_MESSAGE("illegal shorthand: 0x%x", cmd_low);
        return;
    };
}


void CPU::apic_command_startup(uint32_t cmd_low)
{
    switch (cmd_low & APIC_CMD_SHORTHAND_MASK) {
    case APIC_CMD_SHORTHAND_DEST: {
        APIC_DEBUG("APIC_CMD_SHORTHAND_DEST");
        uint dest = _apic_regs[APIC_OFFSET_CMD_HIGH] >> APIC_CMD_HIGH_DEST_SHIFT;
        populate_dest_mask(dest, !!(cmd_low & APIC_CMD_LOGICAL_MASK));

        for (uint i = 0; i < num_cpus; i++) {
            if (test_bit(_cpu_dest_mask, i)) {
                if (cpus[i] == this) {
                    continue;
                }
                cpus[i]->startup(cmd_low & APIC_CMD_VECTOR_MASK);
            }
        }
        break;
    }
    case APIC_CMD_SHORTHAND_EXCLUD: {
        APIC_DEBUG("APIC_CMD_SHORTHAND_EXCLUD");

        for (uint i = 0; i < num_cpus; i++) {
            if (cpus[i] != this) {
                cpus[i]->startup(cmd_low & APIC_CMD_VECTOR_MASK);
            }
        }
        break;
    }
    default:
        W_MESSAGE("illegal shorthand: 0x%x", cmd_low);
        return;
    };
}


void CPU::apic_command_nmi(uint32_t cmd_low)
{
    bool level_mode = !!(cmd_low & APIC_CMD_MODE_MASK);
    bool level = !!(cmd_low & APIC_CMD_LEVEL_MASK);

    if (level_mode && !level) {
        W_MESSAGE("illegal combination: level-sensitive and level is clear 0x%x", cmd_low);
        return;
    }

    switch (cmd_low & APIC_CMD_SHORTHAND_MASK) {
    case APIC_CMD_SHORTHAND_DEST: {
        APIC_DEBUG("APIC_CMD_SHORTHAND_DEST");
        uint dest = _apic_regs[APIC_OFFSET_CMD_HIGH] >> APIC_CMD_HIGH_DEST_SHIFT;
        populate_dest_mask(dest, !!(cmd_low & APIC_CMD_LOGICAL_MASK));

        for (uint i = 0; i < num_cpus; i++) {
            if (test_bit(_cpu_dest_mask, i)) {
                cpus[i]->_nmi = true;

                if (cpus[i] == this) {
                    continue;
                }

                cpus[i]->force_exec_loop();
            }
        }
        break;
    }
    case APIC_CMD_SHORTHAND_EXCLUD: {
        APIC_DEBUG("APIC_CMD_SHORTHAND_EXCLUD");

        for (uint i = 0; i < num_cpus; i++) {
            if (cpus[i] != this) {
                cpus[i]->_nmi = true;
                cpus[i]->force_exec_loop();
            }
        }
        break;
    }
    default:
        W_MESSAGE("illegal shorthand: 0x%x", cmd_low);
        return;
    };
}


inline void CPU::apic_command(uint32_t cmd_low)
{
    cmd_low &= ~(APIC_CMD_LOW_MBZ_MASK | APIC_CMD_LOW_RO_MASK);
    cmd_low |= _apic_regs[APIC_OFFSET_CMD_LOW] & APIC_CMD_LOW_RO_MASK;
    _apic_regs[APIC_OFFSET_CMD_LOW] = cmd_low;

    switch (cmd_low & APIC_CMD_MT_MASK) {
    case APIC_CMD_MT_FIXED:
        apic_command_fixed(cmd_low);
        break;
    case APIC_CMD_MT_LOWEST:
        W_MESSAGE("not implemented: APIC_CMD_MT_LOWEST. sleeping forever...");
        for (;;) sleep(2);
        break;
    case APIC_CMD_MT_SMI:
        W_MESSAGE("not implemented: APIC_CMD_MT_SMI. sleeping forever...");
        for (;;) sleep(2);
        break;
    case APIC_CMD_MT_READ:
        W_MESSAGE("not implemented: APIC_CMD_MT_READ. sleeping forever...");
        for (;;) sleep(2);
        break;
    case APIC_CMD_MT_NMI:
        apic_command_nmi(cmd_low);
        break;
    case APIC_CMD_MT_INIT:
        apic_command_init(cmd_low);
        break;
    case APIC_CMD_MT_STARTUP:
        apic_command_startup(cmd_low);
        break;
    case APIC_CMD_MT_EXTERNAL:
        W_MESSAGE("not implemented: APIC_CMD_MT_EXTERNAL. sleeping forever...");
        for (;;) sleep(2);
        break;
    }
}


void CPU::apic_update_priority_isr(int isr)
{
    ASSERT(isr >= 0 && isr < 0x100);

    if ((_apic_regs[APIC_OFFSET_TPR] >> 4) >= (isr >> 4)) {
        _apic_regs[APIC_OFFSET_PROCESSOR_PRIORITY] = _apic_regs[APIC_OFFSET_TPR];
    } else {
        _apic_regs[APIC_OFFSET_PROCESSOR_PRIORITY] = isr & 0xf0;
    }

    isr = MAX(isr, find_high_interrupt(_apic_regs + APIC_OFFSET_IRR));

    if ((_apic_regs[APIC_OFFSET_TPR] >> 4) >= (isr >> 4)) {
        _apic_regs[APIC_OFFSET_ARBITRATION_PRIORITY] = _apic_regs[APIC_OFFSET_TPR];
    } else {
        _apic_regs[APIC_OFFSET_ARBITRATION_PRIORITY] = isr & 0xf0;
    }
}


void CPU::apic_update_priority_irr(int irr)
{
    ASSERT(irr >= 0 && irr < 0x100);

    if ((_apic_regs[APIC_OFFSET_ARBITRATION_PRIORITY] >> 4) < (irr >> 4)) {
        _apic_regs[APIC_OFFSET_ARBITRATION_PRIORITY] = irr & 0xf0;
    }
}


inline void CPU::apic_update_priority()
{
    apic_update_priority_isr((_current_interrupt == -1) ? 0 : _current_interrupt);
    _test_interrupts = true;
}


void CPU::apic_set_timer(uint32_t val)
{
    _apic_timer->disarm();

    _apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT] = val;
    _apic_regs[APIC_OFFSET_TIMER_INIT_COUNT] = val;

    if (!val) {
        _apic_timer_start_tsc = 0;
        return;
    }

    _apic_timer_start_tsc = get_monolitic_time();

    if (!(_apic_regs[APIC_OFFSET_LVT_TIMER] & (1 << APIC_LVT_MASK_BIT))) {
        _apic_timer->arm(val * _apic_timer_div, true);
    }
}


void CPU::apic_put_irr(int irr, bool level)
{
    if (irr < 16) {
        D_MESSAGE("0-15 are reserved");
        // todo: post error interrupt
        return;

    }

    set_bit(&_apic_regs[APIC_OFFSET_IRR], irr);

    if (level) {
        set_bit(&_apic_regs[APIC_OFFSET_TMR], irr);
    } else {
        clear_bit(&_apic_regs[APIC_OFFSET_TMR], irr);
    }

    apic_update_priority_irr(irr);
    _test_interrupts = true;
}


void CPU::apic_update_timer()
{
    if (!_apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT]) {
        return;
    }

    //todo: convert time to cpu cycles
    uint64_t delta = get_monolitic_time() - _apic_timer_start_tsc;
    _apic_timer_start_tsc += delta;

    delta /= _apic_timer_div;

    Lock lock(_apic_mutex);

    if (delta < _apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT]) {
        _apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT] -= delta;
        return;
    }

    if (_apic_regs[APIC_OFFSET_LVT_TIMER] & (1 << APIC_LVT_TIMER_MODE_BIT)) {
        // maybe compensate, sub (delta - _apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT])
        // from _apic_timer_start_tsc
        _apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT] = _apic_regs[APIC_OFFSET_TIMER_INIT_COUNT];
    } else {
        _apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT] = 0;
        _apic_timer->disarm();
    }

    if (!(_apic_regs[APIC_OFFSET_LVT_TIMER] & (1 << APIC_LVT_MASK_BIT))) {
        apic_put_irr(_apic_regs[APIC_OFFSET_LVT_TIMER] & 0xff, false);
    }
}


void CPU::apic_write(uint32_t offset, uint32_t n, uint8_t* src)
{
    if (offset & 0x0f || n != 4) {
        D_MESSAGE("ignoring offset 0x%x length %u", offset, n);
        return;
    }

    uint32_t val = *(uint32_t*)src;
    offset >>= 4;

    Lock lock(_apic_mutex);

    switch (offset) {
    case APIC_OFFSET_EOI: {
        int vector = apic_eoi();

        if (vector != -1) {
            lock.unlock();
            io_apic->eoi(vector);
        }
        break;
    }
    case APIC_OFFSET_TPR:
        _apic_regs[APIC_OFFSET_TPR] = val & 0xff;
        apic_update_priority();
        break;
    case APIC_OFFSET_ID:
        // support setting apic id is modul depended, assuming it can be rejected
        D_MESSAGE("ignoring set appic id 0x%x from _id to 0x%x", _id, val & (0xff << 24));
        break;
    case APIC_OFFSET_LVT_TIMER:
        if (!is_apic_soft_enabled()) {
            val |= (1 << APIC_LVT_MASK_BIT);
        }
        _apic_regs[APIC_OFFSET_LVT_TIMER] = val & APIC_LVT_TIMER_MASK;
        apic_rearm_timer();
        break;
    case APIC_OFFSET_DIV_CONF: {
        _apic_regs[APIC_OFFSET_DIV_CONF] = val & APIC_DIV_CONF_MASK;
        uint timer_div_val = (_apic_regs[APIC_OFFSET_DIV_CONF] & 0x3) |
                             ((_apic_regs[APIC_OFFSET_DIV_CONF] & 0x8) >> 1);
        _apic_timer_div = (2 << timer_div_val) == 256 ? 1 : (2 << timer_div_val);

        break;
    }
    case APIC_OFFSET_TIMER_INIT_COUNT:
        apic_set_timer(val);
        break;
    case APIC_OFFSET_LVT_INT_0:
    case APIC_OFFSET_LVT_INT_1:
        if (!is_apic_soft_enabled()) {
            val |= (1 << APIC_LVT_MASK_BIT);
        }
        _apic_regs[offset] = val & APIC_LVT_LINT_MASK;
        break;
    case APIC_OFFSET_LVT_PERFORMANCE:
        if (!is_apic_soft_enabled()) {
            val |= (1 << APIC_LVT_MASK_BIT);
        }
        _apic_regs[APIC_OFFSET_LVT_PERFORMANCE] = val & APIC_LVT_PERFORMANCE_MASK;
        break;
    case APIC_OFFSET_LVT_THERMAL:
        if (!is_apic_soft_enabled()) {
            val |= (1 << APIC_LVT_MASK_BIT);
        }
        _apic_regs[APIC_OFFSET_LVT_THERMAL] = val & APIC_LVT_THERMAL_MASK;
        break;
    case APIC_OFFSET_LVT_ERROR:
        if (!is_apic_soft_enabled()) {
            val |= (1 << APIC_LVT_MASK_BIT);
        }
        _apic_regs[APIC_OFFSET_LVT_ERROR] = val & APIC_LVT_ERROR_MASK;
        break;
    case APIC_OFFSET_ERROR:
        apic_update_error();
        break;
    case APIC_OFFSET_SPURIOUS:
        apic_set_spurious(val);
        break;
    case APIC_OFFSET_CMD_LOW:
        lock.unlock();
        apic_command(val);
        break;
    case APIC_OFFSET_CMD_HIGH:
        _apic_regs[APIC_OFFSET_CMD_HIGH] = val & APIC_CMD_HIGH_MASK;
        break;
    case APIC_OFFSET_LOGICAL_DEST:
        _apic_regs[APIC_OFFSET_LOGICAL_DEST] = val & APIC_LOGICAL_DEST_MASK;
        break;
    case APIC_OFFSET_DEST_FORMAT:
        _apic_regs[APIC_OFFSET_DEST_FORMAT] = val & APIC_DEST_FORMAT_MASK;
        break;
    }
}


void CPU::apic_read(uint32_t offset, uint32_t n, uint8_t* dest)
{
    // APIC registers are aligned to 16-byte offsets and must be accessed using DWORD size
    // read and writes. All other accesses cause undefined behavior.

    if ((offset & 0x0f) || n != 4) {
        if ((offset & 0x0f) || n > 4) {
            D_MESSAGE("ignoring offset 0x%x length %u", offset, n);
            return;
        }

        memcpy(dest, &_apic_regs[offset >> 4], n);
        return;
    }

    offset >>= 4;

    if (offset == APIC_OFFSET_TIMER_CURRENT_COUNT) {
        apic_update_timer();
    }

    *(uint32_t*)dest = _apic_regs[offset];
}


void CPU::handle_mmio()
{
    uint len = _kvm_run->mmio.len;

    if (len > sizeof(_kvm_run->mmio.data) ||
        _kvm_run->mmio.phys_addr + len < _kvm_run->mmio.phys_addr) {
        THROW("invalid args");
    }

    if (_kvm_run->mmio.phys_addr >= _apic_start && _kvm_run->mmio.phys_addr < _apic_end) {
        uint32_t offset = _kvm_run->mmio.phys_addr - _apic_start;
        if (_kvm_run->mmio.is_write) {
            apic_write(offset, len, _kvm_run->mmio.data);
        } else {
            apic_read(offset, len, _kvm_run->mmio.data);
        }
        return;
    }

    if (_kvm_run->mmio.is_write) {
        memory_bus->write(_kvm_run->mmio.data, len, _kvm_run->mmio.phys_addr);
    } else {
        memory_bus->read(_kvm_run->mmio.phys_addr, len, _kvm_run->mmio.data);
    }
}


#ifdef NOX_DEBUG
void CPU::back_trace_64(address_t rip, address_t frame_pointer, int depth)
{
    D_MESSAGE("");
    for (int i = 0; i < depth; i++) {
        D_MESSAGE("\t\t0x%lx", rip);
        struct kvm_translation translation;
        translation.linear_address = frame_pointer + 8;

        if (ioctl(_vcpu_fd.get(), KVM_TRANSLATE, &translation) == -1 || !translation.valid) {
            D_MESSAGE("translate failed");
            return;
        }

        memory_bus->read(translation.physical_address, 8, &rip);

        translation.linear_address = frame_pointer;

        if (ioctl(_vcpu_fd.get(), KVM_TRANSLATE, &translation) == -1 || !translation.valid) {
            D_MESSAGE("translate failed");
            return;
        }

        memory_bus->read(translation.physical_address, 8, &frame_pointer);
    }
}


void CPU::backtrace_64()
{
    ASSERT(vcpu == this);

    struct kvm_regs regs;

    if (ioctl(_vcpu_fd.get(), KVM_GET_REGS, &regs) == -1) {
        THROW("failed %d", errno);
    }

    back_trace_64(regs.rip, regs.rbp, 5);
}
#endif


void CPU::set_single_step()
{
    ASSERT(vcpu == this || _cpu_state != RUNNING);

    struct kvm_guest_debug debug;

    memset(&debug, 0, sizeof(debug));
    debug.control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_SINGLESTEP | KVM_GUESTDBG_USE_SW_BP;

    if (ioctl(_vcpu_fd.get(), KVM_SET_GUEST_DEBUG, &debug) == -1) {
        THROW("failed %d", errno);
    }
}


void CPU::enter_debug_mode(void_callback_t cb, void* opaque)
{
    ASSERT(_cpu_state != RUNNING);

    struct kvm_guest_debug debug;

    memset(&debug, 0, sizeof(debug));
    debug.control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_USE_SW_BP;

    if (ioctl(_vcpu_fd.get(), KVM_SET_GUEST_DEBUG, &debug) == -1) {
        THROW("failed %d", errno);
    }

    _debug_cb = cb;
    _debug_opaque = opaque;
}


void CPU::cancle_single_step()
{
    ASSERT(_cpu_state != RUNNING);

    struct kvm_guest_debug debug;

    memset(&debug, 0, sizeof(debug));
    debug.control = KVM_GUESTDBG_ENABLE | KVM_GUESTDBG_USE_SW_BP;

    if (ioctl(_vcpu_fd.get(), KVM_SET_GUEST_DEBUG, &debug) == -1) {
        THROW("failed %d", errno);
    }

    _debug_trap = false;
}


void CPU::exit_debug_mode()
{
    ASSERT(_cpu_state != RUNNING);

    struct kvm_guest_debug debug;

    memset(&debug, 0, sizeof(debug));

    if (ioctl(_vcpu_fd.get(), KVM_SET_GUEST_DEBUG, &debug) == -1) {
        THROW("failed %d", errno);
    }

    _debug_cb = NULL;
    _debug_opaque = NULL;
    _debug_trap = false;
}


void CPU::trigger_debug_trap()
{
    ASSERT(_cpu_state == RUNNING);

    if (!_debug_cb) {
        return;
    }

    set_single_step();
}


void CPU::get_regs(CPURegs& regs)
{
    ASSERT(_cpu_state != RUNNING);

    struct kvm_regs kvm_regs;

    if (ioctl(_vcpu_fd.get(), KVM_GET_REGS, &kvm_regs) == -1) {
        THROW("get regs failed %d", errno);
    }

    regs.r[CPU_REG_A_INDEX] = kvm_regs.rax;
    regs.r[CPU_REG_B_INDEX] = kvm_regs.rbx;
    regs.r[CPU_REG_C_INDEX] = kvm_regs.rcx;
    regs.r[CPU_REG_D_INDEX] = kvm_regs.rdx;
    regs.r[CPU_REG_SI_INDEX] = kvm_regs.rsi;
    regs.r[CPU_REG_DI_INDEX] = kvm_regs.rdi;
    regs.r[CPU_REG_SP_INDEX] = kvm_regs.rsp;
    regs.r[CPU_REG_BP_INDEX] = kvm_regs.rbp;
    regs.r[CPU_REG_8_INDEX] = kvm_regs.r8;
    regs.r[CPU_REG_9_INDEX] = kvm_regs.r9;
    regs.r[CPU_REG_10_INDEX] = kvm_regs.r10;
    regs.r[CPU_REG_11_INDEX] = kvm_regs.r11;
    regs.r[CPU_REG_12_INDEX] = kvm_regs.r12;
    regs.r[CPU_REG_13_INDEX] = kvm_regs.r13;
    regs.r[CPU_REG_14_INDEX] = kvm_regs.r14;
    regs.r[CPU_REG_15_INDEX] = kvm_regs.r15;
    regs.r[CPU_REG_IP_INDEX] = kvm_regs.rip;
    regs.r[CPU_REG_FLAGS_INDEX] = kvm_regs.rflags;

    struct kvm_sregs sys_regs;

    if (ioctl(_vcpu_fd.get(), KVM_GET_SREGS, &sys_regs) == -1) {
        THROW("get sregs failed %d", errno);
    }

    regs.seg[CPU_SEG_CS] = sys_regs.cs.selector;
    regs.seg[CPU_SEG_DS] = sys_regs.ds.selector;
    regs.seg[CPU_SEG_ES] = sys_regs.es.selector;
    regs.seg[CPU_SEG_FS] = sys_regs.fs.selector;
    regs.seg[CPU_SEG_GS] = sys_regs.gs.selector;
    regs.seg[CPU_SEG_SS] = sys_regs.ss.selector;
}


#ifdef NOX_DEBUG
bool CPU::get_vector_entry_32(uint vector, uint64_t& a)
{
    ASSERT(_cpu_state != RUNNING);

    struct kvm_sregs sys_regs;

    if (ioctl(_vcpu_fd.get(), KVM_GET_SREGS, &sys_regs) == -1) {
        THROW("get sregs failed %d", errno);
    }

    struct kvm_translation translation;
    translation.linear_address = sys_regs.idt.base;

    if (ioctl(_vcpu_fd.get(), KVM_TRANSLATE, &translation) == -1 || !translation.valid) {
        D_MESSAGE("translate failed");
        return false;
    }

    uint64_t address = translation.physical_address;

    if (vector * 8 + 7 > sys_regs.idt.limit) {
        D_MESSAGE("no int descriptor");
        return false;
    }

    address += vector * 8;

    if ((address & GUEST_PAGE_MASK) != ((address + 8 -1) & GUEST_PAGE_MASK)) {
        D_MESSAGE("page error");
        return false;
    }

    uint64_t descriptor;

    memory_bus->read(address, 8, &descriptor);
    uint type = (descriptor >> (32 + 8)) & 0xf;

    if (!(descriptor & (1UL << (32 + 15)))) {
        D_MESSAGE("not present error");
        return false;
    }

    if ((descriptor & (1UL << (32 + 12)))) {
        D_MESSAGE("not system");
        return false;
    }

    if (type != 0xe) { // 32-bit interrupt gate
        D_MESSAGE("type error");
        return false;
    }

    uint64_t offset =  (descriptor & 0xffff) | ((descriptor >> 32) & (0xffff0000));
    uint64_t selector =  (descriptor >> 16) & 0xffff;

    if (selector & (1 << 2)) {
        D_MESSAGE("local");
        return false;
    }

    selector >>= 3;

    if (!selector) {
        D_MESSAGE("null selector");
        return false;
    }

    if (selector * 8 + 7 > sys_regs.gdt.limit) {
        D_MESSAGE("no gtd descriptor");
        return false;
    }

    translation.linear_address = sys_regs.gdt.base;

    if (ioctl(_vcpu_fd.get(), KVM_TRANSLATE, &translation) == -1 || !translation.valid) {
        D_MESSAGE("translate failed");
        return false;
    }

    address = translation.physical_address + selector * 8;

    if ((address & GUEST_PAGE_MASK) != ((address + 8 -1) & GUEST_PAGE_MASK)) {
        D_MESSAGE("page error");
        return false;
    }

    memory_bus->read(address, 8, &descriptor);
    type = (descriptor >> (32 + 8)) & 0xf;

    if (!(descriptor & (1UL << (32 + 15)))) {
        D_MESSAGE("not present error");
        return false;
    }

    if (!(descriptor & (1UL << (32 + 12)))) {
        D_MESSAGE("not data");
        return false;
    }

    if (type != 11) { // Execute/Read, accessed
        D_MESSAGE("type error");
        return false;
    }

    a = ((descriptor >> 16) & 0xffff) + ((descriptor >> 16) & 0xff0000) +
        ((descriptor >> 32) & 0xff000000) + offset;

    return true;
}
#endif


bool CPU::translate(uint64_t address, uint64_t& pysical)
{
    ASSERT(_cpu_state != RUNNING);

    struct kvm_translation translation;
    translation.linear_address = address;

    if (ioctl(_vcpu_fd.get(), KVM_TRANSLATE, &translation) == -1 || !translation.valid) {
        D_MESSAGE("translate failed");
        return false;
    }

    pysical = translation.physical_address;
    return true;
}


inline bool CPU::is_apic_enabled()
{
    return _apic_address & APIC_ENABLE_MASK;
}


inline bool CPU::is_apic_soft_enabled()
{
    /* The ASE bit when set to 0 disables the local APIC temporarily. When the local
       APIC is disabled, SMI, NMI, INIT, Startup, Remote Read, and LINT interrupts may
       be accepted; pending interrupts in the ISR and IRR are held, but further fixed,
       lowest-priority, and ExtInt interrupts are not accepted. All LVT entry mask bits
       are set and cannot be cleared. Setting the ASE bit to 1, enables the local APIC.
    */

    return is_apic_enabled() &&
           (_apic_regs[APIC_OFFSET_SPURIOUS] & APIC_SPURIOUS_APIC_ENABLED_MASK);
}


bool CPU::is_dirct_interrupt_pending()
{
    return false;
}


uint CPU::get_direct_interrupt()
{
    return INVALID_INTERRUPT;
}


uint CPU::get_interrupt()
{
    if (!is_apic_enabled()) {
        return pic->get_interrupt();
    }

    // If software sets the task priority in the TPR to 0, the processor will handle all
    // interrupts; it is it set to 15, all interrupts are inhibited from  being handled,
    // except those delivered with the NMI, SMI, INIT, ExtINT, INIT-deassert, and start-up
    // delivery mode

    // Message Type[3:0] indicating the type of interrupt to be presented to the local APIC. For
    // Fixed and Lowest Priority message types, the interrupt is processed through the target
    // local APIC. For all other message types, the interrupt is sent directly to the destination
    // CPU core.

    uint interrupt = get_direct_interrupt();

    if (interrupt != INVALID_INTERRUPT) {
        return interrupt;
    }

    if (!is_apic_soft_enabled()) {
        return pic->get_interrupt();
    }

    int irr = find_high_interrupt(_apic_regs + APIC_OFFSET_IRR);

    if (irr != -1 && (irr >> 4) > (_apic_regs[APIC_OFFSET_PROCESSOR_PRIORITY] >> 4)) {
         clear_bit(_apic_regs + APIC_OFFSET_IRR, irr);
         set_bit(_apic_regs + APIC_OFFSET_ISR, irr);
         _apic_regs[APIC_OFFSET_PROCESSOR_PRIORITY] = irr & 0xf0;
         _apic_regs[APIC_OFFSET_ARBITRATION_PRIORITY] = _apic_regs[APIC_OFFSET_PROCESSOR_PRIORITY];
         _current_interrupt = irr;
         return irr;
    }

    return pic->get_interrupt();
}


bool CPU::interrupt_test()
{
    if (is_apic_enabled()) {

        if (is_dirct_interrupt_pending()) {
            return true;
        }

        if (is_apic_soft_enabled()) {
            int irr = find_high_interrupt(_apic_regs + APIC_OFFSET_IRR);

            if (irr != -1 && (irr >> 4) > (_apic_regs[APIC_OFFSET_PROCESSOR_PRIORITY] >> 4)) {
                return true;
            }
        }
    }

    return pic->interrupt_test();
}


inline void CPU::set_apic_address(address_t address)
{
    if (address == _apic_address) {
        return;
    }

    if ((address & APIC_ENABLE_MASK)) {

        if ((_apic_address & APIC_ENABLE_MASK) != (address & APIC_ENABLE_MASK)) {
            apic_reset();
        }

        _apic_start = address & GUEST_PAGE_MASK;
        _apic_end = _apic_start + GUEST_PAGE_SIZE;
    } else {
        _apic_start = _apic_end = 0;
        _apic_timer->disarm();
    }

    _apic_address = address;
}


inline void CPU::sync_tpr()
{
    if ((_apic_regs[APIC_OFFSET_TPR] >> 4) == _kvm_run->cr8) {
        return;
    }

    Lock lock(_apic_mutex);
    _apic_regs[APIC_OFFSET_TPR] = _kvm_run->cr8 << 4;

    apic_update_priority();
}


void CPU::apic_rearm_timer()
{
    if (!is_apic_enabled()) {
        return;
    }

    if (_apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT] &&
                                  !(_apic_regs[APIC_OFFSET_LVT_TIMER] & (1 << APIC_LVT_MASK_BIT))) {
        _apic_timer->arm(_apic_regs[APIC_OFFSET_TIMER_CURRENT_COUNT] * _apic_timer_div, true);
    }
}


void CPU::run_loop()
{
    apic_rearm_timer();

    trap_wait();

    for (;;) {
        if (_execution_break) {
            if (_execution_break & (1 << EXECUTION_BREAK_INIT_BIT)) {
                init();
                continue;
            }

            if (_execution_break & (1 << EXECUTION_BREAK_SLEEP_BIT)) {
                throw SleepException();
            }
            break;
        }

        //block SIGUSR1

        _kvm_run->request_interrupt_window = 0;

        _executing = true;

        __sync_synchronize();

        if (_need_timer_update) {
            _need_timer_update = false;
            apic_update_timer();
        }

        if (_nmi) {
            _nmi = false;

            if (ioctl(_vcpu_fd.get(), KVM_NMI)) {
                int err = errno;
                THROW("inject NMI failed %d", err);
            }

            _kvm_run->request_interrupt_window = _interrupt_mark_set != _interrupt_mark_get ||
                                                 _test_interrupts;
        } else if (_interrupt_mark_set != _interrupt_mark_get || _test_interrupts) {
            _interrupt_mark_get = _interrupt_mark_set;
            _test_interrupts = false;

            struct kvm_interrupt interrupt;

            if (!_kvm_run->ready_for_interrupt_injection || !_kvm_run->if_flag) {
                _kvm_run->request_interrupt_window = 1;
                _test_interrupts = true;
            } else {
                interrupt.irq = get_interrupt();

                if (interrupt.irq != PIC::INVALID_IRQ) {
                    if (ioctl(_vcpu_fd.get(), KVM_INTERRUPT, &interrupt.irq)) {
                        int err = errno;
                        THROW("inject irq failed %d", err);
                    }
                    _test_interrupts = interrupt_test();
                    _kvm_run->request_interrupt_window = _test_interrupts;
                }
            }
        }

        if (ioctl(_vcpu_fd.get(), KVM_RUN, 0) == -1) {
            if (errno != EINTR) {
                int err = errno;
                THROW("failed %d", err);
            }
        }

        _executing = false;

        __sync_synchronize();

        sync_tpr();
        set_apic_address(_kvm_run->apic_base);

        switch (_kvm_run->exit_reason) {
        case KVM_EXIT_IO:
            handle_io();
            break;
        case KVM_EXIT_IRQ_WINDOW_OPEN:
            break;
        case KVM_EXIT_MMIO:
            handle_mmio();
            break;
        case KVM_EXIT_INTR:
            break;
        case KVM_EXIT_SET_TPR:
            break;
        case KVM_EXIT_HLT:
            set_halt_trap();
            break;
        case KVM_EXIT_DEBUG:
            set_debug_trap();
            break;
        default:
            THROW("unhandle kvm exit reason %d", _kvm_run->exit_reason);
        }
    }

    _apic_timer->disarm();
}


void CPU::set_cpu_state(CPUState state)
{
    if (state == _cpu_state) {
        return;
    }

    Lock lock(_cpu_state_mutex);
    _cpu_state = state;
    _cpu_state_condition.broadcast();

    if (_cpu_state == _state_change_target) {
        _state_change_target = INVALID;
        transition_done();
    }
}


void CPU::run()
{
    _thread.enable_signal(SIGUSR1);
    create();

    for (;;) {

        Lock lock(_command_mutex);

        while (_command == WAIT) {
            set_cpu_state(WAITING);
            _command_condition.wait(_command_mutex);
        }

        Command command = _command;
        _command = WAIT;
        lock.unlock();

        switch (command) {
        case WAIT:
            break;
        case RUN:
            set_cpu_state(RUNNING);
            try {
                run_loop();
            } catch (ResetException& e) {
                get_nox().vm_restart(NULL, NULL);
            } catch (SoftOffException& e) {
                get_nox().vm_power_off();
            } catch (SleepException& e) {
                __sync_or_and_fetch(&_execution_break, (1 << EXECUTION_BREAK_SLEEP_BIT));
                get_nox().vm_sleep(*this);
            }
            break;
        case TERMINATE:
            set_cpu_state(TERMINATING);
            //terminate();
            set_cpu_state(TERMINATED);
            return;
        }
    }
}


bool CPU::start()
{
    ASSERT(_cpu_state == WAITING);

    __sync_and_and_fetch(&_execution_break, ~(1 << EXECUTION_BREAK_ADMIN_BIT));

    Lock lock(_command_mutex);
    _command = RUN;
    _command_condition.signal();
    lock.unlock();

    Lock state_lock(_cpu_state_mutex);

    switch (_cpu_state) {
    case RUNNING:
        return true;
    case WAITING:
        _state_change_target = RUNNING;
        break;
    default:
        THROW("cpu start failed");
    }

    return false;
}


bool CPU::pending_sleep_request()
{
    return _execution_break & (1 << EXECUTION_BREAK_SLEEP_BIT);
}


void CPU::clear_sleep_request()
{
    ASSERT(get_state() == VMPart::SLEEPING);
    __sync_and_and_fetch(&_execution_break, ~(1 << EXECUTION_BREAK_SLEEP_BIT));
}


bool CPU::stop()
{
    ASSERT(_cpu_state == RUNNING || _cpu_state == WAITING);

    Lock lock(_trap_mutex);

    __sync_or_and_fetch(&_execution_break, 1 << EXECUTION_BREAK_ADMIN_BIT);
    _trap_condition.signal();
    lock.unlock();

    _thread.signal(SIGUSR1);

    Lock state_lock(_cpu_state_mutex);

    switch (_cpu_state) {
    case RUNNING:
        _state_change_target = WAITING;
        break;
    case WAITING:
        return true;
    default:
        THROW("cpu start failed");
    }

    return false;
}


void* CPU::thread_main()
{
    vcpu = this;

    try {
        run();
    } catch (Exception& e) {
        E_MESSAGE("unhndled exception -> %s", e.what());
       // therminate();
        set_cpu_state(ERROR);
    } catch (std::exception& e) {
         E_MESSAGE("unhndled exception -> %s", e.what());
        //therminate();
         set_cpu_state(ERROR);
    } catch (...) {
         E_MESSAGE("unhndled exception");
        //therminate();
         set_cpu_state(ERROR);
    }

    vcpu = NULL;

    return NULL;
}


void CPU::force_exec_loop()
{
    Lock lock(_trap_mutex);
    if (_trap) {
       _trap_condition.signal();
       return;
    }

    if(_executing) {
        D_MESSAGE("possible race");
    }

    _thread.signal(SIGUSR1); // BUG
}


void CPU::output_trigger()
{
    // todo: maybe move interrupt logic into signal handler

    _interrupt_mark_set++;

    __sync_synchronize();

    if (_executing && !_kvm_run->request_interrupt_window) {
        //possible race
        _thread.signal(SIGUSR1);
    }

    Lock lock(_trap_mutex);
    if (_trap == &CPU::halt_trap) {
        _trap_condition.signal();
    }
}


void CPU::apic_timer_cb()
{
    _need_timer_update = true;
    output_trigger();
}


void CPU::apic_deliver_interrupt_physical(uint vector, uint dest, bool level)
{
    if (dest < num_cpus) {
        Lock lock(cpus[dest]->_apic_mutex);
        cpus[dest]->apic_put_irr(vector, level);
        lock.unlock();
        cpus[dest]->output_trigger();
    } else if (dest == 0xff) {
        for (uint i = 0; i < num_cpus; i++) {
            Lock lock(cpus[i]->_apic_mutex);
            cpus[i]->apic_put_irr(vector, level);
            lock.unlock();
            cpus[i]->output_trigger();
        }
    } else {
        D_MESSAGE("invalid cpu id");
    }
}


inline bool CPU::apic_in_logical_dest(uint dest)
{
    uint32_t logical_dest = _apic_regs[APIC_OFFSET_LOGICAL_DEST];

    if (!_apic_regs[APIC_OFFSET_DEST_FORMAT]) { // cluster
        return  ((logical_dest >> (APIC_ID_SHIFT + 4)) == (dest >> 4)) ?
                                    !!((logical_dest >> APIC_ID_SHIFT) & dest & 0xf) : false;
    } else {
        return !!((logical_dest >> APIC_ID_SHIFT) & dest & 0xff);
    }
}


void CPU::apic_deliver_interrupt_logical(uint vector, uint dest, bool level)
{
    for (uint i = 0; i < num_cpus; i++) {
        if (cpus[i]->apic_in_logical_dest(dest)) {
            Lock lock(cpus[i]->_apic_mutex);
            cpus[i]->apic_put_irr(vector, level);
            lock.unlock();
            cpus[i]->output_trigger();
        }
    }
}


void CPU::__apic_deliver_interrupt_logical(uint vector, uint dest, bool level)
{
    for (uint i = 0; i < num_cpus; i++) {
        if (cpus[i]->apic_in_logical_dest(dest)) {
            if (cpus[i] == this) {
                Lock lock(cpus[i]->_apic_mutex);
                apic_put_irr(vector, level);
            } else {
                Lock lock(cpus[i]->_apic_mutex);
                cpus[i]->apic_put_irr(vector, level);
                lock.unlock();
                cpus[i]->output_trigger();
            }
        }
    }
}


void CPU::__apic_deliver_interrupt_all(uint vector, bool level)
{
    for (uint i = 0; i < num_cpus; i++) {
        if (cpus[i] == this) {
            Lock lock(_apic_mutex);
            apic_put_irr(vector, level);
        } else {
            Lock lock(cpus[i]->_apic_mutex);
            cpus[i]->apic_put_irr(vector, level);
            lock.unlock();
            cpus[i]->output_trigger();
        }
    }
}


void CPU::__apic_deliver_interrupt_exclude(uint vector, bool level)
{
    for (uint i = 0; i < num_cpus; i++) {
        if (cpus[i] == this) {
            continue;
        }

        Lock lock(cpus[i]->_apic_mutex);
        cpus[i]->apic_put_irr(vector, level);
        lock.unlock();
        cpus[i]->output_trigger();
    }
}



void CPU::apic_deliver_interrupt_lowest(uint vector, uint dest, bool level)
{
    uint lowest = ~0;
    uint target = ~0;

    for (uint i = 0; i < num_cpus; i++) {
        if (!cpus[i]->apic_in_logical_dest(dest)) {
            continue;
        }

        if (cpus[i]->_apic_regs[APIC_OFFSET_ARBITRATION_PRIORITY] <= lowest) {
            lowest = cpus[i]->_apic_regs[APIC_OFFSET_ARBITRATION_PRIORITY];
            target = i;
        }
    }

    if (target < num_cpus) {
        Lock lock(cpus[target]->_apic_mutex);
        cpus[target]->apic_put_irr(vector, level);
        lock.unlock();
        cpus[target]->output_trigger();
    }
}


void CPU::apic_deliver_nmi_physical(uint dest)
{
    if (dest < num_cpus) {
        cpus[dest]->nmi();
    } else if (dest == 0xff) {
        for (uint i = 0; i < num_cpus; i++) {
            cpus[i]->nmi();
        }
    } else {
        D_MESSAGE("invalid cpu id");
    }
}


void CPU::apic_deliver_nmi_logical(uint dest)
{
    for (uint i = 0; i < num_cpus; i++) {
        if (cpus[i]->apic_in_logical_dest(dest)) {
             cpus[i]->nmi();
        }
    }
}


void CPU::nmi()
{
    _nmi = true;

    if (vcpu != this) {
        __sync_synchronize();
        force_exec_loop();
    }
}

