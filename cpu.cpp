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

CPU::CPU(NoxVM& vm, uint id)
    : VMPart ("cpu", vm)
    , _id (id)
    , _state (INITIALIZING)
    , _command (WAIT)
    , _kvm_run (NULL)
    , _execution_break (false)
    , _io_bus (vm.get_io_bus())
    , _thread (&CPU::thread_main, this)
    , _executing (false)
    , _test_interrupts(false)
    , _interrupt_mark_set(0)
    , _interrupt_mark_get (0)
    , _halt (false)
{
    pic->attach_notify_target((void_callback_t)&CPU::output_trigger, this);
}

CPU::~CPU()
{
    _thread.join();

    if (_kvm_run) {
        munmap(_kvm_run, _vcpu_mmap_size);
    }
}


void CPU::create()
{
    KVM& kvm = get_vm().get_kvm();

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


    //KVM_SET_SIGNAL_MASK
    // KVM_INTERRUPT
}

#define CPU_STEPPING 0
#define CPU_MODUL 0
#define CPU_EXT_MODUL 0
#define CPU_FAMILY 0
#define CPU_EXT_FAMILY 0

void CPU::reset_regs()
{
    struct kvm_regs regs;

    memset(&regs, 0, sizeof(regs));
    regs.rip = 0x000000000000fff0;
    regs.rflags = 0x0000000000000002;
    regs.rdx = CPU_STEPPING | (CPU_MODUL << 4) | (CPU_EXT_MODUL << 16) |
               (CPU_FAMILY << 8) | (CPU_EXT_FAMILY << 20);
    if (ioctl(_vcpu_fd.get(), KVM_SET_REGS, &regs) == -1) {
        THROW("failed %d", errno);
    }
}


void CPU::reset_sys_regs()
{
    struct kvm_sregs sys_regs;

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
#define MSR_IA32_APICBASE_ENABLE        (1<<11)
    sys_regs.apic_base = 0x00000000fee00000 | MSR_IA32_APICBASE_ENABLE;


    if (ioctl(_vcpu_fd.get(), KVM_SET_SREGS, &sys_regs) == -1) {
        THROW("failed %d", errno);
    }
}

void CPU::reset()
{
    reset_regs();
    reset_sys_regs();
    //KVM_SET_MSRS
    //KVM_SET_FPU
    //KVM_SET_CPUID
    //KVM_SET_CPUID2
    //KVM_SET_LAPIC
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


void CPU::halt()
{
    //D_MESSAGE("");

    Lock lock(_halt_mutex);
    _halt = true;

    for (;;) {

        if (_kvm_run->if_flag && pic->intterupt_test()) {
            break;
        }

        _halt_condition.wait(_halt_mutex);
    }

     _halt = false;
}

void CPU::run_loop()
{
    while (!_execution_break) {

        //block SIGUSR1

        _kvm_run->request_interrupt_window = 0;

        _executing = true;

        __sync_synchronize();

        if (_interrupt_mark_set != _interrupt_mark_get || _test_interrupts) {
            _interrupt_mark_get = _interrupt_mark_set;
            _test_interrupts = false;

            struct kvm_interrupt interrupt;

            if (!_kvm_run->ready_for_interrupt_injection || !_kvm_run->if_flag) {
                _kvm_run->request_interrupt_window = 1;
                _test_interrupts = true;
            } else {
                interrupt.irq = pic->get_intterupt();

                if (interrupt.irq != PIC::INVALID_IRQ) {
                    if (ioctl(_vcpu_fd.get(), KVM_INTERRUPT, &interrupt.irq)) {
                        int err = errno;
                        THROW("inject irq failed %d", err);
                    }
                    _test_interrupts = pic->intterupt_test();
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

        switch (_kvm_run->exit_reason) {
        case KVM_EXIT_IO:
            handle_io();
            break;
        case KVM_EXIT_IRQ_WINDOW_OPEN:
            break;
        case KVM_EXIT_INTR:
            break;
        case KVM_EXIT_HLT:
            halt();
            break;
        default:
            THROW("unhandle kvm exit reason %d", _kvm_run->exit_reason);
        }
    }
}


void CPU::set_state(State state)
{
    if (state == _state) {
        return;
    }

    Lock lock(_state_mutex);
    _state = state;
    _state_condition.broadcast();
}

void CPU::run()
{
    create();
    reset();

    for (;;) {

        Lock lock(_command_mutex);
        while (_command == WAIT) {
            set_state(WAITING);
            _command_condition.wait(_command_mutex);
        }
        Command command = _command;
        _command = WAIT;
        lock.unlock();

        switch (command) {
        case WAIT:
            break;
        case RUN:
            set_state(RUNNING);
            run_loop();
            break;
        case TERMINATE:
            set_state(TERMINATING);
            //terminate();
            set_state(TERMINATED);
            return;
        }
    }
}


void CPU::start()
{
    Lock lock(_command_mutex);
    _command = RUN;
    _command_condition.signal();
}


void* CPU::thread_main(void* ioaque)
{
    CPU* cpu = (CPU*)ioaque;
    try {
        cpu->run();
    } catch (Exception& e) {
        E_MESSAGE("unhndled exception -> %s", e.what());
       // therminate();
    } catch (std::exception& e) {
         E_MESSAGE("unhndled exception -> %s", e.what());
        //therminate();
    } catch (...) {
         E_MESSAGE("unhndled exception");
        //therminate();
    }

    return NULL;
}

void CPU::output_trigger()
{
    _interrupt_mark_set++;

    __sync_synchronize();

    if (_executing && !_kvm_run->request_interrupt_window) {
        _thread.signal(SIGUSR1);
    }

    Lock lock(_halt_mutex);
    if (_halt) {
        _halt_condition.signal();
    }
}

