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

#include "pm_controller.h"
#include "nox_vm.h"
#include "pci_bus.h"
#include "nox.h"
#include "pci.h"
#include "application.h"
#include "admin_server.h"

#define TIMER_FLIP_MASK (1 << 31)
#define PM_TIMER_HZ 3579545

static const double TICK = double(1000) * 1000 * 1000 / PM_TIMER_HZ;


enum {
    PM_STATUS_TIMER = (1 << 0),
    PM_STATUS_BUS_MASTER = (1 << 4),
    PM_STATUS_GLOBAL_LOCK = (1 << 5),
    PM_STATUS_POWER_BUTTON = (1 << 8),
    PM_STATUS_SLEEP_BUTTON = (1 << 9),
    PM_STATUS_RTC = (1 << 10), // assert when the rtc alarm rais the RTC IRQ
    PM_STATUS_WAKEUP = (1 << 15),

    PM_STATUS_SCI_MASK = PM_STATUS_TIMER | PM_STATUS_GLOBAL_LOCK | PM_STATUS_POWER_BUTTON |
                         PM_STATUS_SLEEP_BUTTON,
};


enum {
    PM_ENABLE_TIMER = (1 << 0),
    PM_ENABLE_GLOBAL_LOCK = (1 << 5),
    PM_ENABLE_POWER_BUTTON = (1 << 8),
    PM_ENABLE_SLEEP_BUTTON = (1 << 9),
    PM_ENABLE_RTC = (1 << 10),

    PM_ENABLE_MASK = PM_ENABLE_TIMER | PM_ENABLE_GLOBAL_LOCK | PM_ENABLE_POWER_BUTTON |
                     PM_ENABLE_SLEEP_BUTTON | PM_ENABLE_RTC,
};


enum {
    PM_CONTROL_SCI = (1 << 0),
    PM_CONTROL_BUS_MASTER = (1 << 1),
    PM_CONTROL_GLOBAL_LOCK = (1 << 2),
    PM_CONTROL_SLEEP_TYPE_SHIFT = 10,
    PM_CONTROL_SLEEP_MASK = 0x7 << PM_CONTROL_SLEEP_TYPE_SHIFT,
    PM_CONTROL_SLEEP = (1 << 13),

    PM_COTROL_WRITE_MASK = PM_CONTROL_SLEEP_MASK,
};


PMController::PMController(NoxVM& vm)
    : PCIDevice("pm_controller", *pci_bus, NOX_PCI_VENDOR_ID, NOX_PCI_DEV_ID_PM_CONTROLLER,
                NOX_PCI_DEV_PM_CONTROLLER_REV, mk_pci_class_code(PCI_CLASS_SYSTEM,
                                                                   PCI_SUBCLASS_SYSTEM_OTHER, 0),
                true)
    , _vm (vm)
    , _timer (application->create_timer((void_callback_t)&PMController::timer_cb, this))
{
    add_io_region(0, PM_IO_END, this, NULL,
                  (io_write_byte_proc_t)&PMController::io_write_byte,
                  (io_read_word_proc_t)&PMController::io_read_word,
                  (io_write_word_proc_t)&PMController::io_write_word,
                  (io_read_dword_proc_t)&PMController::io_read_dword);

    pci_bus->add_device(*this);
    register_admin_commands();
}


void PMController::register_admin_commands()
{
    AdminServer* admin = application->get_admin();

    va_type_list_t output_args(2);

    output_args[0] = VA_UINT32_T;
    output_args[1] = VA_UTF8_T;

    va_names_list_t output_names(2);

    output_names[0] = "result";
    output_names[1] = "error-string";


    admin->register_command("power", "power button", "???",
                            empty_va_type_list, empty_names_list, output_args, output_names,
                            (admin_command_handler_t)&PMController::power_button, this);

    admin->register_command("sleep", "sleep button", "???",
                            empty_va_type_list, empty_names_list, output_args, output_names,
                            (admin_command_handler_t)&PMController::sleep_button, this);
}


uint PMController::get_hard_id()
{
    return PM_CONTROLLER_SLOT;
}


void PMController::reset()
{
    PCIDevice::reset();
    _state = 0;
    _enable = 0;
    _control = PM_CONTROL_SCI;
    _base_time =  get_monolitic_time();
    _freeze_time = _base_time;
}


bool PMController::stop()
{
    _timer->disarm();
    _freeze_time = get_monolitic_time();

    return PCIDevice::stop();
}


bool PMController::start()
{
    ASSERT(_freeze_time);

    _base_time += get_monolitic_time() - _freeze_time;
    _freeze_time = 0;

    if ((_enable & PM_ENABLE_TIMER)) {
        update_timer();
        _timer->arm(next_flip_delta(), true);
    }

    update_irq_level();

    return PCIDevice::start();
}


void PMController::update_irq_level()
{
    set_interrupt_level(_state & _enable & PM_STATUS_SCI_MASK);
}


void PMController::update_timer()
{
    uint32_t prev_val = _timer_val;
    _timer_val = double(get_monolitic_time() - _base_time) / TICK;

    if ((prev_val & TIMER_FLIP_MASK) != (_timer_val & TIMER_FLIP_MASK)) {
        _state |= PM_STATUS_TIMER;
        update_irq_level();
    }
}


uint16_t PMController::io_read_word(uint16_t port)
{
    Lock lock(_mutex);

    port -= get_region_address(0);

    switch (port) {
    case PM_IO_STATUS:
        update_timer();
        return _state;
    case PM_IO_ENABLE:
        return _enable;
    case PM_IO_CONTROL:
        return _control;
    default:
        D_MESSAGE("unhandled 0x%x", port);
        return ~0;
    }
}


void PMController::do_sleep(uint type)
{
    switch (type) {
    case NOX_PM1_SLP_TYPE_WORKING:
        D_MESSAGE("nop");
        break;
    case NOX_PM1_SLP_TYPE_SLEEP:
        throw SleepException();
    case NOX_PM1_SLP_TYPE_SOFT_OFF:
        throw SoftOffException();
    default:
        D_MESSAGE("invalid sleep type 0x%x", type);
    }
}


void PMController::io_write_word(uint16_t port, uint16_t val)
{
    Lock lock(_mutex);

    port -= get_region_address(0);

    switch (port) {
    case PM_IO_STATUS:
        _state &= ~val;
        update_irq_level();
        break;
    case PM_IO_ENABLE:
        val &= PM_ENABLE_MASK;
        _enable = val;

        if ((_enable & PM_ENABLE_TIMER)) {
            update_timer();
            _timer->arm(next_flip_delta(), true);
        } else {
            _timer->disarm();
        }

        update_irq_level();
        break;
    case PM_IO_CONTROL:
        _control &= ~PM_COTROL_WRITE_MASK;
        _control |= (val & PM_COTROL_WRITE_MASK);

        if ((val & PM_CONTROL_GLOBAL_LOCK)) {
            _state |= PM_STATUS_GLOBAL_LOCK;
            update_irq_level();
        }

        if ((val & PM_CONTROL_SLEEP)) {
            do_sleep((_control & PM_CONTROL_SLEEP_MASK) >> PM_CONTROL_SLEEP_TYPE_SHIFT);
        }

        break;
    default:
        D_MESSAGE("unhandled 0x%x", port);
    }
}


uint32_t PMController::io_read_dword(uint16_t port)
{
    Lock lock(_mutex);

    port -= get_region_address(0);

    if (port != PM_IO_TIMER) {
        D_MESSAGE("unhandled 0x%x", port);
        return ~0;
    }

    update_timer();
    return _timer_val;
}


void PMController::io_write_byte(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    port -= get_region_address(0);

    if (port != PM_IO_RESET) {
        D_MESSAGE("unhandled 0x%x", port);
        return;
    }

    if (val != PM_RESET_MAGIC) {
        D_MESSAGE("bad reset maigic 0x%x", val);
        return;

    }

    throw ResetException();
}


nox_time_t PMController::next_flip_delta()
{
    uint32_t delta_tiks = TIMER_FLIP_MASK - (_timer_val & ~TIMER_FLIP_MASK);
    return TICK * delta_tiks;
}


void PMController::timer_cb()
{
    Lock lock(_mutex);
    update_timer();
    _timer->modify(next_flip_delta());
}


void PMController::alarm()
{
    Lock lock(_mutex);
    _state |= PM_STATUS_RTC;

    if (get_state() == VMPart::RUNNING) {
        update_irq_level();
    } else {
        ASSERT(get_state() == VMPart::SLEEPING);
        _state |= PM_STATUS_WAKEUP;
        get_nox().vm_start(NULL, NULL);
    }
}


void PMController::button_press_common(AdminReplyContext* context, uint button)
{
    RLock state_lock(get_nox().get_state_lock());

    Lock lock(_mutex);

    switch (get_state()) {
    case VMPart::RUNNING:
        _state |= button;
        update_irq_level();
        break;
    case VMPart::SLEEPING:
        _state |= PM_STATUS_WAKEUP | button;
        get_nox().vm_start(NULL, NULL);
        break;
    default:
        context->command_reply(1, "failed");
        return;
    }

    context->command_reply(0, "");
}


void PMController::power_button(AdminReplyContext* context)
{
    button_press_common(context, PM_STATUS_POWER_BUTTON);
}


void PMController::sleep_button(AdminReplyContext* context)
{
    button_press_common(context, PM_STATUS_SLEEP_BUTTON);
}

