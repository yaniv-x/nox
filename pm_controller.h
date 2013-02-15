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

#ifndef _H_PM_CONTROLLER
#define _H_PM_CONTROLLER

#include "pci_device.h"

class NoxVM;
class Timer;
class AdminReplyContext;

class PMController : public PCIDevice {
public:
    PMController(NoxVM& vm);
    virtual ~PMController();
    void alarm();

protected:
    virtual uint get_hard_id();

    virtual void reset();
    virtual bool start();
    virtual bool stop();

private:
    uint16_t io_read_word(uint16_t port);
    uint32_t io_read_dword(uint16_t port);
    void io_write_byte(uint16_t port, uint8_t val);
    void io_write_word(uint16_t port, uint16_t val);

    nox_time_t next_flip_delta();
    void timer_cb();
    void update_timer();
    void update_irq_level();
    void do_sleep(uint type);

    void register_admin_commands();
    void button_press_common(AdminReplyContext* context, uint button);
    void power_button(AdminReplyContext* context);
    void sleep_button(AdminReplyContext* context);

private:
    NoxVM& _vm;
    Mutex _mutex;
    Timer* _timer;
    uint16_t _state;
    uint16_t _enable;
    uint16_t _control;
    uint32_t _timer_val;
    nox_time_t _base_time;
    nox_time_t _freeze_time;
};

#endif

