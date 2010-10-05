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

#ifndef _H_APPLICATION
#define _H_APPLICATION

#include "common.h"
#include "threads.h"

#define USE_TIMER_FD

class NoxVM;

class Timer {
public:
    static Timer* create(void_callback_t proc, void* opaque);

    virtual void destroy(bool sync = false) = 0;
    virtual void arm(nox_time_t delte, bool auto_arm) = 0;
    virtual void disarm(bool sync = false) = 0;
    virtual void modifay(nox_time_t delte) = 0;

protected:
    Timer() {}
    virtual ~Timer() {}
};

class Event {
public:
    static Event* create_event(void_callback_t proc, void* opaque);

    virtual void trigger() = 0;
    virtual void destroy(bool sync = false) = 0;

protected:
    Event() {}
    virtual ~Event() {}
};

class Application: private NonCopyable {
public:
    Application();

    void wakeup();

    static ErrorCode Main(int argc, const char** argv);

    class InternalItem;
    class EpollEvent;
    class InternalEvent;

private:
    int get_timeout_val();
    void run_timers();
    void init();
    void run();
    void sync();

private:
    AutoFD _epoll;
#ifdef USE_TIMER_FD
    AutoFD _timer_fd;
#endif
    std::auto_ptr<NoxVM> _vm;
    ErrorCode _exit_code;
    Mutex _dead_list_mutex;

    std::list<InternalItem*> _dead_list;
    Mutex _sync_lock;
    Condition _sync_condition;
    Event* _wakeup_event;

#ifndef USE_TIMER_FD
    struct timespec _poll_timeout;
#endif

    friend class EpollEvent;
    friend class IntervalTimer;
};

#endif

