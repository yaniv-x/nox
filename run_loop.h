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

#ifndef _H_RUN_LOOP
#define _H_RUN_LOOP

#include "utils.h"
#include "threads.h"
#include "ring.h"

class Timer {
public:
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
    virtual void trigger() = 0;
    virtual void destroy(bool sync = false) = 0;

protected:
    Event() {}
    virtual ~Event() {}
};

class FDEvent {
public:
    virtual void destroy(bool sync = false) = 0;

protected:
    FDEvent() {}
    virtual ~FDEvent() {}
};

class RunLoop: public NonCopyable {
public:
    RunLoop();
    ~RunLoop();

    void run();
    void wakeup();

    Timer* create_timer(void_callback_t proc, void* opaque);
    Event* create_event(void_callback_t proc, void* opaque);
    FDEvent* create_fd_event(int fd, void_callback_t proc, void* opaque);

    bool is_self_thread_equal() { return pthread_equal(pthread_self(), _run_loop_thread);}

    class InternalItem;
    class InternalEvent;
    class EpollEvent;

protected:
    void set_exit_code(ErrorCode code) { _exit_code = code;}
    ErrorCode get_exit_code() { return _exit_code;}

private:
    int get_timeout_val();
    void run_timers();
    void run_tasks();
    void sync();

private:
    AutoFD _epoll;
    AutoFD _timer_fd;
    ErrorCode _exit_code;
    Event* _wakeup_event;
    pthread_t _run_loop_thread;

    Mutex _dead_list_mutex;
    std::list<InternalItem*> _dead_list;

    Mutex _timers_mutex;
    Ring _timers;
    EpollEvent* _timer_event;

    Mutex _sync_lock;
    Condition _sync_condition;

    friend class EpollEvent;
    friend class IntervalTimer;
};


#endif

