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

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include "common.h"
#include "application.h"
#include "nox_vm.h"
#include "utils.h"
#include "ring.h"



enum {
    EPOLL_INIT_SIZE = 20,
    EPOLL_NUM_EVENTS = 20,
};

Application* application = NULL;
static pthread_t main_pthread;

static Mutex _timers_mutex;
static Ring _timers;

class Application::InternalItem : public NonCopyable {
public:
    InternalItem() {}
    virtual ~InternalItem() {}
};

class Application::EpollEvent: public Application::InternalItem {
public:
    EpollEvent(int fd)
        : _fd (fd)
        , _valid (true)
    {
         if (_fd == -1) {
             THROW("create event fd failed");
         }

         struct epoll_event event;
         event.events = EPOLLIN | EPOLLOUT | EPOLLET;
         event.data.ptr = this;

         if (epoll_ctl(application->_epoll.get(), EPOLL_CTL_ADD, _fd, &event) == -1) {
             THROW("epoll ctl failed %d", errno);
         }
    }

    virtual ~EpollEvent() {}

    virtual void destroy(bool sync)
    {
        ASSERT(_valid);

        _valid = false;
        epoll_ctl(application->_epoll.get(), EPOLL_CTL_DEL, _fd, NULL);

        Lock lock(application->_dead_list_mutex);
        application->_dead_list.push_back(this);

        if (sync) {
             lock.unlock();
             application->sync();
        }
    }

    virtual void action() {}

    bool is_valid() { return _valid;};

protected:
    int _fd;
    bool _valid;
};

class Application::InternalEvent: public Application::EpollEvent, public Event {
public:
    InternalEvent(void_callback_t proc, void* opaque)
        : EpollEvent(eventfd(0, EFD_NONBLOCK))
        , _proc (proc)
        , _opaque (opaque)
    {
    }

    virtual ~InternalEvent()
    {
        close(_fd);
    }

    virtual void action()
    {
        eventfd_t val;

        for (;;) {
            int n = read(_fd, &val, sizeof(val));

            if (n == -1) {
                if (errno == EINTR) {
                    continue;
                }

                if (errno == EAGAIN) {
                    return;
                }

                THROW("event fd read failed %d", errno);
            }

            ASSERT(n == sizeof(val));

            _proc(_opaque);

            return;
        }
    }

    virtual void trigger()
    {
        if (!is_valid()) {
            return;
        }

        eventfd_t val = 1;

        for (;;) {
            int n = write(_fd, &val, sizeof(val));

            if (n == -1) {
                if (errno == EINTR) {
                    continue;
                }
                THROW("event fd write failed %d", errno);
            }

            ASSERT(n == sizeof(val));

            return;
        }
    }

    virtual void destroy(bool sync)
    {
        EpollEvent::destroy(sync);
    }

private:
    void_callback_t _proc;
    void* _opaque;
};


Event* Event::create_event(void_callback_t proc, void* opaque)
{
    return new Application::InternalEvent(proc, opaque);
}


static void nop_proc(void *) {}


Application::Application()
    : _epoll (epoll_create(EPOLL_INIT_SIZE))
#ifdef USE_TIMER_FD
    , _timer_fd (timerfd_create(CLOCK_MONOTONIC, 0))
#endif
    , _vm (NULL)
    , _exit_code (ERROR_OK)
    , _wakeup_event (NULL)
{

}


void Application::init()
{
    application = this;

    if (!_epoll.is_valid()) {
        THROW("epoll create failed %u", errno);
    }

#ifdef USE_TIMER_FD
    new Application::EpollEvent(_timer_fd.get());
#endif

    _wakeup_event = Event::create_event(nop_proc, NULL);

    _vm.reset(new NoxVM());

    if (!_vm->init()) {
        E_MESSAGE("vm initialization failed");
        _exit_code = ERROR_VM_INIT_FAILED;
    }
}


void Application::wakeup()
{
    _wakeup_event->trigger();
}


class IntervalTimer: public Application::InternalItem, public Timer {
public:
    typedef void (*callback_t)(void*);

    IntervalTimer(void_callback_t proc, void* opaque)
        : _proc (proc)
        , _opaque (opaque)
        , _next_time (0)
        , _interval (0)
    {
    }

    virtual ~IntervalTimer()
    {
        Lock lock(_timers_mutex);
        if (_timers.is_linked(_link)) {
            _timers.remove(_link);
        }
    }

    virtual void arm(nox_time_t delte, bool auto_arm);
    virtual void modifay(nox_time_t delte) { _interval = delte;}

    virtual void disarm(bool sync = false)
    {
        _next_time = 0;
        _interval = 0;

        if (sync) {
            application->sync();
        }
    }

    virtual void destroy(bool sync = false)
    {
        Lock lock(application->_dead_list_mutex);
        application->_dead_list.push_back(this);
        lock.unlock();

        _proc = NULL;

        if (sync) {
            application->sync();
        }
    }

    void rearm();

    inline void execute()
    {
        if (!is_valid() || !is_armed()) {
            return;
        }

        _proc(_opaque);
    }

    inline bool expired(nox_time_t now)
    {
        return _next_time <= now;
    }

    inline bool is_valid()
    {
        return _proc != NULL;
    }

    inline bool is_armed()
    {
        return _next_time != 0;
    }

    inline bool is_interval()
    {
        return _interval != 0;
    }

    RingItem& get_link() { return _link;}
    nox_time_t get_next_time() {return _next_time;}

    static inline IntervalTimer* convert(RingItem* item)
    {
        if (item == NULL) {
            return NULL;
        }

        IntervalTimer* timer = NULL;
        return (IntervalTimer*)((unsigned long)item - (unsigned long)&timer->_link);
    }

private:
    void push();

private:
    void_callback_t _proc;
    void* _opaque;
    RingItem _link;
    nox_time_t _next_time;
    nox_time_t _interval;
};


void IntervalTimer::push()
{
    RingItem* now = _timers.head();

    for (; now; now = _timers.next(*now)) {

        IntervalTimer* timer = IntervalTimer::convert(now);

        if (timer->_next_time > _next_time) {
            _timers.insert_before(_link, *now);
            return;
        }
    }

    _timers.push_back(_link);
}


void IntervalTimer::rearm()
{
    if (!_interval || Ring::is_linked(_link)) {
        return;
    }

    _next_time = get_monolitic_time() + _interval;
    push();
}


void IntervalTimer::arm(nox_time_t delte, bool auto_arm)
{
    Lock lock(_timers_mutex);

    if (_timers.is_linked(_link)) {
        _timers.remove(_link);
    }

    _interval = (auto_arm) ? delte : 0;
    _next_time = get_monolitic_time() + delte;
    push();

    if (_timers.head() == &_link && !pthread_equal(pthread_self(), main_pthread)) {
        application->wakeup();
    }
}


Timer* Timer::create(void_callback_t proc, void* opaque)
{
    return new IntervalTimer(proc, opaque);
}


void Application::run_timers()
{
    nox_time_t now = get_monolitic_time();

    Lock lock(_timers_mutex);

    for (;;) {

        if (_timers.empty()) {
            return;
        }

        IntervalTimer* timer = IntervalTimer::convert(_timers.head());

        if (!timer->expired(now)) {
            return;
        }

        _timers.remove(timer->get_link());

        lock.unlock();

        timer->execute();

        lock.lock();

        timer->rearm();
    }
}


#ifdef USE_TIMER_FD

int Application::get_timeout_val()
{
    Lock lock(_timers_mutex);

    IntervalTimer* timer = IntervalTimer::convert(_timers.head());

    if (!timer) {
        return -1;
    }

    nox_time_t next_time = timer->get_next_time();
    nox_time_t now = get_monolitic_time();

    if (now >= next_time) {
        return 0;
    }

    next_time -= now;

    struct itimerspec wakeup_time;
    wakeup_time.it_value.tv_sec = next_time / (1000 * 1000 * 1000);
    wakeup_time.it_value.tv_nsec = next_time % (1000 * 1000 * 1000);
    wakeup_time.it_interval.tv_sec = 0;
    wakeup_time.it_interval.tv_nsec = 0;

    if (timerfd_settime(_timer_fd.get(), 0, &wakeup_time, NULL)) {
        D_MESSAGE("set time failed");
        return 0;
    }

    return -1;
}

#else

enum {
    TIMEOUT_IMIDIAT = 0,
    TIMEOUT_INFINIT = ~0,
    TIMEOUT_USE_TIME_SPEC = ~1,
};

int Application::get_timeout_val()
{
    Lock lock(_timers_mutex);

    IntervalTimer* timer = IntervalTimer::convert(_timers.head());

    if (!timer) {
        return TIMEOUT_INFINIT;
    }

    nano_time_t next_time = timer->get_next_time();
    nano_time_t now = get_monolitic_time();

    if (now >= next_time) {
        return TIMEOUT_IMIDIAT;
    }

    next_time -= now;
    _poll_timeout.tv_sec = next_time / (1000 * 1000 * 1000);
    _poll_timeout.tv_nsec = next_time % (1000 * 1000 * 1000);

    return TIMEOUT_USE_TIME_SPEC;
}
#endif


void Application::run()
{
    int epoll = _epoll.get();

#ifndef USE_TIMER_FD
    fd_set read_file_set;
    FD_ZERO(&read_file_set);
    FD_SET(epoll, &read_file_set);
#endif

    while (true) {
        struct epoll_event events[EPOLL_NUM_EVENTS];
        int timeout_val = get_timeout_val();

#ifndef USE_TIMER_FD
        if (timeout_val == TIMEOUT_USE_TIME_SPEC) {
            pselect(epoll + 1, &read_file_set, NULL, NULL, &_poll_timeout, NULL);
            timeout_val = 0;
        }
#endif
        int n = epoll_wait(epoll, events, EPOLL_NUM_EVENTS, timeout_val);

        if (n == -1) {
            if (errno !=  EINTR) {
                THROW("epoll wait failed %u", errno);
            }
            continue;
        }

        run_timers();

        struct epoll_event* now = events;
        struct epoll_event* end = now + n;

        for (; now < end; now++) {
            ((EpollEvent*)now->data.ptr)->action();
        }

        if (!_dead_list.empty()) {
            Lock lock(_dead_list_mutex);

            while(!_dead_list.empty()) {
                delete *_dead_list.begin();
                _dead_list.pop_front();
            }
        }

        _sync_condition.broadcast();
    }
}


void Application::sync()
{
    if (pthread_equal(pthread_self(), main_pthread)) {
        return;
    }

    Lock lock(_sync_lock);
    wakeup();
    _sync_condition.wait(_sync_lock);
}


static void sig_handler(int sig)
{
}

static void init_sig_handlers()
{
    struct sigaction act;

    memset(&act, 0, sizeof(act));
    sigfillset(&act.sa_mask);
    act.sa_handler = sig_handler;

    if (sigaction(SIGUSR1, &act, NULL) == -1) {
        THROW("sigaction failed %d", errno);
    }
}


ErrorCode Application::Main(int argc, const char** argv)
{
    main_pthread = pthread_self();

    init_sig_handlers();

    std::auto_ptr<Application> app(new Application());

    app->init();

    if (!IS_ERROR(app->_exit_code)) {
        app->run();
    }

    return app->_exit_code;
}

