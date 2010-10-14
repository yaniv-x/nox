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
#include <sys/timerfd.h>
#include "run_loop.h"


enum {
    EPOLL_INIT_SIZE = 20,
    EPOLL_NUM_EVENTS = 20,
};


static void nop_proc(void *) {}


class RunLoop::InternalItem : public NonCopyable {
public:
    InternalItem() {}
    virtual ~InternalItem() {}
};

class RunLoop::EpollEvent: public RunLoop::InternalItem {
public:
    EpollEvent(RunLoop& loop, int fd)
        : _loop (loop)
        , _fd (fd)
        , _valid (true)
    {
         if (_fd == -1) {
             THROW("create event fd failed");
         }

         struct epoll_event event;
         event.events = EPOLLIN | EPOLLOUT | EPOLLET;
         event.data.ptr = this;

         if (epoll_ctl(_loop._epoll.get(), EPOLL_CTL_ADD, _fd, &event) == -1) {
             THROW("epoll ctl failed %d", errno);
         }
    }

    virtual ~EpollEvent() {}

    virtual void destroy(bool sync)
    {
        ASSERT(_valid);

        _valid = false;
        epoll_ctl(_loop._epoll.get(), EPOLL_CTL_DEL, _fd, NULL);

        Lock lock(_loop._dead_list_mutex);
        _loop._dead_list.push_back(this);

        if (sync) {
             lock.unlock();
             _loop.sync();
        }
    }

    virtual void action() {}

    bool is_valid() { return _valid;};

protected:
    RunLoop& _loop;
    int _fd;
    bool _valid;
};




RunLoop::RunLoop()
    : _epoll (epoll_create(EPOLL_INIT_SIZE))
    , _timer_fd (timerfd_create(CLOCK_MONOTONIC, 0))
    , _exit_code (ERROR_OK)
    , _wakeup_event (NULL)
    , _run_loop_thread (-1)
{

    if (!_epoll.is_valid()) {
        THROW("epoll create failed");
    }

    if (!_timer_fd.is_valid()) {
        THROW("timer fd create failed");
    }

    _timer_event = new RunLoop::EpollEvent(*this, _timer_fd.get());
    _wakeup_event = create_event(nop_proc, NULL);
}


RunLoop::~RunLoop()
{
    _timer_event->destroy(false);
    _wakeup_event->destroy();

    while(!_dead_list.empty()) {
        delete *_dead_list.begin();
        _dead_list.pop_front();
    }
}


void RunLoop::wakeup()
{
    _wakeup_event->trigger();
}


class _IntervalTimer: public RunLoop::InternalItem, public RLTimer {
public:
    typedef void (*callback_t)(void*);

    _IntervalTimer(RunLoop& loop, void_callback_t proc, void* opaque)
        : _loop (loop)
        , _proc (proc)
        , _opaque (opaque)
        , _next_time (0)
        , _interval (0)
    {
    }

    virtual ~_IntervalTimer()
    {
        Lock lock(_loop._timers_mutex);
        if (_loop._timers.is_linked(_link)) {
            _loop._timers.remove(_link);
        }
    }

    virtual void arm(nox_time_t delte, bool auto_arm);
    virtual void modifay(nox_time_t delte) { _interval = delte;}

    virtual void disarm(bool sync = false)
    {
        _next_time = 0;
        _interval = 0;

        if (sync) {
            _loop.sync();
        }
    }

    virtual void destroy(bool sync = false)
    {
        Lock lock(_loop._dead_list_mutex);
        _loop._dead_list.push_back(this);
        lock.unlock();

        _proc = NULL;

        if (sync) {
            _loop.sync();
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

    static inline _IntervalTimer* convert(RingItem* item)
    {
        if (item == NULL) {
            return NULL;
        }

        _IntervalTimer* timer = NULL;
        return (_IntervalTimer*)((unsigned long)item - (unsigned long)&timer->_link);
    }

private:
    void push();

private:
    RunLoop& _loop;
    void_callback_t _proc;
    void* _opaque;
    RingItem _link;
    nox_time_t _next_time;
    nox_time_t _interval;
};


void _IntervalTimer::push()
{
    RingItem* now = _loop._timers.head();

    for (; now; now = _loop._timers.next(*now)) {

        _IntervalTimer* timer = _IntervalTimer::convert(now);

        if (timer->_next_time > _next_time) {
            _loop._timers.insert_before(_link, *now);
            return;
        }
    }

    _loop._timers.push_back(_link);
}


void _IntervalTimer::rearm()
{
    if (!_interval || Ring::is_linked(_link)) {
        return;
    }

    _next_time = get_monolitic_time() + _interval;
    push();
}


void _IntervalTimer::arm(nox_time_t delte, bool auto_arm)
{
    Lock lock(_loop._timers_mutex);

    if (_loop._timers.is_linked(_link)) {
        _loop._timers.remove(_link);
    }

    _interval = (auto_arm) ? delte : 0;
    _next_time = get_monolitic_time() + delte;
    push();

    if (_loop._timers.head() == &_link && !pthread_equal(pthread_self(), _loop._run_loop_thread)) {
        _loop.wakeup();
    }
}


RLTimer* RunLoop::create_timer(void_callback_t proc, void* opaque)
{
    return new _IntervalTimer(*this, proc, opaque);
}


class RunLoop::InternalEvent: public RunLoop::EpollEvent, public RLEvent {
public:
    InternalEvent(RunLoop& loop, void_callback_t proc, void* opaque)
        : EpollEvent(loop, eventfd(0, EFD_NONBLOCK))
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


RLEvent* RunLoop::create_event(void_callback_t proc, void* opaque)
{
    return new InternalEvent(*this, proc, opaque);
}


class InternalFDEvent: public RunLoop::EpollEvent, public FDEvent {
public:
    InternalFDEvent(RunLoop& loop, int fd, void_callback_t proc, void* opaque)
        : EpollEvent(loop, fd)
        , _proc (proc)
        , _opaque (opaque)
    {
    }

    virtual void action()
    {
        if (!is_valid()) {
            return;
        }

        _proc(_opaque);
    }

    virtual void destroy(bool sync)
    {
        RunLoop::EpollEvent::destroy(sync);
    }

private:
    void_callback_t _proc;
    void* _opaque;
};


FDEvent* RunLoop::create_fd_event(int fd, void_callback_t proc, void* opaque)
{
    return new InternalFDEvent(*this, fd, proc, opaque);
}


void RunLoop::run_timers()
{
    nox_time_t now = get_monolitic_time();

    Lock lock(_timers_mutex);

    for (;;) {

        if (_timers.empty()) {
            return;
        }

        _IntervalTimer* timer = _IntervalTimer::convert(_timers.head());

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

void RunLoop::run_tasks()
{
}

int RunLoop::get_timeout_val()
{
    Lock lock(_timers_mutex);

    _IntervalTimer* timer = _IntervalTimer::convert(_timers.head());

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


void RunLoop::run()
{
    int epoll = _epoll.get();
    _run_loop_thread = pthread_self();

    while (true) {
        struct epoll_event events[EPOLL_NUM_EVENTS];
        int timeout_val = get_timeout_val();

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

        run_tasks();

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


void RunLoop::sync()
{
    if (pthread_equal(pthread_self(), _run_loop_thread)) {
        return;
    }

    Lock lock(_sync_lock);
    wakeup();
    _sync_condition.wait(_sync_lock);
}

