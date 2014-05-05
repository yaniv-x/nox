/*
    Copyright (c) 2014 Yaniv Kamay,
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

#include "worker.h"

Worker::Worker()
    : _state (INITIALIZING)
    , _cmd (CMD_WAIT)
    , _new_cmd (false)
{
    AutoFD epoll(epoll_create(3));
    AutoFD event_fd(eventfd(0, EFD_NONBLOCK));
    AutoFD timer_fd(timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK));

    if (!epoll.is_valid() || !event_fd.is_valid() || !timer_fd.is_valid()) {
        THROW("create failed");
    }

    struct epoll_event event;
    memset(&event, 0, sizeof(event));
    event.events = EPOLLIN | EPOLLOUT | EPOLLET;

    event.data.u32 = TIMEOUT_EVENT_ID;

    if (epoll_ctl(epoll.get(), EPOLL_CTL_ADD, timer_fd.get(), &event) == -1) {
        THROW_SYS_ERROR("epoll ctl failed");
    }

    event.data.u32 = WAKEUP_EVENT_ID;

    if (epoll_ctl(epoll.get(), EPOLL_CTL_ADD, event_fd.get(), &event) == -1) {
        THROW_SYS_ERROR("epoll ctl failed");
    }

    _event_fd =  event_fd.release();
    _timer_fd = timer_fd.release();
    _epoll = epoll.release();
}


Worker::~Worker()
{
    close(_event_fd);
    close(_timer_fd);
    close(_epoll);
}


void Worker::command(int cmd)
{
    Lock lock(_cmd_mutex);
    _new_cmd = true;
    _cmd = cmd;
    _cmd_condition.signal();
    wakeup();
}


void Worker::wait_state(int state)
{
    Lock lock(_state_mutex);

    while (_state != state) {
        _state_condition.wait(_state_mutex);
    }
}


void Worker::wakeup()
{
    eventfd_t val = 1;

    for (;;) {
        int n = write(_event_fd, &val, sizeof(val));

        if (n == -1) {
            if (errno == EINTR) {
                continue;
            }

            THROW_SYS_ERROR("event fd write failed");
        }
        return;
    }
}


int Worker::wait_cmd()
{
    Lock lock(_cmd_mutex);

    while (_cmd == CMD_WAIT) {
        set_state(STATE_WAITING);
        _cmd_condition.wait(_cmd_mutex);
    }

    _new_cmd = false;

    return _cmd;
}


inline void Worker::read_fd_event()
{
    eventfd_t val;

    for (;;) {
        int n = read(_event_fd, &val, sizeof(val));

        if (n == -1) {
            if (errno == EINTR) {
                continue;
            }

            if (errno == EAGAIN) {
                return;
            }

            THROW_SYS_ERROR("event fd read failed");
        }
    }
}


void Worker::wait_event(uint64_t time_out_nano)
{
    int epoll_time_out;

    if (time_out_nano) {
        if (time_out_nano != ~uint64_t(0)) {
            struct itimerspec arm_val;

            arm_val.it_value.tv_sec = time_out_nano / (1000 * 1000 * 1000);
            arm_val.it_value.tv_nsec = time_out_nano % (1000 * 1000 * 1000);
            arm_val.it_interval.tv_sec = 0;
            arm_val.it_interval.tv_nsec = 0;

            if (timerfd_settime(_timer_fd, 0, &arm_val, NULL)) {
                D_MESSAGE("set time failed");
            }
        }

        epoll_time_out = -1;
    } else {
        epoll_time_out = 0;
    }

    struct epoll_event events[4];

    int n = epoll_wait(_epoll, events, 4, epoll_time_out);

    if (n == -1 && errno !=  EINTR) {
        THROW_SYS_ERROR("epoll wait failed");
    }

    for (int i = 0; i < n; i++) {
        if (events[i].data.u32 == WAKEUP_EVENT_ID) {
            read_fd_event();
        }
    }

    if (_new_cmd) {
        _new_cmd = false;
        throw NewCmdException();
    }
}


void Worker::set_state(int state)
{
    Lock lock(_state_mutex);
    _state = state;
    _state_condition.broadcast();
}


void Worker::add_fd(int fd, int flags)
{
    if (fd == -1 || flags == 0) {
        D_MESSAGE("nop");
        return;
    }

    remove_fd(fd, true);

    struct epoll_event event;
    memset(&event, 0, sizeof(event));

    if (flags & POLL_READ) {
        event.events |= EPOLLIN;
    }

    if (flags & POLL_WRITE) {
        event.events |= EPOLLOUT;
    }

    if (flags & POLL_EDGE) {
        event.events |= EPOLLET;
    }

    event.data.u32 = USER_EVENT_ID;

    if (epoll_ctl(_epoll, EPOLL_CTL_ADD, fd, &event) == -1) {
        THROW_SYS_ERROR("epoll ctl failed");
    }

    user_fd_list.push_front(fd);
}


void Worker::remove_fd(int fd, bool internal)
{
    if (fd == -1) {
        D_MESSAGE("nop");
        return;
    }

    std::list<int> ::iterator iter = user_fd_list.begin();

    for (; iter != user_fd_list.end(); iter++) {
        if ((*iter) != fd) {
            continue;
        }

         user_fd_list.erase(iter);

        if (epoll_ctl(_epoll, EPOLL_CTL_DEL, fd, NULL) == -1) {
            W_MESSAGE("failed %d", errno)
        }
        return;
    }

    if (!internal) {
        D_MESSAGE("not found");
    }
}


void Worker::remove_fd(int fd)
{
    remove_fd(fd, false);
}


void Worker::reset()
{
    while (!user_fd_list.empty()) {
        remove_fd(*user_fd_list.begin());
    }
}

