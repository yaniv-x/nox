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

#ifndef _H_WORKER
#define _H_WORKER

#include "non_copyable.h"
#include "utils.h"
#include "threads.h"


class Worker: public NonCopyable {
public:
    Worker();
    virtual ~Worker();

    // controller
    void command(int cmd);
    void wait_state(int);
    void wakeup();


    // slave
    int wait_cmd();
    void wait_event(nox_time_t time_out);
    void add_fd(int fd, int flags);
    void remove_fd(int fd);
    void set_state(int state);
    void reset();


    enum {
        CMD_WAIT = 0,
        FIRST_USER_CMD,

        INITIALIZING = 0,
        STATE_WAITING,
        FIRST_USER_STATE,

        POLL_READ = (1 << 0),
        POLL_WRITE = (1 << 1),
        POLL_EDGE = (1 << 2),
    };

    class NewCmdException {};

private:
    void read_fd_event();
    void remove_fd(int fd, bool internal);

    enum {
        TIMEOUT_EVENT_ID = 1,
        WAKEUP_EVENT_ID,
        USER_EVENT_ID,
    };

private:
    int _state;
    Mutex _state_mutex;
    Condition _state_condition;
    int _cmd;
    bool _new_cmd;
    Mutex _cmd_mutex;
    Condition _cmd_condition;
    int _epoll;
    int _event_fd;
    int _timer_fd;

    std::list<int> user_fd_list;
};

#endif

