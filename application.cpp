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



/*enum {
    EPOLL_INIT_SIZE = 20,
    EPOLL_NUM_EVENTS = 20,
};

static pthread_t main_pthread;

static Mutex _timers_mutex;
static Ring _timers;*/


Application* application = NULL;



Application::Application()
    : _vm (NULL)
{

}


void Application::init()
{
    application = this;

    _vm.reset(new NoxVM());

    if (!_vm->init()) {
        E_MESSAGE("vm initialization failed");
        set_exit_code(ERROR_VM_INIT_FAILED);
    }
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
    init_sig_handlers();

    std::auto_ptr<Application> app(new Application());

    app->init();

    if (!IS_ERROR(app->get_exit_code())) {
        app->run();
    }

    return app->get_exit_code();
}

