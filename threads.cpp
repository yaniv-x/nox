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

#include <signal.h>
#include "threads.h"

std::list<Thread*> threads;
__thread Thread* thread = NULL;


Thread::Thread(start_proc_t start_proc, void* opaque)
{
    sigset_t sig_mask;
    sigset_t curr_sig_mask;

    sigfillset(&sig_mask);
    sigdelset(&sig_mask, SIGKILL);
    sigdelset(&sig_mask, SIGSTOP);
    sigdelset(&sig_mask, SIGSEGV);
    sigdelset(&sig_mask, SIGFPE);
    sigdelset(&sig_mask, SIGILL);

    pthread_sigmask(SIG_SETMASK, &sig_mask, &curr_sig_mask);

    int error = pthread_create(&_thread, NULL, start_proc, opaque);

    pthread_sigmask(SIG_SETMASK, &curr_sig_mask, NULL);

    if (error) {
        THROW("failed %d", error);
    }

    ASSERT(_thread != -1);
}

void Thread::join()
{
    void* retval;

    pthread_join(_thread, &retval);
}


void Thread::enable_signal(int signal)
{
    sigset_t sigset;

    if (pthread_sigmask(SIG_SETMASK, NULL, &sigset)) {
        THROW("get mask filed");
    }

    if (sigdelset(&sigset, SIGUSR1)) {
        THROW("sigaddset filed");
    }

    if (pthread_sigmask(SIG_SETMASK, &sigset, NULL)) {
        THROW("set mask filed");
    }
}



void Thread::set_normal_priority()
{
    struct sched_param param;
    param.__sched_priority = 0;

    if (pthread_setschedparam(_thread, SCHED_OTHER, &param)) {
        W_MESSAGE("failed");
    }
}


void Thread::set_high_priority()
{
    struct sched_param param;
    param.__sched_priority = sched_get_priority_min(SCHED_RR);

    if (pthread_setschedparam(_thread, SCHED_RR, &param)) {
        W_MESSAGE("failed");
    }
}

