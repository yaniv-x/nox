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

//#include <signal.h>
#include "threads.h"

std::list<Thread*> threads;
__thread Thread* thread = NULL;
Mutex exclucive;


Thread::Thread(start_proc_t start_proc, void* opaque)
{
    /*sigset_t sig_mask;
    sigset_t curr_sig_mask;

    sigfillset(&sig_mask);
    sigdelset(&sig_mask, SIGUSR1);
    sigdelset(&sig_mask, SIGSEGV);
    sigdelset(&sig_mask, SIGFPE);
    sigdelset(&sig_mask, SIGILL);

    pthread_sigmask(SIG_SETMASK, &sig_mask, &curr_sig_mask);*/

    int error = pthread_create(&_thread, NULL, start_proc, opaque);

    //pthread_sigmask(SIG_SETMASK, &sig_mask, &curr_sig_mask);

    if (error) {
        THROW("failed %d", error);
    }
}

void Thread::join()
{
    void* retval;

    pthread_join(_thread, &retval);
}


void Thread::suspend_other_threads(Thread* thread)
{

}

void Thread::resume_other_threads(Thread* thread)
{

}


void Thread::exclucive_inc()
{
    Thread* t = thread;

    ASSERT(t);

    if (t->_exclusive) {
        t->_exclusive++;
        return;
    }

    Lock lock(exclucive);
    t->_exclusive = 1;
    suspend_other_threads(thread);

}

void Thread::exclucive_dec()
{
    Thread* t = thread;

    Lock lock(exclucive);
    ASSERT(t->_exclusive > 0);

    if (!--t->_exclusive) {
        resume_other_threads(thread);
    }
}

