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

#ifndef _H_THREADS
#define _H_THREADS

#include <pthread.h>
#include <signal.h>

#include "common.h"
#include "non_copyable.h"

class Atomic {
public:
    Atomic(uint32_t val = 0)
        : _val (val)
    {
    }

    uint32_t inc()
    {
        return __sync_add_and_fetch(&_val, 1);
    }

    uint32_t dec()
    {
        return __sync_sub_and_fetch(&_val, 1);
    }

private:
    uint32_t _val;
};

class Mutex: public NonCopyable {
public:

    Mutex()
    {
        if (pthread_mutex_init(&_mutex, NULL)) {
            THROW("failed");
        }
    }

    ~Mutex()
    {
        if (pthread_mutex_destroy(&_mutex)) {
            THROW("failed");
        }
    }

private:
    pthread_mutex_t* get() { return &_mutex;}

    friend class Lock;
    friend class Condition;

protected:
    pthread_mutex_t _mutex;
};


class RecursiveMutex: public Mutex {
public:
    RecursiveMutex()
    {
        pthread_mutexattr_t attr;

        if (pthread_mutexattr_init(&attr) ||
            pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE)) {

            THROW("init attr failed");
        }

        if (pthread_mutex_init(&_mutex, &attr)) {
            THROW("failed");
        }
    }
};



class Lock: public NonCopyable {
public:
    Lock(Mutex& mutex)
        : _mutex (mutex.get())
        , _looked (true)
    {
        pthread_mutex_lock(_mutex);
    }

    ~Lock() { if (_looked) unlock();}

    void lock()
    {
        ASSERT(!_looked);
        pthread_mutex_lock(_mutex);
        _looked = true;;
    }

    void unlock()
    {
        ASSERT(_looked);
        pthread_mutex_unlock(_mutex);
        _looked = false;
    }

private:
    pthread_mutex_t* _mutex;
    bool _looked;
};

class Condition: public NonCopyable {
public:
    Condition()
    {
        if (pthread_cond_init(&_condition, NULL)) {
            THROW("failed");
        }
    }

    ~Condition()
    {
        if (pthread_cond_destroy(&_condition)) {
            THROW("failed");
        }
    }

    void signal()
    {
        pthread_cond_signal(&_condition);
    }

    void broadcast()
    {
        pthread_cond_broadcast(&_condition);
    }

    void wait(Mutex& mutex)
    {
        pthread_cond_wait(&_condition, mutex.get());
    }

private:
    pthread_cond_t _condition;
};


class RWLock: public NonCopyable {
public:
    RWLock()
    {
        if (pthread_rwlock_init(&_rwlock, NULL)) {
            THROW("failed");
        }
    }

    ~RWLock()
    {
        pthread_rwlock_destroy(&_rwlock);
    }

    pthread_rwlock_t* get() {return &_rwlock;}

private:
    pthread_rwlock_t _rwlock;
};

class RLock : public NonCopyable {
public:
    RLock(RWLock& lock)
        : _rwlock (lock.get())
    {
        if (pthread_rwlock_rdlock(_rwlock)) {
            THROW("failed");
        }
    }

    ~RLock()
    {
        if (pthread_rwlock_unlock(_rwlock)) {
            THROW("failed");
        }
    }

private:
    pthread_rwlock_t* _rwlock;
};


class WLock : public NonCopyable {
public:
    WLock(RWLock& lock)
        : _rwlock (lock.get())
    {
        if (pthread_rwlock_wrlock(_rwlock)) {
            THROW("failed");
        }
    }

    ~WLock()
    {
        if (pthread_rwlock_unlock(_rwlock)) {
            THROW("failed");
        }
    }

private:
    pthread_rwlock_t* _rwlock;
};


class Thread: public NonCopyable {
public:
    typedef void* (*start_proc_t)(void*);

    Thread(start_proc_t start_proc, void* opaque);

    void join();
    void signal(int signal) { pthread_kill(_thread, signal);}
    void enable_signal(int signal);

    static void suspend_other_threads(Thread* t);
    static void resume_other_threads(Thread* t);
    static void exclucive_inc();
    static void exclucive_dec();

private:
    pthread_t _thread;
    uint32_t _exclusive;
};



#endif

