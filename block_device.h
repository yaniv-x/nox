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

#ifndef _H_BLOCK_DEVICE
#define _H_BLOCK_DEVICE

#include "common.h"
#include "threads.h"

class Block {
public:
    Block(uint64_t in_address, uint8_t* in_data)
        : address(in_address)
        , data (in_data)
    {
    }

    uint64_t address;
    uint8_t* data;
};

class BlockDeviceCallback {
public:
    virtual void block_io_done(Block* block) = 0;
    virtual void block_io_error(Block* block, int error) = 0;
    virtual void sync_done(void*) {}
    virtual void sync_failed(void*, int error) {}
};

#if 0

class RunLoop;
struct FDEvent;

class BlockDevice {
public:
    BlockDevice(const std::string& file_name, uint block_size, uint n_requests,
                RunLoop& loop, BlockDeviceCallback& call_back);
    ~BlockDevice();

    uint64_t get_size() { return _size;}
    void read(Block* block);
    void write(Block* buf);

private:
    enum JobType {
        READ,
        WRITE,
    };

    struct IOJob {
        struct iocb cb;
        JobType type;
        Block* block;
    };

    struct QueuedReques {
        QueuedReques(JobType in_type, Block* in_block)
            : type (in_type)
            , block (in_block)
        {
        }

        JobType type;
        Block* block;
    };

    void init_io(RunLoop& loop);

    void handle_io();
    void push_queued();
    void clear_eventfd();

private:
    Mutex _mutex;
    AutoFD _file;
    AutoFD _event_fd;
    FDEvent* _loop_event;
    uint64_t _size;
    io_context_t _io_context;
    uint _block_size;
    uint _num_requests;

    IOJob* _jobs;
    std::list<IOJob*> _free_cbs;
    std::list<QueuedReques> _queue;

    BlockDeviceCallback& _call_back;
};

#endif


class BlockDevice {
public:
    BlockDevice(const std::string& file_name, uint block_size,
                 BlockDeviceCallback& call_back, bool read_only);
    virtual ~BlockDevice();

    uint64_t get_size() { return __size * _block_size;}
    void read(Block* block);
    void write(Block* block);
    void sync(void* mark);

private:
    class Task {
    public:
        enum Type {
            READ,
            WRITE,
            SYNC,
            QUIT,
        };

        Task(Type in_type, void* in_data)
            : type (in_type)
            , data (in_data)
        {
        }

        Type type;
        void* data;
    };

    void read(Task& task);
    void write(Task& task);
    void sync(Task& task);
    void* thread_main();

private:
    AutoFD _file;
    Mutex _mutex;
    Condition _condition;
    Thread* _thread;
    uint64_t __size;
    uint _block_size;
    BlockDeviceCallback& _call_back;

    typedef std::list<Task> TaskList;
    TaskList _tasks;
};

#endif

