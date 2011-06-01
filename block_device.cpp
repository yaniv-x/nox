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

#include <fcntl.h>
#include <sys/stat.h>

#include "block_device.h"
#include "run_loop.h"

#if 0
BlockDevice::BlockDevice(const std::string& file_name,
                         uint block_size,
                         uint n_requests,
                         RunLoop& loop,
                         BlockDeviceCallback& call_back)
    : _file (open(file_name.c_str(), O_RDWR | O_DIRECT | O_DSYNC | O_LARGEFILE))
    , _event_fd (eventfd(0, EFD_NONBLOCK))
    , _loop_event (NULL)
    , _io_context (NULL)
    , _block_size (block_size)
    , _num_requests (n_requests)
    , _jobs (new IOJob[_num_requests])
    , _call_back (call_back)
{
    if (!_file.is_valid()) {
        THROW("open %s failed", file_name.c_str());
    }

    if (!_event_fd.is_valid()) {
        THROW("create eventfd failed");
    }

    struct stat stat;

    if (fstat(_file.get(), &stat) == -1) {
        THROW("fstat failed");
    }

    if ((stat.st_size % _block_size)) {
        THROW("invalid file size: %lu is not align on %d bytes", stat.st_size, _block_size);
    }

    _size = stat.st_size;

    init_io(loop);
}

BlockDevice::~BlockDevice()
{
    io_destroy(_io_context);
    _loop_event->destroy();
}


void BlockDevice::init_io(RunLoop& loop)
{
    int r = io_setup(_num_requests, &_io_context);

    if (r) {
        THROW("io setup failed %d", r);
    }

    for (int i = 0; i < _num_requests; i++) {
        _free_cbs.push_back(_jobs + i);
    }

    try {
        _loop_event = loop.create_fd_event(_event_fd.get(),
                                           (void_callback_t)&BlockDevice::handle_io,
                                           this);
    } catch (...) {
        io_destroy(_io_context);
        throw;
    }
}


void BlockDevice::clear_eventfd()
{
    for (;;) {
        eventfd_t val;

        int n = ::read(_event_fd.get(), &val, sizeof(val));

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
    }
}


void BlockDevice::handle_io()
{
    struct io_event events[_num_requests];

    struct timespec timeout;

    clear_eventfd();

    timeout.tv_sec = 0;
    timeout.tv_nsec = 0;

    long n = io_getevents(_io_context, 0, _num_requests, events, &timeout);

    if (n <= 0) {
        //D_MESSAGE("no events %d", n);
        return;
    }

    for (int i = 0; i < n; i++) {
        IOJob* job =  CONTAINEROF(events[i].obj, IOJob, cb);

        Block* block = job->block;

        if (events[i].res == _block_size) {
            _call_back.block_io_done(job->block);
        } else {
            D_MESSAGE("failed: type %u address 0x%lx result %lu",
                      job->type, block->address, events[i].res)
            int err = (events[i].res < 0) ? -events[i].res : EIO;
            _call_back.block_io_error(job->block, err);
        }

        Lock lock(_mutex);
        _free_cbs.push_front(job);
    }

    Lock lock(_mutex);
    push_queued();
}


void BlockDevice::push_queued()
{
    while (!_queue.empty() && !_free_cbs.empty()) {
        QueuedReques request = _queue.front();
        _queue.pop_front();

        IOJob* job = _free_cbs.front();
        Block* block = request.block;
        job->block = block;
        job->type = request.type;
        _free_cbs.pop_front();

        struct iocb* cb_ptr = &job->cb;

        switch (request.type) {
        case READ:
            io_prep_pread(cb_ptr, _file.get(), block->data, _block_size,
                          block->address * _block_size);
            break;
        case WRITE:
            io_prep_pwrite(cb_ptr, _file.get(), block->data, _block_size,
                           block->address * _block_size);
            break;
        default:
            PANIC("invalid");
        }

        io_set_eventfd(cb_ptr, _event_fd.get());

        int r = io_submit (_io_context, 1, &cb_ptr);

        if ( r != 1) {
            W_MESSAGE("io_submit failed %d", r);
            _queue.push_front(request);
            _free_cbs.push_front(job);
        }
    }
}


void BlockDevice::read(Block* block)
{
    Lock lock(_mutex);

    push_queued();

    if (_free_cbs.empty()) {
         _queue.push_back(QueuedReques(READ, block));
        return;
    }

    IOJob* job = _free_cbs.front();
    job->type = READ;
    job->block = block;
    _free_cbs.pop_front();

    struct iocb* cb_ptr = &job->cb;

    io_prep_pread(cb_ptr, _file.get(), block->data, _block_size, block->address * _block_size);
    io_set_eventfd(cb_ptr, _event_fd.get());

    int r = io_submit (_io_context, 1, &cb_ptr);

    if ( r != 1) {
        W_MESSAGE("io_submit failed %d", r);
        _queue.push_back(QueuedReques(READ, block));
        _free_cbs.push_front(job);
    }
}


void BlockDevice::write(Block* block)
{
    Lock lock(_mutex);

    push_queued();

    if (_free_cbs.empty()) {
         _queue.push_back(QueuedReques(WRITE, block));
        return;
    }

    IOJob* job = _free_cbs.front();
    job->type = WRITE;
    job->block = block;
    _free_cbs.pop_front();

    struct iocb* cb_ptr = &job->cb;

    io_prep_pwrite(cb_ptr, _file.get(), block->data, _block_size, block->address * _block_size);
    io_set_eventfd(cb_ptr, _event_fd.get());

    int r = io_submit(_io_context, 1, &cb_ptr);

    if ( r != 1) {
        W_MESSAGE("io_submit failed %d", r);
        _queue.push_back(QueuedReques(WRITE, block));
        _free_cbs.push_front(job);
    }
}

#endif


BlockDevice::BlockDevice(const std::string& file_name, uint block_size,
                         BlockDeviceCallback& call_back, bool read_only)
    : _file (open(file_name.c_str(), (read_only) ? O_RDONLY : O_RDWR | O_LARGEFILE))
    , _block_size (block_size)
    , _call_back (call_back)
{
    if (!_file.is_valid()) {
        THROW("open %s failed", file_name.c_str());
    }

    struct stat stat;

    if (fstat(_file.get(), &stat) == -1) {
        THROW("fstat failed");
    }

    if ((stat.st_size % _block_size)) {
        THROW("invalid file size: %lu is not align on %d bytes", stat.st_size, _block_size);
    }

    __size = stat.st_size / _block_size;

    _thread = new Thread((Thread::start_proc_t)&BlockDevice::thread_main, this);
}


BlockDevice::~BlockDevice()
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::QUIT, NULL));
    _condition.signal();
    lock.unlock();

    _thread->join();
    delete _thread;
}


void BlockDevice::read(Block* block)
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::READ, block));
    _condition.signal();
}


void BlockDevice::write(Block* block)
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::WRITE, block));
    _condition.signal();
}


void BlockDevice::sync(void* mark)
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::SYNC, mark));
    _condition.signal();
}


void BlockDevice::read(Task& task)
{
    Block* block = (Block*)task.data;

    if (block->address >= __size) {
        _call_back.block_io_error(block, 0);
        return;
    }

    uint64_t from = block->address * _block_size;
    uint8_t* to = block->data;
    uint size = _block_size;

    for (;;) {
        ssize_t n = pread64(_file.get(), to, size, from);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            _call_back.block_io_error(block, n == 0 ? 0 : errno);
            return;
        }

        if ((size -= n) == 0) {
            _call_back.block_io_done(block);
            return;
        }

        to += n;
        from += n;
    }
}


void BlockDevice::write(Task& task)
{
    Block* block = (Block*)task.data;

    if (block->address >= __size) {
        _call_back.block_io_error(block, 0);
        return;
    }

    uint64_t to = block->address * _block_size;
    uint8_t* from = block->data;
    uint size = _block_size;

    for (;;) {
        ssize_t n = pwrite64(_file.get(), from, size, to);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            _call_back.block_io_error(block, n == 0 ? 0 : errno);
            return;
        }

        if ((size -= n) == 0) {
            _call_back.block_io_done(block);
            return;
        }

        to += n;
        from += n;
    }
}


void BlockDevice::sync(Task& task)
{
    int r = fdatasync(_file.get());

    if (!r) {
        _call_back.sync_done(task.data);
    } else {
        _call_back.sync_failed(task.data, errno);
    }
}


void* BlockDevice::thread_main()
{
    for (;;) {
        Lock lock(_mutex);

        if (_tasks.empty()) {
            _condition.wait(_mutex);
        }

        Task task = _tasks.front();
        _tasks.pop_front();
        lock.unlock();

        switch (task.type) {
        case Task::READ:
            read(task);
            break;
        case Task::WRITE:
            write(task);
            break;
        case Task::SYNC:
            sync(task);
            break;
        case Task::QUIT:
            return NULL;
        }
    }
}

