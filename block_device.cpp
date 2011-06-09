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
#include <sys/uio.h>

#include "block_device.h"
#include "run_loop.h"
#include "application.h"

#define BLOCK_DEVICE_COALESCE


BlockDevice::BlockDevice(const std::string& file_name, uint block_size,
                         BlockDeviceCallback& call_back, bool read_only)
    : _file (open(file_name.c_str(), (read_only) ? O_RDONLY : O_RDWR | O_LARGEFILE))
    , _thread (NULL)
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

    _size = stat.st_size / _block_size;
}


void BlockDevice::start()
{
    _thread = new Thread((Thread::start_proc_t)&BlockDevice::thread_main, this);
}


BlockDevice::~BlockDevice()
{
    if (!_thread) {
        return;
    }

    Lock lock(_mutex);
    _tasks.push_back(Task(Task::QUIT, NULL));
    _condition.signal();
    lock.unlock();

    _thread->join();
    delete _thread;
}


void BlockDevice::readv(IOVec* vec)
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::READV, vec));
    _condition.signal();
}


void BlockDevice::writev(IOVec* vec)
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::WRITEV, vec));
    _condition.signal();
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


void BlockDevice::sync(IOSync* sync_obj)
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::SYNC, sync_obj));
    _condition.signal();
}


void iovec_skip(struct iovec*& io_vec, uint& vec_size, uint size)
{
    while (io_vec->iov_len >= size) {
        size -= io_vec->iov_len;
        io_vec++;

        if (!--vec_size) {
            PANIC("no space");
        }
    }

    io_vec->iov_len -= size;
}


void BlockDevice::readv(Task& task)
{
    IOVec* vec = (IOVec*)task.data;
    uint64_t address = vec->address;

    if (address >= _size) {
        vec->cb(vec->opaque, vec, EFAULT);
        return;
    }

    uint vec_size = vec->vec_size;

    if (vec_size > IOV_MAX) {
        D_MESSAGE("invalid io vec size");
        vec->cb(vec->opaque, vec, EFAULT);
        return;
    }

    uint64_t from = address * _block_size;

    if (lseek(get_fd_for_read(address), from, SEEK_SET) != from) {
        D_MESSAGE("seek failed %d", errno);
        vec->cb(vec->opaque, vec, EFAULT);
        return;
    }

    uint size = vec->size;
    struct iovec* io_vec = vec->vec;

    for (;;) {
        ssize_t n = ::readv(get_fd_for_read(address), io_vec, vec_size);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            vec->cb(vec->opaque, vec, EFAULT);
            return;
        }

        if (n != size) {
            iovec_skip(io_vec, vec_size, n);
            size -= n;
            continue;
        }

        vec->cb(vec->opaque, vec, 0);

        break;
    }
}


void BlockDevice::writev(Task& task)
{
    IOVec* vec = (IOVec*)task.data;
    uint64_t address = vec->address;

    if (address >= _size) {
        vec->cb(vec->opaque, vec, EFAULT);
        return;
    }

    uint vec_size = vec->vec_size;

    if (vec_size > IOV_MAX) {
        D_MESSAGE("invalid io vec size");
        vec->cb(vec->opaque, vec, EFAULT);
        return;
    }

    uint64_t from = address * _block_size;

    if (lseek(get_fd_for_write(), from, SEEK_SET) != from) {
        D_MESSAGE("seek failed %d", errno);
        vec->cb(vec->opaque, vec, EFAULT);
        return;
    }

    uint size = vec->size;
    struct iovec* io_vec = vec->vec;

    for (;;) {
        ssize_t n = ::writev(get_fd_for_write(), io_vec, vec_size);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            vec->cb(vec->opaque, vec, EFAULT);
            return;
        }

        if (n != size) {
            iovec_skip(io_vec, vec_size, n);
            size -= n;
            continue;
        }

        vec->cb(vec->opaque, vec, 0);

        break;
    }
}


void BlockDevice::read(Task& task)
{
#ifdef BLOCK_DEVICE_COALESCE
    Block* block = (Block*)task.data;
    uint64_t address = block->address;

    if (address >= _size) {
        _call_back.block_io_error(block, 0);
        return;
    }

    uint64_t from = address * _block_size;

    if (lseek(get_fd_for_read(address), from, SEEK_SET) != from) {
        D_MESSAGE("seek failed %d", errno);
        _call_back.block_io_error(block, 0);
        return;
    }

    #define MAX_COALESCE MIN(128, IOV_MAX - 1)

    struct iovec iov[MAX_COALESCE + 1];
    uint coalesce = 0;

    iov[0].iov_base = block->data;
    iov[0].iov_len = _block_size;

    Lock lock(_mutex);

    TaskList::iterator iter = _tasks.begin();
    for (; iter != _tasks.end() && coalesce < MAX_COALESCE; ++iter) {

        if ((*iter).type != Task::READ) {
            break;
        }

        Block* block = (Block*)(*iter).data;

        if (block->address != address + coalesce + 1) {
            break;
        }

        coalesce++;

        iov[coalesce].iov_base = block->data;
        iov[coalesce].iov_len = _block_size;
    }

    lock.unlock();

    for (;;) {
        ssize_t n = ::readv(get_fd_for_read(address), iov, coalesce + 1);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            _call_back.block_io_error(block, n == 0 ? 0 : errno);
            return;
        }

        if (n != (coalesce + 1) * _block_size) {
            D_MESSAGE("not handled, sleep...");
            for (;;) sleep(3);
        }

        _call_back.block_io_done(block);

        while (coalesce--) {
            Lock lock(_mutex);
            Task task = _tasks.front();
            _tasks.pop_front();
            lock.unlock();
            Block* block = (Block*)task.data;
            _call_back.block_io_done(block);
        }
        break;
    }

#else

    uint64_t from = address * _block_size;
    uint8_t* to = block->data;
    uint size = _block_size;

    for (;;) {
        ssize_t n = pread64(get_fd_for_read(address), to, size, from);

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
#endif
}


void BlockDevice::write(Task& task)
{
    Block* block = (Block*)task.data;
    uint64_t address = block->address;

    if (address >= _size) {
        _call_back.block_io_error(block, 0);
        return;
    }

    uint64_t to = address * _block_size;
    uint8_t* from = block->data;
    uint size = _block_size;

    for (;;) {
        ssize_t n = pwrite64(get_fd_for_write(), from, size, to);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            _call_back.block_io_error(block, n == 0 ? 0 : errno);
            return;
        }

        if ((size -= n) == 0) {
            on_write_done(address);
            _call_back.block_io_done(block);
            return;
        }

        to += n;
        from += n;
    }
}


void BlockDevice::sync(Task& task)
{
    int r = fdatasync(get_fd_for_write());

    IOSync* sync_obj = (IOSync*)task.data;
    sync_obj->cb(sync_obj->opaque, sync_obj, r);
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
        case Task::READV:
            readv(task);
            break;
        case Task::WRITEV:
            writev(task);
            break;
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


ROBlockDevice::ROBlockDevice(const std::string& file_name, uint block_size,
                             BlockDeviceCallback& call_back)
    : BlockDevice(file_name, block_size, call_back, true)
{
    std::string tmp_file;

    sprintf(tmp_file, "%s/tmp/%s_%u.tmp", Application::get_nox_dir().c_str(),
            basename(file_name.c_str()), rand());

    _tmp.reset(open(tmp_file.c_str(), O_CREAT | O_EXCL | O_LARGEFILE | O_NOATIME | O_RDWR));

    if (!_tmp.is_valid()) {
        THROW("create %s failed", tmp_file.c_str());
    }

    unlink(tmp_file.c_str());

    if (ftruncate(_tmp.get(), get_size()) == -1) {
        THROW_SYS_ERROR("truncate failed");
    }

    start();
}


int ROBlockDevice::get_fd_for_read(uint64_t address)
{
     return (_tmp_blocks.find(address) != _tmp_blocks.end()) ? _tmp.get() : get_file();
}


int ROBlockDevice::get_fd_for_write()
{
     return  _tmp.get();
}


void ROBlockDevice::on_write_done(uint64_t address)
{
    _tmp_blocks.insert(address);
}

