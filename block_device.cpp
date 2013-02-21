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
#include <sys/file.h>

#include "block_device.h"
#include "application.h"


BlockDevice::BlockDevice(const std::string& file_name, uint block_size, bool read_only)
    : _file (open(file_name.c_str(), ((read_only) ? O_RDONLY : O_RDWR) | O_LARGEFILE))
    , _thread (NULL)
    , _block_size (block_size)
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

    uint flock_op = read_only ? LOCK_SH : LOCK_EX;

    if (flock(_file.get(), flock_op | LOCK_NB) == -1) {
        THROW_SYS_ERROR("%s lock failed %s", (read_only) ? "share": "exclusive", file_name.c_str());
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


void BlockDevice::sync(IOSync* sync_obj)
{
    Lock lock(_mutex);
    _tasks.push_back(Task(Task::SYNC, sync_obj));
    _condition.signal();
}


static void iovec_skip(struct iovec*& io_vec, uint& vec_size, uint size)
{
    while (size >= io_vec->iov_len) {
        size -= io_vec->iov_len;
        io_vec++;

        if (!--vec_size) {
            PANIC("no space");
        }
    }

    io_vec->iov_len -= size;
    io_vec->iov_base = (uint8_t*)io_vec->iov_base + size;
}


static void iovec_split(struct iovec*& io_vec, uint& vec_size, uint size, struct iovec* split_vec,
                 uint& split_size)
{
    split_size = 0;

    while (size >= io_vec->iov_len) {
        *split_vec = *io_vec;
        split_vec++;
        split_size++;

        size -= io_vec->iov_len;
        io_vec++;

        if (!--vec_size) {
            PANIC("no space");
        }
    }

    if (!size) {
        return;
    }

    split_vec->iov_base = io_vec->iov_base;
    split_vec->iov_len = size;
    split_size++;
    io_vec->iov_len -= size;
    io_vec->iov_base = (uint8_t*)io_vec->iov_base + size;
}


static uint read_vector(int fd, uint64_t from, uint size, struct iovec* io_vec, uint vec_size)
{
    ASSERT(size);

    if (vec_size > IOV_MAX) {
        D_MESSAGE("invalid io vec size");
        return EFAULT;
    }

    if (lseek(fd, from, SEEK_SET) != from) {
        D_MESSAGE("seek failed %d", errno);
        return EFAULT;
    }

    for (;;) {
        ssize_t n = ::readv(fd, io_vec, vec_size);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            return EFAULT;
        }

        if (n != size) {
            iovec_skip(io_vec, vec_size, n);
            size -= n;
            continue;
        }

        return 0;
    }
}


void BlockDevice::write_vector(IOVec& vec)
{
    uint64_t address = vec.address;
    uint64_t end_address = address + vec.num_blocks;

    if (end_address <= address || end_address > _size) {
        vec.cb(vec.opaque, &vec, EFAULT);
        return;
    }

    uint vec_size = vec.vec_size;

    if (vec_size > IOV_MAX) {
        D_MESSAGE("invalid io vec size");
        vec.cb(vec.opaque, &vec, EFAULT);
        return;
    }

    uint64_t from = address * _block_size;

    int fd = get_fd_for_write();

    if (lseek(fd, from, SEEK_SET) != from) {
        D_MESSAGE("seek failed %d", errno);
        vec.cb(vec.opaque, &vec, EFAULT);
        return;
    }

    uint size = vec.num_blocks * _block_size;
    struct iovec* io_vec = vec.vec;

    for (;;) {
        ssize_t n = ::writev(fd, io_vec, vec_size);

        if (n <= 0) {
            if (n == -1 && errno == EINTR) {
                continue;
            }

            vec.cb(vec.opaque, &vec, EFAULT);
            return;
        }

        if (n != size) {
            iovec_skip(io_vec, vec_size, n);
            size -= n;
            continue;
        }

        for (; address < end_address; address++) {
            on_write_done(address);
        }

        vec.cb(vec.opaque, &vec, 0);
        break;
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
            read_vector(*(IOVec*)task.data);
            break;
        case Task::WRITEV:
            write_vector(*(IOVec*)task.data);
            break;
        case Task::SYNC:
            sync(task);
            break;
        case Task::QUIT:
            return NULL;
        }
    }
}


PBlockDevice::PBlockDevice(const std::string& file_name, uint block_size, bool read_only)
    : BlockDevice(file_name, block_size, read_only)
{
    start();
}


void PBlockDevice::read_vector(IOVec& vec)
{
    uint64_t start_address = vec.address;
    uint64_t end_address = start_address + vec.num_blocks;

    if (end_address <= start_address || end_address > get_num_blocks()) {
        vec.cb(vec.opaque, &vec, EFAULT);
        return;
    }

    uint err = ::read_vector(get_file(), start_address * get_block_size(),
                             vec.num_blocks * get_block_size(),
                             vec.vec, vec.vec_size);
    vec.cb(vec.opaque, &vec, err);
}


ROBlockDevice::ROBlockDevice(const std::string& file_name, uint block_size)
    : BlockDevice(file_name, block_size, true)
{
    std::string tmp_file;

    sprintf(tmp_file, "%s/tmp/%s_%u.tmp", Application::get_nox_dir().c_str(),
            basename(file_name.c_str()), rand());

    _tmp.reset(open(tmp_file.c_str(), O_CREAT | O_EXCL | O_LARGEFILE | O_NOATIME | O_RDWR));

    if (!_tmp.is_valid()) {
        THROW("create %s failed", tmp_file.c_str());
    }

    if (flock(_tmp.get(), LOCK_EX | LOCK_NB) == -1) {
        THROW_SYS_ERROR("lock %s failed", tmp_file.c_str());
    }

    unlink(tmp_file.c_str());

    if (ftruncate(_tmp.get(), get_size()) == -1) {
        THROW_SYS_ERROR("truncate failed");
    }

    start();
}


int ROBlockDevice::get_fd_for_read(uint64_t address)
{
     return (_modified_blocks.find(address) != _modified_blocks.end()) ? _tmp.get() : get_file();
}


void ROBlockDevice::read_vector(IOVec& vec)
{
    uint64_t address = vec.address;
    uint64_t end_address = address + vec.num_blocks;

    if (end_address <= address || end_address > get_num_blocks()) {
        vec.cb(vec.opaque, &vec, EFAULT);
        return;
    }

    struct iovec* io_vec = vec.vec;
    uint vec_size = vec.vec_size;

    int fd = get_fd_for_read(address);
    uint64_t run_end = address + 1;

    for (;;) {

        if (run_end == end_address) {
            uint err = ::read_vector(fd, address * get_block_size(),
                                     (run_end - address) * get_block_size(),
                                     io_vec, vec_size);
            vec.cb(vec.opaque, &vec, err);
            return;
        }

        int next_fd = get_fd_for_read(run_end);

        if (next_fd == fd) {
            run_end++;
            continue;
        }

        struct iovec split_vec[vec_size];
        uint split_size;
        uint run_size = (run_end - address) * get_block_size();

        iovec_split(io_vec, vec_size, run_size, split_vec, split_size);

        uint err = ::read_vector(fd, address * get_block_size(), run_size, split_vec, split_size);

        if (err) {
            vec.cb(vec.opaque, &vec, err);
            return;
        }

        address = run_end;
        run_end = address + 1;
        fd = next_fd;
    }
}


int ROBlockDevice::get_fd_for_write()
{
     return  _tmp.get();
}


void ROBlockDevice::on_write_done(uint64_t address)
{
    _modified_blocks.insert(address);
}

