/*
    Copyright (c) 2013-2014 Yaniv Kamay,
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

#include <set>

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

class IOVec;

typedef void (*io_vec_cb_t)(void*, IOVec*, uint err);

class IOVec {
public:
    IOVec () : vec (NULL) {}

    IOVec(uint64_t in_address, uint in_num_blocks, struct iovec* in_vec, uint in_vec_size,
          io_vec_cb_t in_cb, void* in_opaque)
        : address(in_address)
        , num_blocks (in_num_blocks)
        , vec (in_vec)
        , vec_size (in_vec_size)
        , cb (in_cb)
        , opaque (in_opaque)
    {
    }

    uint64_t address;
    uint num_blocks;
    struct iovec* vec;
    uint vec_size;
    io_vec_cb_t cb;
    void* opaque;
};


class IOSync;

typedef void (*io_sync_cb_t)(void*, IOSync*, uint err);

class IOSync {
public:
    IOSync(io_sync_cb_t in_cb, void* in_opaque)
        : cb (in_cb)
        , opaque (in_opaque)
    {
    }

    io_sync_cb_t cb;
    void* opaque;
};


class BlockDevice {
public:
    BlockDevice(const std::string& file_name, uint block_size, bool read_only);
    virtual ~BlockDevice();

    uint64_t get_size() { return _size * _block_size;}

    void readv(IOVec* vec);
    void writev(IOVec* vec);
    void sync(IOSync* obj);

    virtual void set_sync_mode(bool sync) { _sync = sync;}

protected:
    virtual int get_fd_for_read(uint64_t address) = 0;
    virtual void on_write_done(uint64_t address) = 0;
    virtual int get_fd_for_write() = 0;
    virtual void read_vector(IOVec& vec) = 0;
    uint64_t get_num_blocks() { return _size;}
    uint64_t get_block_size() { return _block_size;}

    int get_file() { return _file.get();}
    void start();

private:
    class Task {
    public:
        enum Type {
            READV,
            WRITEV,
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

    void write_vector(IOVec& vec);
    void sync(Task& task);
    void* thread_main();

private:
    AutoFD _file;
    Mutex _mutex;
    Condition _condition;
    Thread* _thread;
    uint64_t _size;
    uint _block_size;
    bool _sync;

    typedef std::list<Task> TaskList;
    TaskList _tasks;
};


class PBlockDevice: public BlockDevice {
public:
    PBlockDevice(const std::string& file_name, uint block_size, bool read_only);

private:
    virtual int get_fd_for_read(uint64_t address) { return get_file();}
    virtual void on_write_done(uint64_t address) {}
    virtual int get_fd_for_write() { return get_file();}

    virtual void read_vector(IOVec& vec);
};


class ROBlockDevice: public BlockDevice {
public:
    ROBlockDevice(const std::string& file_name, uint block_size);

    virtual void set_sync_mode(bool sync);

private:
    virtual int get_fd_for_read(uint64_t address);
    virtual void on_write_done(uint64_t address);
    virtual int get_fd_for_write();


    virtual void read_vector(IOVec& vec);

private:
    AutoFD _tmp;

    typedef std::set<uint64_t> BlocksSet;
    BlocksSet _modified_blocks;
};


#endif

