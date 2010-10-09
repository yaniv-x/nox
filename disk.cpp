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
#include <libaio.h>
#include <sys/stat.h>
#include "disk.h"

//io_cancel(2), io_destroy(2), io_setup(2), io_submit(2),


enum {
    SECTOR_SHIFT = 9,
    SECTOR_SIZE = (1 << 9),
    SECTOR_MASK = SECTOR_SIZE - 1,
};

Disk::Disk(const char* file_name, uint read_cache, uint write_cache)
    : _file (open(file_name, O_RDWR /*| O_DIRECT*/ | O_LARGEFILE))
{
    if (!_file.is_valid()) {
        THROW("open %s failed", file_name);
    }

    struct stat stat;

    if (fstat(_file.get(), &stat) == -1) {
        THROW("fstat failed");
    }

    if (stat.st_size & SECTOR_MASK) {
        THROW("invalid file size ", stat.st_size);
    }

    _size = stat.st_size;
}

Disk::~Disk()
{
}

bool Disk::read(uint64_t sector, uint8_t* buf)
{
    sector <<= SECTOR_SHIFT;

    if (sector + SECTOR_SIZE > _size) {
        return false;
    }

    ssize_t n = ::lseek(_file.get(), sector, SEEK_SET);
    if (n != sector) {
        if (n == -1) {
            int err = errno;
            W_MESSAGE("seek failed %d", err);
        }
        return false;
    }

    n = ::read(_file.get(), buf, SECTOR_SIZE);
    if ( n != SECTOR_SIZE) {
        if (n == -1) {
            int err = errno;
            W_MESSAGE("write failed %d", err);
        }
        return false;
    }

    return true;
}


bool Disk::write(uint64_t sector, const uint8_t* buf)
{
    sector <<= SECTOR_SHIFT;

    if (sector + SECTOR_SIZE > _size) {
        return false;
    }

    if (::lseek(_file.get(), SEEK_SET, sector) != sector) {
        return false;
    }

    if (::write(_file.get(), buf, SECTOR_SIZE) != SECTOR_SIZE) {
        return false;
    }

    return true;
}
