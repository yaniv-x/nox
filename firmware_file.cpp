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
#include "firmware_file.h"


FirmwareFile::FirmwareFile()
    : _fd (-1)
{
}


FirmwareFile::~FirmwareFile()
{
    if (is_valid()) {
        close(_fd);
    }
}


bool FirmwareFile::open(const char* path)
{
    if (is_valid()) {
        THROW("double open");
    }

    AutoFD fd(::open(path, O_RDONLY));

    if (!fd.is_valid()) {
        return false;
    }

    struct stat stat;

    if (fstat(fd.get(), &stat) == -1) {
        THROW("fstat failed");
    }

    if (!stat.st_size) {
        THROW("bad file size %u", stat.st_size);
    }

    if ((stat.st_size % GUEST_PAGE_SIZE)) {
        D_MESSAGE("bad file size %u", stat.st_size);
    }

    _file_size = stat.st_size;

    if (stat.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH)) {
        W_MESSAGE("%s is writable. trying to remove write permission", path);
        fchmod(fd.get(), stat.st_mode & ~(S_IWUSR | S_IWGRP | S_IWOTH));
    }

    _fd = fd.release();
    _num_pages = ALIGN(_file_size, GUEST_PAGE_SIZE) >> GUEST_PAGE_SHIFT;

    return true;
}


void FirmwareFile::read_all(void* buf)
{
    uint8_t* dest = (uint8_t*)buf;
    ::read_all(_fd, 0, dest, _file_size);
    memset(dest + _file_size, 0, (_num_pages << GUEST_PAGE_SHIFT) - _file_size);
}

