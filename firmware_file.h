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

#ifndef _H_FIRMWARE_FILE
#define _H_FIRMWARE_FILE

#include "common.h"
#include "non_copyable.h"

class FirmwareFile : public NonCopyable {
public:
    FirmwareFile();
    virtual ~FirmwareFile();

    bool open(const char* path);
    void read_all(void* buf);
    bool is_valid() { return _fd != -1;}
    uint num_pages() { return _num_pages;}

private:
    int _fd;
    uint _file_size;
    uint _num_pages;
};

#endif

