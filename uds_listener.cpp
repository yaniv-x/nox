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

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/fcntl.h>

#include "uds_listener.h"
#include "run_loop.h"

UDSListener::UDSListener(RunLoop& loop, const std::string& uds_file,
                         int_callback_t callback, void* opaque)
    : _name (uds_file)
    , _callback (callback)
    , _opaque (opaque)
{
    struct sockaddr_un address;

    if (uds_file.length() > sizeof(address.sun_path) - 1) {
        THROW("bad file name. max length is %d", sizeof(address.sun_path) - 1);
    }

    if (unlink(uds_file.c_str()) == -1 && errno != ENOENT) {
        THROW_SYS_ERROR("unlink failed");
    }

    _socket = socket(PF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);

    if (_socket == -1) {
        THROW_SYS_ERROR("socket failed");
    }

    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, uds_file.c_str());

    if (bind(_socket, (struct sockaddr*) &address, sizeof(address)) == -1) {
        THROW_SYS_ERROR("bind failed");
    }

    if (listen(_socket, 4) == -1) {
        THROW_SYS_ERROR("listen failed");
    }

    _acccept_event = loop.create_fd_event(_socket, (void_callback_t)&UDSListener::accept, this);

    ASSERT(_acccept_event);
}


UDSListener::~UDSListener()
{
    _acccept_event->destroy();
    close(_socket);
    unlink(_name.c_str());
}


void UDSListener::accept()
{
    int connection = ::accept(_socket, NULL, NULL);
    int flags;

    if (connection == -1) {
        W_MESSAGE("failed %d (%s)", errno, strerror(errno));
        return;
    }

    if ((flags = fcntl(connection, F_GETFL)) == -1 ||
        fcntl(connection, F_SETFL, flags | O_NONBLOCK) == -1) {
        W_MESSAGE("accept failed, %s", strerror(errno));
        close(connection);
        return;
    }

    _callback(_opaque, connection);
}

