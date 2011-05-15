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

#ifndef _H_ADMIN_SERVER
#define _H_ADMIN_SERVER

#include "common.h"
#include "threads.h"
#include "admin_common.h"

class UDSListener;

class AdminServer : public NonCopyable {
public:
    AdminServer(const std::string& uds_name);
    ~AdminServer();

    void register_command(const std::string& name,
                          const std::string& description,
                          const std::string& help,
                          const va_type_list_t& input_list,
                          const va_type_list_t& output_list,
                          admin_command_handler_t proc, void* opaque);

    void command_reply(void* context, ...);
    void command_failure(void* context, VAErrorCode error, const std::string& error_str);

private:
    class Connection;
    class CommandHandler;

    void on_new_connection(int socket);
    void remove_connection(Connection* connection);

    AdminLocalCommand* get_handler(uint32_t command_code);
    AdminLocalCommand* find_handler(const std::string& name);
    void enum_commands(AdminReplyContext* context, uint32_t index);

private:
    std::auto_ptr<UDSListener> _listener;
    uint _next_command_code;

    typedef std::list<Connection*> ConectionsList;
    ConectionsList _connections;
    Mutex _connections_mutex;

    typedef std::list<AdminLocalCommand*> HandlersList;
    HandlersList _handlers;
    Mutex _handlers_mutex;

    friend class Connection;
};


#endif

