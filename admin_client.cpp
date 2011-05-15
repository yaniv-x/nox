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

#include <stdio.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <alloca.h>

#include "options_parser.h"
#include "admin.h"
#include "admin_common.h"


class AdminClient: public NonCopyable, public AdminTransmitContext {
public:
    AdminClient(const char* vm_name);
    ~AdminClient();

    void connect();
    void wait_reply();

    AdminRemoteCommand* get_remote_command(const std::string& name);
    uint32_t alloc_call_serial(AdminRemoteCommand* command);

private:
    std::string get_uds_file_name();
    void link();

    void read(void* dest, uint size);
    void write(const void* source, uint size);


    void enum_commands(AdminReplyContext* context, uint32_t index);
    void enum_commands_reply(uint32_t code, const char* name, const char* description,
                             const char* help, const va_type_list_t& inputs,
                             const va_type_list_t& outputs);

    void generic_call_reply_handler(...);
    void generic_call_error_handler(VAErrorCode code, const std::string& str);

    void refresh_commands_list();

    AdminRemoteCommand* get_remote_command(uint command_code);
    AdminLocalCommand* get_local_command(uint command_code);

    virtual void transmit(AdminBuf* buf);

private:
    std::string _vm_name;
    int _socket;

    typedef  std::list<AdminLocalCommand*> LocalCmdList;
    LocalCmdList _local_commands;

    typedef  std::list<AdminRemoteCommand*> RemoteCmdList;
    RemoteCmdList _remote_commands;

    typedef  std::list<AdminBuf*> TransmitList;
    TransmitList _transmit_list;

    AutoRef<AdminRemoteCommand> _active_call;
    uint32_t _call_serial;

    AutoRef<AdminLocalCommand> _active_command;
    uint32_t _command_serial;

    bool _enum_done;
};


AdminClient::AdminClient(const char* vm_name)
    : _vm_name (vm_name)
    , _socket (-1)
    , _call_serial (0)
{

    va_type_list_t input_args(1);
    input_args[0] = VA_UINT32_T;      // command index

    va_type_list_t output_args(6);
    output_args[0] = VA_UINT32_T;     // command id
    output_args[1] = VA_UTF8_T;       // command name
    output_args[2] = VA_UTF8_T;       // command description
    output_args[3] = VA_UTF8_T;       // command help
    output_args[4] = VA_UINT32V_T;    // inputs args
    output_args[5] = VA_UINT32V_T;    // outputs args

    AdminLocalCommand* local;

    local = new AdminLocalCommand(VA_CMD_ENUM_COMMANDS, "@enum-commands", "", "", input_args,
                                  output_args, (admin_command_handler_t)&AdminClient::enum_commands,
                                  this);
    _local_commands.push_back(local);

    AdminRemoteCommand* remote;

    admin_reply_handler_t reply_handler = &AdminClient::enum_commands_reply;
    admin_error_handler_t error_handler = &AdminClient::generic_call_error_handler;
    remote = new AdminRemoteCommand(VA_CMD_ENUM_COMMANDS, "@enum-commands", "", "", input_args,
                                    output_args, reply_handler, error_handler, this);
    _remote_commands.push_back(remote);
}


AdminClient::~AdminClient()
{
    if (_socket != -1) {
        close(_socket);
    }

    _active_command.reset(NULL);
    _active_call.reset(NULL);

    while (!_transmit_list.empty()) {
        delete _transmit_list.front();
        _transmit_list.pop_front();
    }

    while (!_local_commands.empty()) {
        _local_commands.front()->unref();
        _local_commands.pop_front();
    }

    while (!_remote_commands.empty()) {
        _remote_commands.front()->unref();
        _remote_commands.pop_front();
    }
}


uint32_t AdminClient::alloc_call_serial(AdminRemoteCommand* command)
{
    _active_call.reset((AdminRemoteCommand*)command->ref());
    return ++_call_serial;
}


void AdminClient::enum_commands(AdminReplyContext* context, uint32_t index)
{
    va_type_list_t empty_vec(0);

    if (!_active_command.get()) {
        THROW("no active command");
    }

    AutoRef<AdminLocalCommand> tmp(_active_command.release());
    tmp->reply(_command_serial, this, 0, "", "", "", &empty_vec, &empty_vec);
}


void AdminClient::enum_commands_reply(uint32_t code, const char* name, const char* description,
                                      const char* help, const va_type_list_t& inputs,
                                      const va_type_list_t& outputs)
{
    if (!code) {
        _enum_done = true;
        return;
    }

    AdminRemoteCommand* command;

    admin_reply_handler_t reply_handler = &AdminClient::generic_call_reply_handler;
    admin_error_handler_t err_handler = &AdminClient::generic_call_error_handler;

    command = new AdminRemoteCommand(code, name, description, help, inputs, outputs,
                                    reply_handler, err_handler, this);

    _remote_commands.push_back(command);
}


void AdminClient::generic_call_reply_handler(...)
{
    D_MESSAGE("");
}


void AdminClient::generic_call_error_handler(VAErrorCode code, const std::string& str)
{
    THROW("comand failed %u %s", code, str.c_str());
}


std::string AdminClient::get_uds_file_name()
{
    std::string admin_file;

    char* home_dir = getenv("HOME");

    if (!home_dir || strlen(home_dir) < 2) {
        return admin_file;
    }

    sprintf(admin_file, "%s%snox/active/%s.uds",
            home_dir,
            (home_dir[strlen(home_dir) - 1] != '/') ? "/" : "",
            _vm_name.c_str());

    return admin_file;
}


void AdminClient::read(void* dest, uint size)
{
    uint8_t* ptr = (uint8_t*)dest;

    while (size) {
        ssize_t n = ::read(_socket, ptr, size);

        if (n > 0) {
            size -= n;
            ptr += n;
            continue;
        }

        if (n == -1 && errno == EINTR) {
            continue;
        }

        THROW("failed");
    }
}


void AdminClient::write(const void* source, uint size)
{
    const uint8_t* ptr = (const uint8_t*)source;

    while (size) {
        ssize_t n = ::write(_socket, ptr, size);

        if (n > 0) {
            size -= n;
            ptr += n;
            continue;
        }

        if (n == -1 && errno == EINTR) {
            continue;
        }

        THROW("failed");
    }
}


void AdminClient::link()
{
    LinkMessage link_messsage;
    link_messsage.magic = NOX_ADMIN_MAGIC;
    link_messsage.version = NOX_ADMIN_VERSION;

    write(&link_messsage, sizeof(link_messsage));

    LinkReply reply;

    read(&reply, sizeof(reply));

    if (reply.magic != NOX_ADMIN_MAGIC || reply.version != NOX_ADMIN_VERSION ||
        reply.error != VA_ERROR_OK || reply.size < sizeof(LinkReply)) {
        THROW("failed");
    }

    uint more = reply.size - sizeof(LinkReply);

    if (more) {
        AutoArray<uint8_t> buf(new uint8_t[more]);
        read(buf.get(), sizeof(more));
    }
}


void AdminClient::transmit(AdminBuf* buf)
{
    _transmit_list.push_back(buf);
}


AdminRemoteCommand* AdminClient::get_remote_command(uint command_code)
{
    RemoteCmdList::iterator iter = _remote_commands.begin();

    for (; iter != _remote_commands.end(); iter++) {
        if ((*iter)->get_command_code() == command_code) {
            return (AdminRemoteCommand*)(*iter)->ref();
        }
    }

    return NULL;
}


AdminRemoteCommand* AdminClient::get_remote_command(const std::string& name)
{
    RemoteCmdList::iterator iter = _remote_commands.begin();

    for (; iter != _remote_commands.end(); iter++) {
        if ((*iter)->get_name() == name) {
            return (AdminRemoteCommand*)(*iter)->ref();
        }
    }

    return NULL;
}


AdminLocalCommand* AdminClient::get_local_command(uint command_code)
{
    LocalCmdList::iterator iter = _local_commands.begin();

    for (; iter != _local_commands.end(); iter++) {
        if ((*iter)->get_command_code() == command_code) {
            return (AdminLocalCommand*)(*iter)->ref();
        }
    }

    return NULL;
}


void AdminClient::wait_reply()
{
    for (;;) {
        while (!_transmit_list.empty()) {
            std::auto_ptr<AdminBuf> buf(_transmit_list.front());
            _transmit_list.pop_front();
            write(buf->data(), buf->size());
        }

        MessageHeader header;

        read(&header, sizeof(header));

        AutoArray<uint8_t> body(new uint8_t[header.size]);

        read(body.get(), header.size);

        switch (header.type) {
        case VA_MESSAGE_TYPE_COMMAND: {
            if (_active_command.get()) {
                THROW("MESSAGE_TYPE_COMMAND: command in progess");
            }

            if (header.size < sizeof(Command)) {
                THROW("bad command");
            }

            Command* command = (Command*)body.get();

            _active_command.reset(get_local_command(command->command_code));

            if (!_active_command.get()) {
                THROW("MESSAGE_TYPE_COMMAND: command not found");
            }

            _command_serial = command->command_serial;


            _active_command->process_command(NULL, (uint8_t*)(command + 1),
                                             header.size - sizeof(Command));

            break;
        }
        case VA_MESSAGE_TYPE_REPLY: {
            if (!_active_call.get()) {
                THROW("MESSAGE_TYPE_REPLY: no active call");
            }

            if (header.size < sizeof(CommandReply)) {
                THROW("bad reply");
            }

            CommandReply* reply = (CommandReply*)body.get();

            if (reply->command_serial != _call_serial) {
                THROW("incorrect serial");
            }

            _active_call->process_reply((uint8_t*)(reply + 1), header.size - sizeof(CommandReply));
            _active_call.reset(NULL);
            return;
        }
        case VA_MESSAGE_TYPE_ERROR:
            if (!_active_call.get()) {
                THROW("VA_MESSAGE_TYPE_ERROR: no active call");
            }

            if (header.size < sizeof(CommandError)) {
                THROW("bad error message");
            }

            CommandError* err = (CommandError*)body.get();

            if (err->command_serial != _call_serial) {
                THROW("incorrect serial");
            }

            std::string error_str;

            if (err->message_string) {
                char *start = (char *)err + err->message_string;
                char *now = start;
                char *end = (char*)body.get() + header.size;

                for (;; now++) {
                    if (now >= end) {
                        THROW("bad error message");
                    }

                    if (*now == 0) {
                        break;
                    }
                }

                error_str = start;
            } else {
                error_str = "";
            }

            _active_call->report_error((VAErrorCode)err->error_code, error_str);

            return;
        }
    }
}


void AdminClient::refresh_commands_list()
{
    if (_active_call.get()) {
        THROW("command in progress");
    }

    AutoRef<AdminRemoteCommand> command(get_remote_command(VA_CMD_ENUM_COMMANDS));

    if (!command.get()) {
        THROW("command not found");
    }

    uint32_t index = 0;
    _enum_done = false;

    do {
        _active_call->call(alloc_call_serial(command.get()), this, index++);
        wait_reply();
    } while (!_enum_done);
}


void AdminClient::connect()
{
    std::string file_name = get_uds_file_name();
    struct sockaddr_un address;

    if (file_name.length() == 0 || file_name.length() + 1 > sizeof(address.sun_path)) {
        THROW("invalid file name");
    }

    _socket = socket(PF_UNIX, SOCK_STREAM, 0);

    if(_socket == -1) {
        THROW_SYS_ERROR("create socket failed");
    }

    address.sun_family = AF_UNIX;

    strcpy(address.sun_path, file_name.c_str());

    if(::connect(_socket, (struct sockaddr *) &address, sizeof(address)) != 0) {
        THROW_SYS_ERROR("connect failed");
    }

    link();
    refresh_commands_list();
}


int main(int argc, const char** argv)
{
    OptionsParser parser;

    parser.set_front_positional_minmax(1, ~0);

    if (!parser.parse(argc, argv)) {
        return -1;
    }

    int option;
    const char* arg;
    const char* vm_name = NULL;
    const char* command_name = NULL;
    std::string command_args;

    while ((option = parser.next(&arg)) != OptionsParser::OPT_ID_DONE) {
        switch (option) {
        case OptionsParser::OPT_ID_FRONT_POSITIONAL:
            if (vm_name == NULL) {
                vm_name = arg;
            } else if (command_name == NULL) {
                command_name = arg;
            } else {
                command_args += " ";
                command_args += arg;
            }

            break;
        case OptionsParser::OPT_ID_HELP:
            parser.help();
            return 0;
        }
    }

    AdminClient client(vm_name);

    try {
        client.connect();
    } catch (std::exception& e) {
        E_MESSAGE("%s", e.what());
        return -1;
    } catch (...) {
        return -1;
    }

    if (command_name) {
        AutoRef<AdminRemoteCommand> command(client.get_remote_command(command_name));

        if (!command.get()) {
            printf("command \"%s\" not found\n", command_name);
        } else {
            command->calls(client.alloc_call_serial(command.get()), &client, command_args);
            client.wait_reply();
        }
    }

    return 0;
}

