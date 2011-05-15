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
#include <stdarg.h>

#include "admin_server.h"
#include "uds_listener.h"
#include "application.h"
#include "admin_common.h"


static AdminServer* server = NULL;

class AdminServer::Connection : public NonCopyable, public AdminReplyContext,
                                public AdminTransmitContext {
public:
    Connection(int socket)
        : _refs (1)
        , _socket (socket)
        , _event (application->create_fd_event(_socket,
                                               (void_callback_t)&Connection::on_event,
                                               this))
        , _recive_buf (new uint8_t[sizeof(LinkMessage)])
        , _recive_start (NULL)
        , _recive_end (NULL)
        , _transmit_start (NULL)
        , _transmit_end (NULL)
        , _active (false)
    {
        _recive_start = _recive_buf;
        _recive_end = _recive_buf + sizeof(LinkMessage);
        _recive_done = &Connection::handle_link;
    }

    Connection* ref() {_refs.inc(); return this;}
    void unref() { if (!_refs.dec()) delete this;}

    void detach()
    {
        Lock lock(_detach_mutex);

        if (!_event) {
            return;
        }

        shut_connection();

        _event->destroy();
        _event = NULL;

        lock.unlock();

        server->remove_connection(this);

        unref();
    }

    void command_reply(...)
    {
         va_list ap;
         va_start(ap, this);

        if (!_current_handler.get()) {
            E_MESSAGE("reply while no active command");
            shut_connection();
            return;
        } else {
            Lock lock(_transmit_mutex);
            _current_handler.get()->replyv(_command_serial, this, ap);
            _current_handler.release();
            lock.unlock();
            process_out();
        }

        va_end(ap);
        unref();
    }

    void command_error(VAErrorCode error_code, const std::string& error_str)
    {
        if (!_current_handler.get()) {
            E_MESSAGE("reply while no active command");
            shut_connection();
        } else {
            uint size = sizeof(MessageHeader) + sizeof(CommandError) +
                        (error_str.length() ? error_str.length() + 1 : 0);
            AdminBuf* buf = new AdminBuf(size);

            MessageHeader* header = (MessageHeader*)buf->data();
            header->type = VA_MESSAGE_TYPE_ERROR;
            header->size = size - sizeof(MessageHeader);
            CommandError* err = (CommandError*)(header + 1);
            err->command_serial = _command_serial;
            err->error_code = error_code;

            if (!error_str.length()) {
                err->message_string = 0;
            } else {
                err->message_string = sizeof(CommandError);
                strcpy((char *)err + err->message_string, error_str.c_str());
            }

            _current_handler.release();

            Lock lock(_transmit_mutex);
            transmit(buf);
            lock.unlock();
            process_out();
        }

        unref();
    }


private:
    ~Connection()
    {
        close(_socket);
        delete[] _recive_buf;

        while (!_transmit_list.empty()) {
             delete _transmit_list.front();
            _transmit_list.pop_front();
        }
    }

    void shut_connection()
    {
        shutdown(_socket, SHUT_RDWR);
    }

    void transmit(AdminBuf* buf)
    {
        ASSERT(buf);

        _transmit_list.push_back(buf);

        if (_transmit_start != _transmit_end) {
            return;
        }

        ASSERT(_transmit_list.size() == 1);

        _transmit_start = buf->data();
        _transmit_end = _transmit_start + buf->size();
    }

    void handle_link()
    {
        LinkMessage link = *(LinkMessage*)_recive_buf;
        delete[] _recive_buf;
        _recive_buf = NULL;

        if (link.magic != NOX_ADMIN_MAGIC) {
            shutdown(_socket, SHUT_RDWR);
            return;
        }

        AdminBuf* buf = new AdminBuf(sizeof(LinkReply));
        _transmit_list.push_back(buf);

        LinkReply* link_reply = (LinkReply*)buf->data();

        _transmit_start = buf->data();
        _transmit_end = _transmit_start + sizeof(LinkReply);

        link_reply->magic = NOX_ADMIN_MAGIC;
        link_reply->size = sizeof(LinkReply);
        link_reply->version = NOX_ADMIN_VERSION;
        link_reply->message_string = 0;

        if (link.version != NOX_ADMIN_VERSION) {
            link_reply->error = VA_ERROR_VERSION_MISSMATCH;
            _transmit_done = &Connection::shut_connection;
        } else {
            link_reply->error = VA_ERROR_OK;
            _transmit_done = &Connection::start_handle_commands;
        }
    }

    void start_handle_commands()
    {
        _active = true;
        start_recive();
    }

    void start_recive()
    {
        _recive_start = (uint8_t*)&_message_header;
        _recive_end = _recive_start + sizeof(_message_header);
        _recive_done = &Connection::handle_header;
    }

    void handle_header()
    {
        if (!_message_header.size) {
            E_MESSAGE("size iz zero");
            shutdown(_socket, SHUT_RDWR);
            return;
        }

        try {
            _recive_buf = new uint8_t[_message_header.size];
        } catch (...) {
            E_MESSAGE("alloc_failed");
            shutdown(_socket, SHUT_RDWR);
            return;
        }

        _recive_start = _recive_buf;
        _recive_end = _recive_buf + _message_header.size;
        _recive_done = &Connection::handle_message;
    }

    void handle_message()
    {
        uint8_t* buf = _recive_buf;
        _recive_buf = NULL;
        process_message(buf);
        start_recive();
    }

    void process_message(uint8_t* data)
    {
        AutoArray<uint8_t> auto_data(data);

        switch (_message_header.type) {
        case VA_MESSAGE_TYPE_COMMAND: {
            if (_current_handler.get()) {
                E_MESSAGE("recived new command while command executing is in progress");
                shut_connection();
                break;
            }

            if (_message_header.size < sizeof(Command)) {
                E_MESSAGE("invalid command message size %u", _message_header.size);
                shut_connection();
                break;
            }

            Command* cmd = (Command*)data;

            _current_handler.reset(server->get_handler(cmd->command_code));

            if (!_current_handler.get()) {
                E_MESSAGE("invalid command %u", cmd->command_code);
                shut_connection();
                break;
            }

            _command_serial = cmd->command_serial;
            if (!_current_handler->process_command(ref(), data + sizeof(Command),
                                                   _message_header.size - sizeof(Command))) {
                W_MESSAGE("command abort");
                unref();
                // todo: keep connection and send error
                shut_connection();
                break;
            }
            break;
        }
        case VA_MESSAGE_TYPE_REPLY:
            E_MESSAGE("recive unexpected reply");
            shut_connection();
            break;
        case VA_MESSAGE_TYPE_ERROR:
            E_MESSAGE("recive unexpected error");
            shut_connection();
            break;
        default:
            E_MESSAGE("invalid message type %u", _message_header.type);
            shut_connection();
        }
    }

    void on_event()
    {
        AutoRef<Connection> autoref(ref());
        process_in();
        process_out();
    }

    void process_in()
    {
        while (_recive_start != _recive_end) {
            ssize_t n = read(_socket, _recive_start, _recive_end - _recive_start);

            if (n <= 0) {

                if (n == 0) {
                    D_MESSAGE("connection closed");
                    detach();
                    return;
                }

                if (errno == EAGAIN) {
                    break;
                }

                if (errno == EINTR) {
                    continue;
                }

                D_MESSAGE("connection closed %d %s", errno, strerror(errno));
                detach();
                return;
            }

            if (_recive_start += n) {
                (this->*_recive_done)();
            }
        }
    }

    void process_out()
    {
        Lock lock(_transmit_mutex);

        while (_transmit_start != _transmit_end) {
            ssize_t n = write(_socket, _transmit_start, _transmit_end - _transmit_start);
            if (n <= 0) {

                if (n == 0) {
                    D_MESSAGE("connection closed");
                    detach();
                    return;
                }

                if (errno == EAGAIN) {
                    break;
                }

                if (errno == EINTR) {
                    continue;
                }

                D_MESSAGE("connection closed %d %s", errno, strerror(errno));
                detach();
                return;
            }

            if ((_transmit_start += n) == _transmit_end) {
                delete _transmit_list.front();
                _transmit_list.pop_front();

                if (!_transmit_list.empty()) {
                    _transmit_start = _transmit_list.front()->data();
                    _transmit_end = _transmit_start + _transmit_list.front()->size();
                }

                (this->*_transmit_done)();
            }
        }
    }

private:
    Atomic _refs;
    int _socket;
    FDEvent* _event;
    Mutex _detach_mutex;

    MessageHeader _message_header;
    uint8_t* _recive_buf;

    uint8_t* _recive_start;
    uint8_t* _recive_end;
    void (Connection::*_recive_done)();

    typedef std::list<AdminBuf*> TransmitList;
    TransmitList _transmit_list;
    uint8_t* _transmit_start;
    uint8_t* _transmit_end;
    void (Connection::*_transmit_done)();
    Mutex _transmit_mutex;

    bool _active;
    AutoRef<AdminLocalCommand> _current_handler;
    uint32_t _command_serial;
};


AdminServer::AdminServer(const std::string& uds_name)
    : _listener(new UDSListener(*application, uds_name,
                                (int_callback_t)&AdminServer::on_new_connection, this))
    , _next_command_code (VA_FIRST_USER_CMD)
{
    server = this;

    va_type_list_t input_args(1);
    input_args[0] = VA_UINT32_T;      // command index

    va_type_list_t output_args(6);
    output_args[0] = VA_UINT32_T;     // command id
    output_args[1] = VA_UTF8_T;       // command name
    output_args[2] = VA_UTF8_T;       // command description
    output_args[3] = VA_UTF8_T;       // command help
    output_args[4] = VA_UINT32V_T;    // inputs args
    output_args[5] = VA_UINT32V_T;    // outputs args

    AdminLocalCommand* command;
    command = new AdminLocalCommand(VA_CMD_ENUM_COMMANDS, "@unum-commands", "???", "???",
                                    input_args, output_args,
                                    (admin_command_handler_t)&AdminServer::enum_commands, this);
    _handlers.push_back(command);
}


AdminServer::~AdminServer()
{
    while (!_connections.empty()) {
        _connections.front()->detach();
    }
}


void AdminServer::enum_commands(AdminReplyContext* context, uint32_t index)
{
    Lock lock (_handlers_mutex);

    index++;

    if (index >= _handlers.size()) {
        std::vector<uint32_t> empty_vec(0);
        context->command_reply(0, "", "", "", &empty_vec, &empty_vec);
        return;
    }

    HandlersList::iterator iter = _handlers.begin();

    while (index--) {
        iter++;
    }

    AdminLocalCommand* handler = *iter;

    context->command_reply(handler->get_command_code(), handler->get_name(),
                           handler->get_description(), handler->get_help(),
                           &handler->get_input_list(), &handler->get_output_list());
}


void AdminServer::on_new_connection(int socket)
{
    Lock lock(_connections_mutex);
    _connections.push_back(new Connection(socket));
}


void AdminServer::remove_connection(Connection* connection)
{
    Lock lock(_connections_mutex);
    ConectionsList::iterator iter = _connections.begin();

    for (; iter != _connections.end(); iter++) {
        if ((*iter) == connection) {
            _connections.erase(iter);
            return;
        }
    }
}


AdminLocalCommand* AdminServer::find_handler(const std::string& name)
{
    HandlersList::iterator iter = _handlers.begin();

    for (; iter != _handlers.end(); iter++) {
        if ((*iter)->get_name() == name) {
            return *iter;
        }
    }

    return NULL;
}


AdminLocalCommand* AdminServer::get_handler(uint32_t command_code)
{
    Lock lock (_handlers_mutex);

    HandlersList::iterator iter = _handlers.begin();

    for (; iter != _handlers.end(); iter++) {
        if ((*iter)->get_command_code() == command_code) {
            return (AdminLocalCommand*)(*iter)->ref();
        }
    }

    return NULL;
}


void AdminServer::register_command(const std::string& name,
                                   const std::string& description,
                                   const std::string& help,
                                   const va_type_list_t& input_list,
                                   const va_type_list_t& output_list,
                                   admin_command_handler_t handler,
                                   void* opaque)
{
    Lock lock (_handlers_mutex);

    if (!AdminCommand::is_valid_name(name)) {
         D_MESSAGE("%s bad command_name", name.c_str());
         return;
    }

    if (find_handler(name)) {
        D_MESSAGE("%s exist", name.c_str());
        return;
    }

    _handlers.push_back(new AdminLocalCommand(_next_command_code++, name, description, help,
                                              input_list, output_list, handler, opaque));
}

