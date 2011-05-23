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

#ifndef _H_ADMIN_COMMON
#define _H_ADMIN_COMMON

#include <stdarg.h>

#include "common.h"
#include "admin.h"
#include "threads.h"

typedef std::vector<uint32_t> va_type_list_t;
typedef std::vector<const char*> va_names_list_t;


class AdminBuf: public NonCopyable {
public:
    AdminBuf(uint size)
        : _size (size)
        , _data (new uint8_t[size])
    {
    }

    ~AdminBuf()
    {
        delete[] _data;
    }

    uint size() { return _size;}
    uint8_t* data() { return _data;}

private:
    uint _size;
    uint8_t* _data;
};

class ArgSource;

class AdminCommand: public NonCopyable {
public:
    AdminCommand(uint command_code, const std::string& name,
                 const std::string& description,
                 const std::string& help,
                 const va_type_list_t& input_list,
                 const va_names_list_t& input_names,
                 const va_type_list_t& output_list,
                 const va_names_list_t& output_names);

    AdminCommand* ref() {_refs.inc(); return this;}
    void unref() { if (!_refs.dec()) delete this;}

    static bool is_valid_name(const std::string& name);

    const char* get_name() { return _name.c_str();}
    const char* get_description() { return _description.c_str();}
    const char* get_help() { return _help.c_str();}
    uint get_command_code() { return _command_code;}
    const va_type_list_t& get_input_list() { return _input_list;};
    const va_type_list_t& get_output_list() { return _output_list;};
    const va_names_list_t& get_input_names() { return _input_names;};
    const va_names_list_t& get_output_names() { return _output_names;};

    struct BadCommand {};

protected:
    virtual ~AdminCommand();

    uint get_fixed_output_size() { return _fixed_output_size;}
    uint get_fixed_input_size() { return _fixed_input_size;}
    uint num_output_variable_args() { return _output_var_args;}
    uint num_input_variable_args() { return _input_var_args;}
    uint64_t get_arg_val(uint type, uint& offset, uint8_t* data, uint size);
    void arg_clean_up(uint stop_item, uint64_t* vec, const va_type_list_t& arg_list);
    void build(ArgSource& source, const va_type_list_t& types, uint num_variable_args,
               uint8_t* ptr, uint32_t var_data_offset, AdminBuf** var_buf);
    bool execute(uint8_t* data, uint size, const va_type_list_t& arg_list,
                 uint64_t* vec, uint num_args, uint index, void* handler);
    void names_cleanup();

private:
    void init_io_params(const va_type_list_t& types, uint& fixed_size,
                        uint& var_args);
private:
    Atomic _refs;
    uint _command_code;
    std::string _name;
    std::string _description;
    std::string _help;
    va_type_list_t _input_list;
    va_names_list_t _input_names;
    va_type_list_t _output_list;
    va_names_list_t _output_names;
    uint _fixed_output_size;
    uint _output_var_args;
    uint _fixed_input_size;
    uint _input_var_args;
};


class AdminTransmitContext {
public:
    virtual void transmit(AdminBuf* buf) = 0;
};


class AdminReplyContext {
public:
    virtual void command_reply(...) = 0;
    virtual void command_error(VAErrorCode error, const std::string& error_str) = 0;
};


typedef void (*admin_command_handler_t)(void* opaque, AdminReplyContext* context, ...);

class AdminLocalCommand: public AdminCommand {
public:
    AdminLocalCommand(uint command_code, const std::string& name,
                      const std::string& description,
                      const std::string& help,
                      const va_type_list_t& input_list,
                      const va_names_list_t& input_names,
                      const va_type_list_t& output_list,
                      const va_names_list_t& output_names,
                      admin_command_handler_t handler,
                      void* opaque)
        : AdminCommand(command_code, name, description, help, input_list,
                       input_names, output_list, output_names)
        , _handler (handler)
        , _opaque (opaque)
    {
    }

    bool process_command(AdminReplyContext* context, uint8_t* data, uint size);
    void replyv(uint32_t serial, AdminTransmitContext* context, va_list ap);
    void reply(uint32_t serial, AdminTransmitContext* context, ...);

private:
    virtual ~AdminLocalCommand() {}

private:
    admin_command_handler_t _handler;
    void* _opaque;
};


typedef void (*admin_reply_handler_t)(void* opaque, ...);
typedef void (*admin_error_handler_t)(void* opaque, VAErrorCode code, const std::string& str);

class AdminRemoteCommand: public AdminCommand {
public:
    AdminRemoteCommand(uint command_code, const std::string& name,
                      const std::string& description,
                      const std::string& help,
                      const va_type_list_t& input_list,
                       const va_names_list_t& input_names,
                      const va_type_list_t& output_list,
                       const va_names_list_t& output_names,
                      admin_reply_handler_t reply_handler,
                      admin_error_handler_t error_handler,
                      void* opaque)
        : AdminCommand(command_code, name, description, help, input_list, input_names,
                       output_list, output_names)
        , _reply_handler (reply_handler)
        , _error_handler (error_handler)
        , _opaque (opaque)
    {
    }

    void callv(uint32_t serial, AdminTransmitContext* context, va_list ap);
    void call(uint32_t serial, AdminTransmitContext* context, ...);
    void calls(uint32_t serial, AdminTransmitContext* context, const std::string& args);
    bool process_reply(uint8_t* data, uint size);
    void report_error(VAErrorCode code, const std::string& str);
    void reply_to_string(va_list args, std::string& result);

private:
    virtual ~AdminRemoteCommand() {}

    void call_common(uint32_t serial, AdminTransmitContext* context, ArgSource& args);

private:
    admin_reply_handler_t _reply_handler;
    admin_error_handler_t _error_handler;
    void* _opaque;
};

#endif

