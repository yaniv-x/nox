/*
    Copyright (c) 2013-2017 Yaniv Kamay,
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

#include <stdarg.h>
#include <sstream>

#include "admin_common.h"
#include "dynamic_call.h"


AdminCommand::AdminCommand(uint command_code, const std::string& name,
                           const std::string& description,
                           const std::string& help,
                           const va_type_list_t& input_list,
                           const va_names_list_t& input_names,
                           const va_type_list_t& output_list,
                           const va_names_list_t& output_names)
    : _refs (1)
    , _command_code (command_code)
    , _name (name)
    , _description (description)
    , _help (help)
    , _input_list (input_list)
    , _output_list (output_list)
{
    init_io_params(_output_list, _fixed_output_size, _output_var_args);
    init_io_params(_input_list, _fixed_input_size, _input_var_args);

    if (input_list.size() != input_names.size() || output_list.size() != output_names.size()) {
        THROW("invalid args");
    }

    try {
        for (int i = 0; i < input_names.size(); i++) {
            _input_names.push_back(copy_cstr(input_names[i]));
        }

        for (int i = 0; i < output_names.size(); i++) {
            _output_names.push_back(copy_cstr(output_names[i]));
        }
    } catch (...) {
        names_cleanup();
    }
}


void AdminCommand::names_cleanup()
{
    while (!_input_names.empty()) {
        delete[] _input_names.back();
        _input_names.pop_back();
    }

    while (!_output_names.empty()) {
        delete[] _output_names.back();
        _output_names.pop_back();
    }
}

AdminCommand::~AdminCommand()
{
    names_cleanup();
}


void AdminCommand::init_io_params(const va_type_list_t& types, uint& fixed_size,
                                   uint& var_args)
{
    fixed_size = 0;
    var_args = 0;

    va_type_list_t::const_iterator iter = types.begin();

    for (; iter != types.end(); iter++) {
        switch (*iter) {
        case VA_UINT32_T:
        case VA_INT32_T:
            fixed_size += sizeof(uint32_t);
            break;
        case VA_UINT64_T:
        case VA_INT64_T:
            fixed_size += sizeof(uint64_t);
            break;
        case VA_UINT8_T:
        case VA_INT8_T:
            fixed_size += sizeof(uint8_t);
            break;
        case VA_UTF8_T:
            fixed_size += sizeof(uint16_t);
            var_args++;
            break;
        case VA_UINT32V_T:
            fixed_size += 2 * sizeof(uint16_t);
            var_args++;
            break;
        case VA_UTF8V_T:
            fixed_size += 2 * sizeof(uint16_t);
            var_args++;
            break;
        default:
            PANIC("invalid type");
        }
    }
}


static const char* get_str(uint& offset, uint8_t* data, uint size)
{
    const char* str = (const char*)data + offset;

    for (;;) {
        if (offset >= size) {
            throw AdminCommand::BadCommand();
        }

        if (data[offset++] == 0) {
            break;
        }
    }

    return copy_cstr(str);
}

uint64_t AdminCommand::get_arg_val(uint type, uint& offset, uint8_t* data, uint size)
{
    uint max_size = size - offset;
    uint64_t ret;

    switch (type) {
    case VA_UINT32_T:
    case VA_INT32_T:
        if (max_size < sizeof(uint32_t)) {
            throw BadCommand();
        }
        ret = *(uint32_t*)(data + offset);
        offset += sizeof(uint32_t);
        break;
    case VA_UINT64_T:
    case VA_INT64_T:
        if (max_size < sizeof(uint64_t)) {
            throw BadCommand();
        }
        ret = *(uint64_t*)(data + offset);
        offset += sizeof(uint64_t);
        break;
    case VA_UINT8_T:
    case VA_INT8_T:
        if (max_size < sizeof(uint8_t)) {
            throw BadCommand();
        }
        ret = *(data + offset);
        offset += sizeof(uint8_t);
        break;
    case VA_UTF8_T: {
        if (max_size < sizeof(uint16_t)) {
            throw BadCommand();
        }

        uint str_offset = *(uint16_t*)(data + offset);
        offset += sizeof(uint16_t);

        if (!str_offset) {
            throw BadCommand();
        }

        ret = (uint64_t)get_str(str_offset, data, size);
        break;
    }
    case VA_UINT32V_T: {
        if (size < sizeof(uint32_t)) {
            throw BadCommand();
        }

        uint data_offset = *(uint16_t*)(data + offset);
        offset += sizeof(uint16_t);

        if (!data_offset) {
            throw BadCommand();
        }

        uint len = *(uint16_t*)(data + offset);
        offset += sizeof(uint16_t);

        if ((data_offset + len * sizeof(uint32_t)) > size) {
            throw BadCommand();
        }

        std::vector<uint32_t>* v = new std::vector<uint32_t>(len);
        memcpy(&(*v)[0], data + data_offset, len * sizeof(uint32_t));
        ret = (uint64_t)v;
        break;
    }
    case VA_UTF8V_T: {
        if (size < sizeof(uint32_t)) {
            throw BadCommand();
        }

        uint data_offset = *(uint16_t*)(data + offset);
        offset += sizeof(uint16_t);

        if (!data_offset) {
            throw BadCommand();
        }

        uint len = *(uint16_t*)(data + offset);
        offset += sizeof(uint16_t);

        std::vector<const char*>* v = new std::vector<const char*>(len);

        for (int i = 0; i < len; i++) {
            (*v)[i] = get_str(data_offset, data, size);
        }

        ret = (uint64_t)v;
        break;
    }
    default:
        PANIC("invalid type");
    }

    return ret;
}


void AdminCommand::arg_clean_up(uint stop_item, uint64_t* vec, const va_type_list_t& arg_list)
{
    for (uint i = 0; i != stop_item; i++) {
        uint type = arg_list[i];
        switch (type) {
        case VA_UINT32_T:
        case VA_INT32_T:
        case VA_UINT64_T:
        case VA_INT64_T:
        case VA_UINT8_T:
        case VA_INT8_T:
            break;
        case VA_UTF8_T:
            delete[] (char*)vec[i];
            break;
        case VA_UINT32V_T:
            delete (std::vector<uint32_t>*)vec[i];
            break;
        case VA_UTF8V_T:
            std::vector<const char*>* str_vec = (std::vector<const char*>*)vec[i];
            while (!str_vec->empty()) {
                delete[] str_vec->back();
                str_vec->pop_back();
            }
            delete str_vec;
            break;
        }
    }
}


bool AdminCommand::execute(uint8_t* data, uint size, const va_type_list_t& arg_list,
                           uint64_t* vec, uint num_args, uint index, void* handler)
{
    uint offset = index;

    try {
        uint data_offset = 0;

        for (; index < num_args; index++) {
            uint type = arg_list[index - offset];
            vec[index] = get_arg_val(type, data_offset, data, size);
        }

        uint num_reg_args = (num_args > 6) ? 6 : num_args;
        dynamic_sys_v_amd64_call(handler, num_reg_args, vec,
                                 num_args - num_reg_args,
                                 vec + 6);
        arg_clean_up(index - offset, vec + offset, arg_list);
    } catch (...) {
        arg_clean_up(index - offset, vec + offset, arg_list);
        return false;
    }

    return true;
}


bool AdminCommand::is_valid_name(const std::string& name)
{
    if (!name.length() || !isalnum(name[0])) {
        return false;
    }

    for (int i = 1; i < name.length(); i++) {
        if (!isalnum(name[i]) && name[i] != '.' &&  name[i] != '-') {
            return false;
        }
    }

    return true;
}


class ArgSource {
public:
    virtual const char* get_utf8() = 0;
    virtual uint32_t get_uint32() = 0;
    virtual uint64_t get_uint64() = 0;
    virtual uint8_t get_uint8() = 0;
    virtual const std::vector<uint32_t>* get_uint32v() = 0;
    virtual const std::vector<const char*>* get_utf8v() = 0;
};


class StdargSource: public ArgSource {
public:
    StdargSource(va_list ap)
    {
        va_copy(_ap, ap);
    }

    virtual const char* get_utf8()
    {
        return va_arg(_ap, const char*);
    }

    virtual uint32_t get_uint32()
    {
        return va_arg(_ap, uint32_t);
    }

    virtual uint64_t get_uint64()
    {
        return va_arg(_ap, uint64_t);
    }

    virtual uint8_t get_uint8()
    {
        return va_arg(_ap, int);/* uint8_t is promoted to int*/
    }

    virtual const std::vector<uint32_t>* get_uint32v()
    {
        return va_arg(_ap, const std::vector<uint32_t>*);
    }

    virtual const std::vector<const char*>* get_utf8v()
    {
        return va_arg(_ap, const std::vector<const char*>*);
    }

private:
    va_list _ap;
};


struct VariableInfo {
    VariableInfo()
        : depth (NULL)
    {
    }

    uint to;
    const void* from;
    uint length;
    uint depth_count;
    VariableInfo* depth;
};


static void var_info_cleanup(VariableInfo *info)
{
    if (!info) {
        return;
    }

    var_info_cleanup(info->depth);
    delete[] info;
}


static void copy_var_data(VariableInfo *info, uint n, uint8_t* dest_base)
{
    VariableInfo* end = info + n;

    for (; info < end; info++) {
        memcpy(dest_base + info->to, info->from, info->length);

        if (info->depth) {
            copy_var_data(info->depth, info->depth_count, dest_base);
        }
    }
}


void AdminCommand::build(ArgSource& source, const va_type_list_t& types, uint num_variable_args,
                         uint8_t* ptr, uint32_t var_data_offset, AdminBuf** var_buf)
{
    VariableInfo* var_vec = new VariableInfo[num_variable_args];

    try {
        uint32_t variable_data_size = 0;
        uint32_t var_index = 0;

        va_type_list_t::const_iterator iter = types.begin();

        for (; iter != types.end(); iter++) {
            switch (*iter) {
            case VA_UINT32_T:
            case VA_INT32_T: {
                *(uint32_t*)ptr = source.get_uint32();
                ptr += sizeof(uint32_t);
                break;
            }
            case VA_UINT64_T:
            case VA_INT64_T: {
                *(uint64_t*)ptr = source.get_uint64();
                ptr += sizeof(uint64_t);
                break;
            }
            case VA_UTF8_T:
                var_vec[var_index].from = source.get_utf8();

                if (!var_vec[var_index].from) {
                    PANIC("null string");
                }

                *(uint16_t*)ptr = variable_data_size + var_data_offset;
                ptr += sizeof(uint16_t);
                var_vec[var_index].to = variable_data_size;
                var_vec[var_index].length = strlen((char*)var_vec[var_index].from) + 1;
                variable_data_size += var_vec[var_index].length;
                var_index++;
                break;
            case VA_UINT8_T:
            case VA_INT8_T:
                *(uint8_t*)ptr = source.get_uint8();
                ptr += sizeof(uint8_t);
                break;
            case VA_UINT32V_T: {
                const std::vector<uint32_t>* v = source.get_uint32v();

                if (!v) {
                    PANIC("null vector");
                }

                var_vec[var_index].from = &(*v)[0];
                *(uint16_t*)ptr = variable_data_size + var_data_offset;
                ptr += sizeof(uint16_t);
                *(uint16_t*)ptr = v->size();
                ptr += sizeof(uint16_t);
                var_vec[var_index].to = variable_data_size;
                var_vec[var_index].length = v->size() * sizeof(uint32_t);
                variable_data_size += var_vec[var_index].length;
                var_index++;
                break;
            }
            case VA_UTF8V_T: {
                const std::vector<const char*>* v = source.get_utf8v();

                if (!v) {
                    PANIC("null vector");
                }

                var_vec[var_index].from = NULL;
                *(uint16_t*)ptr = variable_data_size + var_data_offset;
                ptr += sizeof(uint16_t);
                *(uint16_t*)ptr = v->size();
                ptr += sizeof(uint16_t);
                var_vec[var_index].to = 0;
                var_vec[var_index].length = 0;
                var_vec[var_index].depth_count = v->size();
                var_vec[var_index].depth = new VariableInfo[v->size()];

                for (int i = 0; i < var_vec[var_index].depth_count; i++) {
                    const char* str = (*v)[i];
                    VariableInfo* inf = &var_vec[var_index].depth[i];

                    if (!str) {
                        PANIC("null string");
                    }

                    inf->from = str;
                    inf->to = variable_data_size;
                    inf->length = strlen(str) + 1;
                    variable_data_size += inf->length;
                }

                var_index++;
                break;
            }
            default:
                PANIC("invalid type");
            }
        }

        if (!variable_data_size) {
            *var_buf = NULL;
            return;
        }

        if (variable_data_size > (1 << 16) - var_data_offset) {
            THROW("overflow");
        }

        *var_buf = new AdminBuf(variable_data_size);
        copy_var_data(var_vec, var_index, (*var_buf)->data());
    } catch (...) {
        var_info_cleanup(var_vec);
        throw;
    }

    var_info_cleanup(var_vec);
}


bool AdminLocalCommand::process_command(AdminReplyContext* context, uint8_t* data, uint size)
{
    //Lock lock(_mutex);
    // if (!_active) {
    // return false
    //}

    uint num_args = get_input_list().size() + 2;
    AutoArray<uint64_t> args(new uint64_t[num_args]);
    uint64_t* vec = args.get();
    vec[0] = (uint64_t)_opaque;
    vec[1] = (uint64_t)context;

    return execute(data, size, get_input_list(), vec, num_args, 2, (void*)_handler);
}


void AdminLocalCommand::replyv(uint32_t serial, AdminTransmitContext* context, va_list ap)
{
    uint fixed_output_size = get_fixed_output_size();
    uint buf_size = sizeof(VAMessageHeader) + sizeof(VACommandReply) + fixed_output_size;
    std::unique_ptr<AdminBuf> buf(new AdminBuf(buf_size));

    VAMessageHeader* header = (VAMessageHeader*)buf->data();
    header->type = VA_MESSAGE_TYPE_REPLY;
    header->size = buf_size - sizeof(VAMessageHeader);
    VACommandReply* reply = (VACommandReply*)(header + 1);
    reply->command_serial = serial;

    AdminBuf* var_buf;
    StdargSource arg_source(ap);

    build(arg_source, get_output_list(), num_output_variable_args(), (uint8_t*)(reply + 1),
          buf->size() - sizeof(VAMessageHeader) - sizeof(VACommandReply), &var_buf);

    if (var_buf) {
        std::unique_ptr<AdminBuf> tmp(var_buf);

        header->size += var_buf->size();
        context->transmit(buf.release());
        context->transmit(tmp.release());

        return;
    }

    context->transmit(buf.release());
}


void AdminLocalCommand::reply(uint32_t serial, AdminTransmitContext* context, ...)
{
    va_list ap;
    va_start(ap, context);
    replyv(serial, context, ap);
    va_end(ap);
}


void AdminRemoteCommand::call_common(uint32_t serial, AdminTransmitContext* context,
                                     ArgSource& args)
{
    uint fixed_input_size = get_fixed_input_size();
    uint buf_size = sizeof(VAMessageHeader) + sizeof(VACommand) + fixed_input_size;
    std::unique_ptr<AdminBuf> buf(new AdminBuf(buf_size));

    VAMessageHeader* header = (VAMessageHeader*)buf->data();
    header->type = VA_MESSAGE_TYPE_COMMAND;
    header->size = buf_size - sizeof(VAMessageHeader);
    VACommand* command = (VACommand*)(header + 1);
    command->command_code = get_command_code();
    command->command_serial = serial;

    AdminBuf* var_buf = NULL;

    build(args, get_input_list(), num_input_variable_args(), (uint8_t*)(command + 1),
          buf->size() - sizeof(VAMessageHeader) - sizeof(VACommand), &var_buf);

    if (var_buf) {
        std::unique_ptr<AdminBuf> tmp(var_buf);

        header->size += var_buf->size();
        context->transmit(buf.release());
        context->transmit(tmp.release());

        return;
    }

    context->transmit(buf.release());
}


void AdminRemoteCommand::callv(uint32_t serial, AdminTransmitContext* context, va_list ap)
{
    StdargSource arg_source(ap);
    call_common(serial, context, arg_source);
}


void AdminRemoteCommand::call(uint32_t serial, AdminTransmitContext* context, ...)
{
    va_list ap;
    va_start(ap, context);
    StdargSource arg_source(ap);
    call_common(serial, context, arg_source);
    va_end(ap);
}


static void conv_u32(const char* str, uint64_t &bin_val)
{
    char* end;
    uint64_t ret = strtoul(str, &end, 0);

    if (ret > UINT32_MAX  || *end) {
        THROW("conv failed");
    }

    bin_val = ret;
}


static void conv_i32(const char* str, uint64_t &bin_val)
{
    char* end;
    int64_t ret = strtol(str, &end, 0);

    if (ret > INT32_MAX || ret < INT32_MIN || *end) {
        THROW("conv failed");
    }

    bin_val = ret;
}


static void conv_u64(const char* str, uint64_t &bin_val)
{
    char* end;
    uint64_t ret = strtoul(str, &end, 0);

    if (*end || errno == ERANGE) {
        THROW("conv failed");
    }

    bin_val = ret;
}


static void conv_i64(const char* str, uint64_t &bin_val)
{
    char* end;
    int64_t ret = strtol(str, &end, 0);

    if (*end || errno == ERANGE) {
        THROW("conv failed");
    }

    bin_val = ret;
}


static void conv_u8(const char* str, uint64_t &bin_val)
{
    char* end;
    uint64_t ret = strtoul(str, &end, 0);

    if (ret > UINT8_MAX  || *end) {
        THROW("conv failed");
    }

    bin_val = ret;
}


static void conv_i8(const char* str, uint64_t &bin_val)
{
    char* end;
    int64_t ret = strtol(str, &end, 0);

    if (ret > INT8_MAX || ret < INT8_MIN || *end) {
        THROW("conv failed");
    }

    bin_val = ret;
}


static const char* skip_blank(const char* str)
{
    for (; *str && isblank(*str); str++);

    return str;
}


static void conv_uint32v(const char* str, uint64_t &bin_val)
{
    std::unique_ptr<std::vector<uint32_t> > array( new std::vector<uint32_t>());

    str = skip_blank(str);

    if (!*str) {
        bin_val = (uint64_t)array.release();
        return;

    }

    for (uint i = 0; ; i++, str++) {

        char* el_end;
        uint64_t el_val = strtoul(str, &el_end, 0);

        if (el_end == str || el_val > UINT32_MAX) {
            THROW("conv failed");
        }

        array->push_back(el_val);

        if (!*(str = skip_blank(el_end))) {
            break;
        }

        if (*str != ',') {
            THROW("conv failed");
        }
    }

    bin_val = (uint64_t)array.release();
}

typedef int (*run_stop_test_t)(char ch);


static int default_stop_test(char ch)
{
    return isblank(ch);
}

static const char* get_next_run(const char* str, std::string& next,
                                run_stop_test_t test = default_stop_test)
{
    str = skip_blank(str);

    if (!*str) {
        THROW("failed");
    }

    const char* start = str;

    for (; *str && !test(*str); str++);

    next.assign((char*)(start), str - start);

    return str;
}


static const char* get_next_string(const char* str, std::string& next,
                                   run_stop_test_t test = default_stop_test)
{
    str = skip_blank(str);

    if (*str != '"') {
        return get_next_run(str, next, test);
    }

    const char* start = ++str;

    for (;; str++) {
        if (*str == 0) {
            THROW("failed");
        }

        if (*str != '"') {
            continue;
        }

        uint slash_count = 0;

        for (const char* back = str - 1; back >= start && *back == '\\'; slash_count++, back--);

        if (!(slash_count & 1)) {
            break;
        }
    }

    next.assign((char*)start, str - start);

    if (!isblank(*++str) && *str != 0) {
        THROW("failed");
    }

    return str;
}


static const char* get_next_array(const char* str, std::string& next)
{
    str = skip_blank(str);

    if (*str != '[') {
        THROW("failed");
    }

    str++;

    const char* end = strchr(str, ']');

    if (!end || (*(end + 1) && !isblank(*(end + 1)))) {
        THROW("failed");
    }

    next.assign((char*)str, end - str);

    return end + 1;
}


static int utf8v_stop_test(char ch)
{
    return ch == ',' || isblank(ch);
}

static void conv_utf8v(const char* str, uint64_t &bin_val)
{
    std::unique_ptr<std::vector<const char*> > array( new std::vector<const char*>());

    str = skip_blank(str);

    if (!*str) {
        bin_val = (uint64_t)array.release();
        return;

    }

    for (uint i = 0; ; i++, str++) {
        std::string val;

        str = get_next_string(str, val, utf8v_stop_test);
        array->push_back(copy_cstr(val.c_str()));

        if (!*(str = skip_blank(str))) {
            break;
        }

        if (*str != ',') {
            THROW("conv failed");
        }
    }

    bin_val = (uint64_t)array.release();
}


class InternalSource: public ArgSource {
public:
    InternalSource(uint64_t* vec)
        : _vec (vec)
        , _pos (0)
    {
    }

    virtual const char* get_utf8()
    {
        return (const char*)_vec[_pos++];
    }

    virtual uint32_t get_uint32()
    {
        return (uint32_t)_vec[_pos++];
    }

    virtual uint64_t get_uint64()
    {
        return _vec[_pos++];
    }

    virtual uint8_t get_uint8()
    {
        return (uint8_t)_vec[_pos++];
    }

    virtual const std::vector<uint32_t>* get_uint32v()
    {
        return (const std::vector<uint32_t>*)_vec[_pos++];
    }

    virtual const std::vector<const char*>* get_utf8v()
    {
        return (const std::vector<const char*>*)_vec[_pos++];
    }

private:
    uint64_t* _vec;
    uint _pos;
};


void AdminRemoteCommand::calls(uint32_t serial, AdminTransmitContext* context,
                               const std::string& args)
{
    AutoArray<uint64_t> bin_args(new uint64_t[get_input_list().size()]);
    const char* str = args.c_str();
    std::string val;
    uint i = 0;

    try {
        for (i = 0; i < get_input_list().size(); i++) {
            switch (get_input_list()[i]) {
            case VA_UINT32_T:
                str = get_next_run(str, val);
                conv_u32(val.c_str(), bin_args[i]);
                break;
            case VA_INT32_T:
                str = get_next_run(str, val);
                conv_i32(val.c_str(), bin_args[i]);
                break;
            case VA_UINT64_T:
                str = get_next_run(str, val);
                conv_u64(val.c_str(), bin_args[i]);
                break;
            case VA_INT64_T:
                str = get_next_run(str, val);
                conv_i64(val.c_str(), bin_args[i]);
                break;
            case VA_UINT8_T:
                str = get_next_run(str, val);
                conv_u8(val.c_str(), bin_args[i]);
                break;
            case VA_INT8_T:
                str = get_next_run(str, val);
                conv_i8(val.c_str(), bin_args[i]);
                break;
            case VA_UTF8_T: {
                str = get_next_string(str, val);
                char* tmp = new char[val.length() + 1];
                strcpy(tmp, val.c_str());
                bin_args[i] = (uint64_t)tmp;
                break;
            }
            case VA_UINT32V_T:
                str = get_next_array(str, val);
                conv_uint32v(val.c_str(), bin_args[i]);
                break;
            case VA_UTF8V_T:
                str = get_next_array(str, val);
                conv_utf8v(val.c_str(), bin_args[i]);
                break;
            }
        }

        str = skip_blank(str);

        if (*str != 0) {
            THROW("invalid arg list");
        }

        InternalSource arg_source(bin_args.get());

        call_common(serial, context, arg_source);


        arg_clean_up(i, bin_args.get(), get_input_list());
    } catch (...) {
        arg_clean_up(i, bin_args.get(), get_input_list());
        throw;
    }
}


bool AdminRemoteCommand::process_reply(uint8_t* data, uint size)
{
    //Lock lock(_mutex);
    // if (!_active) {
    // return false
    //}

    uint num_args = get_output_list().size() + 1;
    AutoArray<uint64_t> args(new uint64_t[num_args]);
    uint64_t* vec = args.get();
    vec[0] = (uint64_t)_opaque;

    return execute(data, size, get_output_list(), vec, num_args, 1, (void*)_reply_handler);
}


void AdminRemoteCommand::report_error(VAErrorCode code, const std::string& str)
{
    //Lock lock(_mutex);
    // if (!_active) {
    // return false
    //}

    _error_handler(_opaque, code, str);
}


void AdminRemoteCommand::reply_to_string(va_list args, std::string& result)
{
    StdargSource source(args);
    std::ostringstream os;

    int len = get_output_list().size();

    if (!len) {
        return;
    }

    for (int i = 0; ;) {
        switch (get_output_list()[i]) {
        case VA_UINT32_T:
            os << source.get_uint32();
            break;
        case VA_INT32_T:
            os << int32_t(source.get_uint32());
            break;
        case VA_UINT64_T:
            os << source.get_uint64();
            break;
        case VA_INT64_T:
            os << source.get_uint64();
            break;
        case VA_UINT8_T:
            os << source.get_uint8();
            break;
        case VA_INT8_T:
            os << int8_t(source.get_uint8());
            break;
        case VA_UTF8_T:
            os << "\"";
            os << source.get_utf8();
            os << "\"";
            break;
        case VA_UINT32V_T: {
            os << "[";
            const std::vector<uint32_t>* v = source.get_uint32v();
            uint v_size = v->size();

            if (v_size) {
                for (uint j = 0;  ;) {
                    os << (*v)[j];

                    if (++j == v_size) {
                        break;
                    }

                    os << ", ";
                }
            }

            os << "]";
            break;
        }
        case VA_UTF8V_T: {
            os << "[";
            const std::vector<const char*>* v = source.get_utf8v();
            uint v_size = v->size();

            if (v_size) {
                for (uint j = 0;  ;) {
                    os << "\"";
                    os << (*v)[j];
                    os << "\"";

                    if (++j == v_size) {
                        break;
                    }

                    os << ", ";
                }
            }

            os << "]";
            break;
        }
        default:
            PANIC("invalid type");
        }

        if (++i == len) {
            break;
        }

        os << " ";
    }

    result = os.str();
}


va_type_list_t admin_types(uint argc, ...)
{
    va_type_list_t list(argc);
    va_list ap;

    va_start(ap, argc);

    for (uint i = 0; i < argc; i++) {
        list[i] = va_arg(ap, uint32_t);
    }

    va_end(ap);

    return list;
}


va_names_list_t admin_names(uint argc, ...)
{
    va_names_list_t list(argc);
    va_list ap;

    va_start(ap, argc);

    for (uint i = 0; i < argc; i++) {
        list[i] = va_arg(ap, char*);
    }

    va_end(ap);

    return list;
}

