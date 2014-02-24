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

#include "options_parser.h"


enum OptionType {
    NO_VALUE = 1,
    MANDATORY_VALUE,
    OPTIONAL_VALUE,
};


class OptionsParser::Option: public NonCopyable {
public:
    Option(uint id, const char* name, OptionType type, const char* description, uint flags,
           const char* _arg_name)
        : _id (id)
        , _name (name)
        , _type (type)
        , _description (description)
        , _flags (flags)
        , _short_name (0)
        , _arg_name (_arg_name)
        , _hits (0)
    {
    }

    void set_short_name(char name) { _short_name = name;}
    const std::string&  get_name() { return _name;}
    char get_short_name() { return _short_name;}
    const std::string&  get_description() { return _description;}
    const std::string&  get_arg_name() { return _arg_name;}
    uint  get_id() { return _id;}
    void hit() { _hits++;}
    bool was_hit() { return _hits != 0;}

    bool is_optional_val() { return _type == OPTIONAL_VALUE;}
    bool is_mandatory_val() { return _type == MANDATORY_VALUE;}
    bool is_mandatory() { return !!(_flags & MANDATORY);}
    bool is_exclusive_arg() { return !!(_flags & EXCLUSIVE_ARG);}
    bool is_exclusive_opt() { return !!(_flags & EXCLUSIVE_OPT);}

private:
    uint _id;
    std::string _name;
    OptionType _type;
    std::string _description;
    uint _flags;
    char _short_name;
    std::string _arg_name;
    uint _hits;
};


class OptionsParser::BuildItem {
public:
    BuildItem(Option* in_option, const char* in_val)
        : option (in_option)
        , val (in_val)
    {
    }

    Option* option;
    const char* val;
};


enum {
    INIT,
    PARSE_FRONT_POSITIONAL,
    PARSE_OPTIONS,
    PARSE_BACK_POSITIONAL,
    READY,
    ERROR,
};


OptionsParser::OptionsParser()
    : _state (INIT)
    , _min_front_positional (0)
    , _max_front_positional (0)
    , _min_back_positional (0)
    , _max_back_positional (0)
    , _back_positional (false)
{
}


OptionsParser::~OptionsParser()
{
    while (!_argv.empty()) {
        delete[] *_argv.begin();
        _argv.pop_front();
    }

    while (!_options.empty()) {
        delete *_options.begin();
        _options.pop_front();
    }

    while (!_build_list.empty()) {
        delete *_build_list.begin();
        _build_list.pop_front();
    }
}


OptionsParser::Option* OptionsParser::get_option(char* str)
{
    Option* ret;
    char* end_pos;

    if ((end_pos = strchr(str, '='))) {
        *end_pos = 0;
    }

    if (str[0] != '-') {

        if (strlen(str) != 1) {
            ret =  NULL;
        } else {
            ret = find(str[0]);
        }
    } else {
        ret = find(str + 1);
    }

    if (end_pos) {
        *end_pos = '=';
    }

    return ret;
}


bool OptionsParser::verify_positional(uint min, uint max, uint count, const char* word)
{
    if (count < min) {
        printf("%s: too few %s positional arguments\n", _prog_name.c_str(), word);
        return false;
    }

    if (count > max) {
        printf("%s: too many %s positional arguments\n", _prog_name.c_str(), word);
        return false;
    }

    return true;
}


bool OptionsParser::verify()
{
    OptionList::iterator opt_iter = _options.begin();

    for (; opt_iter != _options.end(); ++opt_iter) {
        Option* option = *opt_iter;

        if (!option->was_hit() || !option->is_exclusive_arg()) {
            continue;
        }

        if (_build_list.size() == 1) {
            return true;
        }

        printf("%s: \"--%s\" is exclusive argument\n",
               _prog_name.c_str(), option->get_name().c_str());
        return false;
    }

    for (opt_iter = _options.begin(); opt_iter != _options.end(); ++opt_iter) {
        Option* option = *opt_iter;

        if (option->was_hit() || !option->is_mandatory()) {
            continue;
        }

        printf("%s: \"--%s\" is mandatory\n", _prog_name.c_str(),
               option->get_name().c_str());
        return false;
    }

    BuildList::iterator build_iter = _build_list.begin();
    uint front_positionals = 0;
    uint back_positionals = 0;
    uint options = 0;
    Option* exclusive_opt = NULL;
    bool count_back = false;

    for (; build_iter != _build_list.end(); ++build_iter) {
        BuildItem* item = *build_iter;
        Option* option = item->option;

        if (!option) {
            if (count_back) {
                back_positionals++;
            } else {
                front_positionals++;
            }
            continue;
        }

        options++;
        count_back = true;

        if (option->is_exclusive_opt()) {
            exclusive_opt = option;
        }

        if (option->is_mandatory_val() && !item->val) {
            printf("%s: value for \"--%s\" is mandatory\n",
                   _prog_name.c_str(), option->get_name().c_str());
            return false;
        }
    }

    if (!verify_positional(_min_front_positional, _max_front_positional,
                          front_positionals, "front") ||
        !verify_positional(_min_back_positional, _max_back_positional,
                          back_positionals, "back")) {
        return false;
    }

    if (exclusive_opt && (options > 1)) {
        printf("%s: \"--%s\" is exclusive option\n", _prog_name.c_str(),
               exclusive_opt->get_name().c_str());
        return false;
    }

    return true;
}


static bool verify_argv(int argc, const char** argv)
{
    if (argc < 1 || !argv) {
        return false;
    }

    const char** end = argv + argc;

    for (; argv < end; argv++) {
        if (!*argv) {
            return false;
        }
    }

    return true;
}


bool OptionsParser::parse(int argc, const char** argv)
{
    ASSERT(_state == INIT);

    if (!verify_argv(argc, argv)) {
        THROW("invalig argc argv");
    }

    _prog_name = basename(*argv);

    const char** end = argv + argc;

    for (argv++; argv < end; argv++) {
        _argv.push_back(copy_cstr(*argv));
    }

    _options.push_back(new Option(OPT_ID_HELP, "help", NO_VALUE,
                                  "show command help", EXCLUSIVE_ARG, ""));

    _state = PARSE_FRONT_POSITIONAL;
    ArgsList::iterator iter = _argv.begin();

    for (; iter != _argv.end(); iter++) {
        char* now = *iter;

        if (now[0] == '-') {
            if ( _state == PARSE_FRONT_POSITIONAL) {
                _state = PARSE_OPTIONS;
            } else if (_state != PARSE_OPTIONS) {
                _state = ERROR;
                printf("%s: positional arguments are restricted to the end of the command line.\n",
                       _prog_name.c_str());
                return false;
            }

            Option* opt = get_option(now + 1);

            if (!opt) {
                 printf("%s: invalid option \"%s\"\n", _prog_name.c_str(), now);
                _state = ERROR;
                return false;
            }

            char* optinal_val = strchr(now, '=');

            if (optinal_val) {
                if (!opt->is_optional_val()) {
                    printf("%s: \"%s\", = is valid only in case of optional argument\n",
                           _prog_name.c_str(), now);
                    _state = ERROR;
                    return false;
                }

                if (strlen(optinal_val + 1) == 0) {
                    printf("%s: \"%s\" missing optional argumant value\n", _prog_name.c_str(), now);
                    _state = ERROR;
                    return false;
                }
                optinal_val++;
            }
            opt->hit();
            _build_list.push_back(new BuildItem(opt, optinal_val));
        } else {
            BuildItem* item = _build_list.empty() ? NULL : _build_list.back();

            if (item && item->option && item->option->is_mandatory_val() &&  !item->val) {
                item->val = now;
            } else {
                if (_state != PARSE_FRONT_POSITIONAL) {
                    _state = PARSE_BACK_POSITIONAL;
                }
                _build_list.push_back(new BuildItem(NULL, now));
            }
        }
    }

    if (!verify()) {
        _state = ERROR;
        return false;
    }

    _get_iter = _build_list.begin();
    _state = READY;
    return true;
}


int OptionsParser::next(const char** arg)
{
    ASSERT(_state == READY);
    ASSERT(arg);

    if (_get_iter == _build_list.end()) {
        *arg = NULL;
        return OPT_ID_DONE;
    }

    BuildItem* item = *_get_iter;
    int id;

    if (item->option) {
        _back_positional = true;
        id = item->option->get_id();
    } else {
        id = (_back_positional) ? OPT_ID_BACK_POSITIONAL : OPT_ID_FRONT_POSITIONAL;
    }

    *arg = item->val;
    _get_iter++;

    return id;
}


static bool is_valid_name(const char* name)
{
    size_t len = strlen(name);

    if (!len) {
        return false;
    }

    const char* end = name + len;

    for (; name < end; name++) {
        if (!isalnum(*name) && !*name == '-') {
            return false;
        }
    }

    return true;
}


void OptionsParser::add_option(int id, const char* name, const char* description, uint flags)
{
    ASSERT(id >= 0);
    ASSERT(!find(id));
    ASSERT(name && is_valid_name(name));
    ASSERT(description && strlen(description));
    ASSERT(!find(name));
    ASSERT(_state == INIT);

    _options.push_back(new Option(id, name, NO_VALUE, description, flags, ""));
}


void OptionsParser::add_option_with_arg(int id, const char* name, ArgType type,
                                        const char* arg_name, const char* description,
                                        uint flags)
{
    ASSERT(id >= 0);
    ASSERT(!find(id));
    ASSERT(name && description && arg_name);
    ASSERT(strlen(description) && strlen(arg_name));
    ASSERT(is_valid_name(name));
    ASSERT(!find(name));
    ASSERT(_state == INIT);

    OptionType opt_type = (type == ONE_ARGUMENT) ? MANDATORY_VALUE : OPTIONAL_VALUE;

    _options.push_back(new Option(id, name, opt_type, description, flags, arg_name));
}


const char* OptionsParser::get_option_name(int id)
{
    Option* opt = find(id);

    ASSERT(opt);

    return opt->get_arg_name().c_str();
}


void OptionsParser::set_short_name(int id, char name)
{
    ASSERT(find(id) && !find(name));
    ASSERT(isalnum(name));
    find(id)->set_short_name(name);
}


void OptionsParser::set_front_positional_minmax(uint min, uint max)
{
    ASSERT(_state == INIT);
    _min_front_positional = min;
    _max_front_positional = max;
}


void OptionsParser::set_back_positional_minmax(uint min, uint max)
{
    ASSERT(_state == INIT);
    _min_back_positional = min;
    _max_back_positional = max;
}


OptionsParser::Option* OptionsParser::find(const char* name)
{
    OptionList::iterator iter = _options.begin();

    for (; iter != _options.end(); ++iter) {
        if ((*iter)->get_name() == name) {
            return *iter;
        }
    }

    return NULL;
}


OptionsParser::Option* OptionsParser::find(int id)
{
    OptionList::iterator iter = _options.begin();

    for (; iter != _options.end(); ++iter) {
        if ((*iter)->get_id() == id) {
            return *iter;
        }
    }

    return NULL;
}


OptionsParser::Option* OptionsParser::find(char short_name)
{
    OptionList::iterator iter = _options.begin();

    for (; iter != _options.end(); ++iter) {
        if ((*iter)->get_short_name() == short_name) {
            return *iter;
        }
    }

    return NULL;
}


void OptionsParser::print_help_description(const char* in_str, uint skip, uint width)
{
    char* str_copy = copy_cstr(in_str);
    char* start = str_copy;

    while (strlen(start)) {

        for (int i = 0; i < skip; i++) {
            printf(" ");
        }

        char* end = start;
        char* space = NULL;

        for (; end - start < width; end++) {
            if (*end == 0 || *end == '\n') {
                break;
            }

            if (*end == ' ') {
                space = end;
            }
        }

        switch (*end) {
        case 0:
            printf("%s\n", start);
            start = end;
            break;
        case '\n':
            *end = 0;
            printf("%s\n", start);
            start = end + 1;
            break;
        default:
            if (space) {
                *space = 0;
                printf("%s\n", start);
                start = space + 1;
            } else {
                char ch = *end;
                *end = 0;
                printf("%s\n", start);
                *end = ch;
                start = end;
            }
        }
    }

    delete[] str_copy;
}


void OptionsParser::help()
{
    printf("\n%s - short discription\n\n", _prog_name.c_str());

    OptionList::iterator iter = _options.begin();

    for (; iter != _options.end(); ++iter) {
        Option* option = *iter;

        if (option->get_short_name()) {
            printf(" -%c, ", option->get_short_name());
        } else {
            printf("     ");
        }

        printf("--%s", option->get_name().c_str());

        if (option->is_mandatory_val()) {
            printf(" <%s>", option->get_arg_name().c_str());
        } else if (option->is_optional_val()) {
            printf("[=%s]", option->get_arg_name().c_str());
        }

        printf("\n");
        print_help_description(option->get_description().c_str(), 12, 68);
        printf("\n");
    }
}

