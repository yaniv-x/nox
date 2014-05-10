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

#include <sstream>


enum OptionType {
    NO_VALUE = 1,
    MANDATORY_VALUE,
    OPTIONAL_VALUE,
};


class OptionsParser::Option: public NonCopyable {
public:
    Option(uint id, const char* name, const char* description, uint flags,
           const char* val_descriptor = "", bool opt_val = false)
        : _id (id)
        , _name (name)
        , _description (description)
        , _flags (flags)
        , _short_name (0)
        , _val_descriptor (val_descriptor)
        , _hits (0)
        , _type (_val_descriptor.empty() ? NO_VALUE : (opt_val ? OPTIONAL_VALUE : MANDATORY_VALUE))
    {
    }

    void set_short_name(char name) { _short_name = name;}
    const std::string&  get_name() { return _name;}
    char get_short_name() { return _short_name;}
    const std::string&  get_description() { return _description;}
    const std::string&  get_val_descriptor() { return _val_descriptor;}
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
    std::string _description;
    uint _flags;
    char _short_name;
    std::string _val_descriptor;
    uint _hits;
    OptionType _type;
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

    _options.push_back(new Option(OPT_ID_HELP, "help", "show command help", EXCLUSIVE_ARG));

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

    _options.push_back(new Option(id, name, description, flags));
}


void OptionsParser::add_option(int id, const char* name, const char* arg_name, bool optional_val,
                               const char* description, uint flags)
{
    ASSERT(id >= 0);
    ASSERT(!find(id));
    ASSERT(name && description && arg_name);
    ASSERT(strlen(description) && strlen(arg_name));
    ASSERT(is_valid_name(name));
    ASSERT(!find(name));
    ASSERT(_state == INIT);

    _options.push_back(new Option(id, name, description, flags, arg_name, optional_val));
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
            printf(" %s", option->get_val_descriptor().c_str());
        } else if (option->is_optional_val()) {
            printf("[=%s]", option->get_val_descriptor().c_str());
        }

        printf("\n");
        print_help_description(option->get_description().c_str(), 12, 68);
        printf("\n");
    }
}


class InnerImp: public OptionsParser::Inner {
public:
    InnerImp(const std::string& descriptor);
    bool parse(const std::string& val, uint min_positional, uint max_positional,
               const std::string& option_name, const std::string& program_name);
    virtual const char* get_positional(uint pos);
    virtual const char* get_option(const std::string& name);
    virtual bool switch_test(const std::string& name);
    void add_positional(std::string& split);
    void add_pair(std::string& name, std::string& val);
    void add_switch(std::string& name);
    void add_segment(std::string& str, bool optional);

    struct PairsVal {
        PairsVal(const std::string& in_val, bool in_switches)
            : val (in_val)
            , switches (in_switches)
        {
        }

        std::string val;
        bool switches;
    };

    typedef std::map<std::string, PairsVal> PairsMap;
    PairsMap pair_descriptors;
    std::list<std::string> positional_descriptors;
    std::set<std::string> switch_descriptors;

    std::vector<std::string> positionals;
    std::set<std::string> switches;
    PairsMap pairs;
};


InnerImp::InnerImp(const std::string& in_descriptor)
{
    std::string descriptor(in_descriptor);

    bool first = true;

    for (;;) {
        size_t opt_start = descriptor.find('[');

        if (opt_start == std::string::npos) {
            add_segment(descriptor, false);
            break;
        }

        if (opt_start > 0) {
            std::string split = descriptor.substr(0, opt_start);
            add_segment(split, false);
            first = false;

        }

        size_t opt_end = descriptor.find(']');
        ASSERT(opt_end != std::string::npos);
        ASSERT(first || descriptor[opt_start + 1] == ',');

        if (!first) {
            opt_start += 2;
        } else {
            opt_start += 1;
        }

        ASSERT(opt_end  > opt_start);
        std::string split = descriptor.substr(opt_start, opt_end - opt_start);
        ASSERT(split.find('[') == std::string::npos);

        add_segment(split, true);

        if (descriptor[opt_end + 1] == 0) {
            break;
        }

        ASSERT(descriptor[opt_end + 1] == ',' || descriptor[opt_end + 1] == '[');

        if (descriptor[opt_end + 1] == ',') {
            opt_end += 1;
        }

        descriptor = descriptor.substr(opt_end + 1);
        ASSERT(descriptor.length());

        first = false;
    }
}


void InnerImp::add_positional(std::string& val)
{
    ASSERT(val.length());
    ASSERT(val.find_first_of("<>[],|") == std::string::npos);
    positional_descriptors.push_back(val);
}


void InnerImp::add_pair(std::string& name, std::string& val)
{
    ASSERT(name.length() && val.length());
    ASSERT(name.find_first_of("<>[],|") == std::string::npos);
    ASSERT(pair_descriptors.find(name) == pair_descriptors.end());

    size_t variable_start = val.find('<');

    if (variable_start != std::string::npos) {
        size_t end = val.find('>');
        ASSERT(variable_start == 0 && end == val.size() - 1 && val.length() > 2);
        variable_start += 1;
        std::string split = val.substr(variable_start, end - variable_start);
        PairsMap::value_type pair(name, PairsVal(val, false));
        pair_descriptors.insert(pair);
        return;
    }

    PairsMap::value_type pair(name, PairsVal(val, true));
    pair_descriptors.insert(pair);

}


void InnerImp::add_switch(std::string& name)
{
    ASSERT(name.length());
    ASSERT(name.find_first_of("<>[],|") == std::string::npos);
    ASSERT(switch_descriptors.find(name) == switch_descriptors.end());
    switch_descriptors.insert(name);
}


void InnerImp::add_segment(std::string& str, bool optional)
{
    std::istringstream is(str);
    std::string line;

    while (std::getline(is, line, ',')) {
        size_t pair_split = line.find('=');

        if (pair_split != std::string::npos) {
            std::string name = line.substr(0, pair_split);
            std::string val = line.substr(pair_split + 1);
            add_pair(name, val);
            continue;
        }

        size_t positional_start = line.find('<');

        if (positional_start != std::string::npos) {
            size_t end = line.find('>');

            ASSERT(positional_start == 0 && end == line.size() - 1);
            positional_start +=1;
            std::string split = line.substr(positional_start, end - positional_start);

            ASSERT(split.length() && split.find('<') == std::string::npos);

            add_positional(split);

            continue;
        }

        std::istringstream is(line);
        std::string name;

        while (std::getline(is, name, '|')) {
            add_switch(name);
        }
    }
}


const char* InnerImp::get_positional(uint pos)
{
    if (pos >= positionals.size()) {
        return NULL;
    }

    return positionals[pos].c_str();
}


const char* InnerImp::get_option(const std::string& name)
{
    PairsMap::iterator iter = pairs.find(name);
    return (iter == pairs.end()) ? NULL : (*iter).second.val.c_str();
}


bool InnerImp::switch_test(const std::string& name)
{
    return switches.find(name) != switches.end();
}


static bool is_valid_val(const std::string& val, const std::string& options)
{
    std::istringstream is(options);
    std::string split;

    while (std::getline(is, split, '|')) {
       if (split == val) {
           return true;
       }
    }

    return false;
}


bool InnerImp::parse(const std::string& arg, uint min_positional, uint max_positional,
                     const std::string& option_name, const std::string& prog_name)
{
    std::istringstream is(arg);
    std::string line;
    bool front = true;

    while (std::getline(is, line, ',')) {

        if (!line.length()) {
            printf("%s: in \"--%s\": invalid args \"%s\"\n", prog_name.c_str(),
                   option_name.c_str(), arg.c_str());
            return false;
        }

        size_t pair_split = line.find('=');

        if (pair_split != std::string::npos) {
            std::string name = line.substr(0, pair_split);
            std::string val = line.substr(pair_split + 1);

            if (val.length() == 0) {
                printf("%s: in \"--%s\": invalid empty val of \"%s\"\n", prog_name.c_str(),
                       option_name.c_str(), name.c_str());
                return false;
            }

            PairsMap::iterator item = pair_descriptors.find(name);

            if (item == pair_descriptors.end()) {
                printf("%s: in \"--%s\": invalid arg name \"%s\"\n", prog_name.c_str(),
                       option_name.c_str(), name.c_str());
                return false;
            }

            if ((*item).second.switches && !is_valid_val(val, (*item).second.val)) {
                printf("%s: in \"--%s\": \"%s\" is not a valid val of \"%s\"\n", prog_name.c_str(),
                       option_name.c_str(), val.c_str(), name.c_str());
                return false;
            }

            if (pairs.find(name) != pairs.end()) {
                pairs.erase(name);
            }

            PairsMap::value_type pair(name, PairsVal(val, (*item).second.switches));
            pairs.insert(pair);

            front = false;
            continue;
        }

        if (switch_descriptors.find(line) != switch_descriptors.end()) {
            if (switches.find(line) == switches.end()) {
                switches.insert(line);
                front = false;
            }
            continue;
        }

        if (front && !positional_descriptors.empty()) {
            positionals.push_back(line);
            continue;
        }

        printf("%s: in \"--%s\": invalid switch \"%s\"\n", prog_name.c_str(), option_name.c_str(),
               line.c_str());

        return false;
    }

    if (positionals.size() < min_positional) {
        printf("%s: in \"--%s\": missing front positional arguments\n", prog_name.c_str(),
               option_name.c_str());
        return false;
    }

    if (positionals.size() > max_positional) {
        printf("%s: in \"--%s\": too many positional arguments\n", prog_name.c_str(),
               option_name.c_str());
        return false;
    }

    return true;
}


OptionsParser::Inner* OptionsParser::parse_val(int option_id, const char* val,
                                               uint min_positional,
                                               uint max_positional)
{
    Option* option = find(option_id);

    ASSERT(option);

    std::auto_ptr<InnerImp> iner(new InnerImp(option->get_val_descriptor()));

    return (iner->parse(val, min_positional, max_positional, option->get_name(),
                        _prog_name)) ? iner.release() : NULL;

}

