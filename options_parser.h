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

#ifndef _H_OPTIONS_PARSER
#define _H_OPTIONS_PARSER

#include "common.h"


class OptionsParser: public NonCopyable {
public:
    OptionsParser();
    ~OptionsParser();

    enum ArgType {
        OPTIONAL_ARGUMENT,
        ONE_ARGUMENT,
        //MULTIPLE_ARGUMENTS
    };

    enum OptionFlags {
        MANDATORY = 1 << 0,
        EXCLUSIVE_ARG = 1 << 1,
        EXCLUSIVE_OPT = 1 << 2,
    };

    enum {
        OPT_ID_DONE = -1,
        OPT_ID_FRONT_POSITIONAL = -2,
        OPT_ID_BACK_POSITIONAL = -3,
        OPT_ID_HELP = -4,
    };

    bool parse(int argc, const char** argv);

    void add_option(int id, const char* name, const char* description, uint flags = 0);
    void add_option_with_arg(int id, const char* name, ArgType type, const char* arg_name,
                             const char* description, uint flags = 0);
    void set_short_name(int id, char name);
    void set_front_positional_minmax(uint min, uint max);
    void set_back_positional_minmax(uint min, uint max);

    int next(const char** arg);
    const char* get_option_name(int id);
    void help();
    const char* get_prog_name() { return _prog_name.c_str();}

private:
    class Option;

    Option* get_option(char* str);
    bool verify_positional(uint min, uint max, uint count, const char* word);
    bool verify();
    void print_help_description(const char* in_str, uint skip, uint width);

    Option* find(const char* name);
    Option* find(int id);
    Option* find(char short_name);

private:
    uint _state;
    uint _min_front_positional;
    uint _max_front_positional;
    uint _min_back_positional;
    uint _max_back_positional;
    std::string _prog_name;

    typedef std::list<char*> ArgsList;
    ArgsList _argv;

    class Option;
    typedef std::list<Option*> OptionList;
    OptionList _options;

    class BuildItem;
    typedef std::list<BuildItem*> BuildList;
    BuildList _build_list;
    BuildList::iterator _get_iter;
    bool _back_positional;
};

#endif

