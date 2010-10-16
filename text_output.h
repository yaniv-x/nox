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

#ifndef _H_TEXT_OUTPUT
#define _H_TEXT_OUTPUT

#include "utils.h"

static inline std::string strip_pretty_function(const std::string& pretty_function)
{
    size_t end;

    if ((end = pretty_function.find('(')) != std::string::npos ||
         (end = pretty_function.find('<')) != std::string::npos) {

        size_t start;

        if ((start = pretty_function.rfind(' ', end)) == std::string::npos) {
            start = 0;
        } else {
            start += 1;
        }

        return pretty_function.substr(start, end - start);
    }

    return pretty_function;
}

#define FUNC_NAME  strip_pretty_function(__PRETTY_FUNCTION__).c_str()

#define OUTPUT_MESSAGE(type, format, ...) {                                         \
    std::string log_message;                                                        \
    sprintf(log_message, #type": %s: " format"\n", FUNC_NAME, ## __VA_ARGS__);      \
    printf(log_message.c_str());                                                    \
}

#define E_MESSAGE(format, ...) OUTPUT_MESSAGE(ERROR, format, ## __VA_ARGS__)
#define W_MESSAGE(format, ...) OUTPUT_MESSAGE(WARNING, format, ## __VA_ARGS__)
#define I_MESSAGE(format, ...) OUTPUT_MESSAGE(INFO, format, ## __VA_ARGS__)
#define D_MESSAGE(format, ...) OUTPUT_MESSAGE(DEBUG, format, ## __VA_ARGS__)


#define OUTPUT_MESSAGE_SOME(how_much, type, format, ...)   {        \
    static uint show_count = how_much;                              \
    if (show_count) {                                               \
        show_count--;                                               \
        W_MESSAGE(format, ## __VA_ARGS__);                          \
    }                                                               \
}

#define W_MESSAGE_SOME(how_much, format, ...) \
    OUTPUT_MESSAGE_SOME(how_much, WARNING, format, ## __VA_ARGS__)


#define OUTPUT_MESSAGE_ONCE(type, format, ...)   {          \
    static bool show_message = true;                        \
    if (show_message) {                                     \
        show_message = false;                               \
        W_MESSAGE(format, ## __VA_ARGS__);                  \
    }                                                       \
}

#define W_MESSAGE_ONCE(format, ...) OUTPUT_MESSAGE_ONCE(WARNING, format, ## __VA_ARGS__)

#endif

