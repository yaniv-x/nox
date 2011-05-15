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

#ifndef _H_EXCEPTION
#define _H_EXCEPTION

#include <exception>
#include "error_codes.h"

class Exception: public std::exception {
public:
    Exception(const std::string& str) : _error (ERROR_UNKNOWN), _mess (str) {}
    Exception(ErrorCode err, const std::string& str) : _error (err), _mess (str) {}
    virtual ~Exception() throw () {}

    virtual const char* what() const throw () {return _mess.c_str();}
    ErrorCode get_error_code() { return _error;}

private:
    ErrorCode _error;
    std::string _mess;
};

#define THROW(format, ...)  {                                               \
    std::string exption_string;                                             \
    sprintf(exption_string, "%s: "format, FUNC_NAME, ## __VA_ARGS__ );      \
    throw Exception(exption_string);                                        \
}

#define THROW_ERROR(error_code, format, ...)  {                             \
    std::string exption_string;                                             \
    sprintf(exption_string, "%s: "format, FUNC_NAME, ## __VA_ARGS__ );      \
    throw Exception(error_code, exption_string);                            \
}

#define THROW_SYS_ERROR(format, ...)  {                                     \
    std::string exption_string;                                             \
    std::string tmp_string;                                                 \
    int err = errno;                                                        \
                                                                            \
    sprintf(tmp_string, "%s: "format, FUNC_NAME, ## __VA_ARGS__ );          \
    sprintf(exption_string, "%s. %d (%s)", tmp_string.c_str(), err, strerror(err)); \
    throw Exception(ERROR_SYS_ERROR, exption_string);                       \
}

#endif

