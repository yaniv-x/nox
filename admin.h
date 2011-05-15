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

#ifndef _H_ADMIN
#define _H_ADMIN


#define NOX_ADMIN_MAGIC *(uint32_t*)"VADM"
#define NOX_ADMIN_VERSION 1


enum VAMessageType {
    VA_MESSAGE_TYPE_INVALID,
    VA_MESSAGE_TYPE_COMMAND,
    VA_MESSAGE_TYPE_REPLY,
    VA_MESSAGE_TYPE_ERROR,
};


enum VACommands {
    VA_CMD_INVALID = 0,
    VA_CMD_ENUM_COMMANDS,
    VA_CMD_DESCRIBE_COMMAND,
    VA_CMD_NOTIFY,
    VA_CMD_ASK,

    VA_FIRST_USER_CMD,
};


enum VAErrorCode {
    VA_ERROR_OK,
    VA_ERROR_FAILED,
    VA_ERROR_VERSION_MISSMATCH,
    VA_ERROR_NOT_SUPORTED,
};


enum VAType {
    VA_INVALID_T,
    VA_UINT32_T,
    VA_INT32_T,
    VA_UINT8_T,
    VA_INT8_T,
    VA_UTF8_T,      // uint16_t offset
    VA_UINT32V_T,   //(uint16_t offset, uint16_t size)
};


struct __attribute__ ((__packed__)) LinkMessage {
    uint32_t magic;
    uint32_t version;
};


struct __attribute__ ((__packed__)) LinkReply {
    uint32_t magic;
    uint32_t version;
    uint16_t size;
    uint16_t message_string;
    uint32_t error;
};


struct __attribute__ ((__packed__)) MessageHeader {
    uint32_t type; //VA_MESSAGE_TYPE_?
    uint16_t size;
    uint16_t reserved_mbz;
};


struct __attribute__ ((__packed__)) Command {
    uint32_t command_code;
    uint32_t command_serial;
};


struct __attribute__ ((__packed__)) CommandReply {
    uint32_t command_serial;
};


struct __attribute__ ((__packed__)) CommandError {
    uint32_t command_serial;
    uint32_t error_code;
    uint16_t message_string;
    uint16_t reserved_mbz;
};

#endif

