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

#ifndef _H_NOX_TYPES
#define _H_NOX_TYPES

typedef unsigned int uint;
typedef uint64_t address_t;
typedef uint64_t page_address_t;
typedef uint8_t mac_addr_t[6];

typedef void (*read_mem_proc_t)(void* opaque, uint64_t src, uint64_t length, uint8_t* dest);
typedef void (*write_mem_proc_t)(void* opaque, const uint8_t* src, uint64_t length, uint64_t dest);

typedef uint8_t (*io_read_byte_proc_t)(void* opaque, uint16_t port);
typedef uint16_t (*io_read_word_proc_t)(void* opaque, uint16_t port);
typedef uint32_t (*io_read_dword_proc_t)(void* opaque, uint16_t port);

typedef void (*io_write_byte_proc_t)(void* opaque, uint16_t port, uint8_t val);
typedef void (*io_write_word_proc_t)(void* opaque, uint16_t port, uint16_t val);
typedef void (*io_write_dword_proc_t)(void* opaque, uint16_t port, uint32_t val);

typedef void (*void_callback_t)(void*);
typedef void (*int_callback_t)(void*, int);

#endif

