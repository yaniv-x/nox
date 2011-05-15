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

#ifndef _H_COMMON
#define _H_COMMON

#include <stdlib.h>
#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>
#include <auto_ptr.h>
#include <string.h>
#include <string>
#include <list>
#include <vector>
#include <map>

#include "exception.h"
#include "utils.h"
#include "text_output.h"
#include "debug.h"
#include "non_copyable.h"
#include "error_codes.h"
#include "nox_types.h"

#define KB (1024UL)
#define MB (KB * 1024)
#define GB (MB * 1024)

#define GUEST_PAGE_SHIFT 12
#define GUEST_PAGE_SIZE (1 << GUEST_PAGE_SHIFT)
#define GUEST_PAGE_OFFSET_MASK page_address_t(GUEST_PAGE_SIZE - 1)
#define GUEST_PAGE_MASK ~GUEST_PAGE_OFFSET_MASK

#define EXCLISIC_EXEC()
#define IS_ERROR(x) (!!(x))

#endif

