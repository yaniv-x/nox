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

#include "dynamic_call.h"

void dynamic_sys_v_amd64_call(void* func, uint64_t n_reg_args, uint64_t* regs_args,
                              uint64_t n_stack_args, uint64_t* stack_args)
{
    __asm__ (
        "   sub $128, %%rsp #skip the red zone\n"
        "   mov %0, %%rax\n"
        "   and %%rax, %%rax\n"
        "   jz no_stack_args\n"
        "   mov $8, %%rdx\n"
        "   mul %%rdx\n"
        "   sub %%rax, %%rsp\n"
        "   mov %0, %%rax\n"
        "   and $0xfffffffffffffff0, %%rsp\n"
        "   mov %1, %%rcx\n"
        "   mov %%rsp, %%r11\n"
        "more_sregs:\n"
        "   mov (%%rcx), %%r9\n"
        "   mov %%r9, (%%r11)\n"
        "   add $8, %%r11\n"
        "   add $8, %%rcx\n"
        "   dec %%rax\n"
        "   jnz more_sregs\n"
        "   jmp set_regs\n"
        "no_stack_args:\n"
        "   and $0xfffffffffffffff0, %%rsp\n"

        "set_regs:\n"
        "   mov %2, %%rax\n"
        "   and %%rax, %%rax\n"
        "   jz do_call\n"
        "   mov %3, %%r11\n"
        "   mov (%%r11), %%rdi\n"
        "   dec %%rax\n"
        "   jz do_call\n"
        "   mov 0x08(%%r11), %%rsi\n"
        "   dec %%rax\n"
        "   jz do_call\n"
        "   mov 0x10(%%r11), %%rdx\n"
        "   dec %%rax\n"
        "   jz do_call\n"
        "   mov 0x18(%%r11), %%rcx\n"
        "   dec %%rax\n"
        "   jz do_call\n"
        "   mov 0x20(%%r11), %%r8\n"
        "   dec %%rax\n"
        "   jz do_call\n"
        "   mov 0x28(%%r11), %%r9\n"
        "do_call:\n"
        "   xor %%rax, %%rax # al is upper bound on the number of SSE registers used \n"
        "   mov %4, %%r11\n"
        "   callq *%%r11\n"

    :: "g" (n_stack_args), "g" (stack_args), "g" (n_reg_args), "g" (regs_args), "g" (func));
}

