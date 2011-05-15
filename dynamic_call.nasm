%if 0
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
%endif

BITS 64


; void dynamic_sys_v_amd64_call(void* func, uint64_t n_reg_args, uint64_t* regs_args,
;                               uint64_t n_stack_args, uint64_t* stack_args)

global dynamic_sys_v_amd64_call
dynamic_sys_v_amd64_call:
    push rbp
    mov rbp, rsp

    mov [rbp - 0x08], rdi
    mov [rbp - 0x10], rsi
    mov [rbp - 0x18], rdx
    mov [rbp - 0x20], rcx
    mov [rbp - 0x28], r8
    sub rsp, 0x80

    mov rax, [rbp - 0x20]
    and rax, rax
    jz  .no_stack_args
    mov rdx, 0x08
    mul rdx
    sub rsp, rax
    and rsp, 0xfffffffffffffff0
    mov rax, [rbp - 0x20]
    mov rcx, [rbp - 0x28]
    mov r11, rsp
.more_sregs:
    mov r9, [rcx]
    mov [r11], r9
    add rcx, 8
    add r11, 8
    dec rax
    jnz .more_sregs
    jmp .set_regs 

.no_stack_args:
    and rsp, 0xfffffffffffffff0

.set_regs:
    mov rax, [rbp - 0x10]
    and rax, rax
    jz .call
    mov r11, [rbp - 0x18]
    mov rdi, [r11]
    dec rax
    jz .call
    mov rsi, [r11 + 0x08]
    dec rax
    jz .call
    mov rdx, [r11 + 0x10]
    dec rax
    jz .call
    mov rcx, [r11 + 0x18]
    dec rax
    jz .call
    mov r8, [r11 + 0x20]
    dec rax
    jz .call
    mov r9, [r11 + 0x28]

.call:
    xor rax, rax
    mov r11, [rbp - 0x08]
    call r11
    leave
    ret





