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

#ifndef _H_KVM
#define _H_KVM

#include "common.h"

typedef uint32_t KvmMapRef;
#define INVALID_KVM_MAP_REF 0

typedef unsigned long KvmCpuRef;
#define INVALID_KVM_CPY_REF 0

typedef std::list<uint32_t> FreeSlotsList;

class KVM: private NonCopyable {
public:
    KVM();

    KvmMapRef map_mem_slot(page_address_t start, uint64_t num_pages,
                        page_address_t host_address);
    void unmap_mem_slot(KvmMapRef map_ref);

    int get_dev_fd() { return _devfd.get();}
    int get_vm_fd() { return _vmfd.get();}
    int get_vcpu_mmap_size() { return _vcpu_mmap_size;}
    const struct kvm_msr_list& get_msrs_list() { return *(struct kvm_msr_list*)_msrs_list.get();}
    int get_max_cpus() { return _max_vcpu;}

private:
    void init_msrs();
    void init();
    int check_extention(int extension);
    bool is_active_slot(uint32_t slot);

private:
    AutoFD _devfd;
    AutoFD _vmfd;
    int _max_vcpu;
    int _num_mem_slot;
    int _vcpu_mmap_size;
    FreeSlotsList _free_slots;
    AutoArray<uint8_t> _msrs_list;
};

#endif

