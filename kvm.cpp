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

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/user.h>

#include "kvm.h"
#include "common.h"


KVM::KVM()
    : _max_vcpu (0)
    , _num_mem_slot (0)
    , _vcpu_mmap_size (0)
{
    init();
}


int KVM::check_extention(int extension)
{
    int r = ioctl(_devfd.get(), KVM_CHECK_EXTENSION, extension);

    if (r == -1) {
        int err = errno;
        D_MESSAGE("failed: errno %d (%s)", err, strerror(err));
        r = 0;
    }

    return r;
}


void KVM::init_msrs()
{
    struct kvm_msr_list test_list;

    test_list.nmsrs = 0;

    int r = ioctl(_devfd.get(), KVM_GET_MSR_INDEX_LIST, &test_list);

    if (r == 0 || errno != E2BIG || test_list.nmsrs == 0) {
        THROW("test msrs count failed");
    }

    uint size = sizeof(test_list) + test_list.nmsrs * sizeof(test_list.indices[0]);
    AutoArray<uint8_t> list(new uint8_t[size]);
    struct kvm_msr_list* list_ptr = (struct kvm_msr_list*)list.get();
    memset(list_ptr, 0, size);

    list_ptr->nmsrs = test_list.nmsrs;

    if (ioctl(_devfd.get(), KVM_GET_MSR_INDEX_LIST, list_ptr)) {
        THROW("get msrs list failed");
    }

    _msrs_list.set(list.release());
}


void KVM::init()
{
    static const char* kvm_file_name = "/dev/kvm";

    _devfd.reset(::open(kvm_file_name, O_RDWR));

    if (!_devfd.is_valid()) {
        int err = errno;
        THROW("open %s failed: errno %d (%s)", kvm_file_name, err, strerror(err));
    }

    int version = ioctl(_devfd.get(), KVM_GET_API_VERSION, 0);

    if (version != KVM_API_VERSION) {
        if (version == -1) {
            int err = errno;
            THROW("get kvm api version failed: errno %d (%s)", err, strerror(err));
        }

        THROW("kvm api version mismatch kvm %d self %d", version, KVM_API_VERSION);
    }

    if (!check_extention(KVM_CAP_USER_MEMORY)) {
        THROW("no user memory extension");
    }

    if (!check_extention(KVM_CAP_SET_TSS_ADDR)) {
        THROW("no set tss extension");
    }

    if (!(_max_vcpu = ioctl(_devfd.get(), KVM_CHECK_EXTENSION, KVM_CAP_NR_VCPUS))) {
        THROW("unable to get max vcpu");
    }

    if (!(_num_mem_slot = ioctl(_devfd.get(), KVM_CHECK_EXTENSION, KVM_CAP_NR_MEMSLOTS))) {
        THROW("unable to get max mem slots");
    }

    if ((_vcpu_mmap_size = ioctl(_devfd.get(), KVM_GET_VCPU_MMAP_SIZE, 0)) < 0) {
        THROW("unable to get vcpu mmap size");
    }

    init_msrs();

    _vmfd.reset(ioctl(_devfd.get(), KVM_CREATE_VM, 0));

    if (!_vmfd.is_valid()) {
        int err = errno;
        THROW("vm create failed: errno %d (%s)", err, strerror(err));
    }

    for (uint32_t i = 0; i < _num_mem_slot; i++) {
        _free_slots.push_back(i);
    }

    //todo: set tss KVM_SET_TSS_ADDR
}


KvmMapRef KVM::map_mem_slot(page_address_t start, uint64_t num_pages,
                            page_address_t host_address)
{
    struct kvm_userspace_memory_region userspace_mem_slot;



    ASSERT(num_pages > 0);
    ASSERT((start << GUEST_PAGE_SHIFT) + (num_pages << GUEST_PAGE_SHIFT) >
           (start << GUEST_PAGE_SHIFT));
    ASSERT((host_address << GUEST_PAGE_SHIFT) + (num_pages << GUEST_PAGE_SHIFT) >
           (host_address << GUEST_PAGE_SHIFT));

    if (_free_slots.empty()) {
        W_MESSAGE("no more slots");
        return INVALID_KVM_MAP_REF;
    }

    memset(&userspace_mem_slot, 0, sizeof(userspace_mem_slot));

    userspace_mem_slot.slot = *_free_slots.begin();
    userspace_mem_slot.guest_phys_addr = start << GUEST_PAGE_SHIFT;
    userspace_mem_slot.memory_size = num_pages << GUEST_PAGE_SHIFT;
    userspace_mem_slot.userspace_addr = host_address << PAGE_SHIFT;

    if (ioctl(_vmfd.get(), KVM_SET_USER_MEMORY_REGION, &userspace_mem_slot) == -1) {
        int err = errno;
        W_MESSAGE("set user memory failed: errno %d (%s)", err, strerror(err));
        return INVALID_KVM_MAP_REF;
    }

    _free_slots.pop_front();

    return userspace_mem_slot.slot + 1;
}


bool KVM::is_active_slot(uint32_t slot)
{
    if (slot >= _num_mem_slot) {
        return false;
    }

    FreeSlotsList::iterator iter = _free_slots.begin();

    for (; iter != _free_slots.end(); iter++ ) {
        if ((*iter) == slot){
            return false;
        }
    }

    return true;
}

void KVM::unmap_mem_slot(KvmMapRef map_ref)
{
    struct kvm_userspace_memory_region userspace_mem_slot;

    memset(&userspace_mem_slot, 0, sizeof(userspace_mem_slot));

    map_ref--;

    if (!is_active_slot(map_ref)) {
        THROW("invalid mem ref");
    }

    userspace_mem_slot.slot = map_ref;

    if (ioctl(_vmfd.get(), KVM_SET_USER_MEMORY_REGION, &userspace_mem_slot) == -1) {
        int err = errno;
        THROW("set user memory failed: errno %d (%s)", err, strerror(err));
    }

    _free_slots.push_front(map_ref);
}

