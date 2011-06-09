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

#ifndef _H_ATA_DISK
#define _H_ATA_DISK

#include "ata_device.h"
#include "ata_host.h"
#include "block_device.h"

class ATADisk: public ATADevice, public BlockDeviceCallback {
public:
    ATADisk(VMPart& owner, Wire& wire, const std::string& file_name, bool read_only);
    virtual ~ATADisk();

    uint64_t get_size();

    virtual uint8_t io_read(uint16_t port);

protected:
    virtual void set_signature();
    virtual void do_command(uint8_t command);
    virtual void reset(bool cold);

private:
    bool is_valid_sectors_range(uint64_t start, uint64_t end);
    uint64_t get_sector_address();
    uint64_t get_sector_address_ext();
    uint get_sector_count();
    uint get_sector_count_ext();

    void do_read_sectors_common(uint64_t start, uint64_t end, uint bunch);
    void do_read_sectors();
    void do_read_sectors_ext();
    void do_read_multi();
    void do_read_multi_ext();
    void do_read_verify_sectors();
    void do_read_verify_sectors_ext();
    void do_read_dma_common(uint64_t start, uint64_t end);
    void do_read_dma();
    void do_read_dma_ext();
    void do_write_sectors_common(uint64_t start, uint64_t end, uint bunch);
    void do_write_sectors();
    void do_write_sectors_ext();
    void do_write_multi();
    void do_write_multi_ext();
    void do_write_dma_common(uint64_t start, uint64_t end);
    void do_write_dma();
    void do_write_dma_ext();
    void do_identify_device();
    void do_set_multi_mode();
    void do_set_features();
    void do_initialize_device_parameters();
    void do_flush();
    void do_idle_immediate();
    void do_standby_immediate();

    void sync(void* mark);

    virtual void sync_done(void* mark);
    virtual void sync_failed(void*, int error);
    virtual void block_io_done(Block* block) {}
    virtual void block_io_error(Block* block, int error) {}

private:
    std::auto_ptr<BlockDevice> _block_dev;

    bool _sync_mode;
    uint _heads_per_cylinder;
    uint _sectors_per_track;
    uint _multi_mode;

    friend class ReadTask;
    friend class ReadDMATask;
    friend class WriteTask;
    friend class WriteDMATask;
    friend class IdentifyTask;
    friend class SyncTask;
};


class ATADiskFactory: public ATADeviceFactory {
public:
    ATADiskFactory(const std::string& file_mame, bool read_only);
    virtual ATADevice* creat_device(VMPart &owner, Wire &wire);
    uint64_t get_size() { return _device->get_size();}

private:
    std::string _file_name;
    ATADisk* _device;
    bool _read_only;
};

#endif

