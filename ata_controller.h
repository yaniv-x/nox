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

#ifndef _H_ATA_CONTROLLER
#define _H_ATA_CONTROLLER

#include "vm_part.h"
#include "threads.h"

class NoxVM;
class PICWire;
class Disk;

class ATAController: public VMPart {
public:
    ATAController(NoxVM& nox, uint irq);

    virtual void reset();
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    void set_disk(Disk* disk) { _disk = disk;}

private:
    void soft_reset();
    uint8_t io_alt_status(uint16_t port);
    void io_control(uint16_t port, uint8_t val);
    uint8_t io_read(uint16_t port);
    void io_write(uint16_t port, uint8_t val);
    uint16_t io_read_word(uint16_t port);
    void io_write_word(uint16_t port, uint16_t data);
    void set_signature();
    void command_abort_error();
    void clear_HOB();
    void set_config();
    void identify_device();
    void do_read_sectors();
    void do_write_sectors();
    void do_command(uint8_t command);
    void raise();
    bool is_valid_sectors_range(uint64_t start, uint64_t end);
    uint get_sector_count();
    uint64_t get_num_sectors();
    uint64_t get_sector_address();

private:
    Mutex _mutex;
    Disk* _disk;
    std::auto_ptr<PICWire> _irq;

    union {
        uint16_t _identity[256];
        uint8_t _sector[512];
    };

    uint _status;
    uint _control;
    uint _error;
    uint _device;
    uint _count;
    uint _lba_low;
    uint _lba_mid;
    uint _lba_high;
    uint _feature;

    uint _heads_per_cylinder;
    uint _sectors_per_track;
    uint16_t* _data_in;
    uint16_t* _data_in_end;
    uint64_t _next_sector;
    uint64_t _end_sector;
};

#endif

