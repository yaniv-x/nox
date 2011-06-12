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

#ifndef _H_ATA_HOST
#define _H_ATA_HOST

#include "pci_device.h"

class ATADeviceFactory;
class ATADevice;
class Wire;


class ATAHost: public PCIDevice {
public:
    ATAHost();

    void set_device_0(ATADeviceFactory& factory);
    void set_device_1(ATADeviceFactory& factory);

protected:
    virtual void reset();

    virtual void on_io_enabled();
    virtual void on_io_disabled();

private:
    uint8_t io_channel_0_alt_status(uint16_t port);
    void io_channel_0_control(uint16_t port, uint8_t val);
    uint8_t io_channel_0_read(uint16_t port);
    void io_channel_0_write(uint16_t port, uint8_t val);
    uint16_t io_channel_0_read_word(uint16_t port);
    void io_channel_0_write_word(uint16_t port, uint16_t data);

    uint8_t io_channel_1_alt_status(uint16_t port);
    void io_channel_1_control(uint16_t port, uint8_t val);
    uint8_t io_channel_1_read(uint16_t port);
    void io_channel_1_write(uint16_t port, uint8_t val);
    uint16_t io_channel_1_read_word(uint16_t port);
    void io_channel_1_write_word(uint16_t port, uint16_t data);

    uint8_t io_bus_maste_read(uint16_t port);
    void io_bus_maste_write(uint16_t port, uint8_t val);

    void set_bm_command(uint8_t val, uint8_t* reg, ATADevice* device);
    void set_bm_status(uint8_t val, uint8_t* reg);

private:
    std::auto_ptr<Wire> _channel_0_wire;
    std::auto_ptr<Wire> _channel_1_wire;
    std::auto_ptr<ATADevice> _channel_0;
    std::auto_ptr<ATADevice> _channel_1;

    Mutex _bm_mutex;
    uint16_t _bus_master_io_base;
    uint8_t _bus_master_regs[16];
};


class ATADeviceFactory: public NonCopyable {
public:
    virtual ATADevice* creat_device(VMPart& owner, Wire& wire) = 0;
};


#endif

