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

#ifndef _H_PCI_BUS
#define _H_PCI_BUS

#include "vm_part.h"

class NoxVM;
class PCIDevice;

class PCIBus: public VMPart {
public:
    PCIBus(NoxVM& nox);
    virtual ~PCIBus();

    virtual void load(InStream &stream) {}
    virtual void power() {}
    virtual void reset() { _config_address = 0;}
    virtual void save(OutStream &stream) {}
    virtual void start() {}
    virtual void stop() {}

    void add_device(PCIDevice& device);

private:
    uint32_t io_get_config_address(uint16_t port);
    void io_set_config_address(uint16_t port, uint32_t val);
    uint8_t io_read_config_byte(uint16_t port);
    void io_write_config_byte(uint16_t port, uint8_t val);
    uint16_t io_read_config_word(uint16_t port);
    void io_write_config_word(uint16_t port, uint16_t val);
    uint32_t io_read_config_dword(uint16_t port);
    void io_write_config_dword(uint16_t port, uint32_t val);


    bool is_valid_address();
    PCIDevice* get_target();

    enum {
        PCI_MAX_DEVICES = 32,
        PCI_MAX_FUNCTIONS = 8,
    };

private:
    uint32_t _config_address;
    PCIDevice* _devices[PCI_MAX_DEVICES];

};

#endif

