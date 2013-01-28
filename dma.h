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

#ifndef _H_DMA
#define _H_DMA

#include "vm_part.h"

class NoxVM;

class DMA: public VMPart {
public:
    DMA(NoxVM& vm);
    virtual ~DMA();

    virtual void reset();
    virtual bool start() { return true;}
    virtual bool stop() { return true;}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

private:

    enum {
        NUM_CHIPS = 2,
        NUM_CHANELS = 4,
        NUM_IO_REGIONS = 7,
    };

    struct Channel {
        uint address_reda_flipflop;
        uint address_write_flipflop;
        uint counter_reda_flipflop;
        uint counter_write_flipflop;

        uint address;
        uint counter;
        uint page;
        uint mode;
        uint mask;
    };

    struct Chip {
        Channel chanels[NUM_CHANELS];
    };

    uint8_t read_byte(uint16_t port);
    void write_byte(uint16_t port, uint8_t val);
    void disable_dma1();
    void disable_dma2();

private:
    Chip _chips[NUM_CHIPS];
};

#endif

