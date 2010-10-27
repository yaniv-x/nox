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

#ifndef _H_VGA
#define _H_VGA

#include "vm_part.h"
#include "threads.h"

class NoxVM;
class Timer;
class PhysicalRam;
class MMIORegion;
class VGABackEndImp;

class SharedBuf: public NonCopyable {
public:
    SharedBuf(uint size)
        : _refs (1)
        , _size (size)
        , _data (new uint8_t[size])
    {
    }

    SharedBuf* ref() {_refs.inc(); return this;}
    void unref() { if (!_refs.dec()) delete this;}

    uint8_t* get() { return _data;}
    uint size() { return _size;}

protected:
    virtual ~SharedBuf() { delete[] _data;}

private:
    Atomic _refs;
    uint _size;
    uint8_t* _data;
};

class VGAFrontEnd {
public:
    virtual void on_size_changed(uint32_t width, uint32_t hight) = 0;
    virtual void invalid() = 0;
};

class VGABackEnd {
public:
    virtual void detach() = 0;
    virtual const uint8_t* get_fb() = 0;
    virtual void get_size(uint32_t* width, uint32_t* hight) = 0;
};

class VGA: public VMPart {
public:
    VGA(NoxVM& nox);
    virtual ~VGA();

    virtual void reset();
    virtual void start() {}
    virtual void stop() {}
    virtual void power() {}
    virtual void save(OutStream& stream) {}
    virtual void load(InStream& stream) {}

    VGABackEnd* attach_front_end(VGAFrontEnd* front_and);

private:
    uint8_t io_read_byte(uint16_t port);
    void io_write_byte(uint16_t port, uint8_t val);
    void set_misc_reg(uint8_t val);
    void reset_sequencer();
    uint8_t fetch_pix_16(const uint8_t* fb_ptr, uint offset);
    uint32_t text_color(uint nibble);
    uint32_t foreground_color_at(uint fb_pos);
    void show_caret();
    void hide_caret();
    void update_caret();
    void update();
    void reset_io();
    void reset_fb();
    void blank_screen();
    void on_crt_mode_cahnge();

    void vram_load_one(uint32_t offset, uint8_t& dest);
    void vram_read_one(uint32_t src, uint8_t& dest);
    void vram_read(uint64_t src, uint64_t length, uint8_t* dest);

    void vram_store_byte(uint64_t offset, uint8_t val);
    void vram_write_one(uint64_t dest, uint8_t val);
    void vram_write(const uint8_t* src, uint64_t length, uint64_t dest);

    bool font_bit(uint8_t ch, int i, int j, uint char_w, uint char_h);
    void draw_char(uint8_t ch, uint8_t attrib, uint32_t* dest, uint char_w, uint char_h);

    enum {
        ATTRIB_NUM_REGS = 32,
        SEQUENCER_NUM_REGS = 8,
        GRAPHICS_NUM_REGS = 16,
        PALETTE_SIZE = 256,
        CRT_NUM_REGS = 0x25,
    };

private:
    Mutex _mutex;
    Timer* _update_timer;
    IORegion* _region1;
    IORegion* _region2;
    PhysicalRam* _physical_ram;
    uint8_t* _vram;
    uint8_t* _vram_end;
    MMIORegion* _mmio;
    uint8_t _last_io_delta;
    uint8_t _mmap_state;
    bool _active;

    uint8_t _attrib_control_index;
    bool _write_attrib;
    uint8_t _attributes_regs[ATTRIB_NUM_REGS];
    uint8_t _misc_output;
    uint8_t _sequencer_index;
    uint8_t _sequencer_regs[SEQUENCER_NUM_REGS];
    uint8_t _color_index_mask;
    uint8_t _dac_state;
    uint8_t _palette_read_index;
    int8_t _palette_read_comp;
    uint8_t _palette_write_index;
    int8_t _palette_write_comp;

    uint8_t _graphics_index;
    uint8_t _graphics_regs[GRAPHICS_NUM_REGS];

    uint8_t _feature_cntrol;

    uint8_t _crt_index;
    uint8_t _crt_regs[CRT_NUM_REGS];

    struct PaletteEnt {
        union {
            uint8_t components[4];
            uint32_t color;
        };
    } _palette[PALETTE_SIZE];

    AutoRef<SharedBuf> _fb;
    uint _width;
    uint _height;
    bool _dirty;

    typedef std::list<VGABackEndImp*> FrontEndList;
    FrontEndList _front_ends;

    nox_time_t _caret_tick;
    bool _caret_visable;

    friend class VGABackEndImp;
};

#endif

