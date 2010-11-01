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

#include "vga.h"
#include "nox_vm.h"
#include "io_bus.h"
#include "memory_bus.h"
#include "application.h"

#define VGA_D_MESSAGE(format, ...)

// todo: odd even bits in multiple regs

enum {
    IO_VGA_BASE = 0x3c0,
    IO_VGA_END = 0x3d0,

    VRAM_START = 0xa0000,
    VRAM_SIZE = 256 * KB,

    CARET_RATE = 1000 * 1000 * 1000 / 2,

    IO_ATTRIB_CONTROL_INDEX = 0x3c0,
    IO_ATTRIB_READ = 0x3c1,
    IO_INPUT_STATUS_0 = 0x3c2,
    IO_MISC_OUTPUT_W = 0x3c2,
    IO_SEQUENCER_INDEX = 0x3c4,
    IO_SEQUENCER_DATA = 0x3c5,
    IO_COLOR_INDEX_MASK = 0x3c6,
    IO_DEC_DAC_STATE = 0x3c7,
    IO_PALETTE_READ_INDEX = 0x3c7,
    IO_PALETTE_WRITE_INDEX = 0x3c8,
    IO_PALETTE_DATA = 0x3c9,
    IO_FEATURE_CONTROL_R = 0x3ca,
    IO_MISC_OUTPUT_R = 0x3cc,
    IO_GRAPHICS_INDEX = 0x3ce,
    IO_GRAPHICS = 0x3cf,
    IO_INPUT_STATUS_1 = 0x3da,
    IO_INPUT_STATUS_1_MDA = 0x3ba,
    IO_FEATURE_CONTROL_W = 0x3da,
    IO_FEATURE_CONTROL_W_MDA = 0x3ba,

    IO_CRT_CONTROL_INDEX = 0x3d4,
    IO_CRT_CONTROL_INDEX_MDA = 0x3b4,
    IO_CRT_CONTROL = 0x3d5,
    IO_CRT_CONTROL_MDA = 0x3b5,

    INPUT_STATUS_0_IRQ_PENDING = (1 << 7),
    INPUT_STATUS_0_COLOR_DISPLAY_MASK = (1 << 4),


    INPUT_STATUS1_V_RETRACE_MASK = (1 << 3), // Bits 4 and 5 of the Vertical Retrace End
                                             // Register (CR11) can program this bit to
                                             // generate an interrupt at the start of the
                                             // vertical retrace interval.
    INPUT_STATUS1_DISPLAY_ENABLE_MASK = (1 << 0),


    MISC_FB_ACCESS_MASK = (1 << 1),
    MISC_IO_ADDRESS_SELECT_MASK = (1 << 0),
    MISC_PAGE_SELECT_MASK = (1 << 5),


    CRT_REG_WIDTH_TOTAL = 0x00,
    CRT_REG_WIDTH = 0x01,
    CRT_REG_HEIGHT_TOTAL = 0x06,
    CRT_REG_OVERFLOW = 0x07,
    CRT_REG_MAX_SCAN_LINE = 0x09,
    CRT_REG_CURSOR_START = 0x0a,
    CRT_REG_CURSOR_END = 0x0b,
    CRT_REG_START_ADDRESS_HIGH = 0x0c,
    CRT_REG_START_ADDRESS_LOW = 0x0d,
    CRT_REG_CURSOR_LOCATION_HIGH = 0x0e,
    CRT_REG_CURSOR_LOCATION_LOW = 0x0f,
    CRT_REG_VERTICAL_RETRACE = 0x11,
    CRT_REG_HEIGHT = 0x12,
    CRT_REG_MODE = 0x17,
    CRT_REG_TEST = 0x24,

    CRT_MODE_ACTIVE_MASK = (1 << 7),

    CRT_OVERFLAOW_HT8_BIT = 0,
    CRT_OVERFLAOW_HT9_BIT = 5,
    CRT_OVERFLAOW_H8_BIT = 1,
    CRT_OVERFLAOW_H9_BIT = 6,

    CRT_CURSOR_START_OFF = (1 << 5),
    CRT_CURSOR_START_MASK = (1 << 5) - 1,

    CRT_CURSOR_END_MASK = (1 << 5) - 1,

    CRT_MAX_SCAN_LINE_TEXT_HIGHET_MASK = (1 << 5) - 1,


    ATTRIB_INDEX_MASK = (1 << 5) - 1,

    ATTRIB_REG_MODE = 0x10,
    ATTRIB_REG_OVERSCAN = 0x11,
    ATTRIB_REG_MEM_PLANE = 0x12,
    ATTRIB_REG_COLOR_SELECT = 0x14,

    ATTRIB_INDEX_DISABLE_MASK =  (1 << 5),
    ATTRIB_MODE_PAL_FIXED_4_5_MASK = (1 << 7),
    ATTRIB_MODE_GRAPHICS_BIT = 0,
    ATTRIB_MEM_PLANE_STATUS_MUX_SHIFT = 4,
    ATTRIB_MEM_PLANE_STATUS_MUX_MASK = (0x3 << ATTRIB_MEM_PLANE_STATUS_MUX_SHIFT),


    SEQUENCER_INDEX_MASK = (1 << 3) - 1,

    SEQUENCER_REG_CLOCKING = 0x01,
    SEQUENCER_REG_PLANE = 0x02,
    SEQUENCER_REG_FONT = 0x03,
    SEQUENCER_REG_MEM_MODE = 0x04,

    SEQUENCER_CLOCKIND_BLANK_BIT = 5,
    SEQUENCER_CLOCKIND_DOTS_BIT = 0,

    SEQUENCER_FONT_MAPA_BIT0 = 5,
    SEQUENCER_FONT_MAPA_BIT1 = 2,
    SEQUENCER_FONT_MAPB_BIT0 = 4,
    SEQUENCER_FONT_MAPB_BIT1 = 0,

    SEQUENCER_MEM_MODE_CHAIN_MASK =  (1 << 3),
    SEQUENCER_MEM_MODE_ODD_EVEN_MASK =  (1 << 2),


    GRAPHICS_INDEX_MASK = (1 << 4) - 1,

    GRAPHICS_REG_SET_RESET = 0x00,
    GRAPHICS_REG_ENABLE_SET_RESET = 0x01,
    GRAPHICS_REG_ROTATE = 0x03,
    GRAPHICS_REG_READ_PLAN = 0x04,
    GRAPHICS_REG_MODE = 0x05,
    GRAPHICS_REG_MISC = 0x06,

    GRAPHICS_ROTATE_COUNT_MASK = (1 << 3) - 1,
    GRAPHICS_ROTATE_FUNC_MASK = (0x03 << 3),
    GRAPHICS_ROTATE_FUNC_NONE = (0 << 3),
    GRAPHICS_ROTATE_FUNC_AND = (1 << 3),
    GRAPHICS_ROTATE_FUNC_OR = (2 << 3),
    GRAPHICS_ROTATE_FUNC_XOR = (3 << 3),

    GRAPHICS_READ_PLAN_MASK = (1 << 2) - 1,

    GRAPHICS_MODE_256C_MASK = (1 << 6),
    GRAPHICS_MODE_4C_MASK = (1 << 5),
    GRAPHICS_MODE_ODD_EVEN = (1 << 4),
    GRAPHICS_MODE_READ_MODE_MASK = (1 << 3),
    GRAPHICS_MODE_WRITE_MODE_MASK = (1 << 2) - 1,

    GRAPHICS_MISK_ODD_EVANE_BIT = 5,
    GRAPHICS_MISK_FB_ADDR_MASK_SHIFT = 2,
    GRAPHICS_MISK_FB_ADDR_A0000_BFFFF = 0,
    GRAPHICS_MISK_FB_ADDR_A0000_AFFFF = 1,
    GRAPHICS_MISK_FB_ADDR_B0000_B7FFF = 2,
    GRAPHICS_MISK_FB_ADDR_B8000_BFFFF = 3,
};


//this naive imp. need to improve thread safety and detaching
class VGABackEndImp : public NonCopyable, public VGABackEnd {
public:
    VGABackEndImp(VGA* vga, VGAFrontEnd* front_end)
        : _vga (vga)
        , _fb (_vga->_fb->ref())
        , _front_end (front_end)
    {
    }

    ~VGABackEndImp()
    {
        _fb->unref();
    }

    virtual void detach()
    {
        Lock lock(_mutex);
        VGA::FrontEndList::iterator iter = _vga->_front_ends.begin();

        for (; iter != _vga->_front_ends.end(); iter++) {
            if ((*iter) == this) {
                _vga->_front_ends.erase(iter);
                delete this;
                return;
            }
        }
    }

    virtual const uint8_t* get_fb()
    {
        return _fb->get();
    }

    virtual void get_size(uint32_t* width, uint32_t* hight)
    {
        *width = _vga->_width;
        *hight = _vga->_height;
    }

    VGAFrontEnd* get_front_end() { return _front_end;}

public:
    Mutex _mutex;
    VGA* _vga;
    SharedBuf* _fb;
    VGAFrontEnd* _front_end;
};


enum {
    MAX_WIDTH = 740,
    MAX_HIGHT = 480,
};


VGA::VGA(NoxVM& nox)
    : VMPart("vga", nox)
    , _mmio (NULL)
    , _fb (new SharedBuf(MAX_WIDTH * MAX_HIGHT * sizeof(uint32_t)))
    , _width (640)
    , _height (480)
    , _caret_tick (0)
    , _caret_visable (0)
{
    IOBus& io_bus = nox.get_io_bus();

    add_io_region(io_bus.register_region(*this, IO_VGA_BASE, IO_VGA_END - IO_VGA_BASE, this,
                                         (io_read_byte_proc_t)&VGA::io_read_byte,
                                         (io_write_byte_proc_t)&VGA::io_write_byte));

    _update_timer = application->create_timer((void_callback_t)&VGA::update, this);
    _update_timer->arm(1000 * 1000 * 1000 / 30, true); // arm/disarm in start/stop

    _physical_ram = memory_bus->alloc_physical_ram(*this, VRAM_SIZE / GUEST_PAGE_SIZE, "vga ram");
    _vram = memory_bus->get_physical_ram_ptr(_physical_ram);
    _vram_end = _vram + VRAM_SIZE;

    reset();
}


VGA::~VGA()
{
    _update_timer->destroy();

    io_bus->unregister_region(_region1);
    io_bus->unregister_region(_region2);

    memory_bus->unregister_mmio(_mmio);
    memory_bus->release_physical_ram(_physical_ram);
}


bool VGA::font_bit(uint8_t ch, int i, int j, uint char_w, uint char_h)
{
    // no font select support (SR03 Character Font)
    uint offset = uint(ch) * 32 + i;
    uint8_t c = _vram[(offset << 2) + 2];
    return !!(c & (0x80 >> j));
}


uint32_t VGA::text_color(uint nibble)
{
    uint8_t pal_high_bits;

    ASSERT(nibble < (1 << 4));

    if (_attributes_regs[ATTRIB_REG_MODE] & ATTRIB_MODE_PAL_FIXED_4_5_MASK) {
        pal_high_bits = (_attributes_regs[ATTRIB_REG_COLOR_SELECT] << 4) & 0xf0;
    } else {
        pal_high_bits = (_attributes_regs[ATTRIB_REG_COLOR_SELECT] << 4) & 0xc0;
    }

    uint index = _attributes_regs[nibble] | pal_high_bits;

    return _palette[index & _color_index_mask].color;
}


void VGA::draw_char(uint8_t ch, uint8_t attrib, uint32_t* dest, uint char_w, uint char_h)
{
    // no blinking or font select support

    // todo: prepare actual palette on graphic mode change, on palette entery change or on any
    //       other change that effect translation of index to color

    for (int i= 0; i < char_h; i++) {
        for (int j= 0; j < char_w; j++) {

            if (font_bit(ch, i, j, char_w, char_h)) {
                *(dest + j) = text_color(attrib & 0xf);
            } else {
                *(dest + j) = text_color(attrib >> 4);
            }
        }

        dest += _width;
    }
}


uint8_t VGA::fetch_pix_16(const uint8_t* fb_ptr, uint offset)
{
    uint bit = 7 - offset % 8;
    uint mask = 1 << bit;
    const uint8_t* byte = fb_ptr + offset / 8 * 4;

    return  ((byte[0] & mask) >> bit) << 0 |
            ((byte[1] & mask) >> bit) << 1 |
            ((byte[2] & mask) >> bit) << 2 |
            ((byte[3] & mask) >> bit) << 3;
}


uint32_t VGA::foreground_color_at(uint fb_pos)
{
    return text_color(_vram[(fb_pos << 2) + 1] & 0xf);
}


void VGA::show_caret()
{
    if (_crt_regs[CRT_REG_CURSOR_START] & CRT_CURSOR_START_OFF) {
        D_MESSAGE("off");
        return;
    }

    uint char_w;
    uint char_h;

    char_w = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKIND_DOTS_BIT) ? 8 : 9;
    char_h = (_crt_regs[CRT_REG_MAX_SCAN_LINE] & CRT_MAX_SCAN_LINE_TEXT_HIGHET_MASK) + 1;

    uint line_size = _width / char_w;
    uint lines = _height / char_h;


    uint pos = (uint(_crt_regs[CRT_REG_CURSOR_LOCATION_HIGH]) << 8) |
               _crt_regs[CRT_REG_CURSOR_LOCATION_LOW];

    uint32_t color = foreground_color_at(pos);

    uint32_t fb_offset = (uint32_t(_crt_regs[CRT_REG_START_ADDRESS_HIGH]) << 8) +
                         _crt_regs[CRT_REG_START_ADDRESS_LOW];

    pos -= fb_offset;

    uint cursor_row = pos / line_size;
    uint cursor_col = pos % line_size;

    if (cursor_row >= lines) {
        D_MESSAGE("bad position");
        return;
    }

    uint h_start = _crt_regs[CRT_REG_CURSOR_START] & CRT_CURSOR_START_MASK;
    uint h_end = MIN((_crt_regs[CRT_REG_CURSOR_END] & CRT_CURSOR_END_MASK) + 1, char_h);
    uint32_t* line_start;

    line_start = (uint32_t*)_fb->get() + cursor_row * char_h * _width + cursor_col * char_w;
    line_start += h_start * _width;

    for (; h_start < h_end; ++h_start) {
        for (int j = 0; j < char_w; j++) {
            line_start[j] = color;
        }

        line_start += _width;
    }

    _caret_visable = true;
}


void VGA::hide_caret()
{
    uint cursor_pos = (uint(_crt_regs[CRT_REG_CURSOR_LOCATION_HIGH]) << 8) |
                      _crt_regs[CRT_REG_CURSOR_LOCATION_LOW];

    uint char_w;
    uint char_h;

    char_w = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKIND_DOTS_BIT) ? 8 : 9;
    char_h = (_crt_regs[CRT_REG_MAX_SCAN_LINE] & CRT_MAX_SCAN_LINE_TEXT_HIGHET_MASK) + 1;
    uint line_size = _width / char_w;
    uint lines = _height / char_h;

    uint fb_offset = (uint(_crt_regs[CRT_REG_START_ADDRESS_HIGH]) << 8) +
                      _crt_regs[CRT_REG_START_ADDRESS_LOW];

    uint pos = cursor_pos - fb_offset;

    uint cursor_row = pos / line_size;
    uint cursor_col = pos % line_size;

    if (cursor_row >= lines) {
        D_MESSAGE("bad position");
        return;
    }

    uint32_t* dest = (uint32_t*)_fb->get() + cursor_row * char_h * _width + cursor_col * char_w;
    draw_char(_vram[cursor_pos << 2], _vram[(cursor_pos << 2) + 1], dest, char_w, char_h);

    _caret_visable = false;
}


void VGA::update_caret()
{
    if (!_caret_tick) {
        return;
    }

    nox_time_t now = get_monolitic_time();

    if (now - _caret_tick < CARET_RATE) {
        return;
    }

    _caret_tick = now;

    if (_caret_visable) {
        hide_caret();
    } else {
        show_caret();
    }
}


void VGA::update()
{
    Lock lock(_mutex);

    if (!_active) {
        return;
    }

    if (!_dirty) {
        update_caret();
        return;
    }

    _dirty = false;

    uint width = _width;
    uint height = _height;

    uint32_t fb_offset = (uint32_t(_crt_regs[CRT_REG_START_ADDRESS_HIGH]) << 8) +
                       _crt_regs[CRT_REG_START_ADDRESS_LOW];

    uint8_t* fb_ptr = _vram + (fb_offset << 2);

    bool text_mode = !(_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT));

    if (text_mode) {
         uint char_w;
         uint char_h;

         char_w = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKIND_DOTS_BIT) ? 8 : 9;
         char_h = (_crt_regs[CRT_REG_MAX_SCAN_LINE] & CRT_MAX_SCAN_LINE_TEXT_HIGHET_MASK) + 1;

         uint line_size = width / char_w;
         uint lines = height / char_h;

         uint32_t* dest_line = (uint32_t*)_fb->get();

         lines = MIN(lines, (_vram_end - fb_ptr) / 4 / width);

         for (int i = 0; i < lines; i++) {
             uint32_t* dest_char = dest_line;

             for (int j = 0; j < line_size; j++) {
                 draw_char(*fb_ptr, *(fb_ptr + 1), dest_char, char_w, char_h);
                 fb_ptr += 4;
                 dest_char += char_w;
             }

             dest_line += width * char_h;
         }

         update_caret();

         return;
    }

    if (_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_256C_MASK) {
        D_MESSAGE("256 colors");
    } else if (_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_4C_MASK ) {
        D_MESSAGE("4 colors");
    } else {
        uint8_t pal_high_bits;
        uint8_t pal_mask;

        // todo: prepare actual palette on graphic mode change, on palette entery change or on any
        //       other change that effect translation of index to color

        if (_attributes_regs[ATTRIB_REG_MODE] & ATTRIB_MODE_PAL_FIXED_4_5_MASK) {
            pal_high_bits = (_attributes_regs[ATTRIB_REG_COLOR_SELECT] << 4) & 0xf0;
            pal_mask = 0x0f;
        } else {
            pal_high_bits = (_attributes_regs[ATTRIB_REG_COLOR_SELECT] << 4) & 0xc0;
            pal_mask = 0x3f;
        }

        uint32_t* dest = (uint32_t*)_fb->get();
        uint pixels = height * width;

        pixels = MIN(pixels, (_vram_end - fb_ptr) << 1);

        for (uint i = 0; i < pixels; i++) {
            uint8_t pal_index;

            pal_index = (_attributes_regs[fetch_pix_16(fb_ptr, i)] & pal_mask) | pal_high_bits;
            dest[i] = _palette[pal_index & _color_index_mask].color;
        }
    }

    lock.unlock();
}


void VGA::reset_io()
{
    uint8_t delta = (_misc_output & MISC_IO_ADDRESS_SELECT_MASK) ? 0xd0 - 0xb0 : 0;
    IORegion* region;

    if (_last_io_delta == delta) {
        return;
    }

    region = _region1;
    _region1 = NULL;
    io_bus->unregister_region(region);
    region = _region2;
    _region2 = NULL;
    io_bus->unregister_region(region);

    _region1 = io_bus->register_region(*this, IO_INPUT_STATUS_1_MDA + delta, 1, this,
                                       (io_read_byte_proc_t)&VGA::io_read_byte,
                                       (io_write_byte_proc_t)&VGA::io_write_byte);

    _region2 = io_bus->register_region(*this, IO_CRT_CONTROL_INDEX_MDA + delta, 2, this,
                                       (io_read_byte_proc_t)&VGA::io_read_byte,
                                       (io_write_byte_proc_t)&VGA::io_write_byte);

    _last_io_delta = delta;
}


void VGA::reset_fb()
{
    // setup EXCLUSIVE EXEC trap and relase the lock. The trap will throw ExclusiveRestart
    // exception to the io_bus

    // MISC_PAGE_SELECT_MASK ?

    bool fb_accesibel = !!(_misc_output & MISC_FB_ACCESS_MASK);
    uint8_t fb_address = (_graphics_regs[GRAPHICS_REG_MISC] >> GRAPHICS_MISK_FB_ADDR_MASK_SHIFT);
    fb_address &= 0x03;

    uint8_t new_state = fb_accesibel ? 1 : 0;
    new_state |= (fb_address << 1);

    if (new_state == _mmap_state) {
        return;
    }

     memory_bus->unregister_mmio(_mmio);
    _mmio = NULL;

    if (!fb_accesibel) {
        return;
    }

    switch (fb_address) {
    case GRAPHICS_MISK_FB_ADDR_A0000_BFFFF:
        _mmio = memory_bus->register_mmio(0xa0000 / GUEST_PAGE_SIZE, 128 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        break;
    case GRAPHICS_MISK_FB_ADDR_A0000_AFFFF:
        _mmio = memory_bus->register_mmio(0xa0000 / GUEST_PAGE_SIZE, 64 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        break;
    case GRAPHICS_MISK_FB_ADDR_B0000_B7FFF:
        _mmio = memory_bus->register_mmio(0xb0000 / GUEST_PAGE_SIZE, 32 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        break;
    case GRAPHICS_MISK_FB_ADDR_B8000_BFFFF:
        _mmio = memory_bus->register_mmio(0xb8000 / GUEST_PAGE_SIZE, 32 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        break;
    }
}


void VGA::reset()
{
    _last_io_delta = ~0;
    _mmap_state = ~0;
    _active = false;
    memset(_fb->get(), 0,  _fb->size());

    _misc_output = 0;
    _sequencer_index = 0;
    memset(_sequencer_regs, 0, sizeof(_sequencer_regs));

    _write_attrib = false;
    _attrib_control_index = 0;
    memset(_attributes_regs, 0, sizeof(_attributes_regs));

    _color_index_mask = ~0;
    _dac_state = 0;
    _palette_read_index = 0;
    _palette_read_comp = 2;
    _palette_write_index = 0;
    _palette_write_comp = 2;
    memset(_palette, 0, sizeof(_palette));

    _graphics_index = 0;
    memset(_graphics_regs, 0, sizeof(_graphics_regs));

    _feature_cntrol = 0;

    _crt_index = 0;
    memset(_crt_regs, 0, sizeof(_crt_regs));

    _caret_tick = 0;
    _caret_visable = 0;

    reset_io();
    reset_fb();
}

static uint8_t v_retrace = 0;

uint8_t VGA::io_read_byte(uint16_t port)
{
    Lock lock(_mutex);

    switch (port) {
    case IO_INPUT_STATUS_0:
        return INPUT_STATUS_0_COLOR_DISPLAY_MASK;
    case IO_INPUT_STATUS_1:
    case IO_INPUT_STATUS_1_MDA: {
        v_retrace ^= INPUT_STATUS1_DISPLAY_ENABLE_MASK | INPUT_STATUS1_V_RETRACE_MASK;

        uint8_t status_1 = v_retrace;

        VGA_D_MESSAGE("resetting attribute flipflop");
        _write_attrib = false;

        // bits 5:4
        //      From Intel doc:
        //          "These are diagnostic video bits that are programmably connected to 2 of the 8
        //           color bits sent to the palette."
        //
        //           what is "color bits sent to the palette"
        //
        //      From Cirrus doc:
        //          "These bits follow two of eight outputs of the attribute controller."
        //
        //          what is "outputs of the attribute controller"

        uint8_t attrib_outputs = 0;

        switch ((_attributes_regs[ATTRIB_REG_MEM_PLANE] & ATTRIB_MEM_PLANE_STATUS_MUX_MASK) >>
                                                                ATTRIB_MEM_PLANE_STATUS_MUX_SHIFT) {
        case 0:
            status_1 |= (attrib_outputs & 0x01) << 4;
            status_1 |= (attrib_outputs & 0x04) << 3;
            break;
        case 1:
            status_1 |= (attrib_outputs & 0x30);
            break;
        case 2:
            status_1 |= (attrib_outputs & 0x02) << 3;
            status_1 |= (attrib_outputs & 0x08) << 2;
            break;
        case 3:
            status_1 |= (attrib_outputs & 0xc0) >> 2;
            break;
        }

        return status_1;
    }
    case IO_MISC_OUTPUT_R:
        return _misc_output;
    case IO_SEQUENCER_INDEX:
        return _sequencer_index;
    case IO_SEQUENCER_DATA:
        return _sequencer_regs[_sequencer_index];
    case IO_COLOR_INDEX_MASK:
        return _color_index_mask;
    case IO_DEC_DAC_STATE:
        return _dac_state;
    case IO_PALETTE_DATA:
        if (_palette_read_comp == -1) {
            _palette_read_index++;
            _palette_read_comp = 2;
        }
        return _palette[_palette_read_index].components[_palette_read_comp--] >> 2;
    case IO_ATTRIB_CONTROL_INDEX:
        return _attrib_control_index;
    case IO_ATTRIB_READ:
        return _attributes_regs[_attrib_control_index & ATTRIB_INDEX_MASK];
    case IO_GRAPHICS_INDEX:
        return _graphics_index;
    case IO_FEATURE_CONTROL_R:
        return _feature_cntrol;
    case IO_CRT_CONTROL_INDEX:
        return _crt_index;
    case IO_CRT_CONTROL:
    case IO_CRT_CONTROL_MDA:
        if (_crt_index >= CRT_NUM_REGS) {
            D_MESSAGE("crt index 0x%x is out of range", _crt_index);
            return 0xff;
        }
        return _crt_regs[_crt_index];
    default:
        D_MESSAGE("port 0x%x, inf wait", port);
        for (;;) sleep(1);
    }

    return 0xff;
}


void VGA::set_misc_reg(uint8_t val)
{
    _misc_output = val;
    reset_io();
    reset_fb();
}


void VGA::reset_sequencer()
{
    VGA_D_MESSAGE("");
}


void VGA::blank_screen()
{
    D_MESSAGE("");
}


void VGA::on_crt_mode_cahnge()
{
    bool active = !!(_crt_regs[CRT_REG_MODE] & CRT_MODE_ACTIVE_MASK);;

    if (active == _active) {
        return;
    }

    _active = active;

    if (!_active) {
        return;
    }

    uint overflow = _crt_regs[CRT_REG_OVERFLOW];

    uint height = (((overflow & (1 << CRT_OVERFLAOW_H8_BIT)) << (8 - CRT_OVERFLAOW_H8_BIT)) |
                      ((overflow & (1 << CRT_OVERFLAOW_H9_BIT)) << (9 - CRT_OVERFLAOW_H9_BIT))) +
                        _crt_regs[CRT_REG_HEIGHT] + 1;

    uint dot_clocks;

    dot_clocks = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKIND_DOTS_BIT) ? 8 : 9;

    //maybe vga bios bug?
    if ((_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT))) {
        dot_clocks = 8;
    }

    uint width = (_crt_regs[CRT_REG_WIDTH] + 1) * dot_clocks;

    if (width == _width && height == _height) {
        return;
    }

    if (width > MAX_WIDTH || height > MAX_HIGHT) {
        D_MESSAGE("invalid size");
        return;
    }

    _width = width;
    _height = height;
    _caret_visable = false;

    if (!(_crt_regs[CRT_REG_CURSOR_START] & CRT_CURSOR_START_OFF) &&
        !(_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT))) {
        _caret_tick = get_monolitic_time();
    } else {
        _caret_tick = 0;
    }

    // need to ensure thread safe. Holding lock while calling
    // the front end is a potentail dead lock
    FrontEndList::iterator iter = _front_ends.begin();

    for (; iter != _front_ends.end(); iter++) {
        (*iter)->get_front_end()->on_size_changed(width, height);
    }
}


void VGA::io_write_byte(uint16_t port, uint8_t val)
{
    Lock lock(_mutex);

    switch (port) {
    case IO_MISC_OUTPUT_W:
        set_misc_reg(val);
        VGA_D_MESSAGE("misc_output 0x%x", val);
        break;
    case IO_SEQUENCER_INDEX:
        _sequencer_index = val & SEQUENCER_INDEX_MASK;
        return;
    case IO_SEQUENCER_DATA:
        _sequencer_regs[_sequencer_index] = val;

        if (_sequencer_index == 0) {
            reset_sequencer();
        }

        if (_sequencer_index == SEQUENCER_REG_MEM_MODE) {
            VGA_D_MESSAGE("sequencer[SEQUENCER_REG_MEM_MODE] = 0x%x (%u)", val, val);
        }

        if (_sequencer_index == SEQUENCER_REG_CLOCKING && (val & SEQUENCER_CLOCKIND_BLANK_BIT)) {
            blank_screen();
        }

        VGA_D_MESSAGE("sequencer[%u] = 0x%x (%u)", _sequencer_index, val, val);
        return;
    case IO_COLOR_INDEX_MASK:
        VGA_D_MESSAGE("color index mask 0x%x", val);
        _color_index_mask = val;
        _dirty = true;
        return;
    case IO_PALETTE_READ_INDEX:
        _dac_state = 0x3;
        _palette_read_comp = 2;
        _palette_read_index = val;
        return;
    case IO_PALETTE_WRITE_INDEX:
        _dac_state = 0;
        _palette_write_comp = 2;
        _palette_write_index = val;
        return;
    case IO_PALETTE_DATA:
        if (_palette_write_comp == -1) {
            _palette_write_index++;
            _palette_write_comp = 2;
        }

        VGA_D_MESSAGE("palette[%u].%s = 0x%x", _palette_write_index,
                      _palette_write_comp == 2 ? "red"
                                               : (_palette_write_comp == 1 ? "green" : "blue"),
                      val);

        _palette[_palette_write_index].components[_palette_write_comp--] = val << 2;
        _dirty = true;
        return;
    case IO_ATTRIB_CONTROL_INDEX:
        if (_write_attrib) {
            _write_attrib = false;

            if ((_attrib_control_index & ATTRIB_INDEX_DISABLE_MASK) &&
                (_attrib_control_index & ATTRIB_INDEX_MASK) <= 0x0f) {
                D_MESSAGE("modifay protected reg[0x%x]", _attrib_control_index & ATTRIB_INDEX_MASK);
            }

            _attributes_regs[_attrib_control_index & ATTRIB_INDEX_MASK] = val;
            VGA_D_MESSAGE("attribute[0x%x] = 0x%x", _attrib_control_index & ATTRIB_INDEX_MASK, val);
            _dirty = true;
            return;
        }

        VGA_D_MESSAGE("attribute control index 0x%x", val);
        _write_attrib = true;
        _attrib_control_index = val;

        /*
           if index bit 5 is set the screen displays the color indicated by the
           Overscan register (AR11).
        */

        return;
    case IO_GRAPHICS_INDEX:
        _graphics_index = val & GRAPHICS_INDEX_MASK;
        return;
    case IO_GRAPHICS:

        if (_graphics_index == GRAPHICS_REG_READ_PLAN) {
            val &= GRAPHICS_READ_PLAN_MASK;
        }

        _graphics_regs[_graphics_index] = val;

        if (_graphics_index == GRAPHICS_REG_MISC) {
            reset_fb();
        }

        VGA_D_MESSAGE("graphics[0x%x] = 0x%x", _graphics_index, val);
        return;
    case IO_FEATURE_CONTROL_W:
    case IO_FEATURE_CONTROL_W_MDA:
        _feature_cntrol = val;
        return;
    case IO_CRT_CONTROL_INDEX:
    case IO_CRT_CONTROL_INDEX_MDA:
        _crt_index = val;
        _crt_regs[CRT_REG_TEST] = 0;
        break;
    case IO_CRT_CONTROL:
    case IO_CRT_CONTROL_MDA:
        if (_crt_index > 0x18) {
            if (_crt_index != 0x22 && _crt_index != 0x24) {
                D_MESSAGE("crt index 0x%x is out of range", _crt_index);
            }
            return;
        }

        if (_crt_index == CRT_REG_CURSOR_START &&
            !(_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT)) &&
            (val & CRT_CURSOR_START_OFF) !=
                                  (_crt_regs[CRT_REG_CURSOR_START] & CRT_CURSOR_START_OFF)) {

            if (val & CRT_CURSOR_START_OFF) {
                hide_caret();
                _caret_tick = 0;
            } else {
                _caret_tick = get_monolitic_time() - CARET_RATE;
            }
        }

        _crt_regs[CRT_REG_TEST] = 1 << 7;
        _crt_regs[_crt_index] = val;

        if (_crt_index == CRT_REG_MODE) {
            on_crt_mode_cahnge();
        }

        if ((_crt_index == CRT_REG_CURSOR_LOCATION_HIGH ||
            _crt_index == CRT_REG_CURSOR_LOCATION_LOW) &&
            !(_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT)) &&
            !(_crt_regs[CRT_REG_CURSOR_START] & CRT_CURSOR_START_OFF)) {

            hide_caret();
            _caret_tick = get_monolitic_time() - CARET_RATE;
        }

        if (_crt_index == CRT_REG_START_ADDRESS_HIGH || _crt_index == CRT_REG_START_ADDRESS_LOW) {
            _dirty = true;
        }

        VGA_D_MESSAGE("crt[0x%x] = 0x%x", _crt_index, val);
        break;
    default:
        D_MESSAGE("port 0x%x val %u (0x%x), inf wait", port, val, val);
        for (;;) sleep(1);
    }
}


VGABackEnd* VGA::attach_front_end(VGAFrontEnd* front_and)
{
    VGABackEndImp* back_end = new VGABackEndImp(this, front_and);
    Lock lock(_mutex);
    _front_ends.push_back(back_end);
    return back_end;
}


inline void VGA::vram_load_one(uint32_t offset, uint8_t& dest)
{
    if (_vram + offset >= _vram_end) {
        D_MESSAGE("out of bounds");
        dest = 0xff;
        return;
    }

    dest = _vram[offset];
}


inline void VGA::vram_read_one(uint32_t src, uint8_t& dest)
{
    ASSERT(!!(_sequencer_regs[SEQUENCER_REG_MEM_MODE] & SEQUENCER_MEM_MODE_ODD_EVEN_MASK) ==
           !(_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_ODD_EVEN));

    if (_sequencer_regs[SEQUENCER_REG_MEM_MODE] & SEQUENCER_MEM_MODE_CHAIN_MASK) {
        vram_load_one(src, dest);
    } else if ((_sequencer_regs[SEQUENCER_REG_MEM_MODE] & SEQUENCER_MEM_MODE_ODD_EVEN_MASK)) {
        vram_load_one((src << 2) | _graphics_regs[GRAPHICS_REG_READ_PLAN], dest);
    } else {
        uint32_t plan = (src & 1) | (_graphics_regs[GRAPHICS_REG_READ_PLAN] & 2);
        vram_load_one(((src & ~1) << 1) | plan, dest);
    }
}


void VGA::vram_read(uint64_t src, uint64_t length, uint8_t* dest)
{
    if (!(_misc_output & MISC_FB_ACCESS_MASK)) {
        return;
    }

    if ((_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_READ_MODE_MASK)) {
         W_MESSAGE_SOME(100, "implement me");
         memset(dest, 0xff, length);
        return;
    }

    for (int i = 0; i < length; i++) {
        vram_read_one(src + i, dest[i]);
    }
}


inline void VGA::vram_store_byte(uint64_t offset, uint8_t val)
{
    if (_vram + offset >= _vram_end) {
        return;
    }

    switch (_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_WRITE_MODE_MASK) {
    case 0: {
        uint plan = offset & 0x03;
        if (_graphics_regs[GRAPHICS_REG_ENABLE_SET_RESET] & (1 << plan)) {
            _vram[offset] = (_graphics_regs[GRAPHICS_REG_SET_RESET] & (1 << plan)) ? ~0 : 0;
            return;
        }

        if (_graphics_regs[GRAPHICS_REG_ROTATE] & GRAPHICS_ROTATE_COUNT_MASK) {
            D_MESSAGE_SOME(100, "need to rotate");
        }

        _vram[offset] = val;
        break;
    }
    case 1:
    case 2:
    case 3:
        D_MESSAGE("implement me %u",
                  _graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_WRITE_MODE_MASK);
        _vram[offset] = val;
        break;
    }
}

inline void VGA::vram_write_one(uint64_t dest, uint8_t byte)
{
    uint64_t address;
    uint64_t plan;

    ASSERT(!!(_sequencer_regs[SEQUENCER_REG_MEM_MODE] & SEQUENCER_MEM_MODE_ODD_EVEN_MASK) ==
           !(_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_ODD_EVEN));

    if (_sequencer_regs[SEQUENCER_REG_MEM_MODE] & SEQUENCER_MEM_MODE_CHAIN_MASK) {
        plan = dest & 0x3;

        if (!(_sequencer_regs[SEQUENCER_REG_PLANE] & (1 << plan))) {
            return;
        }

        vram_store_byte(dest, byte);

    } else if (_sequencer_regs[SEQUENCER_REG_MEM_MODE] & SEQUENCER_MEM_MODE_ODD_EVEN_MASK) {

        if (_sequencer_regs[SEQUENCER_REG_PLANE] & 0x01) {
            address = dest << 2;
            vram_store_byte(address, byte);
        }

        if (_sequencer_regs[SEQUENCER_REG_PLANE] & 0x02) {
            address = (dest << 2) | 1;
            vram_store_byte(address, byte);
        }

        if (_sequencer_regs[SEQUENCER_REG_PLANE] & 0x04) {
            address = (dest << 2) | 2;
            vram_store_byte(address, byte);
        }

        if (_sequencer_regs[SEQUENCER_REG_PLANE] & 0x08) {
            address = (dest << 2) | 3;
            vram_store_byte(address, byte);
        }

    } else {
        plan = dest & 1;

        if (_sequencer_regs[SEQUENCER_REG_PLANE] & (1 << plan)) {
            address = ((dest & ~1) << 1) | plan;
            vram_store_byte(address, byte);
        }

        plan += 2;
        if (_sequencer_regs[SEQUENCER_REG_PLANE] & (1 << plan)) {
            address = ((dest & ~1) << 1) | plan;
            vram_store_byte(address, byte);
        }
    }
}

void VGA::vram_write(const uint8_t* src, uint64_t length, uint64_t dest)
{
    if (!(_misc_output & MISC_FB_ACCESS_MASK)) {
        return;
    }

    for (int i = 0; i < length; i++) {
        vram_write_one(dest + i, src[i]);
    }

    _dirty = true;
}

