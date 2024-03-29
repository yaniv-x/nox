/*
    Copyright (c) 2013-2014 Yaniv Kamay,
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
#include "pci.h"
#include "pci_device.h"
#include "pci_bus.h"

#define VGA_D_MESSAGE(format, ...)
#define MAX_LOG_STR_SIZE 256

// todo: odd even bits in multiple regs

enum {
    IO_VGA_BASE = 0x3c0,
    IO_VGA_END = 0x3d0,

    VRAM_START = 0xa0000,
    VGA_VRAM_SIZE = 256 * KB,

    UPDATE_INTERVAL = 1000 * 1000 * 1000 / 30,
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

    SEQUENCER_CLOCKING_BLANK_BIT = 5,
    SEQUENCER_CLOCKING_BLANK_MASK = 1 << SEQUENCER_CLOCKING_BLANK_BIT,
    SEQUENCER_CLOCKING_DOTS_BIT = 0,

    SEQUENCER_FONT_MAPA_BIT0 = 5,
    SEQUENCER_FONT_MAPA_BIT1 = 2,
    SEQUENCER_FONT_MAPB_BIT0 = 4,
    SEQUENCER_FONT_MAPB_BIT1 = 0,

    SEQUENCER_MEM_MODE_CHAIN_MASK =  (1 << 3),
    SEQUENCER_MEM_MODE_ODD_EVEN_MASK =  (1 << 2),


    GRAPHICS_INDEX_MASK = (1 << 4) - 1,

    GRAPHICS_REG_SET_RESET = 0x00,
    GRAPHICS_REG_ENABLE_SET_RESET = 0x01,
    GRAPHICS_REG_COLOR_COMPER = 0x02,
    GRAPHICS_REG_ROTATE = 0x03,
    GRAPHICS_REG_READ_PLAN = 0x04,
    GRAPHICS_REG_MODE = 0x05,
    GRAPHICS_REG_MISC = 0x06,
    GRAPHICS_REG_COLOR_DONT_CARE = 0x07,
    GRAPHICS_REG_MASK = 0x08,

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

enum {
    VBE_MAX_X_RES = 1920,
    VBE_MAX_Y_RES = 1200,
    VBE_MAX_DEPTH = 32,
    VBE_VRAM_SIZE = ALIGN(VBE_MAX_X_RES * VBE_MAX_Y_RES * 4, 64 * KB),

    VBE_REG_DISPLAY_ID = 0,
    VBE_REG_X_RES,
    VBE_REG_Y_RES,
    VBE_REG_DEPTH,
    VBE_REG_COMMAND,
    VBE_REG_BANK,
    VBE_REG_VIRT_WIDTH,
    VBE_REG_VIRT_HEIGHT,
    VBE_REG_X_OFFSET,
    VBE_REG_Y_OFFSET,
    VBE_REG_VIDEO_MEMORY_64K,
    VBE_REG_EDID_WINDOW,
    VBE_REG_EDID_DATA,
    VBE_REG_PALETTE_WRITE_INDEX,
    VBE_REG_PALETTE_DATA,
    VBE_NUM_REGS,

    VBE_COMMAND_ENABLE = (1 << 0),
    VBE_COMMAND_CAPS = (1 << 1),
    VBE_DISPI_8BIT_DAC = (1 << 5),
    VBE_COMMAND_LINEAR = (1 << 6),
    VBE_COMMAN_NOCLEARMEM = (1 << 7),
};


enum {
    IO_PORT_LOG = 0,
    IO_PORT_VBE_REG_SELECT = 2,
    IO_PORT_VBE_DATA = 4,

    IO_PORT_SIZE = IO_PORT_VBE_DATA + 2,
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

    virtual ~VGABackEndImp()
    {
        _fb->unref();
    }

    virtual void detach()
    {
        Lock lock(_vga->_front_ends_mutex);
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
    VGA* _vga;
    SharedBuf* _fb;
    VGAFrontEnd* _front_end;
};

class HostSharedBuf: public SharedBuf {
public:
    HostSharedBuf(uint size)
        : SharedBuf(size, new uint8_t[size])
    {
    }

protected:
    virtual ~HostSharedBuf() { delete[] get();}
};


enum {
    VGA_MAX_WIDTH = 800,
    VGA_MAX_HIGHT = 600,
    VGA_MIN_WIDTH = 320,
    VGA_MIN_HIGHT = 200,
    VGA_PCI_IO_REGION = 0,
    VGA_PCI_FB_REGION = 1,
    VGA_PCI_REVISION = 1,

    EDID_BLOCK_SIZE = 128,
};


VGA::VGA(NoxVM& nox)
    : PCIDevice("vga-pci", *pci_bus, NOX_PCI_VENDOR_ID, NOX_PCI_DEV_ID_VGA, VGA_PCI_REVISION,
                mk_pci_class_code(PCI_CLASS_DISPLAY, PCI_DISPLAY_SUBCLASS_VGA,
                                  PCI_VGA_INTERFACE_VGACOMPAT),
                false)
    , _region0 (NULL)
    , _region1 (NULL)
    , _region2 (NULL)
    , _mmio (NULL)
    , _fb (new HostSharedBuf(VBE_MAX_X_RES * VBE_MAX_Y_RES * sizeof(uint32_t)))
    , _width (640)
    , _height (480)
    , _caret_tick (0)
    , _caret_visable (false)
{
    ASSERT(int(VBE_NUM_REGS) == int(NUM_VBE_REGS));

    _update_timer = application->create_timer((void_callback_t)&VGA::update, this);

    _physical_ram = memory_bus->alloc_physical_ram(*this, VBE_VRAM_SIZE / GUEST_PAGE_SIZE,
                                                   "vga ram");
    _vram = memory_bus->get_physical_ram_ptr(_physical_ram);
    _vga_vram_end = _vram + VGA_VRAM_SIZE;
    _vbe_vram_end = _vram + VBE_VRAM_SIZE;

    add_io_region(VGA_PCI_IO_REGION, IO_PORT_SIZE, this, NULL,
                  (io_write_byte_proc_t)&VGA::io_write_byte,
                  (io_read_word_proc_t)&VGA::io_read_word,
                  (io_write_word_proc_t)&VGA::io_write_word);
    add_mem_region(VGA_PCI_FB_REGION, _physical_ram, false);

    pci_bus->add_device(*this);

    reset();
}


void VGA::put_log_byte(uint8_t val)
{
    Lock lock(_log_mutex);

    switch (val) {
    case '\r':
        break;
    case '\n':
        if (_log_string.empty()) {
            break;
        }

        I_MESSAGE("VGA BIOS: %s", _log_string.c_str());
        _log_string = "";

        break;
    default:
        if (_log_string.size() == MAX_LOG_STR_SIZE) {
            W_MESSAGE_SOME(10, "log string exceed max length");
            break;
        }

        if (!isprint(val)) {
            W_MESSAGE_SOME(10, "invalid char 0x%x", val);
            break;;
        }

        _log_string += val;
    }
}


void VGA::io_write_byte(uint16_t port, uint8_t val)
{
    port -= get_region_address(VGA_PCI_IO_REGION);

    switch (port) {
    case IO_PORT_LOG:
        put_log_byte(val);
        break;
    default:
        D_MESSAGE("invalid port 0x%x", port);
    }
}


VGA::~VGA()
{
    _update_timer->destroy();
    unmap_lagacy_io();
    unmap_lagacy_fb();
    memory_bus->unref_physical_ram(_physical_ram);
}


bool VGA::font_bit(uint8_t ch, uint line, uint pos)
{
    // no font select support (SR03 Character Font)
    uint offset = uint(ch) * 32 + line;
    uint8_t c = _vram[(offset << 2) + 2];
    return !!(c & (0x80 >> pos));
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

    return _effective_palette[index & _color_index_mask].color;
}


void VGA::draw_char(uint8_t ch, uint8_t attrib, uint32_t* dest, uint char_w, uint char_h)
{
    // no blinking or font select support

    // todo: prepare actual palette on graphic mode change, on palette entery change or on any
    //       other change that effect translation of index to color

    for (int i= 0; i < char_h; i++) {
        for (int j= 0; j < char_w; j++) {

            if (font_bit(ch, i, j)) {
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


static uint8_t fetch_pix_4(const uint8_t* fb_ptr, uint offset, uint w)
{
    const uint8_t* byte = fb_ptr + offset / 8 * 4;
    uint8_t ret;

    switch (offset % 8) {
    case 0:
        ret = (byte[0] >> 6) & 0x03;
        break;
    case 1:
        ret = (byte[0] >> 4) & 0x03;
        break;
    case 2:
        ret = (byte[0] >> 2) & 0x03;
        break;
    case 3:
        ret = (byte[0] >> 0) & 0x03;
        break;
    case 4:
        ret = (byte[1] >> 6) & 0x03;
        break;
    case 5:
        ret = (byte[1] >> 4) & 0x03;
        break;
    case 6:
        ret = (byte[1] >> 2) & 0x03;
        break;
    case 7:
        ret = (byte[1] >> 0) & 0x03;
        break;
    default:
        PANIC("");
        ret = 0;
    }

    return ret;
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

    char_w = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKING_DOTS_BIT) ? 8 : 9;
    char_h = (_crt_regs[CRT_REG_MAX_SCAN_LINE] & CRT_MAX_SCAN_LINE_TEXT_HIGHET_MASK) + 1;

    uint line_size = _width / char_w;
    uint lines = _height / char_h;

    uint pos = (uint(_crt_regs[CRT_REG_CURSOR_LOCATION_HIGH]) << 8) |
               _crt_regs[CRT_REG_CURSOR_LOCATION_LOW];

    if (pos > VGA_VRAM_SIZE) {
        D_MESSAGE("invalid position");
        return;
    }

    uint32_t fb_offset = (uint32_t(_crt_regs[CRT_REG_START_ADDRESS_HIGH]) << 8) +
                         _crt_regs[CRT_REG_START_ADDRESS_LOW];

    if (fb_offset > pos) {
        return;
    }

    uint32_t color = foreground_color_at(pos);

    pos -= fb_offset;

    uint cursor_row = pos / line_size;
    uint cursor_col = pos % line_size;

    if (cursor_row >= lines) {
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

    if (cursor_pos > VGA_VRAM_SIZE) {
        D_MESSAGE("invalid position");
        return;
    }

    uint char_w;
    uint char_h;

    char_w = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKING_DOTS_BIT) ? 8 : 9;
    char_h = (_crt_regs[CRT_REG_MAX_SCAN_LINE] & CRT_MAX_SCAN_LINE_TEXT_HIGHET_MASK) + 1;
    uint line_size = _width / char_w;
    uint lines = _height / char_h;

    uint fb_offset = (uint(_crt_regs[CRT_REG_START_ADDRESS_HIGH]) << 8) +
                      _crt_regs[CRT_REG_START_ADDRESS_LOW];

    if (fb_offset > cursor_pos) {
        return;
    }

    uint pos = cursor_pos - fb_offset;

    uint cursor_row = pos / line_size;
    uint cursor_col = pos % line_size;

    if (cursor_row >= lines) {
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


static inline uint32_t rgb565_to_rgb888(uint32_t color)
{
    uint32_t ret;

    ret = ((color & 0x001f) << 3) | ((color & 0x001c) >> 2);
    ret |= ((color & 0x07e0) << 5) | ((color & 0x00600) >> 1);
    ret |= ((color & 0xf800) << 8) | ((color & 0xe000) << 3);

    return ret;
}


static inline uint32_t rgb555_to_rgb888(uint32_t color)
{
    uint32_t ret;

    ret = ((color & 0x001f) << 3) | ((color & 0x001c) >> 2);
    ret |= ((color & 0x03e0) << 6) | ((color & 0x00380) << 1);
    ret |= ((color & 0x7c00) << 9) | ((color & 0x7000) << 4);

    return ret;
}


void VGA::vbe_update_4bpp(uint32_t* dest)
{
    uint32_t stride = _vbe_regs[VBE_REG_VIRT_WIDTH];
    uint now = _vbe_regs[VBE_REG_X_OFFSET] + _vbe_regs[VBE_REG_Y_OFFSET] * stride;
    uint end = now + _height * stride;
    uint gap = stride - _width;

    ASSERT(stride >= _width);

    uint start_offset = now / 8 * 4;
    uint end_offset = end / 8 * 4;

    if (end_offset < start_offset || end_offset > VBE_VRAM_SIZE) {
        D_MESSAGE("invalid area");
        return;
    }

    while (now < end) {
        uint line_end = now +_width;

        for (; now < line_end; now++) {
            *dest++ =  _effective_palette[fetch_pix_16(_vram, now)].color;
        }

        now += gap;
    }
}


void VGA::update()
{
    RLock lock(_rw_lock);

    if (!is_vbe_active()) {
        update_vga();
        return;
    }

    if (_vbe_regs[VBE_REG_DEPTH] == 32) {
        return;
    }

    uint32_t* dest = (uint32_t*)_fb->get();
    uint32_t* dest_end = dest + _width * _height;

    ASSERT(dest < dest_end && (uint8_t*)dest_end <= (uint8_t*)dest + _fb->size());

    uint src_pixel_bytes;

    switch (_vbe_regs[VBE_REG_DEPTH]) {
    case 24:
        src_pixel_bytes = 3;
        break;
    case 16:
    case 15:
        src_pixel_bytes = 2;
        break;
    case 4:
        vbe_update_4bpp(dest);
        return;
    default:
        src_pixel_bytes = 1;
    }

    uint32_t src_stride = _vbe_regs[VBE_REG_VIRT_WIDTH] * src_pixel_bytes;
    uint8_t* src = _vram + _vbe_regs[VBE_REG_X_OFFSET] * src_pixel_bytes +
                   src_stride * _vbe_regs[VBE_REG_Y_OFFSET];

    if (src > _vbe_vram_end || src < _vram) {
        D_MESSAGE("invalid vram src");
        return;
    }

    uint8_t* src_end = src + src_stride * _height;

    if (src_end < src || src_end > _vbe_vram_end) {
        D_MESSAGE("invalid vram src_end");
        return;
    }

    switch (_vbe_regs[VBE_REG_DEPTH]) {
    case 24: {
        uint32_t src_skip = src_stride - _width * 3;

        while (src < src_end) {
            uint32_t* line_end = dest + _width;

            for (; dest < line_end; ++dest, src += 3) {
                *dest = *(uint32_t*)src;
                ((uint8_t*)dest)[3] = 0;
            }

            src += src_skip;
        }
        break;
    }
    case 16:
        while (src < src_end) {
            uint32_t* line_end = dest + _width;
            uint16_t* now = (uint16_t*)src;

            for (; dest < line_end; ++dest, ++now) {
                *dest = rgb565_to_rgb888(*now);
            }

            src += src_stride;
        }
        break;
    case 15:
        while (src < src_end) {
            uint32_t* line_end = dest + _width;
            uint16_t* now = (uint16_t*)src;

            for (; dest < line_end; ++dest, ++now) {
                *dest = rgb555_to_rgb888(*now);
            }

            src += src_stride;
        }
        break;
    default:
        uint32_t src_skip = src_stride - _width;

        while (src < src_end) {
            uint32_t* line_end = dest + _width;

            for (; dest < line_end; ++dest, ++src) {
                *dest = _effective_palette[*src].color;
            }

            src += src_skip;
        }
        break;
    }
}


void VGA::update_vga()
{
    if (!_vga_draw_logic) {
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

         char_w = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKING_DOTS_BIT) ? 8 : 9;
         char_h = (_crt_regs[CRT_REG_MAX_SCAN_LINE] & CRT_MAX_SCAN_LINE_TEXT_HIGHET_MASK) + 1;

         uint line_size = width / char_w;
         uint lines = height / char_h;

         uint32_t* dest_line = (uint32_t*)_fb->get();

         lines = MIN(lines, (_vga_vram_end - fb_ptr) / 4 / width);

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

    D_MESSAGE_ONCE("todo: handle double, plane mask, etc. (AR12 CR17 AR10 SR01...)");

    if (_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_256C_MASK) {

        uint32_t* dest = (uint32_t*)_fb->get();

        for (uint pixel = 0, i = 0; i < height / 2; i++) {
            for (uint j =  0; j < width / 2; j++, pixel++) {
                uint32_t color = _effective_palette[fb_ptr[pixel]].color;
                dest[i * 2 * width + j * 2] = color;
                dest[i * 2 * width + j * 2 + 1] = color;
                dest[(i * 2 + 1) * width + j * 2] = color;
                dest[(i * 2 + 1) * width + j * 2 + 1] = color;
            }
        }
    } else if (_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_4C_MASK ) {
        uint8_t pal_high_bits;
        uint8_t pal_mask;

        if (_attributes_regs[ATTRIB_REG_MODE] & ATTRIB_MODE_PAL_FIXED_4_5_MASK) {
            pal_high_bits = (_attributes_regs[ATTRIB_REG_COLOR_SELECT] << 4) & 0xf0;
            pal_mask = 0x0f;
        } else {
            pal_high_bits = (_attributes_regs[ATTRIB_REG_COLOR_SELECT] << 4) & 0xc0;
            pal_mask = 0x3f;
        }

        uint32_t* dest = (uint32_t*)_fb->get();
        uint offset = 0;

        for (uint i = 0; i < height / 4; i++) {
            for (uint j =  0; j < width; j++, offset++) {
                uint8_t pal_index;

                pal_index = (_attributes_regs[fetch_pix_4(fb_ptr, offset, width)] & pal_mask) |
                                                                                      pal_high_bits;
                dest[i * 4 * width + j] = _effective_palette[pal_index & _color_index_mask].color;
                dest[(i * 4 + 1) * width + j] = dest[i * 4 * width + j];

                pal_index = (_attributes_regs[fetch_pix_4(fb_ptr + (0x2000 << 1), offset, width)]
                                                                        & pal_mask) | pal_high_bits;
                dest[(i * 4  + 2) * width + j] =
                                            _effective_palette[pal_index & _color_index_mask].color;
                dest[(i * 4  + 3) * width + j] = dest[(i * 4  + 2) * width + j] ;
            }
        }
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

        pixels = MIN(pixels, (_vga_vram_end - fb_ptr) << 1);

        for (uint i = 0; i < pixels; i++) {
            uint8_t pal_index;

            pal_index = (_attributes_regs[fetch_pix_16(fb_ptr, i)] & pal_mask) | pal_high_bits;
            dest[i] = _effective_palette[pal_index & _color_index_mask].color;
        }
    }
}


void VGA::unmap_lagacy_io()
{
    io_bus->unregister_region(_region0);
    _region0 = NULL;
    io_bus->unregister_region(_region1);
    _region1 = NULL;
    io_bus->unregister_region(_region2);
    _region2 = NULL;

    _last_io_delta = ~0;
}


void VGA::unmap_lagacy_fb()
{
    memory_bus->unregister_mmio(_mmio);
    _mmio = NULL;
}


void VGA::reset_io()
{
    uint8_t delta = (_misc_output & MISC_IO_ADDRESS_SELECT_MASK) ? 0xd0 - 0xb0 : 0;

    if (_last_io_delta == delta) {
        return;
    }

    unmap_lagacy_io();

    VGA_D_MESSAGE("0x%x 0x%x", IO_INPUT_STATUS_1_MDA + delta, IO_CRT_CONTROL_INDEX_MDA + delta);

    _region0 = io_bus->register_region(*this, IO_VGA_BASE, IO_VGA_END - IO_VGA_BASE, this,
                                       (io_read_byte_proc_t)&VGA::io_vga_read_byte,
                                       (io_write_byte_proc_t)&VGA::io_vga_write_byte);

    _region1 = io_bus->register_region(*this, IO_INPUT_STATUS_1_MDA + delta, 1, this,
                                       (io_read_byte_proc_t)&VGA::io_vga_read_byte,
                                       (io_write_byte_proc_t)&VGA::io_vga_write_byte);

    _region2 = io_bus->register_region(*this, IO_CRT_CONTROL_INDEX_MDA + delta, 2, this,
                                       (io_read_byte_proc_t)&VGA::io_vga_read_byte,
                                       (io_write_byte_proc_t)&VGA::io_vga_write_byte);

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

    unmap_lagacy_fb();

    if (!fb_accesibel) {
        VGA_D_MESSAGE("unmaped");
        return;
    }

    switch (fb_address) {
    case GRAPHICS_MISK_FB_ADDR_A0000_BFFFF:
        _mmio = memory_bus->register_mmio(0xa0000 / GUEST_PAGE_SIZE, 128 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        VGA_D_MESSAGE("map @ 0x%x size %uK", 0xa0000, 128);
        break;
    case GRAPHICS_MISK_FB_ADDR_A0000_AFFFF:
        _mmio = memory_bus->register_mmio(0xa0000 / GUEST_PAGE_SIZE, 64 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        VGA_D_MESSAGE("map @ 0x%x size %uK", 0xa0000, 64);
        break;
    case GRAPHICS_MISK_FB_ADDR_B0000_B7FFF:
        _mmio = memory_bus->register_mmio(0xb0000 / GUEST_PAGE_SIZE, 32 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        VGA_D_MESSAGE("map @ 0x%x size %uK", 0xb0000, 32);
        break;
    case GRAPHICS_MISK_FB_ADDR_B8000_BFFFF:
        _mmio = memory_bus->register_mmio(0xb8000 / GUEST_PAGE_SIZE, 32 / 4,
                                          (read_mem_proc_t)&VGA::vram_read,
                                          (write_mem_proc_t)&VGA::vram_write,
                                          this, *this);
        VGA_D_MESSAGE("map @ 0x%x size %uK", 0xb8000, 32);
        break;
    }
}


void VGA::reset()
{
    _enabled = false;
    _last_io_delta = ~0;
    _mmap_state = ~0;
    _vga_active = false;
    _vga_draw_logic = false;
    memset(_vram, 0,  VBE_VRAM_SIZE);
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
    _palette_read_comp = 0;
    _palette_write_index = 0;
    _palette_write_comp = 0;
    memset(_palette, 0, sizeof(_palette));
    memset(_effective_palette, 0, sizeof(_effective_palette));
    _palette_shift = 2;

    _graphics_index = 0;
    memset(_graphics_regs, 0, sizeof(_graphics_regs));

    _feature_cntrol = 0;

    _crt_index = 0;
    memset(_crt_regs, 0, sizeof(_crt_regs));

    _caret_tick = 0;
    _caret_visable = false;

    _vbe_reg_index = 0;
    memset(_vbe_regs, 0, sizeof(_vbe_regs));
    _vbe_regs[VBE_REG_VIDEO_MEMORY_64K] = VBE_VRAM_SIZE / (64 * KB);

    _vbe_regs[VBE_REG_EDID_WINDOW] = ~0;
    _vbe_regs[VBE_REG_EDID_DATA] = ~0;
    _edid_offset = ~0;
    _vba_palette_expect_red = false;

    _window_start = _vram;
    _window_end = _vga_vram_end;

    memset(_latch, 0, sizeof(_latch));

    _dirty = true;

    unmap_lagacy_io();
    unmap_lagacy_fb();

    _log_string = "";

    PCIDevice::reset();
}

static uint8_t v_retrace = 0;

uint8_t VGA::io_vga_read_byte(uint16_t port)
{
    WLock lock(_rw_lock);

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
    case IO_PALETTE_WRITE_INDEX:
        W_MESSAGE("read from write only port");
        return _palette_write_index;
    case IO_PALETTE_DATA: {
        uint8_t ret = _palette[_palette_read_index].components[_palette_read_comp++];

        if (_palette_read_comp == 3) {
            _palette_read_index++;
            _palette_read_comp = 0;
        }

        return ret;
    }
    case IO_ATTRIB_CONTROL_INDEX:
        return _attrib_control_index;
    case IO_ATTRIB_READ:
        return _attributes_regs[_attrib_control_index & ATTRIB_INDEX_MASK];
    case IO_GRAPHICS_INDEX:
        return _graphics_index;
    case IO_GRAPHICS:
        return _graphics_regs[_graphics_index];
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
    memset(_fb->get(), 0, _fb->size());
}


void VGA::io_vga_write_byte(uint16_t port, uint8_t val)
{
    WLock lock(_rw_lock);

    switch (port) {
    case IO_MISC_OUTPUT_W:
        set_misc_reg(val);
        VGA_D_MESSAGE("misc_output 0x%x", val);
        break;
    case IO_SEQUENCER_INDEX:
        _sequencer_index = val & SEQUENCER_INDEX_MASK;
        return;
    case IO_SEQUENCER_DATA: {
         uint8_t blank_changed;

         blank_changed = _sequencer_index == SEQUENCER_REG_CLOCKING &&
                    (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKING_BLANK_MASK) !=
                                                          (val & SEQUENCER_CLOCKING_BLANK_MASK);

        _sequencer_regs[_sequencer_index] = val;

        if (_sequencer_index == 0) {
            reset_sequencer();
        }

        if (_sequencer_index == SEQUENCER_REG_MEM_MODE) {
            VGA_D_MESSAGE("sequencer[SEQUENCER_REG_MEM_MODE] = 0x%x (%u)", val, val);
        }

        if (_vga_active && blank_changed) {
            if ((val & SEQUENCER_CLOCKING_BLANK_MASK)) {
                _vga_draw_logic = false;
                blank_screen();
            } else {
                _vga_draw_logic = true;
                init_caret_tick();
            }
        }

        conditional_mode_change();

        VGA_D_MESSAGE("sequencer[%u] = 0x%x (%u)", _sequencer_index, val, val);

        return;
    }
    case IO_COLOR_INDEX_MASK:
        VGA_D_MESSAGE("color index mask 0x%x", val);
        _color_index_mask = val;
        _dirty = true;
        return;
    case IO_PALETTE_READ_INDEX:
        _dac_state = 0x3;
        _palette_read_comp = 0;
        _palette_read_index = val;
        return;
    case IO_PALETTE_WRITE_INDEX:
        _dac_state = 0;
        _palette_write_comp = 0;
        _palette_write_index = val;
        return;
    case IO_PALETTE_DATA:
        VGA_D_MESSAGE("palette[%u].%s = 0x%x",
                      _palette_write_index,
                      _palette_write_comp == 0 ? "red" :
                                 (_palette_write_comp == 1 ? "green" : "blue"),
                      val);

        _palette[_palette_write_index].components[_palette_write_comp++] = val;

        if (_palette_write_comp == 3) {
            update_one_effective_palette(_palette_write_index);
            _palette_write_index++;
            _palette_write_comp = 0;
        }

        _dirty = true;
        return;
    case IO_ATTRIB_CONTROL_INDEX:
        if (_write_attrib) {
            _write_attrib = false;

            if ((_attrib_control_index & ATTRIB_INDEX_DISABLE_MASK) &&
                (_attrib_control_index & ATTRIB_INDEX_MASK) <= 0x0f) {
                D_MESSAGE("modify protected reg[0x%x]", _attrib_control_index & ATTRIB_INDEX_MASK);
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

        conditional_mode_change();

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

        if (_vga_draw_logic && _crt_index == CRT_REG_CURSOR_START &&
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

        if (_vga_draw_logic && (_crt_index == CRT_REG_CURSOR_LOCATION_HIGH ||
            _crt_index == CRT_REG_CURSOR_LOCATION_LOW) &&
            !(_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT)) &&
            !(_crt_regs[CRT_REG_CURSOR_START] & CRT_CURSOR_START_OFF)) {

            hide_caret();
            _caret_tick = get_monolitic_time() - CARET_RATE;
        }

        if (_crt_index == CRT_REG_START_ADDRESS_HIGH || _crt_index == CRT_REG_START_ADDRESS_LOW) {
            _dirty = true;
        }

        conditional_mode_change();

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
    Lock lock(_front_ends_mutex);
    _front_ends.push_back(back_end);
    D_MESSAGE("todo: add propagate_fb event");
    return back_end;
}


inline void VGA::vram_read_mode_1(uint32_t src, uint8_t& dest)
{
    uint8_t* quad = _window_start + (src << 2);

    if (quad + 4 < _window_start || quad + 4 > _window_end) {
        D_MESSAGE("out of bounds");
        dest = 0xff;
        return;
    }

    uint32_t* ptr32 = (uint32_t*)_latch;
    *ptr32 = *(uint32_t*)quad;

    uint8_t color_dont_care = _graphics_regs[GRAPHICS_REG_COLOR_DONT_CARE] & 0x0f;
    uint8_t color_comper = _graphics_regs[GRAPHICS_REG_COLOR_COMPER] & color_dont_care;

    uint8_t res = 0;

    for (int i = 7; i >= 0; i--) {
        uint8_t color = ((quad[0] >> i) & 1);
        color |= ((quad[1] >> i) & 1) << 1;
        color |= ((quad[2] >> i) & 1) << 2;
        color |= ((quad[3] >> i) & 1) << 3;

        if ((color & color_dont_care) == color_comper) {
            res |= 1 << i;
        }
    }

    dest = res;
}


inline bool VGA::is_vbe_active()
{
    return !!(_vbe_regs[VBE_REG_COMMAND] & VBE_COMMAND_ENABLE);
}


inline bool VGA::is_crt_active()
{
    return !!(_crt_regs[CRT_REG_MODE] & CRT_MODE_ACTIVE_MASK);
}


bool VGA::is_valid_mode(uint height, uint width)
{
    return width <= VGA_MAX_WIDTH && width >= VGA_MIN_WIDTH &&
           height <= VGA_MAX_HIGHT && height >= VGA_MIN_HIGHT;
}


void VGA::init_caret_tick()
{
    if (!(_crt_regs[CRT_REG_CURSOR_START] & CRT_CURSOR_START_OFF) &&
        !(_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT))) {
        _caret_tick = get_monolitic_time();
    } else {
        _caret_tick = 0;
    }
}

void VGA::conditional_mode_change()
{
    if (is_vbe_active()) {
        return;
    }

    ASSERT(_window_start == _vram && _window_end == _vga_vram_end);

    if (!is_crt_active()) {
        if (_vga_active) {
            _vga_active = false;
            _vga_draw_logic = false;
            blank_screen();
            _update_timer->disarm();
        }
        return;
    }

    uint overflow = _crt_regs[CRT_REG_OVERFLOW];

    uint height = (((overflow & (1 << CRT_OVERFLAOW_H8_BIT)) << (8 - CRT_OVERFLAOW_H8_BIT)) |
                   ((overflow & (1 << CRT_OVERFLAOW_H9_BIT)) << (9 - CRT_OVERFLAOW_H9_BIT))) +
                                                                 _crt_regs[CRT_REG_HEIGHT] + 1;

    uint dot_clocks;

    dot_clocks = (_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKING_DOTS_BIT) ? 8 : 9;

    //maybe vga bios bug?
    if ((_attributes_regs[ATTRIB_REG_MODE] & (1 << ATTRIB_MODE_GRAPHICS_BIT))) {
        dot_clocks = 8;
    }

    uint width = (_crt_regs[CRT_REG_WIDTH] + 1) * dot_clocks;

    if ((_vga_active && height == _height && width == _width) || !is_valid_mode(height, width)) {
        return;
    }

    _height = height;
    _width = width;
    _caret_visable = false;
    _dirty = true;
    _vga_active = true;
    _vga_draw_logic = !(_sequencer_regs[SEQUENCER_REG_CLOCKING] & SEQUENCER_CLOCKING_BLANK_MASK);

    init_caret_tick();

    if (!_vga_draw_logic) {
        blank_screen();
    }

    propagate_fb();

    _update_timer->arm(UPDATE_INTERVAL, true);
}


inline void VGA::vram_load_one(uint32_t offset, uint8_t& dest)
{
    if (_window_start + offset < _window_start || _window_start + offset >= _window_end) {
        D_MESSAGE("out of bounds");
        dest = 0xff;
        return;
    }

    uint32_t plan = offset & 0x03;
    uint32_t* ptr32 = (uint32_t*)_latch;
    *ptr32 = *(uint32_t*)(_window_start + offset - plan);
    dest = _latch[plan];
}


inline void VGA::vram_read_mode_0(uint32_t src, uint8_t& dest)
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
    RLock lock(_rw_lock);

    if (!(_misc_output & MISC_FB_ACCESS_MASK)) {
        return;
    }

    if (is_vbe_active() && _vbe_regs[VBE_REG_DEPTH] != 4) {
        src += uint64_t(_vbe_regs[VBE_REG_BANK]) * 64 * KB;

        ASSERT(_vram + src >= _vram && _vram + src + length >= _vram + src &&
               _vram + src + length <= _vbe_vram_end);

        // protected by limit on _vbe_regs[VBE_REG_BANK] + membus imp
        memcpy(dest, _vram + src, length);
        return;
    }

    if ((_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_READ_MODE_MASK)) {
        for (int i = 0; i < length; i++) {
            vram_read_mode_1(src + i, dest[i]);
        }
    } else {
        for (int i = 0; i < length; i++) {
            vram_read_mode_0(src + i, dest[i]);
        }
    }
}


inline void VGA::vram_store_byte(uint64_t offset, uint8_t val)
{
    if (_window_start + offset < _window_start || _window_start + offset >= _window_end) {
        D_MESSAGE("out of bounds");
        return;
    }

    switch (_graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_WRITE_MODE_MASK) {
    case 0: {
        uint plan = offset & 0x03;
        if (_graphics_regs[GRAPHICS_REG_ENABLE_SET_RESET] & (1 << plan)) {
            val = (_graphics_regs[GRAPHICS_REG_SET_RESET] & (1 << plan)) ? ~0 : 0;
        }

        if (_graphics_regs[GRAPHICS_REG_ROTATE] & GRAPHICS_ROTATE_COUNT_MASK) {
            D_MESSAGE_SOME(100, "rotate, implement me");
        }

        switch (_graphics_regs[GRAPHICS_REG_ROTATE] & GRAPHICS_ROTATE_FUNC_MASK) {
        case GRAPHICS_ROTATE_FUNC_NONE:
            break;
        case GRAPHICS_ROTATE_FUNC_AND:
            val &= _latch[plan];
            break;
        case GRAPHICS_ROTATE_FUNC_OR:
            val |= _latch[plan];
            break;
        case GRAPHICS_ROTATE_FUNC_XOR:
            val ^= _latch[plan];
            break;
        }

        _window_start[offset] = (val & _graphics_regs[GRAPHICS_REG_MASK]) |
                                (_window_start[offset] & ~_graphics_regs[GRAPHICS_REG_MASK]);
        break;
    }
    case 1:
        _window_start[offset] = _latch[offset & 0x03];
        break;
    case 2: {
        uint plan = offset & 0x03;
        if ((1 << plan) & val) {
            val = 0xff;
        } else {
            val = 0;
        }
        _window_start[offset] = (val & _graphics_regs[GRAPHICS_REG_MASK]) |
                                (_latch[offset & 0x03] & ~_graphics_regs[GRAPHICS_REG_MASK]);
        break;
    }
    case 3:
        D_MESSAGE_SOME(10, "implement me %u",
                  _graphics_regs[GRAPHICS_REG_MODE] & GRAPHICS_MODE_WRITE_MODE_MASK);
        _window_start[offset] = val;
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
    RLock lock(_rw_lock);

    if (!(_misc_output & MISC_FB_ACCESS_MASK)) {
        return;
    }

    if (is_vbe_active() && _vbe_regs[VBE_REG_DEPTH] != 4) { // assuming that all other modes can
                                                            // be marked as NOT_VGA_COMPATIBLE in
                                                            // vga bios mode info (vbetables.h)

        dest += uint64_t(_vbe_regs[VBE_REG_BANK]) * 64 * KB;

        ASSERT(_vram + dest >= _vram && _vram + dest + length >= _vram + dest &&
               _vram + dest + length <= _vbe_vram_end);

        // protected by limit on _vbe_regs[VBE_REG_BANK] + membus imp
        memcpy(_vram + dest, src, length);
        _dirty = true;
        return;
    }

    for (int i = 0; i < length; i++) {
        vram_write_one(dest + i, src[i]);
    }

    _dirty = true;
}


static const uint8_t edid_data[] = {
    // 1. The following was generated by "edid-gen 1280x1024".
    // 2. To extract hardware EDID run "monitor-get-edid | xxd -i".

    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
    0x39, 0xf8, 0x5e, 0x11, 0x02, 0xe7, 0xf6, 0x34,
    0x12, 0x18, 0x01, 0x03, 0x80, 0x23, 0x1c, 0x78,
    0x0e, 0x1e, 0xc5, 0xae, 0x4f, 0x34, 0xb1, 0x26,
    0x0e, 0x50, 0x54, 0xa5, 0x4b, 0x00, 0x81, 0x59,
    0x81, 0x19, 0x71, 0x4f, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x91, 0x3b,
    0x00, 0xa0, 0x50, 0x00, 0x23, 0x40, 0x30, 0x20,
    0x36, 0x00, 0x59, 0x14, 0x11, 0x00, 0x00, 0x1a,
    0x00, 0x00, 0x00, 0xfc, 0x00, 0x4e, 0x4f, 0x58,
    0x31, 0x37, 0x2e, 0x34, 0x44, 0x2d, 0x35, 0x45,
    0x0a, 0x20, 0x00, 0x00, 0x00, 0xff, 0x00, 0x33,
    0x34, 0x46, 0x36, 0x45, 0x37, 0x30, 0x32, 0x0a,
    0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfd,
    0x00, 0x3c, 0x64, 0x1f, 0x90, 0x00, 0x00, 0x0a,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0xa8,
};


uint16_t VGA::io_read_word(uint16_t port)
{
    WLock lock(_rw_lock);

    port -= get_region_address(VGA_PCI_IO_REGION);

    switch (port) {
    case IO_PORT_VBE_REG_SELECT:
        return _vbe_reg_index;
    case IO_PORT_VBE_DATA:
        if (_vbe_reg_index >= NUM_VBE_REGS) {
            W_MESSAGE("out of range");
            return 0;
        }

        if (_vbe_regs[VBE_REG_COMMAND] & VBE_COMMAND_CAPS) {
            switch (_vbe_reg_index) {
            case VBE_REG_X_RES:
                return VBE_MAX_X_RES;
            case VBE_REG_Y_RES:
                return VBE_MAX_Y_RES;
            case VBE_REG_DEPTH:
                return VBE_MAX_DEPTH;
            default:
                W_MESSAGE("unhendled get caps for reg %u");
                return 0;
            }
        }

        if (_vbe_reg_index == VBE_REG_EDID_DATA) {
            ASSERT(sizeof(edid_data) == EDID_BLOCK_SIZE);

            if (_edid_offset < EDID_BLOCK_SIZE) {
                const uint8_t* ptr = &edid_data[_edid_offset];
                _vbe_regs[VBE_REG_EDID_DATA] = *(const uint16_t*)ptr;
                _edid_offset += 2;
            } else if (_edid_offset == EDID_BLOCK_SIZE) {
                D_MESSAGE("invalid edid data read");
                _edid_offset += 2;
                _vbe_regs[VBE_REG_EDID_DATA] = 0xffff;
            }
        }

        return _vbe_regs[_vbe_reg_index];
    default:
        W_MESSAGE("invalid port 0x%x", port);
        return 0xffff;
    }
}


void VGA::propagate_fb()
{
    // need to ensure thread safe. Holding lock while calling
    // the front end is a potentail dead lock

    FrontEndList::iterator iter = _front_ends.begin();

    if (!(_vbe_regs[VBE_REG_COMMAND] & VBE_COMMAND_ENABLE) || _vbe_regs[VBE_REG_DEPTH] != 32) {
        for (; iter != _front_ends.end(); iter++) {
            (*iter)->get_front_end()->set(_fb.get(), _width, _height,
                                          _width * sizeof(uint32_t));
        }
    } else {
        for (; iter != _front_ends.end(); iter++) {
            uint32_t stride = _vbe_regs[VBE_REG_VIRT_WIDTH] * sizeof(uint32_t);
            uint8_t* base = _vram + _vbe_regs[VBE_REG_X_OFFSET] * sizeof(uint32_t) +
                            stride * _vbe_regs[VBE_REG_Y_OFFSET];
            uint8_t* end = base + _height * stride;

            if (base < _vram || end > _vbe_vram_end || end < base) {
                D_MESSAGE("invalid fb config");
                return;
            }

            AutoRef<SharedBuf> buf(new SharedBuf(VBE_VRAM_SIZE, base));
            (*iter)->get_front_end()->set(buf.get(), _width, _height,
                                          _vbe_regs[VBE_REG_VIRT_WIDTH] * sizeof(uint32_t));
        }
    }
}


void VGA::update_vga_window()
{
    if (!is_vbe_active()) {
        _window_start = _vram;
        _window_end = _vga_vram_end;
        return;
    }

    _window_start = _vram + uint32_t(_vbe_regs[VBE_REG_BANK]) * 64 * KB;
    ASSERT(_window_start >= _vram && _window_start < _vbe_vram_end);
    _window_end = MIN(_window_start + VGA_VRAM_SIZE, _vbe_vram_end);
}


void VGA::enable_vbe()
{
    VGA_D_MESSAGE("");
    // need to remap fb?
    _vga_active = false;
    _vga_draw_logic = false;

    if (!(_vbe_regs[VBE_REG_COMMAND] & VBE_COMMAN_NOCLEARMEM)) {
        memset(_fb->get(), 0, _fb->size());
        memset(_vram, 0, _vbe_vram_end - _vram);
    }

    if (!(_vbe_regs[VBE_REG_COMMAND] & VBE_COMMAND_LINEAR)) {
        D_MESSAGE("VBE_COMMAND_LINEAR is not set bpp=%u", _vbe_regs[VBE_REG_DEPTH]);
    }

    _palette_shift = (_vbe_regs[VBE_REG_COMMAND] & VBE_DISPI_8BIT_DAC) ? 0 : 2;

    if (_vbe_regs[VBE_REG_DEPTH] == 32) {
        _update_timer->disarm();
    } else {
        _update_timer->arm(UPDATE_INTERVAL, true);
    }

    _width = _vbe_regs[VBE_REG_X_RES];
    _height = _vbe_regs[VBE_REG_Y_RES];

    update_vga_window();

    propagate_fb();
}


void VGA::disable_vbe()
{
    VGA_D_MESSAGE("");
    bool active = !!(_crt_regs[CRT_REG_MODE] & CRT_MODE_ACTIVE_MASK);

    update_vga_window();
    _palette_shift = 2;

    if (!active) {
        return;
    }

    conditional_mode_change();
}


void VGA::nofify_vbe_fb_config()
{
    if (!(_vbe_regs[VBE_REG_COMMAND] & VBE_COMMAND_ENABLE)) {
        return;
    }

    propagate_fb();
}


void VGA::update_one_effective_palette(uint index)
{
    ASSERT(index < PALETTE_SIZE);
    _effective_palette[index].components[0] = _palette[index].components[2] << _palette_shift;
    _effective_palette[index].components[1] = _palette[index].components[1] << _palette_shift;
    _effective_palette[index].components[2] = _palette[index].components[0] << _palette_shift;
}


void VGA::io_write_word(uint16_t port, uint16_t val)
{
    WLock lock(_rw_lock);

    port -= get_region_address(VGA_PCI_IO_REGION);

    switch (port) {
    case IO_PORT_VBE_REG_SELECT:
        _vbe_reg_index = val;
        break;
    case IO_PORT_VBE_DATA:
        switch (_vbe_reg_index) {
        case VBE_REG_DISPLAY_ID:
            _vbe_regs[_vbe_reg_index] = val;
            break;
        case VBE_REG_COMMAND: {
            uint16_t curr_val = _vbe_regs[_vbe_reg_index];
            _vbe_regs[_vbe_reg_index] = val;

            if ((val & VBE_COMMAND_ENABLE) != (curr_val & VBE_COMMAND_ENABLE)) {
                if (val & VBE_COMMAND_ENABLE) {
                    enable_vbe();
                } else {
                    disable_vbe();
                }
            }
            break;
        }
        case VBE_REG_X_RES:
            if (val > VBE_MAX_X_RES) {
                D_MESSAGE("ignore x res %u", val);
                break;
            }
            _vbe_regs[VBE_REG_VIRT_WIDTH] = _vbe_regs[VBE_REG_X_RES] = val;
            break;
        case VBE_REG_Y_RES:
            if (val > VBE_MAX_Y_RES) {
                D_MESSAGE("ignore y res %u", val);
                break;
            }
            _vbe_regs[VBE_REG_VIRT_HEIGHT] = _vbe_regs[VBE_REG_Y_RES] = val;
            break;
        case VBE_REG_DEPTH:
            if (val != 32 && val != 24 && val != 16 && val != 15 && val != 8) {
                D_MESSAGE("%u bpp is not supported", val);
                break;
            }
            VGA_D_MESSAGE("%u bpp", val);
            _vbe_regs[VBE_REG_DEPTH] = val;
            break;
        case VBE_REG_BANK:
            if (val >= _vbe_regs[VBE_REG_VIDEO_MEMORY_64K]) {
                D_MESSAGE("ignore out of bound bank %u", val);
                break;
            }

            _vbe_regs[VBE_REG_BANK] = val;
            update_vga_window();
            break;
        case VBE_REG_X_OFFSET:
            _vbe_regs[VBE_REG_X_OFFSET] = val;
            if (val != 0) {
                D_MESSAGE("x offset %u", val);
            }
            nofify_vbe_fb_config();
            break;
        case VBE_REG_Y_OFFSET:
            _vbe_regs[VBE_REG_Y_OFFSET] = val;
            if (val != 0) {
                D_MESSAGE("y offset %u", val);
            }
            nofify_vbe_fb_config();
            break;
        case VBE_REG_VIRT_WIDTH:
            if (val < _vbe_regs[VBE_REG_X_RES]) {
                D_MESSAGE("ignoring virtual width %u (%u)", val, _vbe_regs[VBE_REG_X_RES]);
                return;
            }
            _vbe_regs[VBE_REG_VIRT_WIDTH] = val;
            nofify_vbe_fb_config();
            break;
        case VBE_REG_VIRT_HEIGHT:
            if (val != _vbe_regs[VBE_REG_Y_RES]) {
                D_MESSAGE("ignoring virtual height %u (%u)", val, _vbe_regs[VBE_REG_Y_RES]);
            }
            _vbe_regs[VBE_REG_VIRT_HEIGHT] = val;
            break;
        case VBE_REG_EDID_WINDOW:
            if (val == 0) {
                _vbe_regs[VBE_REG_EDID_WINDOW] = val;
                _edid_offset = 0;
            } else {
                _vbe_regs[VBE_REG_EDID_WINDOW] = 0xffff;
                _vbe_regs[VBE_REG_EDID_DATA] = 0xffff;
                _edid_offset = EDID_BLOCK_SIZE;
            }
            break;
        case VBE_REG_PALETTE_WRITE_INDEX:
            if (val > 0xff) {
                W_MESSAGE("bad palett index", val);
            }
            _vbe_regs[VBE_REG_PALETTE_WRITE_INDEX] = val & 0xff;
            _vba_palette_expect_red = 0;
            break;
        case VBE_REG_PALETTE_DATA: {
            uint index = _vbe_regs[VBE_REG_PALETTE_WRITE_INDEX];

            if (_vba_palette_expect_red) {
                _palette[index].components[0] = val;
                _vbe_regs[VBE_REG_PALETTE_WRITE_INDEX] = (index + 1) & 0xff;
                update_one_effective_palette(index);
            } else {
                _palette[index].components[2] = val;
                _palette[index].components[1] = val >> 8;
            }

            _vba_palette_expect_red = !_vba_palette_expect_red;
            break;
        }
        default:
            D_MESSAGE("unhandled %u, inf sleep", _vbe_reg_index);
            for (;;) sleep(2);
        }
        break;
    default:
        D_MESSAGE("invalid port 0x%x", port);
    }
}


bool VGA::start()
{
    if (_vga_active || (is_vbe_active() && _vbe_regs[VBE_REG_DEPTH] != 32)) {
        _update_timer->arm(UPDATE_INTERVAL, true);
    }

    return true;
}


bool VGA::stop()
{
    _update_timer->disarm();

    return true;
}


void VGA::on_io_enabled()
{
    WLock lock(_rw_lock);

    if (_enabled) {
        return;
    }

    _enabled = true;

    reset_io();
    reset_fb();
}


void VGA::on_io_disabled()
{
    WLock lock(_rw_lock);

    if (!_enabled) {
        return;
    }

    unmap_lagacy_io();
    unmap_lagacy_fb();

    _enabled = false;
}

