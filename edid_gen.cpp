/*
    Copyright (c) 2014 Yaniv Kamay,
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

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <sstream>
#include <fcntl.h>

#include "nox_types.h"
#include "debug.h"
#include "text_output.h"
#include "options_parser.h"
#include "utils.h"


#define DEFAULT_DPI 94
#define DEFAULT_REFRESH_RATE 100
#define INCH 25.4
#define INVALID_ESTABLISHED_BIT -1

typedef struct Resolution {
    Resolution() {}
    Resolution(uint in_width, uint in_height)
        : width (in_width)
        , height (in_height)
    {
    }

    uint width;
    uint height;
} Resolution;

typedef std::list<Resolution> ResList;

static inline uint16_t reverse_bytes(uint16_t x)
{
    return (x >> 8) | (x << 8);
}

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

#define LE16(x) x
#define LE32(x) x

#define BE16(x) reverse_bytes(uint16_t(x))
#else
    #error "implement me"
#endif

#define EDID_NUM_STD_TIMING 8
#define EDID_MAX_STR_LEN 13
#define EDID_DESCRIPTOR_BLOCK_SIZE 18
#define EDID_DESCRIPTOR_BLOCK_HEADER_SIZE 5

#define EDID_TAG_MONITOR_SERIAL 0xff
#define EDID_TAG_MONITOR_NAME 0xfc
#define EDID_TAG_MONITOR_LIMITS 0xfd


typedef struct __attribute__ ((__packed__)) EDID {
    // header
    uint8_t fixed_pattern[8];
    uint16_t manufacturer;
    uint16_t product_code;
    uint32_t serial_number;
    uint8_t week_of_manufacture;
    uint8_t year_of_manufacture;
    uint8_t edid_version;
    uint8_t edid_revision;

    // basic display parameters
    uint8_t input_parameters_bitmap;
    uint8_t maximum_horizontal_image_size_cm;
    uint8_t maximum_vertical_image_size_cm;
    uint8_t display_gamma;
    uint8_t supported_features_bitmap;
    uint8_t chromaticity_coordinates[10];
    uint8_t established_timing[3];
    uint16_t standard_timing[EDID_NUM_STD_TIMING];
    uint8_t detailed_timing[4][EDID_DESCRIPTOR_BLOCK_SIZE];
    uint8_t number_of_extensions;
    uint8_t checksum;
} EDID ;


struct StandardTiming {
    uint width;
    uint height;
    uint refresh_rate_Hz;
    uint h_frequency_Hz;
    uint pix_frequency_KHz;

    int established;
    uint ratio;
};


enum {
    EDID_RTIO_16_10 = 0,
    EDID_RTIO_4_3,
    EDID_RTIO_5_4,
    EDID_RTIO_16_9,
    EDID_RTIO_INVALID,
};


const StandardTiming standard_timing[] = { // from VESA Display Monitor Timing (DMT)
    //Established Timing I
    { 800,  600,  60, 37900,  40000, 0,  EDID_RTIO_4_3},
    { 640,  480,  75, 37500,  31500, 2,  EDID_RTIO_4_3},
    { 640,  480,  60, 31500,  25175, 5,  EDID_RTIO_4_3},
    { 720,  400,  70, 31469 , 28322, 7,  EDID_RTIO_INVALID}, // exception: data from DELL E153FPTc
                                                        // Service Manual
    //Established Timing II
    { 1280, 1024, 75, 80000, 135000, 8,  EDID_RTIO_5_4},
    { 1024,  768, 75, 70000,  78750, 9,  EDID_RTIO_4_3},
    { 1024,  768, 60, 48400,  65000, 11, EDID_RTIO_4_3},
    {  800,  600, 75, 46900,  49500, 14, EDID_RTIO_4_3},

    //Manufacturer's Timings
    //{ 1152, 870, ?, ?, 23},


    //Standard Timing
    { 1152,  864, 75,  67500, 108000, INVALID_ESTABLISHED_BIT, EDID_RTIO_4_3},
    { 1280,  800, 85,  71600, 122500, INVALID_ESTABLISHED_BIT, EDID_RTIO_16_10},
    { 1280,  960, 85,  85900, 148500, INVALID_ESTABLISHED_BIT, EDID_RTIO_4_3},
    { 1366,  768, 60,  47700,  85500, INVALID_ESTABLISHED_BIT, EDID_RTIO_16_9},
    { 1400, 1050, 85,  93900, 179500, INVALID_ESTABLISHED_BIT, EDID_RTIO_4_3},
    { 1600,  900, 60,  55540,  97750, INVALID_ESTABLISHED_BIT, EDID_RTIO_16_9},
    { 1600, 1200, 85, 106300, 229500, INVALID_ESTABLISHED_BIT, EDID_RTIO_4_3},
    { 1680, 1050, 85,  93900, 214750, INVALID_ESTABLISHED_BIT, EDID_RTIO_16_10},
    //{ 1792, 1344, 60,  83600, 204750, INVALID_ESTABLISHED_BIT, EDID_RTIO_4_3},
    //{ 1856, 1392, 60,  86300, 218250, INVALID_ESTABLISHED_BIT, EDID_RTIO_4_3},
    { 1920, 1080, 60,  67500, 148500, INVALID_ESTABLISHED_BIT, EDID_RTIO_16_9},
    { 1920, 1200, 60,  74600, 193250, INVALID_ESTABLISHED_BIT, EDID_RTIO_16_10},
    { 1920, 1440, 60,  90000, 234000, INVALID_ESTABLISHED_BIT, EDID_RTIO_4_3},
    { 2560, 1600, 60,  99500, 348500, INVALID_ESTABLISHED_BIT, EDID_RTIO_16_10},
};


static uint min_v_rate_Hz = ~0;
static uint max_v_rate_Hz = 0;
static uint min_h_rate_Hz = ~0;
static uint max_h_rate_Hz = 0;
static uint max_pixel_clock_KHz = 0;


static uint32_t jenkins_one_at_a_time_hash(const void *in_key, size_t len)
{
    const uint8_t* key = (const uint8_t*)in_key;
    uint32_t hash;
    uint32_t i;

    for(hash = i = 0; i < len; ++i) {
        hash += key[i];
        hash += (hash << 10);
        hash ^= (hash >> 6);
    }

    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);

    return hash;
}


static void update_frequency_minmax(uint v_rate_Hz, uint h_rate_Hz, uint pixel_clock_KHz)
{
    min_v_rate_Hz = MIN(min_v_rate_Hz, v_rate_Hz);
    max_v_rate_Hz = MAX(max_v_rate_Hz, v_rate_Hz);
    min_h_rate_Hz = MIN(min_h_rate_Hz, h_rate_Hz);
    max_h_rate_Hz = MAX(max_h_rate_Hz, h_rate_Hz);
    max_pixel_clock_KHz = MIN(max_pixel_clock_KHz, pixel_clock_KHz);
}


static void set_one_established(EDID& id, const StandardTiming* timing)
{
    id.established_timing[timing->established / 8] |= 1 << (timing->established % 8);

    update_frequency_minmax(timing->refresh_rate_Hz, timing->h_frequency_Hz,
                            timing->pix_frequency_KHz);
}


static void set_one_standard_timing(EDID& id, const StandardTiming* timing, uint& index)
{
    ASSERT(index < EDID_NUM_STD_TIMING);

    uint8_t* slot = (uint8_t*)&id.standard_timing[index++];
    slot[0] = (timing->width / 8) - 31;
    slot[1] = (timing->refresh_rate_Hz - 60) | (timing->ratio << 6);
    update_frequency_minmax(timing->refresh_rate_Hz, timing->h_frequency_Hz,
                            timing->pix_frequency_KHz);
}


static void set_standard_timing(EDID& id, uint max_width, uint max_height)
{
    int n = sizeof(standard_timing) / sizeof(standard_timing[0]);
    uint std_index = 0;

    for (int i = n - 1; i >= 0; i--) {
        const StandardTiming* now = standard_timing + i;

        if (now->width > max_width || now->height > max_height) {
                continue;
        }

        if (now->established != INVALID_ESTABLISHED_BIT) {
            set_one_established(id, now);
            continue;
        }

        if (std_index == EDID_NUM_STD_TIMING) {
            continue;
        }

        set_one_standard_timing(id, now, std_index);
    }
}


static const StandardTiming* find_std_timing(uint width, uint height)
{
    uint n = sizeof(standard_timing) / sizeof(standard_timing[0]);

    for (uint i = 0; i < n; i++) {
        const StandardTiming* now = standard_timing + i;

        if (now->width == width && now->height == height) {
            return now;;
        }
    }

    return NULL;
}


static void set_standard_timing(EDID& id, const ResList* res_list)
{
    ResList::const_iterator iter(res_list->begin());
    uint std_index = 0;

    for (; iter != res_list->end(); iter++) {
        const StandardTiming* timing = find_std_timing((*iter).width, (*iter).height);

        if (!timing) {
            THROW("no standard timing for info %ux%u", (*iter).width, (*iter).height);
        }

        if (timing->established != INVALID_ESTABLISHED_BIT) {
            set_one_established(id, timing);
            continue;
        }

        if (std_index == EDID_NUM_STD_TIMING) {
            THROW("out of standard timing slots");
        }

        set_one_standard_timing(id, timing, std_index);
    }
}


static void invalidate_standard_timing(EDID& id)
{
    memset(id.standard_timing, 0x01, sizeof(id.standard_timing));
}


static void set_detailed_timing(uint8_t* block, uint width_pix, uint height_pix,
                                uint width_mm, uint hight_mm)
{
    const uint v_rate = DEFAULT_REFRESH_RATE;
    const uint h_blanking = 160;
    const uint v_blanking = 35;
    const uint h_sync_offset = 48;
    const uint h_sync_width = 32;
    const uint v_sync_offset = 3;
    const uint v_sync_width = 6;

    uint pixel_rate = v_rate * (width_pix + h_blanking) * (height_pix + v_blanking);

    update_frequency_minmax(DEFAULT_REFRESH_RATE, pixel_rate / (height_pix + v_blanking),
                            pixel_rate / 1000);

    *(uint16_t*)block = LE16(pixel_rate / 10000);
    block[2] = width_pix;
    block[3] = h_blanking;
    block[4] = ((width_pix & 0xf00) >> 4) | ((h_blanking & 0xf00) >> 8);

    block[5] = height_pix;
    block[6] = v_blanking;
    block[7] = ((height_pix & 0xf00) >> 4) | ((v_blanking & 0xf00) >> 8);

    block[8] = h_sync_offset;
    block[9] = h_sync_width;

    block[10] = ((v_sync_offset & 0x0f) << 4) | (v_sync_width & 0x0f);

    block[11] = ((h_sync_offset & 0x300) >> 2) | ((h_sync_width & 0x300) >> 4) |
                 ((v_sync_offset & 0x30) >> 2) | ((v_sync_width & 0x30) >> 4);

    block[12] = width_mm;
    block[13] = hight_mm;
    block[14] = ((width_mm & 0xf00) >> 4) | ((hight_mm & 0xf00) >> 8);

    block[15] = block[16] = 0; // border

    block[17] = 0x1a; /* from Dell U2410 EDID block*/
}


static void set_descriptor_tag(uint8_t* block, uint8_t tag)
{
    block[0] = 0;
    block[1] = 0;
    block[2] = 0;
    block[3] = tag;
    block[4] = 0;
}


static void terminate_descriptor(uint8_t* block, uint offset)
{
    if (offset == EDID_DESCRIPTOR_BLOCK_SIZE) {
        return;
    }

    block[offset++] = 0x0a;

    for (; offset < EDID_DESCRIPTOR_BLOCK_SIZE; offset++) {
        block[offset] = 0x20;
    }
}


static void set_string(uint8_t* block, const char* str, uint8_t tag)
{
    set_descriptor_tag(block, tag);

    size_t str_len = strlen(str);

    if (str_len > EDID_MAX_STR_LEN) {
        D_MESSAGE("str \"%s\" is too long", str);
    }

    str_len = MIN(EDID_MAX_STR_LEN, str_len);
    memcpy(block + EDID_DESCRIPTOR_BLOCK_HEADER_SIZE, str, str_len);
    terminate_descriptor(block, EDID_DESCRIPTOR_BLOCK_HEADER_SIZE + str_len);
}


static void set_name(uint8_t* block, const char* name)
{
    set_string(block, name, EDID_TAG_MONITOR_NAME);
}


static void set_range_limits(uint8_t* block)
{
    set_descriptor_tag(block, EDID_TAG_MONITOR_LIMITS);

    uint8_t* pos = block + EDID_DESCRIPTOR_BLOCK_HEADER_SIZE;
    *pos++ = min_v_rate_Hz;
    *pos++ = max_v_rate_Hz;
    *pos++ = min_h_rate_Hz / 1000;
    *pos++ = max_h_rate_Hz / 1000;
    *pos++ = max_pixel_clock_KHz / 1000 / 10;

    *pos++ = 0;

    terminate_descriptor(block, pos - block);
}


static void set_serial(uint8_t* block, uint32_t serial)
{
    std::string str;
    sprintf(str, "%08X", serial);
    set_string(block, str.c_str(), EDID_TAG_MONITOR_SERIAL);
}


static uint16_t m_char(char ch)
{
    ASSERT(isupper(ch));
    return (ch - 'A' + 1) & 0x1f;
}


static void init_edid(EDID& id, uint width_pix, uint height_pix, uint width_mm, uint height_mm,
                      const char manufacturer[3], const char* product_name, uint16_t product_code,
                      const ResList* res_list)
{
    const uint8_t fixed_pattern[] = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00};
    const uint8_t chromaticity_coordinates[] = {0x1e, 0xc5, 0xae, 0x4f, 0x34, 0xb1, 0x26,
                                                0x0e, 0x50, 0x54}; /* from Dell U2410 EDID block*/
    struct tm tm;

    time_t t = time(NULL);
    gmtime_r(&t, &tm);

    uint32_t serial = rand();

    memset(&id, 0, sizeof(id));
    invalidate_standard_timing(id);
    memcpy(id.fixed_pattern, fixed_pattern, sizeof(id.fixed_pattern));

    id.manufacturer = BE16((m_char(manufacturer[0]) << 10) | (m_char(manufacturer[1]) << 5) |
                            m_char(manufacturer[2]));
    id.product_code = LE16(product_code);
    id.serial_number = LE32(serial);
    id.week_of_manufacture = tm.tm_yday / 7 + 1;
    id.year_of_manufacture =  tm.tm_year - 90;
    id.edid_version = 1;
    id.edid_revision = 3;

    id.input_parameters_bitmap = (1 << 7); // digital input
    id.maximum_horizontal_image_size_cm = round(double(width_mm) / 10);
    id.maximum_vertical_image_size_cm = round(double(height_mm) / 10);
    id.display_gamma = 120; // gamma is 2.2 (display_gamma = (gamma * 100) - 100;

    id.supported_features_bitmap =  (1 /*RGB 4:4:4 + YCrCb 4:4:4*/ << 3) |
                                    (1 << 2) | /* Standard sRGB colour space. Bytes 25-34 must
                                                contain sRGB standard values */
                                    (1 << 1) ; /* preferred timing mode is indicated in
                                                 the first detailed timing block */
    memcpy(id.chromaticity_coordinates, chromaticity_coordinates,
           sizeof(id.chromaticity_coordinates));

    if (res_list) {
        set_standard_timing(id, res_list);
    } else {
        set_standard_timing(id, width_pix, height_pix);
    }

    set_detailed_timing(&id.detailed_timing[0][0], width_pix, height_pix, width_mm, height_mm);
    set_serial(&id.detailed_timing[2][0], serial);
    set_name(&id.detailed_timing[1][0], product_name);
    set_range_limits(&id.detailed_timing[3][0]);
    id.checksum = checksum8(&id, sizeof(id));
}


static bool str_to_two_dim(const std::string& str, uint& x, uint& y)
{
    size_t end;

    if ((end = str.find('x')) == str.npos && (end = str.find('X')) == str.npos) {
        return false;
    }

    std::string xstr(str, 0, end);
    std::string ystr(str, end + 1, str.length() - end - 1);

    return str_to_uint(xstr.c_str(), x, 10) && x > 0 &&
           str_to_uint(ystr.c_str(), y, 10) && y > 0;
}


static bool compare_res(const Resolution& first, const Resolution& second)
{
    if (first.width != second.width) {
        return first.width < second.width;;
    }

    return first.height < second.height;
}


static bool same_res(const Resolution& first, const Resolution& second)
{
    return first.width == second.width && first.height == second.height;
}


static bool clear_blanks(std::string& str)
{
    std::string::reverse_iterator riter;

    while ((riter = str.rbegin()) != str.rend()) {
        if (*riter == ',') {
            return false;
        }

        if (isblank(*riter)) {
            str.erase(--str.end());
            continue;
        }

        break;
    }

    return true;
}


static bool parse_std_res(std::string str, ResList& res_list)
{
    if (!clear_blanks(str)) {
        return false;
    }

    res_list.clear();

    std::istringstream is(str);
    std::string one_res;

    while (std::getline(is, one_res, ',')) {
        Resolution res;

        if (!str_to_two_dim(one_res, res.width, res.height)) {
            return false;
        }

        res_list.push_back(res);
    }

    res_list.sort(compare_res);
    res_list.unique(same_res);

    return true;
}


static void print_std_res()
{
    ResList res_list;

    uint n = sizeof(standard_timing) / sizeof(standard_timing[0]);

    for (uint i = 0; i < n; i++) {
        res_list.push_back(Resolution(standard_timing[i].width, standard_timing[i].height));
    }

    res_list.sort(compare_res);
    res_list.unique(same_res);

    ResList::iterator iter = res_list.begin();

    for (; iter != res_list.end(); iter++) {
        printf("%ux%u\n", (*iter).width, (*iter).height);
    }
}


static uint32_t hash_std_res(const ResList& res_list)
{
    uint n_rers = res_list.size();
    AutoArray<uint> vec(new uint[n_rers * 2]);

    ResList::const_iterator iter = res_list.begin();

    for (uint i = 0; i < n_rers; i++, iter++) {
        vec[i * 2] = (*iter).width;
        vec[i * 2 + 1] = (*iter).height;
    }

    return jenkins_one_at_a_time_hash(vec.get(), sizeof(vec[0]) * n_rers * 2);
}


static void hex_dump(const EDID& edid)
{
    uint8_t* now = (uint8_t*)&edid;
    uint8_t* end = now + sizeof(edid);

    while (now < end) {
        printf("    0x%.2x,", *now++);
        for (uint i = 0; i < 7 && now < end; i++, now++) {
            printf(" 0x%.2x,", *now);
        }
        printf("\n");
    }
}


static void bin_dump(const EDID& edid, const char* file_name)
{
    AutoFD fd(open(file_name, O_CREAT | O_EXCL | O_WRONLY, 0644));

    if (!fd.is_valid()) {
        THROW_SYS_ERROR("failed");
    }

    write_all(fd.get(), 0, &edid, sizeof(edid));
}


int main(int argc, const char** argv)
{
    try {
        srand(time(NULL));

        OptionsParser parser;

        parser.set_front_positional_minmax(1, 1);

        enum {
            OPT_DISPLAY_SIZE,
            OPT_RES_LIST,
            OPT_BIN_DUMP,
            OPT_SHOW_STD_RES,
        };

        parser.add_option_with_arg(OPT_DISPLAY_SIZE, "size", OptionsParser::ONE_ARGUMENT,
                                   "width'x'height", "display size in mm");
        parser.add_option_with_arg(OPT_RES_LIST, "res-list", OptionsParser::ONE_ARGUMENT,
                                   "x-res'x'y-res,x-res'x'y-res...",
                                   "suported standard resolutions list");
        parser.add_option_with_arg(OPT_BIN_DUMP, "bin-dump", OptionsParser::ONE_ARGUMENT,
                                   "file-name",
                                   "dump EDID block into a file");
        parser.add_option(OPT_SHOW_STD_RES, "show-std-res",
                          "show available standard resolutions",
                          OptionsParser::EXCLUSIVE_ARG);

        if (!parser.parse(argc, argv)) {
            return -1;
        }

        int option;
        const char* arg;
        uint x_res;
        uint y_res;
        uint width = ~0;
        uint height;
        bool explicit_res = false;
        ResList res_list;
        const char* bin_file = NULL;

        while ((option = parser.next(&arg)) != OptionsParser::OPT_ID_DONE) {
            switch (option) {
            case OptionsParser::OPT_ID_FRONT_POSITIONAL:
                if (!str_to_two_dim(arg, x_res, y_res)) {
                    printf("%s: invalid resolution %s\n", parser.get_prog_name(), arg);
                    return -1;
                }

                break;
            case OPT_DISPLAY_SIZE:
                if (!str_to_two_dim(arg, width, height)) {
                    printf("%s: invalid display size %s\n", parser.get_prog_name(), arg);
                    return -1;
                }

                break;
            case OPT_RES_LIST: {
                explicit_res = true;
                if(!parse_std_res(arg, res_list)) {
                    printf("%s: invalid resolutions list %s\n", parser.get_prog_name(), arg);
                    return -1;
                }
                break;
            }
            case OPT_BIN_DUMP:
                bin_file = arg;
                break;
            case OPT_SHOW_STD_RES:
                print_std_res();
                return 0;
            case OptionsParser::OPT_ID_HELP:
                parser.help();
                return 0;
            }
        }

        if (width == ~0) {
            width = double(x_res) / DEFAULT_DPI * INCH;
            height = double(y_res) / DEFAULT_DPI * INCH;
        }

        std::string monitor_name;
        const char* manufacturer;

        const char manufacturer_a[3] = { 'N', 'O', 'X'};
        const char manufacturer_b[3] = { 'X', 'O', 'N'};
        double size = sqrt(pow(width, 2) + pow(height, 2)) / INCH;
        double ratio = double(x_res) / y_res;
        double dpi = double(x_res) / width * INCH;
        uint16_t product_code = (uint(round(size)) << 8) | uint(round(dpi));

        if (explicit_res) {
            uint32_t hash = hash_std_res(res_list);
            hash &= 0xffff;
            hash = (hash & 0xffff) ^ (hash >> 16);

            sprintf(monitor_name, "VM%.1f%X-%X", size, uint(round(dpi)), hash);
            manufacturer = manufacturer_b;
            product_code ^= hash;
        } else {
            sprintf(monitor_name, "NOX%.1f%X-%X", size, uint(round(ratio * 10)), uint(round(dpi)));
            manufacturer = manufacturer_a;
        }

        EDID edid;

        init_edid(edid, x_res, y_res, width, height, manufacturer, monitor_name.c_str(),
                  product_code, explicit_res ? &res_list : NULL);

        if (bin_file) {
            bin_dump(edid, bin_file);
        } else {
            hex_dump(edid);
        }
    } catch (Exception& e) {
        E_MESSAGE("%s", e.what());
        return -1;
    } catch (std::exception& e) {
        printf("%s\n", e.what());
        return -1;
    } catch (...) {
        printf("unknown exception");
        return -1;
    }

    return 0;
}

