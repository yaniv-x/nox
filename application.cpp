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

#include <sys/stat.h>

#include "application.h"
#include "nox_vm.h"
#include "options_parser.h"
#include "admin_server.h"

#define MIN_MEM_SIZE (uint64_t(1) * MB)
#define MAX_MEM_SIZE (uint64_t(8) * GB) // todo: calc dinamically according to system resources
#define VM_NAME_MAX_LENGTH 64

std::string nox_dir;

static uint64_t translate_ram_size(const char* arg)
{
    char* end_ptr;
    uint64_t size = strtoul(arg, &end_ptr, 10);

    if (size > INT_MAX || end_ptr != arg + strlen(arg) - 1) {
        return 0;
    }

    switch (*end_ptr) {
    case 'g':
    case 'G':
        return size * GB;
    case 'm':
    case 'M':
        return size * MB;
    default:
        return 0;
    }
}


static bool is_valid_vm_name(const char* vm_name)
{
    size_t len = strlen(vm_name);

    if (!len || strlen(vm_name) > VM_NAME_MAX_LENGTH) {
        return false;
    }

    for (const char* end = vm_name + len; vm_name < end; vm_name++) {
        if (!isalnum(*vm_name) && *vm_name != '_') {
            return false;
        }
    }

    return true;
}


Application* application = NULL;


Application::Application()
    : _vm (NULL)
{
    application = this;
}


bool Application::init(int argc, const char** argv)
{
    OptionsParser parser;

    parser.set_front_positional_minmax(1, 1);

    enum {
        OPT_RAM_SIZE,
        OPT_HARD_DISK,
        OPT_CDROM,
        OPT_BOOT_DEVICE,
    };

    parser.add_option_with_arg(OPT_RAM_SIZE, "ram-size", OptionsParser::ONE_ARGUMENT,
                               "ram_size<unit>", "specifay ram size. unit can be M/m or G/g",
                               OptionsParser::MANDATORY);
    parser.add_option_with_arg(OPT_HARD_DISK, "hard-disk", OptionsParser::ONE_ARGUMENT,
                               "file_name", "specifay hard disk file name",
                               OptionsParser::MANDATORY);
    parser.add_option_with_arg(OPT_CDROM, "cdrom", OptionsParser::ONE_ARGUMENT, "file_name",
                               "specifay hard disk file name");
    parser.add_option_with_arg(OPT_BOOT_DEVICE, "boot-device", OptionsParser::ONE_ARGUMENT,
                               "device", "specifay boot device \"hd\" pr \"cd\"");

    parser.set_short_name(OPT_RAM_SIZE, 'm');
    parser.set_short_name(OPT_HARD_DISK, 'h');

    if (!parser.parse(argc, argv)) {
        set_exit_code(ERROR_INVALID_COMMAND_LINE);
        return false;
    }

    const char* vm_name = NULL;
    uint64_t ram_size = 0;
    const char* hard_disk = NULL;
    const char* cdrom = NULL;
    bool boot_from_cd = false;

    int option;
    const char* arg;

    while ((option = parser.next(&arg)) != OptionsParser::OPT_ID_DONE) {
        switch (option) {
        case OptionsParser::OPT_ID_FRONT_POSITIONAL:
            vm_name = arg;
            if (!is_valid_vm_name(vm_name)) {
                printf("invalid vm name %s\n", vm_name);
                return false;
            }
            break;
        case OPT_RAM_SIZE: {
            ram_size = translate_ram_size(arg);

            if (ram_size == 0) {
                printf("invalid ram size %s\n", arg);
                set_exit_code(ERROR_INVALID_COMMAND_LINE);
                return false;
            }

            if ( ram_size < MIN_MEM_SIZE || ram_size > MAX_MEM_SIZE) {
                printf("invalid ram size %s, valid size range is %lum to %lug\n",
                       arg, MIN_MEM_SIZE / MB, MAX_MEM_SIZE / GB);
                set_exit_code(ERROR_INVALID_COMMAND_LINE);
                return false;
            }
            break;
        }
        case OPT_HARD_DISK:
            hard_disk = arg;
            break;
        case OPT_CDROM:
            cdrom = arg;
            break;
        case OPT_BOOT_DEVICE:
            if (strcmp(arg, "hd") == 0) {
                boot_from_cd = false;
            } else if (strcmp(arg, "cd") == 0) {
                boot_from_cd = true;
            } else {
                printf("invalid boot-device argumant %s. use \"hd\" or \"cd\"\n", arg);
                set_exit_code(ERROR_INVALID_COMMAND_LINE);
                return false;
            }
            break;
        case OptionsParser::OPT_ID_HELP:
            parser.help();
            set_exit_code(ERROR_OK);
            return false;
        default:
            THROW("invalid option %d", option);
        }
    }

    std::string uds_name;
    sprintf(uds_name, "%s/active/%s.uds", nox_dir.c_str(), vm_name);
    _admin_server.reset(new AdminServer(uds_name));

    try {
        _vm.reset(new NoxVM());

        _vm->set_ram_size(ram_size / MB);
        _vm->set_hard_disk(hard_disk);
        _vm->set_cdrom(cdrom);
        _vm->set_boot_device(boot_from_cd);

        _vm->init();
    } catch (...) {
        E_MESSAGE("vm initialization failed");
        set_exit_code(ERROR_VM_INIT_FAILED);
        return false;
    }

    _vm->vm_reset();
    _vm->vm_start(NULL, NULL);

    return true;
}


static void sig_handler(int sig)
{
}


static void init_sig_handlers()
{
    struct sigaction act;

    memset(&act, 0, sizeof(act));
    sigfillset(&act.sa_mask);
    act.sa_handler = sig_handler;

    if (sigaction(SIGUSR1, &act, NULL) == -1) {
        THROW("sigaction failed %d", errno);
    }
}


static void init_nox_dir()
{
    char* home_dir = getenv("HOME");

    if (!home_dir || strlen(home_dir) < 2) {
        THROW("no home dir");
    }

    nox_dir = home_dir;

    if (nox_dir[nox_dir.length() - 1] != '/') {
        nox_dir += "/";
    }

    nox_dir += "nox";

    if (mkdir(nox_dir.c_str(), 0777) == -1 && errno != EEXIST) {
        THROW("create dir %s failed %d %s", nox_dir.c_str(), errno, strerror(errno));
    }

    std::string sub_dir = nox_dir + "/active";

    if (mkdir(sub_dir.c_str(), 0777) == -1 && errno != EEXIST) {
        THROW("create dir %s failed %d %s", sub_dir.c_str(), errno, strerror(errno));
    }
}


const std::string& Application::get_nox_dir()
{
    return nox_dir;
}


ErrorCode Application::main(int argc, const char** argv)
{
    init_sig_handlers();
    init_nox_dir();

    std::auto_ptr<Application> app(new Application());

    if (app->init(argc, argv)) {
        app->run();
    }

    return app->get_exit_code();
}

