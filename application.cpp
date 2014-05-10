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

#include <sys/stat.h>
#include <fcntl.h>
#include <sys/file.h>

#include "application.h"
#include "nox_vm.h"
#include "options_parser.h"
#include "admin_server.h"
#include "gdb_target.h"

#define MIN_MEM_SIZE (uint64_t(1) * MB)
#define MAX_MEM_SIZE (uint64_t(8) * GB) // todo: calc dinamically according to system resources
#define VM_NAME_MAX_LENGTH 64
#define MAX_CPUS 15 // for now: prevent conflict with ioapic apic-id

std::string nox_dir;
static std::string lock_file_name;
static int lock_fd = -1;

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
    , _quit_event (create_event((void_callback_t)&Application::quit_handler, this))
    , _quitting (false)
{
    init_signals();
    application = this;
}


Application::~Application()
{
    restore_signals();
    _quit_event->destroy();
}


void Application::sig_int_handler(int sig)
{
    W_MESSAGE("");
    application->quit();
}


void Application::sig_term_handler(int sig)
{
    W_MESSAGE("");
    application->quit();
}


void  Application::init_signals()
{
    struct sigaction act;

    memset(&act, 0, sizeof(act));
    sigfillset(&act.sa_mask);

    act.sa_handler = sig_term_handler;

    if (sigaction(SIGTERM, &act, &_prev_term_act) == -1) {
        THROW("sigaction failed %d", errno);
    }

    act.sa_handler = sig_int_handler;

    if (sigaction(SIGINT, &act, &_prev_int_act) == -1) {
        sigaction(SIGTERM, &_prev_term_act, NULL);
        THROW("sigaction failed %d", errno);
    }
}


void Application::restore_signals()
{
    sigaction(SIGTERM, &_prev_term_act, NULL);
    sigaction(SIGINT, &_prev_int_act, NULL);
}


void Application::continue_quitting(bool ok)
{
    run_break();
}


void Application::quit_handler()
{
    if (_quitting) {
        return;
    }

    D_MESSAGE("");
    _quitting = true;
    _vm->vm_down((NoxVM::compleation_routin_t)&Application::continue_quitting, this);
}


void Application::quit()
{
    _quit_event->trigger();
}


void Application::enable_gdb(AdminReplyContext* context, uint32_t port)
{
    if (port > USHRT_MAX) {
        context->command_reply("INVLID_PORT");
        return;
    }

    if (_gdb_target.get()) {
        std::string str;
        sprintf(str, "EXIST: port=%u", _gdb_target->get_port());
        context->command_reply(str.c_str());
        return;
    }

    try {
        _gdb_target.reset(new GDBTarget(*_vm, *this, port));
    } catch (...) {
        context->command_reply("FAILED");
        return;
    }

    context->command_reply("OK");
}


void Application::register_admin_commands()
{
    _admin_server->register_command("enable-gdb", "enable remote debugging", "???",
                                    admin_types(1, VA_UINT32_T),
                                    admin_names(1, "port"),
                                    admin_types(1, VA_UTF8_T),
                                    admin_names(1, "result"),
                                    (admin_command_handler_t)&Application::enable_gdb, this);
}


static void release_lock()
{
    if (lock_fd != -1) {
        unlink(lock_file_name.c_str());
        close(lock_fd);
    }
}


bool Application::ecquire_exclusive_rights(const char* vm_name)
{
    // for now using specific lock file, later on will use ".nox" file

    sprintf(lock_file_name, "%s/tmp/nox-%s.lock", nox_dir.c_str(), vm_name);

    atexit(release_lock);

    int fd = open(lock_file_name.c_str(), O_CREAT | O_RDONLY, S_IRUSR);

    if (fd == -1) {
        int e = errno;
        W_MESSAGE("open lock-file \"%s\" failed: %s", lock_file_name.c_str(), strerror(e));
        return false;
    }

    if (flock(fd, LOCK_EX | LOCK_NB) == -1) {
        int e = errno;
        D_MESSAGE("lock failed: %s", strerror(e));
        close(fd);
        return false;
    }

    lock_fd = fd;

    return true;
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
        OPT_CPU_COUNT,
    };

    parser.add_option(OPT_RAM_SIZE, "ram-size", "<ram_size>unit", false,
                      "specifay ram size. unit can be M/m or G/g",
                       OptionsParser::MANDATORY);
    parser.add_option(OPT_HARD_DISK, "hard-disk", "<file_name>[,ro]", false,
                      "specifay hard-disk image file",
                      OptionsParser::MANDATORY);
    parser.add_option(OPT_CDROM, "dvd-rom", "<file_name>", true, "specifay dvd/cd iso file");
#if 0
    parser.add_option_with_arg(OPT_BOOT_DEVICE, "boot-device", OptionsParser::ONE_ARGUMENT,
                               "device", "specifay boot device \"hd\" or \"cd\"");
#endif
    parser.add_option(OPT_CPU_COUNT, "cpu-count", "<cpu_count>", false, "number of cpus");
    parser.set_short_name(OPT_RAM_SIZE, 'm');
    parser.set_short_name(OPT_HARD_DISK, 'h');

    if (!parser.parse(argc, argv)) {
        set_exit_code(ERROR_INVALID_COMMAND_LINE);
        return false;
    }

    const char* vm_name = NULL;
    uint64_t ram_size = 0;
    std::string hard_disk_file;
    bool ro_hard_disk_file = true;
    bool cdrom = false;
    const char* cdrom_media = NULL;
    bool boot_from_cd = false;
    ulong num_cpus = 1;

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
        case OPT_HARD_DISK: {
            std::auto_ptr<OptionsParser::Inner> iner(parser.parse_val(OPT_HARD_DISK, arg, 1, 1));

            if (!iner.get()) {
                return false;
            }

            const char* file = iner->get_positional(0);

            ASSERT(file);

            if (file[0] != '/') {
                printf("invalid disk file name \"%s\"\n", file);
                return false;
            }

            hard_disk_file = file;
            ro_hard_disk_file = iner->switch_test("ro");

            break;
        }
        case OPT_CDROM:
            cdrom = true;
            cdrom_media = arg;
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
        case OPT_CPU_COUNT:
            if (!str_to_ulong(arg, num_cpus) || !num_cpus || num_cpus > MAX_CPUS) {
                printf("invalid cpu count \"%s\", valid values are in the range 1 through %u\n",
                       arg, MAX_CPUS);
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

    if (!ecquire_exclusive_rights(vm_name)) {
        printf("failed to ecquire exclusive vm rights\n");
        set_exit_code(ERROR_VM_IN_USE);
        return false;
    }


    std::string uds_name;
    sprintf(uds_name, "%s/active/%s.uds", nox_dir.c_str(), vm_name);
    _admin_server.reset(new AdminServer(uds_name));

    try {
        _vm.reset(new NoxVM(vm_name));

        _vm->set_ram_size(ram_size / MB);
        _vm->set_hard_disk(hard_disk_file.c_str(), ro_hard_disk_file);
        _vm->set_num_cpus(num_cpus);

        if (cdrom) {
            _vm->set_cdrom(cdrom_media);
        }

        _vm->set_boot_device(boot_from_cd);

        _vm->init();
    } catch (std::exception& e) {
        E_MESSAGE("vm initialization failed: %s", e.what());
        set_exit_code(ERROR_VM_INIT_FAILED);
        return false;
    } catch (...) {
        E_MESSAGE("vm initialization failed");
        set_exit_code(ERROR_VM_INIT_FAILED);
        return false;
    }

    _vm->vm_reset();
    _vm->vm_start(NULL, NULL);

    register_admin_commands();

    return true;
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

    sub_dir = nox_dir + "/tmp";

    if (mkdir(sub_dir.c_str(), 0777) == -1 && errno != EEXIST) {
        THROW("create dir %s failed %d %s", sub_dir.c_str(), errno, strerror(errno));
    }
}


const std::string& Application::get_nox_dir()
{
    return nox_dir;
}


bool Application::find_firmware(std::string& file, const std::vector< std::string>& names)
{
    for (int i = 0; i < names.size(); i++) {
        struct stat stat_buf;

        file = get_nox_dir() + "/firmware/" + names[i].c_str();

        if (stat(file.c_str(), &stat_buf) != -1) {
            return true;
        }

        if (errno != ENOENT) {
            break;
        }
    }

    file = "";

    return false;
}


ErrorCode Application::main(int argc, const char** argv)
{
    srand(time(NULL));

    init_nox_dir();

    std::auto_ptr<Application> app(new Application());

    if (app->init(argc, argv)) {
        app->run();
    }

    return app->get_exit_code();
}

