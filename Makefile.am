SUFIXES = nasm

.nasm.o:
	nasm -f elf64 -g -F dwarf -o $@ $<


bin_PROGRAMS = nox nox-admin


nox_admin_SOURCES =					\
    options_parser.cpp			\
    utils.cpp				\
    admin_common.cpp        \
    dynamic_call.nasm        \
    admin_client.cpp

nox_SOURCES =					\
	main.cpp				\
	application.cpp				\
	utils.cpp				\
	nox_vm.cpp				\
	vm_part.cpp				\
	kvm.cpp					\
	io_bus.cpp				\
	memory_bus.cpp				\
	cpu.cpp					\
	threads.cpp				\
	cmos.cpp				\
	dma.cpp					\
	pit.cpp					\
	keyboard.cpp				\
	place_holder.cpp			\
	pic.cpp					\
	pci_bus.cpp				\
        pci_device.cpp                          \
	vga.cpp					\
	display.cpp				\
	options_parser.cpp			\
	uds_listener.cpp		\
	admin_server.cpp		\
    admin_common.cpp        \
    dynamic_call.nasm        \
    ata_device.cpp        \
    ata_host.cpp        \
    ata_disk.cpp        \
    atapi_cdrom.cpp        \
    block_device.cpp        \
    speaker.cpp             \
    firmware_file.cpp       \
	run_loop.cpp

INCLUDES =												\
	-Wall -Werror -Wno-sign-compare -Wno-switch-enum						\
	-Wno-pmf-conversions										\
	-DNOX_DEBUG											\
	$(X11_CFLAGS)											\
	$(PULSE_CFLAGS)

nox_LDADD = \
	-lrt -lpthread $(X11_LIBS) $(PULSE_LIBS)

#nox_DEPENDENCIES = yaniv

#yaniv:
#	@echo "******** nox dependency, add updating version file here ********"

