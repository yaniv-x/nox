bin_PROGRAMS = nox

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
	ata_controller.cpp

INCLUDES =												\
	-Wall -Werror -Wno-sign-compare -Wno-switch-enum						\
	-Wno-pmf-conversions										\
	-DNOX_DEBUG

nox_LDADD = \
	-lrt -lpthread

#nox_DEPENDENCIES = yaniv

#yaniv:
#	@echo "******** nox dependency, add updating version file here ********"

