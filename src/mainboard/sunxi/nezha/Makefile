OREBOOT=$(abspath $(CURDIR)/../../../../)
# RV64GCV, ackshully
TARGET     = riscv64gc-unknown-none-elf
QEMU       ?= qemu-system-riscv64
QEMU_FLAGS += -m 1g -machine sifive_u,start-in-flash=true -nographic -device loader,addr=0x20000000,file=${IMAGE} -bios none -smp 4
XFEL       ?= xfel
IMAGE      ?= target/riscv64gc-unknown-none-elf/release/image.bin
# DRAM space starts at 0x40000000, see manual p34
MEMADDR    ?= 0x40000000
include ../../../../Makefile.inc
include ../../../../Makefile.qemu.inc
include ../../../../Makefile.mainboard.inc
cibuild: $(IMAGE)
ciclippy: clippy
citest: skiptest

run:
	$(XFEL) ddr ddr3
	$(XFEL) write $(MEMADDR) $(IMAGE)
	$(XFEL) exec $(MEMADDR)

openocd:
	$(shell which openocd) -f openocd.cfg -c "flash write_image erase unlock $(IMAGE) 0x20000000; shutdown"
