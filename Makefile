#
# Simepl makefile for compiling all .c and .s files in the current folder.
# No dependency tracking, use make clean if a header is changed.
#

SRCS = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SRCS))

GCC_PREFIX ?= arm-none-eabi-

%.o : %.c
	$(GCC_PREFIX)gcc -pedantic -ffreestanding -ffunction-sections -mcpu=cortex-m3 -Wall -O2 -DSTM32F103xB -mthumb -I . -c -o $@ $<

%.o : %.s
	$(GCC_PREFIX)gcc -mcpu=cortex-m3 -mthumb -c -o $@ $<

all : $(OBJS) startup_stm32f103c8tx.o
	$(GCC_PREFIX)ld -nostartfiles -nostdlib --gc-sections -T LinkerScript.ld -o keyhole.elf $(OBJS) startup_stm32f103c8tx.o
	$(GCC_PREFIX)objcopy -O binary keyhole.elf keyhole.bin

clean:
	rm -f *.o *.elf *.bin

