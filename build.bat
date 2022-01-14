@echo off

del /F /Q *.o *.elf *.hex 2>nul

call set OBJS=

for %%f in (*.c) do (
	echo %%f
	arm-none-eabi-gcc -pedantic -ffunction-sections -std=c99 -O2 -Wall -DSTM32F103xB -mcpu=cortex-m3 -mthumb -I. -c -o %%~nf.o %%f
	call set OBJS=%%OBJS%% %%~nf.o
)

for %%f in (*.s) do (
	echo %%f
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -DSTM32F103xB -I. -c -o %%~nf.o %%f
	call set OBJS=%%OBJS%% %%~nf.o
)

arm-none-eabi-ld -nostartfiles -nostdlib --gc-sections -T LinkerScript.ld -o keyhole.elf %OBJS%
arm-none-eabi-objcopy -O binary keyhole.elf keyhole.bin
