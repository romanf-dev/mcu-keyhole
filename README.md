What is keyhole
---------------

Tool for rapid prototyping of simple embedded applications. The idea behind it 
is very simple: the device provides text interface for reading and writing 
internal memory of the MCU so peripheral devices may be controlled by the PC.

Traditional way requires at least the following steps:
- writing program in C/C++
- building
- flashing the app

Keyhole approach is way much simpler: you can write application on any language
you prefer and observe results immediately without reflashing. 

Why 'keyhole': By analogy. Instead of controlling device being 'in-room' (on 
internal flash) with full-speed access to registers and interrupts, you have 
to use small 'hole' to access registers. Also you can see only small piece of 
internal environment like observing room interior via keyhole.


How it works
------------

The device contains internal USB stack and provides USB CDC interface (serial 
port) to the host. The stack contains some fixes taken from 
[here](https://github.com/philrawlings/bluepill-usb-cdc-test).
Host requests are parsed and then the appropriate action is taken. Device 
operation is therefore may be represented as shell-like loop.

    infinite_loop
    {
        request = receive_text_data_from_host_via_USB
        execute_command(request)
    }


Building
--------

The project consists of minimum of code, only the things really needed. 
You don't need Cube, drivers, headers, libc, IDEs, etc. Only vanilla freestanding C compiler.
It is expected that arm-none-eabi- toolchain is available via PATH.


Port settings
-------------

Currently default settings from ST's USB stack are used: __9600/8-N-1__.


Protocol description
--------------------

There are 4 commands:

- Read memory
- Write memory
- Wait for condition
- Info

Command line syntax:

    <cmd>[optional access modifier] <arg1> <arg2> ... LF

Note that device does not output echos so don't expect usual command line
interface using either Putty or similar tools, it is intended to be used by 
programs, not humans!
However, if you prefer to use terminal program you may enable local echos.

| Cmd | Description         |
|-----|---------------------|
| r   | Read MCU memory     |
| w   | Write MCU memory    |
| i   | Device type & FW ver|
| u   | Wait condition      |

Addresses and values are always lowercase hexadecimal strings.
Each command except info may be also augmented by access width modifier:

| Key | Description         |
|-----|---------------------|
| b   | Byte (8 bits)       |
| w   | Word (16 bits)      |
| d   | Doubleword (32 bits)|

Width override keys are optional. By default memory access width is equal to width of "unsigned int".

And yet another couple of facts about the protocol:

- Request string must be terminated by LF.
- All device responses contain CRLF at end of the string.
- Integers wider than specified width are truncated.
- Integers wider than 32-bit are considered as error.
- Writes output just CRLFs.

Examples:

Read value at address 0x11223344:

    r 11223344

Write value 0x55667788 to address 0x11223344:

    w 11223344 55667788

Write byte to the same address:

    wb 11223344 55

Single string may contain more than one request, for example:

    'r 11223344 | r 22334455'

Device will perform two accesses and its responses will be also separated by |.
This approach may be helpful when there is need to perform two actions as fast as possible without 
USB transmission overhead.


Waiting
-------

Most MCU interfaces depend on timing and have 'ready' bits which must be polled 
before either sending or receiving data via that interface.
These bits may be polled by reads, but every read may take too much time since 
it should be sent to host and interpreted there. Separate wait command has been 
added to address this issue.
Instead of polling on PC side, wait command may be part of a batch like this:

    'u <wait arg> | w <write arg>'

This will write immediately after wait condition is satisfied.
Wait takes two arguments: address and wait descriptor described below. 
Also it may have access width modifier just like reads and writes.
Wait descriptor has the following format:

| Bits | Length | Description                                        |
|------|-------------------------------------------------------------|
| 4:0  | 5      | Bit number (0-31)                                  |
| 5    | 1      | Expected value to wait 1 or 0                      |
| 6    | 1      | If 1 - stop the batch if condition is not satisfied|
| 7    | 1      | Reserved                                           |
| 31:8 | 24     | Max iterations or default if 0                     |

For example, wait for 5th bit at address 11223344 to be 1:

    'u 11223344 25'

25 = 0b100101: bits [4:0] = 5, bit 5 = 1.


Platform support
----------------

Linux and Windows 10+ require no additional drivers, just plug and play. OSX should also work but untested.


Troubleshooting
---------------

On some Linux hosts after the device connected the host tries to talk to the 
CDC interface so device send ERROR permanently. Attempts to 'cat' the device
results in 'device busy' response.
Try wait few seconds or try to use root console.
Also it is recommended to send empty string at the beginning to flush out AT commands from input buffer.

Onboard LED is turned on in case of errors, also it blinks for 0.1s after
USB subsystem transition to CONFIGURED state. If it blinks single time on connect
it means the device works properly. If it is in enabled state permanently
the device is likely in HardFault state.


Interrupts
----------

Currently there is no support for interrupts. All interrupts except USB are
masked. Use polling of either NVIC or peripheral register to wait events.


GPIO control example
--------------------

Warning! No sanity or fool-proof checkings are performed on the MCU side. Be 
careful with addresses and values! Using of incorrect accesses will result in 
Hard Fault exception and device operation become impossible until it is 
reconnected. 
Example program controlling PA_1 output on Bluepill using Bash:

    #!/bin/bash
    echo 'w 40010800 44444464' > /dev/ttyACM0
    counter=1
    until [ $counter -gt 10 ]
    do
        echo 'w 40010810 2' > /dev/ttyACM0
        sleep 1
        echo 'w 40010810 20000' > /dev/ttyACM0
        sleep 1
        ((counter++))
    done


License
-------

ST's USB stack uses their own non-opensource license (see headers) and remains property 
of ST. All other code except ST libraries is public domain.


USB stack modifications
-----------------------

Original USB stack is modified significantly to make it more simple and flexible.
Major changes are listed below:

- Descriptors are now static and reside in ROM.
- Device and class objects are allocated at user side, not in the deep of the library.
- Static interface between the stack and the driver (usbd_drv.h header).
- Error codes from errno.h, no ugly functions needed just for status translation from one layer to another.
- Static strings in ROM.
- Alignment from C11 standard (alignas/alignof).
- Wrappers rewritten as static inline functions so they're removed at compile time.
