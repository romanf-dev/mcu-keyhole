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

On Linux use the Makefile. It is expected that arm-none-eabi- toolchain is 
available via PATH. On Windows use .bat file.


Port settings
-------------

Currently default settings from ST's USB stack are used: __9600/8-N-1__.


Protocol description
--------------------

There are 6 command line keys: __a v t b w d__.

Note that device does not output echos so don't expect usual command line
interface using either Putty or similar tools, it is intended to be used by 
programs, not humans!
However, if you prefer to use terminal program you may enable local echos.

| Key | Description         |
|-----|---------------------|
| a   | MCU memory address  |
| v   | Value for writes    |
| t   | Device type & FW ver|

Addresses and values are always lowercase hexadecimal strings.
The latter three keys are required to override default memory access width.

| Key | Description         |
|-----|---------------------|
| b   | Byte (8 bits)       |
| w   | Word (16 bits)      |
| d   | Doubleword (32 bits)|

Width override keys are optional. By default memory access width is equal to
width of "unsigned int" type for the target architecture.

And yet another couple of facts about the protocol:

- Request string must be terminated by CR, LF or CRLF.
- All device responses contain CRLF at end of the string.
- Integers wider than specified width are truncated. 
- Integers wider than 32-bit are considered as error. 
- Writes don't output anything.
- When incorrect string is received the device responds with word 'ERROR'.

Examples:

Read value at address 0x11223344:

    a 11223344

Write value 0x55667788 to address 0x11223344:

    a 11223344 v 55667788

Write byte to the same address:

    a 11223344 v 55 b

Single string may contain more than one request, for example:

    'a 11223344\na 22334455'

Device will perform two accesses. 
This approach may be helpful when there is need to perform two actions as
fast as possible.


Platform support
----------------

Linux and Windows 10 require no additional drivers, just plug and play.
Windows 7/8.1 contain usbser.sys driver but it does not install automatically.
Try install .inf file in the project's folder, it will attach existing driver
to device's VID/PID.


Troubleshooting
---------------

On some Linux hosts after the device connected the host tries to talk to the 
CDC interface so device send ERROR permanently. Attempts to 'cat' the device
results in 'device busy' response.
Try wait few minutes or try to use root console.

Onboard LED is turned on in case of errors, also it blinks for 0.1s after
USB subsystem is completely initialized. If it blinks single time on connect
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
    echo 'a 40010800 v 44444464' > /dev/ttyACM0
    counter=1
    until [ $counter -gt 10 ]
    do
        echo 'a 40010810 v 2' > /dev/ttyACM0
        sleep 1
        echo 'a 40010810 v 20000' > /dev/ttyACM0
        sleep 1
        ((counter++))
    done


License
-------

ST's USB stack uses their own non-opensource license (see headers) and remains property 
of ST. All other code except ST libraries is public domain.
