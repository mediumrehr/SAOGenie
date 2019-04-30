# SAOGenie

The SAO Genie was inspired by the [TPM Genie](https://github.com/nccgroup/TPMGenie), specially crafted for attacking [Shitty Add-ons](https://hackaday.com/2018/06/21/this-is-the-year-conference-badges-get-their-own-badges/) at DEF CON 26. The SAO Genie was designed as a tool for electronic badge and add-on hacking, since it provides an easier interface for sniffing and manipulating data on the serial bus between a badge and targeted add-on. As electronic badges are being used more frequently in conference hacking challenges, the SAO Genie is geared to make it easier to dig into inter-badge/add-on communications.

The SAO Genie works by sitting between an electronic badge and a targeted add-on as a man-in-the-middle kind of way. The SAO Genie is able to do this by acting as a slave to the conference badge, pretending to be the targeted add-on (using the same i2c address), while also playing the role of master to the targeted add-on (forwarding and possibly manipulating) serial data packets between the badge and target add-on. The SAO Genie has both a 2x2 male header for plugging into a conference badge, and a 2x2 female header for providing a connector for the targeted add-on to connect to.

## Functionality

Currently, only monitor/passthrough mode is supported.

#### Monitor/Passthrough Mode

When placed between an electronic badge and targeted add-on, the SAO Genie will forward all communications between the type pieces of hardware, while also repeating any data out through the UART serial port.

Connect the SAO Genie as shown below:



To connect to the UART serial port, attach your serial reader to the RX, TX, and GND pins on the 'handle'.

Serial settings:

```
Baudrate: 9600
Data bits: 8
Parity: None
Stop bits: 1
```

## Installing/Updating Firmware

### Setup

Firmware can be loaded with [Atmel Studio 7.0](https://www.microchip.com/mplab/avr-support/atmel-studio-7), or install the packages below to use with your preferred text editor and compile with arm gcc.

#### arm gcc and gdb

MacOS

```
brew install armmbed/formulae/arm-none-eabi-gcc
```

#### jlink gdb server

Download install file from [segger.com](https://www.segger.com/products/debug-probes/j-link/tools/j-link-gdb-server/about-j-link-gdb-server/)

#### SWD connection

Software debug (SWD) pins are labelled on the back of the board. Connect SWDIO, SWCLK, 3V3, RESET, and GND as labelled. SWD requires the board to be powered external from the SWD port, so it's easiest to power the board from the 3V3 and GND pins on one of the SAO headers.

### Build

```
cd firmware/SAOGenie/
make -C Build/
```

### Load

#### jlink

Start jlink gdb server in one tab

```
JLinkGDBServer -if SWD -device ATSAMD21G18
```

Start gdb in another tba

```
arm-none-eabi-gdb Build/SAOGenie.elf
(gdb) target extended-remote :2331
```

Load new firmware, reset microcontroller, and run new code

```
(gdb) load
(gdb) monitor reset
(gdb) c
```
