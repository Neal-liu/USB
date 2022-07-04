

usb-serial
============

The main purpose in this program is to communication between two serial ports.

Example:
- PC1 - usb interface - SoC (ttyGS0 - ttyS2) - rs232 interface - PC2

Input parameters:
- input: argv[1]: ttyGS0 (UDC gadgets for USB cdc-acm)
- input: argv[2]: ttyS2 (uart port)

Cross compile:
- $ arm-aspeed-linux-gnueabihf-gcc usb_uart.c -o usb-uart

Run on AST2600:
- $ ./usb-uart /dev/ttyGS0 /dev/ttyS2

Then you can key anything either on PC1 or PC2.
