A fork of the original Arduino Renesas bootloader for UNO R4 and Portenta C33 that disables the blinking of the LED on pin 13 (pin P111).

⚙️ `Compilation`
====================================
```bash
git clone https://github.com/Matt5377/arduino-renesas-bootloader-no-led-blink
git clone https://github.com/hathach/tinyusb
cd tinyusb
patch -p1 < ../arduino-renesas-bootloader-no-led-blink/0001-fix-arduino-bootloaders.patch
python3 tools/get_deps.py ra
cd ..
cd arduino-renesas-bootloader-no-led-blink

#For UNO R4 Minima
TINYUSB_ROOT=$PWD/../tinyusb make -f Makefile.minima 

#For UNO R4 WiFi
TINYUSB_ROOT=$PWD/../tinyusb make -f Makefile.wifi

#For Portenta C33
TINYUSB_ROOT=$PWD/../tinyusb make -f Makefile.c33
```
