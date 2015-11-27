AVeRCADE Bootloader Instructions

1: Select correct firmware for your processor and crystal frequency.

2: Burn the firmware (see notes)

3: Burn the fuses:
   
   a. ATMEGA8      Low Fuse = 0x9F   High Fuse = 0xC0  
   b. ATMEGA328p   Low Fuse = 0xF7   High Fuse = 0xDA  Ext Fuse = 0x03

Notes: 
The bootloader shall be burn in the microcontroller before soldering on the board.
Alternatively it is possible to solder a header on the board for programming with USBAsp or another tool of your choice. The connections are:

     ISP Signal   AVeRCADE Pin
	SCK           B3
	MISO          B4
	MOSI          B5 
	GND           GND
	RST        boot jumper
	+Vcc       Anode ot LED


If you program the fuses incorrectly the microcontroller may stop responding to ISP commands and shall be erased by HSP programming or another tool like Fuse Doctor.

