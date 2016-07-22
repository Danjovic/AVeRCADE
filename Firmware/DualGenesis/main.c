/*
 *     ___   __   ___  ___   _   ___  ___ 
 *    /_\ \ / /__| _ \/ __| /_\ |   \| __|
 *   / _ \ V / -_)   / (__ / _ \| |) | _| 
 *  /_/ \_\_/\___|_|_\\___/_/ \_\___/|___|
 *   
 *  Customisable USB adapter for arcade controls.
 *
 *  //////     Dual GENESIS     \\\\\\
 *  
 *  Author: Daniel Jose Viana - danjovic@hotmail.com
 *  
 *  Version 0.9 - 22 July 2016
 */ 


// Based on HID Report Descriptors Tutorial and code by Frank Zhao
// http://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/

// required headers
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/sfr_defs.h>
// V-USB
#include "usbconfig.h"
#include "usbdrv.h"


// SNES protoco, Genesis 6 button protocol and pin mapping on the remarks
// at the end of the file

// Definitions
#define A1 bit_is_clear(PINC,2) 
#define A2 bit_is_clear(PINC,3)
#define A3 bit_is_clear(PINC,4)
#define A4 bit_is_clear(PINC,5)
#define A5 bit_is_clear(PIND,0)
#define A6 bit_is_clear(PIND,1)
#define A7 bit_is_clear(PIND,4)
#define A8 bit_is_clear(PIND,5)
#define A9 bit_is_clear(PIND,6)
           
#define B1 bit_is_clear(PINC,1)
#define B2 bit_is_clear(PINC,0)
#define B3 bit_is_clear(PINB,5)
#define B4 bit_is_clear(PINB,4)
#define B5 bit_is_clear(PINB,3)
#define B6 bit_is_clear(PINB,2)
#define B7 bit_is_clear(PINB,1)
#define B8 bit_is_clear(PINB,0)
#define B9 bit_is_clear(PIND,7)

// Macros for Genesis controllers
#define GENESIS_1_SELECT_LOW()  PORTC &= ~(1<<PC5)
#define GENESIS_1_SELECT_HIGH() PORTC |=  (1<<PC5)
#define GENESIS_2_SELECT_LOW()  PORTB &= ~(1<<PB4)
#define GENESIS_2_SELECT_HIGH() PORTB |=  (1<<PB4)

#define TBASE  50 

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { // Size of report: 100 bytes
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x01,                    //     REPORT_ID (1)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x02,                    //     LOGICAL_MAXIMUM (2)
    0x75, 0x02,                    //     REPORT_SIZE (2)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x04,                    //     REPORT_COUNT (4)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0,                           // END_COLLECTION
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x02,                    //     REPORT_ID (2)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x02,                    //     LOGICAL_MAXIMUM (2)
    0x75, 0x02,                    //     REPORT_SIZE (2)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x04,                    //     REPORT_COUNT (4)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0,                           // END_COLLECTION	
};


typedef struct
{
	uint8_t report_id;
	uint8_t buttons;
	uint8_t axes;
} gamepad_report_t;


static gamepad_report_t genesis_1_report;
static gamepad_report_t genesis_2_report;

static gamepad_report_t genesis_1_report_old;
static gamepad_report_t genesis_2_report_old;

static uint8_t idle_rate = 500 / 4; // see HID1_11.pdf sect 7.2.4
static uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6


usbMsgLen_t usbFunctionSetup(uint8_t data[8])
{
	// see HID1_11.pdf sect 7.2 and http://vusb.wikidot.com/driver-api
	usbRequest_t *rq = (void *)data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_CLASS)
		return 0; // ignore request if it's not a class specific request

	// see HID1_11.pdf sect 7.2
	switch (rq->bRequest)
	{
		case USBRQ_HID_GET_IDLE:
			usbMsgPtr = &idle_rate; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_IDLE:
			idle_rate = rq->wValue.bytes[1]; // read in idle rate
			return 0; // send nothing
		case USBRQ_HID_GET_PROTOCOL:
			usbMsgPtr = &protocol_version; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_PROTOCOL:
			protocol_version = rq->wValue.bytes[1];
			return 0; // send nothing
		case USBRQ_HID_GET_REPORT:
			// check for report ID then send back report
			if (rq->wValue.bytes[0] == 1)
			{
				usbMsgPtr = &genesis_1_report;
				return sizeof(genesis_1_report);
			}
			else if (rq->wValue.bytes[0] == 2)
			{
				usbMsgPtr = &genesis_2_report;
				return sizeof(genesis_2_report);
			} else
			{
				return 0; // no such report, send nothing
			}
		case USBRQ_HID_SET_REPORT: // no "output" or "feature" implemented, so ignore
			return 0; // send nothing
		default: // do not understand data, ignore
			return 0; // send nothing
	}
}

// this function is used to guarantee that the data is sent to the computer once
void usbSendHidReport(uchar * data, uchar len)
{
	while(1)
	{
		usbPoll();
		if (usbInterruptIsReady())
		{
			usbSetInterrupt(data, len);
			break;
		}
	}
}


static inline void Get_genesis_controllers_data(){

	// Initialize Axes (centered) and buttons (none pressed)
	genesis_1_report.buttons=0;
	genesis_1_report.axes=0x5;       // - - - - 0 1 0 1 (X and Y centered);

	genesis_2_report.buttons=0;
	genesis_2_report.axes=0x5;       // - - - - 0 1 0 1 (X and Y centered);


	// Read directionals and buttons B,C with SELECT in HIGH (idle state)
	if ( A5 ) genesis_1_report.buttons |= (1<<1);  // B
	if ( B5 ) genesis_2_report.buttons |= (1<<1); 
	
	if ( A3 ) genesis_1_report.buttons |= (1<<2);  // C
	if ( B3 ) genesis_2_report.buttons |= (1<<2); 

	if ( A9 ) genesis_1_report.axes-=4;  // Up
	if ( B9 ) genesis_2_report.axes-=4; 	

	if ( A8 ) genesis_1_report.axes+=4;	// Down
	if ( B8 ) genesis_2_report.axes+=4;

	if ( A7 ) genesis_1_report.axes-=1;	// Left
	if ( B7 ) genesis_2_report.axes-=1;	

	if ( A6 ) genesis_1_report.axes+=1;	// Right
	if ( B6 ) genesis_2_report.axes+=1;


	GENESIS_1_SELECT_LOW();   // Pulse 1
	GENESIS_2_SELECT_LOW(); 
	_delay_us(100);
	
	
	// Read A and Start with Select in LOW
	if ( A5 ) genesis_1_report.buttons |= (1<<0);  // A
	if ( B5 ) genesis_2_report.buttons |= (1<<0); 	
	
	if ( A3 ) genesis_1_report.buttons |= (1<<7);  // Start
	if ( B3 ) genesis_2_report.buttons |= (1<<7);
	
    GENESIS_1_SELECT_HIGH();
    GENESIS_2_SELECT_HIGH();
	_delay_us(100);
	
	GENESIS_1_SELECT_LOW();   // Pulse 2
	GENESIS_2_SELECT_LOW(); 
	_delay_us(100);
    GENESIS_1_SELECT_HIGH();
    GENESIS_2_SELECT_HIGH();
	_delay_us(100);
	
	GENESIS_1_SELECT_LOW();   // Pulse 3
	GENESIS_2_SELECT_LOW(); 
	_delay_us(100);
    GENESIS_1_SELECT_HIGH();   // Pulse 3, rising edge XYZ available
    GENESIS_2_SELECT_HIGH();
	_delay_us(100);	

	
	if ( A9 ) genesis_1_report.buttons |= (1<<5);  // X
	if ( B9 ) genesis_2_report.buttons |= (1<<5); 

	if ( A8 ) genesis_1_report.buttons |= (1<<4);  // Y
	if ( B8 ) genesis_2_report.buttons |= (1<<4); 

	if ( A7 ) genesis_1_report.buttons |= (1<<3);  // Z
	if ( B7 ) genesis_2_report.buttons |= (1<<3); 

	if ( A6 ) genesis_1_report.buttons |= (1<<6);  // Mode
	if ( B6 ) genesis_2_report.buttons |= (1<<6); 



	GENESIS_1_SELECT_LOW();   // Pulse 4
	GENESIS_2_SELECT_LOW(); 
	_delay_us(100);
    GENESIS_1_SELECT_HIGH();
    GENESIS_2_SELECT_HIGH();
//	_delay_us(500);
	

}


int main()
{
	
	wdt_disable(); // no watchdog, just because I'm lazy
    
	// Configure I/O PORTS - All Digital Inputs (ARCADE)
	DDRB = (1<<4);  // PB4 as output
	DDRC = (1<<5); // PC5 as output
	DDRD = 0;
	// Configure Pullups except for Pins PD2 and PD3
	PORTB = 0xff;
	PORTC = 0xff;
	PORTD = 0xf3;      // 1 1 1 1 0 0 1 1
	 
	// Configure timer 	
	TCCR1B = _BV(CS12) | _BV(CS11); // timer is initialized, used to keep track of idle period
	
	TCCR0B |= (1 << CS00) | (1<<CS02); // timer 0 prescaler 1024, overflow at FOSC/1024/256
	
	
	// Start the show!
	usbInit(); // start v-usb
    usbDeviceDisconnect(); // enforce USB re-enumeration, do this while interrupts are disabled!
	_delay_ms(250);
    usbDeviceConnect();
	
    sei(); // enable interrupts
	
	uint8_t to_send = 1; // boolean, true for first time
	

	
	// Initialize the report IDs 
	genesis_1_report.report_id = 1;
	genesis_2_report.report_id = 2;
	
	// Initialize report. No buttons pressed, directional at center		
	genesis_1_report.buttons=0;
	genesis_1_report.axes=0x5;       // - - - - 0 1 0 1 (X and Y centered);
	genesis_2_report.buttons=0;		
	genesis_2_report.axes=0x5;       // - - - - 0 1 0 1 (X and Y centered);

	
	
	
	while (1)
	{
		usbPoll();
			
		// Sample controllers each 16ms for 16MHz crystal (22ms for 12MHz)
		if (TIFR0 & (1<<TOV0)) {
					Get_genesis_controllers_data();
					TIFR0 |= (1<<TOV0); // reset overflow flag		
		}
 

		
		
		// determine whether or not the report should be sent
		if ((TCNT1 > ((4 * (F_CPU / 1024000)) * idle_rate) || TCNT1 > 0x7FFF) && idle_rate != 0)
		{// using idle rate

			to_send = 1;
		}
		else
		{// or if data has changed			
			if (memcmp(&genesis_1_report, &genesis_1_report_old, sizeof(genesis_1_report)) != 0)
			{
				to_send = 1;
			}
			if (memcmp(&genesis_2_report, &genesis_2_report_old, sizeof(genesis_2_report)) != 0)
			{
				to_send = 1;
			}
		}
		
		usbPoll();
		if (to_send != 0)
		{
			// send the data if needed
			usbSendHidReport(&genesis_1_report, sizeof(genesis_1_report));
			usbSendHidReport(&genesis_2_report, sizeof(genesis_2_report));			
			TCNT1 = 0; // reset timer
		}
		
		usbPoll();	
		memcpy(&genesis_1_report_old, &genesis_1_report, sizeof(genesis_1_report));
		memcpy(&genesis_2_report_old, &genesis_2_report, sizeof(genesis_2_report));			
		to_send = 0; // reset flag
	}
	
	return 0;
}





/*  Pinout, Protocols

AVeRCADE pin mapping

NAME	AVR_PORT	FUNCTION		
A1		   PC2		
A2		   PC3		
A3		   PC4		GEN1_START_C
A4		   PC5		GEN1_SELECT      (output)
A5		   PD0		GEN1_A_B
A6		   PD1		GEN1_RIGHT_0
A7		   PD4		GEN1_LEFT_0_X
A8		   PD5		GEN1_DOWN_DOWN_Y
A9		   PD6		GEN1_UP_UP_Z
	
B1		   PC1		
B2		   PC0		
B3		   PB5		GEN2_START_C
B4		   PB4		GEN2_SELECT      (output)
B5		   PB3		GEN2_A_B
B6		   PB2		GEN2_RIGHT_0
B7		   PB1		GEN2_LEFT_0_X
B8		   PB0		GEN2_DOWN_DOWN_Y
B9		   PD7		GEN2_UP_UP_Z


#  Sega Genesis 6 Buttton Protocol
 
                        Buttons XYZ | available after the 3rd rising edge
           ____________    __    __ V  __    ____________  
  Select               \__/  \__/  \__/  \__/           
  
 
Pin  Select: low  Select: high  Select: pulse-3
--- ------------  ------------  ---------------
 1  joypad up     joypad up     button Z
 2  joypad down   joypad down   button Y
 3  logic low     joypad left   button X
 4  logic low     joypad right
 6  button A      button B
 9  start button  button C
 

DB9      ------SELECT------ 
Pin       LOW   HIGH   3rd 
---      ----- ------ -----
 1        up    up     Z
 2       down  down    Y
 3        0    left    X
 4        0    right
 6        A     B
 9       Start  C
 
 	

// USB Joystick Mapping
           GENESIS
Button 1      A
Button 2      B 
Button 3      C
Button 4      X
Button 5      Y
Button 6      Z
Button 7     Mode  
Button 8    Start


 
 */



 