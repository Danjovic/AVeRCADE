/*
 *     ___   __   ___  ___   _   ___  ___ 
 *    /_\ \ / /__| _ \/ __| /_\ |   \| __|
 *   / _ \ V / -_)   / (__ / _ \| |) | _| 
 *  /_/ \_\_/\___|_|_\\___/_/ \_\___/|___|
 *   
 *  Customisable USB adapter for arcade controls.
 *  
 *  Author: Daniel Jose Viana - danjovic@hotmail.com
 *  
 *  Version 0.9 - 10 October 2015
 *
 *  This code is licensed under GPL V2.0
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

// V-USB
#include "usbconfig.h"
#include "usbdrv/usbdrv.h"

// Arcade Control - Directional plus 14 buttons
const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x01,                    //     REPORT_ID (1)	
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x0e,                    //     USAGE_MAXIMUM (Button 14)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x0e,                    //     REPORT_COUNT (14)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
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
    0xc0                           // END_COLLECTION
};


// Definitions


		#define A_UP    5
		#define A_RIGHT 4		
		#define A_DOWN  3
		#define A_LEFT  2

		
		#define A_B1   1
		#define A_B2   0
		#define A_B3   5	
		#define A_B4   4
		#define A_B5   3		
		#define A_B6   2
		#define A_B7   1		
		#define A_B8   0
		#define A_B9   7					
		#define A_B10  6
		#define A_B11  5
		#define A_B12  4					
		#define A_B13  1
		#define A_B14  0


//typedef struct
//{
///	uint8_t report_id;
//	uint8_t modifier;
//	uint8_t reserved;
//	uint8_t keycodes[6];
//} keyboard_report_t;

//typedef struct
//{
//	uint8_t report_id;
//	uint8_t buttons;
//	int8_t x;
//	int8_t y;
//	int8_t wheel;
//} mouse_report_t;

typedef struct
{
	uint8_t report_id;
	uint8_t buttons8_1;
	uint8_t buttons14_9;
	uint8_t axes;
} gamepad_report_t;

static uint8_t idle_rate = 500 / 4; // see HID1_11.pdf sect 7.2.4
static uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6

//static keyboard_report_t keyboard_report;
//static mouse_report_t mouse_report;
static gamepad_report_t gamepad_report_1;
//static gamepad_report_t gamepad_report_2;
//static keyboard_report_t keyboard_report_old;
//static mouse_report_t mouse_report_old;
static gamepad_report_t gamepad_report_1_old;
//static gamepad_report_t gamepad_report_2_old;

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
//			if (rq->wValue.bytes[0] == 1)
//			{
//				usbMsgPtr = &keyboard_report;
//				return sizeof(keyboard_report);
//			}
//			else if (rq->wValue.bytes[0] == 2)
//			{
//				usbMsgPtr = &mouse_report;
//				return sizeof(mouse_report);
//			}
//			else 
			if (rq->wValue.bytes[0] == 1) //3)
			{
				usbMsgPtr = &gamepad_report_1;
				return sizeof(gamepad_report_1);
			}
//			else if (rq->wValue.bytes[0] == 4)
//			{
//				usbMsgPtr = &gamepad_report_2;
//				return sizeof(gamepad_report_2);
//			}
			else
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

int main()
{
	uint8_t inp_b;
	uint8_t inp_c;
	uint8_t inp_d;
	
	wdt_disable(); // no watchdog, just because I'm lazy
    
	// Configure I/O PORTS - All Digital Inputs (ARCADE)
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	// Configure Pullups except for Pins PD2 and PD3
	PORTB = 0xff;
	PORTC = 0xff;
	PORTD = 0xf3;      // 1 1 1 1 0 0 1 1
	 
	// Configure timer 	
	TCCR1B = _BV(CS12) | _BV(CS11); // timer is initialized, used to keep track of idle period
	
	// Start the show!
	usbInit(); // start v-usb
    usbDeviceDisconnect(); // enforce USB re-enumeration, do this while interrupts are disabled!
	_delay_ms(250);
    usbDeviceConnect();
	
    sei(); // enable interrupts
	
	uint8_t to_send = 1; // boolean, true for first time
	
	while (1)
	{
		usbPoll();
		
		// set the report IDs manually
//		keyboard_report.report_id = 1;
//		mouse_report.report_id = 2;
		gamepad_report_1.report_id = 1;//3;
//		gamepad_report_2.report_id = 4;
		
		/*
		 * this area is where you should set the movement
		 * and button values of the reports using the input
		 * method of your choice
		 *
		*/
		
		// Initialize report. No buttons pressed, directional at center
		gamepad_report_1.buttons8_1=0;
		gamepad_report_1.buttons14_9=0;
		gamepad_report_1.axes=0x5;       // - - - - 0 1 0 1 (X and Y centered);
		
		inp_b = PINB;
		inp_c = PINC;
		inp_d = PIND;		
		
		// Populate directionals
		if ((inp_c & (1<<A_UP   ))==0) gamepad_report_1.axes -=4;
		if ((inp_c & (1<<A_DOWN ))==0) gamepad_report_1.axes +=4;		
		if ((inp_c & (1<<A_LEFT ))==0) gamepad_report_1.axes -=1;
		if ((inp_c & (1<<A_RIGHT))==0) gamepad_report_1.axes +=1;

		// Populate buttons 1-8
		if ((inp_c & (1<<A_B1   ))==0) gamepad_report_1.buttons8_1	+= 1;	
		if ((inp_c & (1<<A_B2   ))==0) gamepad_report_1.buttons8_1	+= 2;		
		if ((inp_b & (1<<A_B3   ))==0) gamepad_report_1.buttons8_1	+= 4;	
		if ((inp_b & (1<<A_B4   ))==0) gamepad_report_1.buttons8_1	+= 8;		
		if ((inp_b & (1<<A_B5   ))==0) gamepad_report_1.buttons8_1	+= 16;	
		if ((inp_b & (1<<A_B6   ))==0) gamepad_report_1.buttons8_1	+= 32;		
		if ((inp_b & (1<<A_B7   ))==0) gamepad_report_1.buttons8_1	+= 64;	
		if ((inp_b & (1<<A_B8   ))==0) gamepad_report_1.buttons8_1	+= 128;				
		
		// Populate buttons 9-14
		if ((inp_d & (1<<A_B9   ))==0) gamepad_report_1.buttons14_9	+= 1;	
		if ((inp_d & (1<<A_B10  ))==0) gamepad_report_1.buttons14_9	+= 2;		
		if ((inp_d & (1<<A_B11  ))==0) gamepad_report_1.buttons14_9	+= 4;	
		if ((inp_d & (1<<A_B12  ))==0) gamepad_report_1.buttons14_9	+= 8;		
		if ((inp_d & (1<<A_B13  ))==0) gamepad_report_1.buttons14_9	+= 16;	
		if ((inp_d & (1<<A_B14  ))==0) gamepad_report_1.buttons14_9	+= 32;  
		
		
		// determine whether or not the report should be sent
		if ((TCNT1 > ((4 * (F_CPU / 1024000)) * idle_rate) || TCNT1 > 0x7FFF) && idle_rate != 0)
		{// using idle rate
			to_send = 1;
		}
		else
		{// or if data has changed
//			if (memcmp(&keyboard_report, &keyboard_report_old, sizeof(keyboard_report_t)) != 0)
//			{
//				to_send = 1;
//			}
//			else if (memcmp(&mouse_report, &mouse_report_old, sizeof(mouse_report_t)) != 0)
//			{
//				to_send = 1;
//			}
//			else 
			if (memcmp(&gamepad_report_1, &gamepad_report_1_old, sizeof(gamepad_report_t)) != 0)
			{
				to_send = 1;
			}
//			else if (memcmp(&gamepad_report_2, &gamepad_report_2_old, sizeof(gamepad_report_t)) != 0)
//			{
//				to_send = 1;
//			}
		}
		
		usbPoll();
		if (to_send != 0)
		{
			// send the data if needed
//			usbSendHidReport(&keyboard_report, sizeof(keyboard_report_t));
//			usbSendHidReport(&mouse_report, sizeof(mouse_report_t));
			usbSendHidReport(&gamepad_report_1, sizeof(gamepad_report_t));
//			usbSendHidReport(&gamepad_report_2, sizeof(gamepad_report_t));
			TCNT1 = 0; // reset timer
		}
		
		usbPoll();
		
		memcpy(&gamepad_report_1_old, &gamepad_report_1, sizeof(gamepad_report_t));
//		memcpy(&gamepad_report_2_old, &gamepad_report_2, sizeof(gamepad_report_t));
//		memcpy(&mouse_report_old, &mouse_report, sizeof(mouse_report_t));
//		memcpy(&keyboard_report_old, &keyboard_report, sizeof(keyboard_report_t));
		
		to_send = 0; // reset flag
	}
	
	return 0;
}
