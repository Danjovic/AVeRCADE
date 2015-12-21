/*
 *     ___   __   ___  ___   _   ___  ___ 
 *    /_\ \ / /__| _ \/ __| /_\ |   \| __|
 *   / _ \ V / -_)   / (__ / _ \| |) | _| 
 *  /_/ \_\_/\___|_|_\\___/_/ \_\___/|___|
 *   
 *  Customisable USB adapter for arcade controls.
 *
 *  //////     DUAL ATARI Firmware     \\\\\\
 *  
 *  Author: Daniel Jose Viana - danjovic@hotmail.com
 *  
 *  Version 0.9 -  20 October 2015
 *          0.93 - 20 December 2015
 *
 *  This code is licensed under GPL V2.0
 */ 


// Based on HID Report Descriptors Tutorial and code by Frank Zhao
// http://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/

// required headers
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/sfr_defs.h>
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
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x02,                    //     LOGICAL_MAXIMUM (2)
    0x75, 0x02,                    //     REPORT_SIZE (2)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)	
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x01,                    //     USAGE_MAXIMUM (Button 1)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x33,                    //     USAGE (Rx)
    0x09, 0x34,                    //     USAGE (Ry)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,               //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)	
    0xc0,                          //   END_COLLECTION
    0xc0,                           // END_COLLECTION
	
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x02,                    //     REPORT_ID (2)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x02,                    //     LOGICAL_MAXIMUM (2)
    0x75, 0x02,                    //     REPORT_SIZE (2)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)	
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x01,                    //     USAGE_MAXIMUM (Button 1)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x33,                    //     USAGE (Rx)
    0x09, 0x34,                    //     USAGE (Ry)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,               //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)	
    0xc0,                          //   END_COLLECTION
    0xc0,                          // END_COLLECTION
};


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



#define AB_PIN  PINC

#define A_PORT PORTC
#define A_DDR  DDRC
#define A_Power 5
#define A_Xaxis 3  
#define A_Yaxis 2  

#define B_PORT PORTC
#define B_DDR  DDRC
#define B_Power 4
#define B_Xaxis 0   
#define B_Yaxis 1   



#ifndef TCCR0B          /* compatibility between ATMega8 and ATMega88 */
#   define TCCR0B   TCCR0
#endif

#ifndef TIFR0          
#   define TIFR0   TIFR
#endif


#define mustPollControllers()	(TIFR0 & (1<<TOV0))
#define clrPollControllers()	do { TIFR0 = (1<<TOV0); } while(0)



typedef struct
{
	uint8_t report_id;
	uint8_t XY_Button1;
	int8_t RXaxis;
	int8_t RYaxis;
} gamepad_report_t;

static uint8_t idle_rate = 500 / 4; // see HID1_11.pdf sect 7.2.4
static uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6


static gamepad_report_t gamepad_report_1;
static gamepad_report_t gamepad_report_2;

static gamepad_report_t gamepad_report_1_old;
static gamepad_report_t gamepad_report_2_old;


// Sample Paddles
void do_a_new_sample(void) {
	uint8_t sample;
	
    // clear timer variable
	uint8_t timer=255;				
	
	// reset external capacitors
	A_PORT &= ~( (1<<A_Xaxis) | (1<<A_Yaxis) );	// short circuit capacitors
	B_PORT &= ~( (1<<B_Xaxis) | (1<<B_Yaxis) );	// 
	A_DDR |= ( (1<<A_Xaxis) | (1<<A_Yaxis) );	// 
	B_DDR |= ( (1<<B_Xaxis) | (1<<B_Yaxis) );	//	
		_delay_us(30);
 	// Release capacitors to charge   
	A_DDR &= ~( (1<<A_Xaxis) | (1<<A_Yaxis) );
	B_DDR &= ~( (1<<B_Xaxis) | (1<<B_Yaxis) );
	_delay_us(10);
	
	// now measure time it takes for each input to flip to HIGH again
	do {
		sample = AB_PIN;
		_delay_us(3);		 											// While capacitor hasn't charged the voltage is low
		if ((sample & (1<<A_Yaxis)) ==0) gamepad_report_1.RYaxis=timer;	// which is the same as the button being pressed 
		if ((sample & (1<<A_Xaxis)) ==0) gamepad_report_1.RXaxis=timer;	// and the 'timer' variable value is copied to the 
																		// axis value.
		if ((sample & (1<<B_Yaxis)) ==0) gamepad_report_2.RYaxis=timer;	// When the capacitor charges it is the same as the 
		if ((sample & (1<<B_Xaxis)) ==0) gamepad_report_2.RXaxis=timer;	// button being released and the 'timer' value will 
	} while (--timer);													// not be copied to the axis value, remaining at the
																		// former 'timer' value 
}




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
				usbMsgPtr = &gamepad_report_1;
				return sizeof(gamepad_report_1);
			}
			else if (rq->wValue.bytes[0] == 2)
			{
				usbMsgPtr = &gamepad_report_2;
				return sizeof(gamepad_report_2);
			}
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
 
	wdt_disable(); // no watchdog, just because I'm lazy
    
	// Configure I/O PORTS - All Digital Inputs (ARCADE)
	DDRB = 0;
	DDRC = 0; 
	DDRD = 0;
	// Configure Pullups except for Pins PD2 and PD3
	PORTB = 0xff;
	PORTC = 0xff;
	PORTD = 0xf3;      // 1 1 1 1 0 0 1 1
	
	// Turn on Power for Paddles
	A_DDR |= (1<<A_Power);
	A_PORT |= (1<<A_Power);
	
	B_DDR |= (1<<B_Power);
	B_PORT |= (1<<B_Power);
	

	// Inicializa timer 0 - prescaler 1024, Clear em overflow -> 61,03Hz
	TCCR0B = ( (1<<CS02) |  (1<<CS00) ); 

	 
	// Configure timer 	
	TCCR1B = _BV(CS12) | _BV(CS11); // timer is initialized, used to keep track of idle period
	
	
	
	// Start the show!
	usbInit(); // start v-usb
    usbDeviceDisconnect(); // enforce USB re-enumeration, do this while interrupts are disabled!
	_delay_ms(250);
    usbDeviceConnect();
	
    sei(); // enable interrupts
	
	uint8_t to_send = 1; // boolean, true for first time


		gamepad_report_1.RXaxis=0;
		gamepad_report_1.RYaxis=0;		
		gamepad_report_2.RXaxis=0;
		gamepad_report_2.RYaxis=0;


	
	while (1)
	{
		usbPoll();
		
		// Initialize the report IDs 
		gamepad_report_1.report_id = 1;
		gamepad_report_2.report_id = 2;
		
		
		// Initialize report. No buttons pressed, directional at center
		gamepad_report_1.XY_Button1=5;
		gamepad_report_2.XY_Button1=5;	

		// Populate X/Y axes and button for - Controller A		
		if ( A5 ) gamepad_report_1.XY_Button1	+= 1;	// Right
		if ( A6 ) gamepad_report_1.XY_Button1	-= 1;   // Left		
		if ( A7 ) gamepad_report_1.XY_Button1	+= 4;	// Down
		if ( A8 ) gamepad_report_1.XY_Button1	-= 4;	// Up
		if ( A9 ) gamepad_report_1.XY_Button1	+= 16;	// Button

		// Populate X/Y axes and button for - Controller B		
		if ( B5 ) gamepad_report_2.XY_Button1	+= 1;	// Right
		if ( B6 ) gamepad_report_2.XY_Button1	-= 1;   // Left		
		if ( B7 ) gamepad_report_2.XY_Button1	+= 4;	// Down
		if ( B8 ) gamepad_report_2.XY_Button1	-= 4;	// Up
		if ( B9 ) gamepad_report_2.XY_Button1	+= 16;	// Button
		
		
		
		// Paddles are scanned at a rate about 61Hz when a Timer0 overflow occurs
		if (mustPollControllers())  {   // Check if its time for a new sample
		    clrPollControllers();       // clear overflow flag
			sleep_enable();	            // Prepare CPU to sleep in order to synchronize 
			sleep_cpu();                // the start of the timing with the end of USB
			sleep_disable();            // interrupt and sleep until it ends
										
										// Resume here after interrupt
			_delay_us(100);             // wait 100us to be sure that no other interrupt will occur
			do_a_new_sample();          // Then perform a new sample
		} // if (mustPollControllers)
				
		
		// determine whether or not the report should be sent
		if ((TCNT1 > ((4 * (F_CPU / 1024000)) * idle_rate) || TCNT1 > 0x7FFF) && idle_rate != 0)
		{// using idle rate
			to_send = 1;
		}
		else
		{// or if data has changed

			if (memcmp(&gamepad_report_1, &gamepad_report_1_old, sizeof(gamepad_report_t)) != 0)
			{
				to_send = 1;
			}
			else if (memcmp(&gamepad_report_2, &gamepad_report_2_old, sizeof(gamepad_report_t)) != 0)
			{
				to_send = 1;
			}
		}
		
		usbPoll();
		if (to_send != 0)
		{
			// send the data if needed
			usbSendHidReport(&gamepad_report_1, sizeof(gamepad_report_t));
			usbSendHidReport(&gamepad_report_2, sizeof(gamepad_report_t));
			TCNT1 = 0; // reset timer
		}
		
		usbPoll();
		
		memcpy(&gamepad_report_1_old, &gamepad_report_1, sizeof(gamepad_report_t));
		memcpy(&gamepad_report_2_old, &gamepad_report_2, sizeof(gamepad_report_t));
	
		to_send = 0; // reset flag
	}
	
	return 0;
}
