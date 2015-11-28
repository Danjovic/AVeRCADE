/*
 *     ___   __   ___  ___   _   ___  ___ 
 *    /_\ \ / /__| _ \/ __| /_\ |   \| __|
 *   / _ \ V / -_)   / (__ / _ \| |) | _| 
 *  /_/ \_\_/\___|_|_\\___/_/ \_\___/|___|
 *   
 *  Customisable USB adapter for arcade controls.
 *
 *  //////     DUAL ANALOG Firmware     \\\\\\
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
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x07,                    //     USAGE_MAXIMUM (Button 7)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x07,                    //     REPORT_COUNT (7)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
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
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x07,                    //     USAGE_MAXIMUM (Button 7)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x07,                    //     REPORT_COUNT (7)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
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

#define A_Xaxis 2  // ADC Channel 2
#define A_Yaxis 3  // ADC Channel 3

#define B_Xaxis 1  // ADC Channel 1
#define B_Yaxis 0  // ADC Channel 0


typedef struct
{
	uint8_t report_id;
	uint8_t buttons7_1;
	int8_t Xaxis;
	int8_t Yaxis;
} gamepad_report_t;

static uint8_t idle_rate = 500 / 4; // see HID1_11.pdf sect 7.2.4
static uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6


static gamepad_report_t gamepad_report_1;
static gamepad_report_t gamepad_report_2;

static gamepad_report_t gamepad_report_1_old;
static gamepad_report_t gamepad_report_2_old;

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
	
	// Configure ADC	
    ADMUX = (1<<REFS0 | 1<<ADLAR );  // AREF = AVcc, Left Justified

    // 12000000/128 = 93750Hz  
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  // ADC Enable and prescaler of 128
	
	 
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
		
		// Initialize the report IDs 
		gamepad_report_1.report_id = 1;
		gamepad_report_2.report_id = 2;
		
		
		// Initialize report. No buttons pressed, directional at center
		gamepad_report_1.buttons7_1=0;
		gamepad_report_1.Xaxis=0;
		gamepad_report_1.Yaxis=0;		

		gamepad_report_2.buttons7_1=0;
		gamepad_report_2.Xaxis=0;
		gamepad_report_2.Yaxis=0;
		
 		
		
		// Populate X and Y Axes  - Controller A
        ADMUX = (ADMUX & 0xF8) | A_Xaxis; // Select X axis 
		ADCSRA |= (1<<ADSC); // start single convertion
		while(ADCSRA & (1<<ADSC)); // wait for conversion to complete		
		gamepad_report_1.Xaxis = 127-ADCH;
		
        ADMUX = (ADMUX & 0xF8) | A_Yaxis; // Select Y axis 
		ADCSRA |= (1<<ADSC); // start single convertion
		while(ADCSRA & (1<<ADSC)); // wait for conversion to complete		
		gamepad_report_1.Yaxis = 127-ADCH;		

		// Populate buttons 1-7 - Controller A
		if ( A3 ) gamepad_report_1.buttons7_1	+= 1;	
		if ( A4 ) gamepad_report_1.buttons7_1	+= 2;		
		if ( A5 ) gamepad_report_1.buttons7_1	+= 4;	
		if ( A6 ) gamepad_report_1.buttons7_1	+= 8;		
		if ( A7 ) gamepad_report_1.buttons7_1	+= 16;	
		if ( A8 ) gamepad_report_1.buttons7_1	+= 32;	
		if ( A9 ) gamepad_report_1.buttons7_1	+= 64;	

		
		// Populate X and Y Axes  - Controller B
        ADMUX = (ADMUX & 0xF8) | B_Xaxis; // Select X axis 
		ADCSRA |= (1<<ADSC); // start single convertion
		while(ADCSRA & (1<<ADSC)); // wait for conversion to complete		
		gamepad_report_2.Xaxis = 127-ADCH;
		
        ADMUX = (ADMUX & 0xF8) | B_Yaxis; // Select Y axis 
		ADCSRA |= (1<<ADSC); // start single convertion
		while(ADCSRA & (1<<ADSC)); // wait for conversion to complete		
		gamepad_report_2.Yaxis = 127-ADCH;		

		// Populate buttons 1-7 - Controller A
		if ( B3 ) gamepad_report_2.buttons7_1	+= 1;	
		if ( B4 ) gamepad_report_2.buttons7_1	+= 2;		
		if ( B5 ) gamepad_report_2.buttons7_1	+= 4;	
		if ( B6 ) gamepad_report_2.buttons7_1	+= 8;		
		if ( B7 ) gamepad_report_2.buttons7_1	+= 16;	
		if ( B8 ) gamepad_report_2.buttons7_1	+= 32;	
		if ( B9 ) gamepad_report_2.buttons7_1	+= 64;		
 
		
		
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
