/*
 * Tritium TRI63 CAN Driver Controls firmware
 * Copyright (c) 2008, Tritium Pty Ltd.  All rights reserved.
 *  
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products 
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * Last modified: J.Kennedy, Tritium Pty Ltd, 18 November 2008
 *
 * Build 01
 *	- Analog input on potentiometer channel 1 (current)
 *	- Analog input on potentiometer channel 2 (velocity)
 *	- Digital input on encoder channel 1 (bus current)
 *	- Reads switch input postions that affect control outputs
 *		- Fwd / Rev
 *		- Brakes
 *		- Enable
 *	- Generates blink signals
 *		- Indicators
 *		- Hazards
 *	- Other switches are read and transmitted in switch bitfield in CAN packet
 *
 *	- Initialises CAN controller (MCP2515) with fixed base address and bit rate
 *	- Transmits serial/ID CAN packet at offset 0 (once only, on startup)
 *	- Transmits current/speed CAN packet at offset 1
 *	- Transmits bus current (power) CAN packet at offset 2
 *	- Transmits switch position CAN packet at offset 4
 *	- Transmits any CAN packet (0-4) if appropriate RTR frame is received
 *
 * Build 02
 *	- Modified code to meet latest (v4) WaveSculptor CAN bus comms spec
 *		- Variables using % values now sent as 0 - 1 not as 0 - 100
 *		- Moved switch output packet to address offset 4
 *
 *	- Still to do:
 *		- Add low-level routines for SPI dataflash
 *		- Add reprogrammable CAN base addresses in SPI flash
 *		- Add motor config selection commands to use pre-programmed motor constants in WaveSculptor controller
 *		- CAN bootloader for MSP430 firmware 
 *
 * Build 03
 *	- Modified code to meet published driver controls CAN bus comms spec (v1)
 *		- ID packet sent repeatedly, not just at startup
 *		- Bus current limit (from encoder 1) handled properly - starts at 100%
 *
 * Build 04
 *	- Fixed error in velocity reception from motor controller (was fp[0], should be fp[1])
 *	- Updated #defines to meet latest (v5) comms spec
 *
 */

// Include files
#include <msp430x13x.h>
#include <signal.h>
#include "tri63.h"
#include "can.h"
#include "spi.h"

// Function prototypes
void clock_init( void );
void io_init( void );
void timerA_init( void );
void timerB_init( void );
void adc_init( void );

// Global variables
volatile unsigned char blink_flag = FALSE;
volatile unsigned char comms_flag = FALSE;
volatile unsigned char activity_flag = FALSE;
volatile unsigned char input_flag = FALSE;

// Main routine
int main( void )
{ 
	// Local variables
	// Switch inputs - same bitfield positions as CAN packet spec
	unsigned int switches_old, switches_new, switches_dif, switches_dif_save;
	// Switch outputs (for blinker flash commands)
	unsigned int switches_out_new, switches_out_dif;
	// Velocity & torque control values
	unsigned int adcvalue1 = 0;
	unsigned int adcvalue2 = 0;
	float actual_velocity = 0.0;
	float set_velocity = 0.0;
	float set_current = 0.0;
	float set_bus_current = 1.0;
	// Indicator (blinker) control
	unsigned char blink_left = 0;
	unsigned char blink_right = 0;
	// Encoders
	int encoder1 = 100;
	int encoder2 = 0;
	unsigned char enc1_old, enc1_new, enc2_old, enc2_new;
	// Comms
	unsigned int comms_event_count = 0;
	
	// Stop watchdog timer
	WDTCTL = WDTPW + WDTHOLD;

	// Initialise I/O ports
	io_init();

	// Initialise SPI port for CAN controller (running with SMCLK, to reset MCP2515)
	spi_init(0);
	
	// Reset CAN controller to give reliable clock output
	// change clock to faster rate (CANCTRL reg, change lower two bits, CLK/4)
	can_init();

	// Initialise clock module now that the MCP2515 is giving us the faster clock
	clock_init();

	// Initialise Timer A (encoder LED output PWM)
	timerA_init();
	
	// Initialise Timer B (10ms timing ticks)
	timerB_init();
  
	// Initialise A/D converter for potentiometer input
	adc_init();

	// Re-initialise SPI port for CAN controller (running with ACLK, full speed)
	spi_init(1);

	// Initialise switch & encoder positions
	switches_dif = 0x0000;
	switches_dif_save = 0x0000;
	switches_new = 0x0000;
	switches_old = P5IN;
	switches_old <<= 4;
	switches_old |= (P4IN >> 4);
	if((P1IN & ENC2_SW) == 0x00) switches_old &= ~SW_ENC2_SW;
	else switches_old |= SW_ENC2_SW;
	if((P1IN & ENC1_SW) == 0x00) switches_old &= ~SW_ENC1_SW;
	else switches_old |= SW_ENC1_SW;
	if((P2IN & DEBUG_nSW) == 0x00) switches_old |= SW_DEBUG;
	else switches_old &= ~SW_DEBUG;	
	switches_out_new = 0x0000;
	switches_out_dif = 0x0000;
	enc1_old = (P1IN & (ENC1_B | ENC1_A));
	enc2_old = (P1IN & (ENC2_B | ENC2_A));	

	// Enable interrupts
	eint();

	// Check switch inputs and generate command packets to motor controller
	while(TRUE){
		// Keep track of encoders
		enc1_new = (P1IN & (ENC1_B | ENC1_A));
		switch(enc1_old){
			case STATE_1_A:							// 00
				if( enc1_new == STATE_1_B ) encoder1--;
				else if( enc1_new == STATE_1_D ) encoder1++;
				break;
			case STATE_1_B:							// 01
				if( enc1_new == STATE_1_C ) encoder1--;
				else if( enc1_new == STATE_1_A ) encoder1++;
				break;
			case STATE_1_C:							// 11
				if( enc1_new == STATE_1_D ) encoder1--;
				else if( enc1_new == STATE_1_B ) encoder1++;
				break;
			case STATE_1_D:							// 10
				if( enc1_new == STATE_1_A ) encoder1--;
				else if( enc1_new == STATE_1_C ) encoder1++;
				break;
			default:
				break;
		}
		enc1_old = enc1_new;		
		
		enc2_new = (P1IN & (ENC2_B | ENC2_A));
		switch(enc2_old){
			case STATE_2_A:							// 00
				if( enc2_new == STATE_2_B ) encoder2--;
				else if( enc2_new == STATE_2_D ) encoder2++;
				break;
			case STATE_2_B:							// 01
				if( enc2_new == STATE_2_C ) encoder2--;
				else if( enc2_new == STATE_2_A ) encoder2++;
				break;
			case STATE_2_C:							// 11
				if( enc2_new == STATE_2_D ) encoder2--;
				else if( enc2_new == STATE_2_B ) encoder2++;
				break;
			case STATE_2_D:							// 10
				if( enc2_new == STATE_2_A ) encoder2--;
				else if( enc2_new == STATE_2_C ) encoder2++;
				break;
			default:
				break;
		}
		enc2_old = enc2_new;
		
		// Monitor switch positions & analog inputs
		if( input_flag == TRUE ){
			input_flag = FALSE;
			
			// Check potentiometer inputs
			ADC12CTL0 |= ADC12SC;               	// Start A/D conversions
			while ((ADC12IFG & BIT1) == 0 );		// Busy wait for both conversions to complete
			
			adcvalue1 = ADC12MEM0;					// Potentiometer 1
			if(adcvalue1 >= ADC_MIN){
				adcvalue1 -= ADC_MIN;				// Make sure 0% is 0
			}
			else{
				adcvalue1 = 0;
			}
			if(adcvalue1 > ADC_MAX){				// Saturate at 100%
				adcvalue1 = ADC_MAX;
			}
			
			adcvalue2 = ADC12MEM1;					// Potentiometer 2
			if(adcvalue2 >= ADC_MIN){
				adcvalue2 -= ADC_MIN;				// Make sure 0% is 0
			}
			else{
				adcvalue2 = 0;
			}
			if(adcvalue2 > ADC_MAX){				// Saturate at 100%
				adcvalue2 = ADC_MAX;
			}
			
			// Grab the current state of the switch inputs
			switches_new = P5IN;
			switches_new <<= 4;
			switches_new |= (P4IN >> 4);
			if((P1IN & ENC2_SW) == 0x00) switches_new &= ~SW_ENC2_SW;
			else switches_new |= SW_ENC2_SW;
			if((P1IN & ENC1_SW) == 0x00) switches_new &= ~SW_ENC1_SW;
			else switches_new |= SW_ENC1_SW;
			if((P2IN & DEBUG_nSW) == 0x00) switches_new |= SW_DEBUG;
			else switches_new &= ~SW_DEBUG;
			
			// Check for changes in state & update record
			switches_dif = switches_old ^ switches_new;
			switches_old = switches_new;
			switches_dif_save |= switches_dif;
			
			// Process changes for switches that we care about
			if(switches_dif & SW_DEBUG){		// Debug switch has changed state
				if(switches_new & SW_DEBUG){	// Switch is now pushed
					P2OUT &= ~DEBUG_nLED;
				}
				else{							// Switch is now released
					P2OUT |= DEBUG_nLED;
				}
			}
			
			if(switches_new & (SW_BRAKE_1 | SW_BRAKE_2)){	// If either brake switch is pushed...
				adcvalue1 = 0;					// Clear command back to zero
				adcvalue2 = 0;
			}
			
			if(switches_dif & SW_REVERSE){		// Direction switch has changed state
				adcvalue1 = 0;					// Clear command back to zero
				adcvalue2 = 0;
			}
			
			if(switches_dif & SW_IND_L){		// Left indicator has changed state
				if(switches_new & SW_IND_L){	// Switch is now pushed
					blink_left = BLINK_SET;	
					blink_right = BLINK_OFF;
				}
				else{							// Switch is now released
					blink_left = BLINK_OFF;
					blink_right = BLINK_OFF;
				}
			}

			if(switches_dif & SW_IND_R){		// Right indicator has changed state
				if(switches_new & SW_IND_R){	// Switch is now pushed
					blink_left = BLINK_OFF;	
					blink_right = BLINK_SET;
				}
				else{							// Switch is now released
					blink_left = BLINK_OFF;
					blink_right = BLINK_OFF;
				}
			}

			if(switches_dif & SW_HAZARD){		// Hazards switch has changed state
				if(switches_new & SW_HAZARD){	// Switch is now pushed
					blink_left = BLINK_SET;	
					blink_right = BLINK_SET;
				}
				else{							// Switch is now released
					blink_left = BLINK_OFF;
					blink_right = BLINK_OFF;
				}
			}
		}

		// Handle blink (status light & indicators) events
		if( blink_flag == TRUE ){
			blink_flag = FALSE;
			// Status light (toggle green, clear red if it is set)
			P4OUT ^= nGREEN_A;
			P4OUT |= nRED;
			// Blink left indicators
			switch(blink_left){
				case BLINK_SET:
					switches_out_new |= SW_BLINK_L;
					switches_out_dif |= SW_BLINK_L;
					blink_left = BLINK_CLR;
					break;
				case BLINK_CLR:
					switches_out_new &= ~SW_BLINK_L;
					switches_out_dif |= SW_BLINK_L;
					blink_left = BLINK_SET;
					break;
				case 0:
					if(switches_out_new & SW_BLINK_L){
						switches_out_new &= ~SW_BLINK_L;
						switches_out_dif |= SW_BLINK_L;
					}
					break;
				default:
					break;
			}
			// Blink right indicators
			switch(blink_right){
				case BLINK_SET:
					switches_out_new |= SW_BLINK_R;
					switches_out_dif |= SW_BLINK_R;
					blink_right = BLINK_CLR;
					break;
				case BLINK_CLR:
					switches_out_new &= ~SW_BLINK_R;
					switches_out_dif |= SW_BLINK_R;
					blink_right = BLINK_SET;
					break;
				case 0:
					if(switches_out_new & SW_BLINK_R){
						switches_out_new &= ~SW_BLINK_R;
						switches_out_dif |= SW_BLINK_R;
					}
					break;
				default:
					break;
			}
		}

		// Handle communications events
		if( comms_flag == TRUE ){
			comms_flag = FALSE;
			// Flash the CAN activity light
			activity_flag = TRUE;
			
			// Transmit current/velocity frame
			if(switches_new & SW_IGN_ON){
				set_current = (float)(adcvalue1) / ADC_MAX * MAX_CURRENT;	
				if(switches_new & SW_REVERSE) set_velocity = (float)(adcvalue2) / ADC_MAX * MAX_VELOCITY_R;
				else set_velocity = (float)(adcvalue2) / ADC_MAX * MAX_VELOCITY_F;
			}
			else{
				set_current = 0.0;
				set_velocity = 0.0;
			}
			can.address = DC_CAN_BASE + DC_DRIVE;
			can.data.data_fp[1] = set_current;
			can.data.data_fp[0] = set_velocity;
			can_transmit();		

			// Transmit bus command frame
			if(encoder1 > 100) encoder1 = 100;
			if(encoder1 < 0) encoder1 = 0;
			set_bus_current = (float)(encoder1) / 100.0;
			can.address = DC_CAN_BASE + DC_POWER;
			can.data.data_fp[1] = set_bus_current;
			can.data.data_fp[0] = 0.0;
			can_transmit();
			
			// Transmit switch position/activity frame and clear switch differences variables
			can.address = DC_CAN_BASE + DC_SWITCH;
			can.data.data_u16[3] = switches_out_dif;
			can.data.data_u16[2] = switches_dif_save;
			can.data.data_u16[1] = switches_out_new;
			can.data.data_u16[0] = switches_new;
			switches_dif_save = 0x0000;
			switches_out_dif = 0x0000;
			can_transmit();
			
			// Transmit our ID frame at a slower rate (every 10 events = 1/second)
			comms_event_count++;
			if(comms_event_count == 10){
				comms_event_count = 0;
				can.address = DC_CAN_BASE;
				can.data.data_u8[7] = 'T';
				can.data.data_u8[6] = 'R';
				can.data.data_u8[5] = 'I';
				can.data.data_u8[4] = 'b';
				can.data.data_u32[0] = DEVICE_SERIAL;
				can_transmit();				
			}
		}

		// Check for CAN packet reception
		if((P2IN & CAN_nINT) == 0x00){
			// IRQ flag is set, so run the receive routine to either get the message, or the error
			can_receive();
			// Check the status
			if(can.status == CAN_OK){
				switch(can.address){
					case MC_CAN_BASE + MC_VELOCITY:
						actual_velocity = can.data.data_fp[1];
						break;
				}
			}
			if(can.status == CAN_RTR){
				switch(can.address){
					case DC_CAN_BASE:
						can.data.data_u8[3] = 'T';
						can.data.data_u8[2] = 'R';
						can.data.data_u8[1] = 'I';
						can.data.data_u8[0] = 'b';
						can.data.data_u32[1] = DEVICE_SERIAL;
						can_transmit();
						break;
					case DC_CAN_BASE + DC_DRIVE:
						can.data.data_fp[1] = set_current;
						can.data.data_fp[0] = set_velocity;
						can_transmit();
						break;
					case DC_CAN_BASE + DC_POWER:
						can.data.data_fp[1] = set_bus_current;
						can.data.data_fp[0] = 0.0;
						can_transmit();
						break;
					case DC_CAN_BASE + DC_SWITCH:
						can.data.data_u16[1] = switches_out_new;
						can.data.data_u16[0] = switches_new;
						can.data.data_u16[3] = switches_out_dif;
						can.data.data_u16[2] = switches_dif_save;
						switches_dif_save = 0x0000;
						switches_out_dif = 0x0000;
						can_transmit();
						break;
				}
			}
			if(can.status == CAN_ERROR){
				P4OUT &= ~nRED;
			}
		}
	}
	
	// Will never get here, keeps compiler happy
	return(1);
}


/*
 * Initialise clock module
 *	- Setup MCLK, ACLK, SMCLK dividers and clock sources
 *	- Make sure input clock (from CAN controller) has stabilised
 *	- ACLK  = CLKIN/1
 *	- MCLK  = CLKIN/1
 *	- SMCLK = 0
 */
void clock_init( void )
{
	unsigned int i;
	
	BCSCTL1 = 0xC0;								// XT1 = HF crystal, ACLK = /1, DCO Rset = 0, XT2 = OFF
	_BIC_SR( OSCOFF );							// Clear OSCOFF bit - start oscillator
	do{
		IFG1 &= ~OFIFG;							// Clear OSCFault flag
		for( i = 255; i > 0; i-- );			// Wait for flag to set
	} while(( IFG1 & OFIFG ) != 0);
	BCSCTL2 = 0xC0;								// Set MCLK to XT1/1, SMCLK to DCOCLK/1
	_BIS_SR( SCG1 );							// Set SCG1 bit - disable SMCLK
	_BIS_SR( SCG0 );							// Set SCG0 bit - disable DCO
}

/*
 * Initialise I/O port directions and states
 *	- Drive unused pins as outputs to avoid floating inputs
 *
 */
void io_init( void )
{
	P1OUT = 0x00;
	P1DIR = ENC1_LED | ENC2_LED | P1_UNUSED;
	
	P2OUT = DEBUG_nLED;
	P2DIR = DEBUG_nLED | P2_UNUSED;
	
	P3OUT = CAN_nCS | FLASH_nCS;
	P3DIR = CAN_nCS | CAN_MOSI | CAN_SCLK | FLASH_nCS | FLASH_MOSI | FLASH_SCLK | P3_UNUSED;
	
	P4OUT = nRED | nYELLOW | nGREEN_A | nGREEN_B;
	P4DIR = nRED | nYELLOW | nGREEN_A | nGREEN_B | P4_UNUSED;
	
	P5OUT = 0x00;
	P5DIR = P5_UNUSED;
	
	P6OUT = 0x00;
	P6DIR = P6_UNUSED;
}

/*
 * Initialise Timer A
 *	- Provides either encoder LED or potentiometer bias output
 *	- 8 bit PWM outputs on ENC_LEDs for channel 1 & 2
 *	- PWM outputs are initialised to 100% to allow their use as POT+ sources
 */
void timerA_init( void )
{
	P1SEL |= ENC1_LED | ENC2_LED;				// P1.2 and P1.3 TA1/2 otions
	TACTL = TASSEL_1 | ID_3 | TACLR;			// ACLK/8, clear TAR
	CCR0 = MAX_LED_PWM-1;						// PWM Period
	CCTL1 = OUTMOD_7;							// CCR1 reset/set
	CCR1 = MAX_LED_PWM;							// CCR1 PWM duty cycle = 100%
	CCTL2 = OUTMOD_7;							// CCR2 reset/set
	CCR2 = MAX_LED_PWM;							// CCR2 PWM duty cycle = 100%
	TACTL |= MC_1;								// Set timer to 'up' count mode	
}

/*
 * Initialise Timer B
 *	- Provides timer tick timebase at 100 Hz
 */
void timerB_init( void )
{
	TBCTL = TBSSEL_1 | ID_3 | TBCLR;			// ACLK/8, clear TBR
	TBCCR0 = (INPUT_CLOCK/8/TICK_RATE);		// Set timer to count to this value = TICK_RATE overflow
	TBCCTL0 = CCIE;								// Enable CCR0 interrrupt
	TBCTL |= MC_1;								// Set timer to 'up' count mode
}

/*
 * Initialise A/D converter
 */
void adc_init( void )
{
	P6SEL |= ENC1_POT | ENC2_POT;				// Enable A/D input channels											
	ADC12CTL0 = ADC12ON | SHT0_8 | MSC;			// Turn on ADC12, set sampling time = 256 ADCCLK, multiple conv											
	ADC12CTL1 = ADC12SSEL_1 | SHP | CONSEQ_1;	// Use sampling timer, ADCCLK = MCLK, run a single sequence per conversion start
	ADC12MCTL0 = INCH_6;						// Set conversion channel 0 to convert input channel 6 (Pot 1)
	ADC12MCTL1 = INCH_7 | EOS;					// Set conversion channel 1 to convert input channel 7 (Pot 2)
	ADC12CTL0 |= ENC;							// Enable conversions
}

/*
 * Timer B CCR0 Interrupt Service Routine
 *	- Interrupts on Timer B CCR0 match at 100Hz
 *	- Sets Time_Flag variable
 */
interrupt(TIMERB0_VECTOR) timer_b0(void)
{
	static unsigned char blink_count = BLINK_SPEED;
	static unsigned char comms_count = COMMS_SPEED;
	static unsigned char activity_count;
	static unsigned char input_count = INPUT_SPEED;
	
	// Trigger blink events (indicators)
	blink_count--;
	if( blink_count == 0 ){
		blink_count = BLINK_SPEED;
		blink_flag = TRUE;
	}
	
	// Trigger comms events (command packet transmission)
	comms_count--;
	if( comms_count == 0 ){
		comms_count = COMMS_SPEED;
		comms_flag = TRUE;
	}
	
	// Check for CAN activity events
	if( activity_flag == TRUE ){
		activity_flag = FALSE;
		activity_count = ACTIVITY_SPEED;
		P4OUT &= ~nGREEN_B;
	}
	if( activity_count == 0 ){
		P4OUT |= nGREEN_B;
	}
	else{
		activity_count--;
	}
	
	// Trigger switch input reads (debounce period)
	input_count--;
	if( input_count == 0 ){
		input_count = INPUT_SPEED;
		input_flag = TRUE;
	}
}

