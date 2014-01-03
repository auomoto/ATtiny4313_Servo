/*
	Check fuses:
	
	avrdude -c -p attiny4313 -U lfuse:r:-:h
	Found the defaults:
	lfuse = 0x64
	hfuse = 0xdf
	efuse = 0xff

	Next, burn fuses for crystal:
	>avrdude -p ATtiny4313 -c usbtiny -U lfuse:w:0xff:m
	The 0xFF in lfuse gives the longest startup time with a crystal

	Using a 14.745600 MHz crystal.

*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>						// for atoi()
#include <string.h>						// for strlen()

#define PWMPINA	PB3
#define COMMANDSENT (UCSRA & _BV(RXC))	// Is there something in the RX buffer?
#define TXREADY (UCSRA & _BV(UDRE))		// Is the transmit buffer empty?

uint8_t EEMEM eeVer[]="Servo Testing v20140102";
void errMsg(uint8_t);
void init_ATtiny4313(void);
uint16_t recvNum(void);
void sendPrompt(void);
void sendByte(uint8_t);
void sendCRLF(void);
void sendNum(int, uint8_t);
void sendString(char *);



int main(void)
{

	float f;

	init_ATtiny4313();


	for (;;) {
		OCR1A = (ICR1 * 0.55) / 20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 2.23) / 20;
		_delay_ms(1000);
		}
	}
/*
		OCR1A = (ICR1 * 2.23)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 0.55)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 2.0)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 0.75)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.75)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.0)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.5)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.25)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.4)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.3)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.4)/20;
		_delay_ms(1000);
		OCR1A = (ICR1 * 1.3)/20;
		_delay_ms(1000);
	}
*/


/*
	recvNum() accepts characters from the serial port and returns the twos complement
	16-bit integer. It echos the characters as they're typed and concludes when a
	carriage return is entered.

	Errors: If more than five (5) characters without sign are entered, or if more
	than six (6) characters with a leading + or - sign are entered, this routine
	sends back a question mark (?), CRLF, and returns zero (0).

*/

uint16_t recvNum(void)
{

	char strBuf[7];
	uint8_t i = 0;

	for (;;) {
		while (!COMMANDSENT) {
			asm("nop");
		}
		strBuf[i] = UDR;
		sendByte(strBuf[i]);
		if (strBuf[i] == '\r') {
			strBuf[i] = 0x00;
			sendCRLF();
			break;
		}
		if (i == 5) {
			if ((strBuf[0] != '+') && (strBuf[0] != '-')) {
				sendCRLF();
				sendByte('?');
				return(0);
			}
		}
		if (i == 6) {
			sendCRLF();
			sendByte('?');
			return(0);
		}
		i++;
	}

	return(atoi(strBuf));

}

void sendByte(uint8_t x)
{

	while (!TXREADY) {
		asm("nop");
	}

	UDR = x;

}

void sendCRLF(void)
{

	sendByte('\n');
	sendByte('\r');

}

void sendNum(int number, uint8_t base)
{

	char strBuf[7];

	if (base == 16) {
		if (number <= 0x0f) {					// send leading '0' for hex
			sendByte('0');
		}
	}
	sendString(itoa(number, strBuf, base));

}

void sendPrompt(void)
{

	sendCRLF();
	sendByte('>');

}

void sendString(char str[])
{

	uint8_t i = 0;

	while (str[i]) {
		sendByte(str[i++]);
	}

}

/*
	errMsg()

*/

void errMsg(uint8_t msgNum)
{

	sendCRLF();
	sendByte('E');
	sendNum(msgNum, 10);

}


/*
===========================================================================

	UBRRL and UBRRH USART Baud Rate Registers. 12 bits, computed from:
	
	UBRR = (F_CPU / (16 * Baudrate)) - 1 where "Baudrate" is the common
	understanding.

===========================================================================

	UCSRB Control STatus Register B
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| RXCIE	| TXCIE	| UDRIE	|  RXEN	|  TXEN	| UCSZ2	|  RXBB	|  TXBB	|
	Set	|	0	|	0	|	0	|	1	|	1	|	0	|	0	|	0	|
	---------------------------------------------------------------------

	RXCIE is RX Complete Interrupt Enable (we won't use this)
	TXCIE is the TX Complete Interrupt ENable
	UDRIE is USART Data Register Empty Interrupt Enable
	RXEN is Receiver Enable
	TXEN is Transmitter Enable
	UCSZ2, combined with UCSZ[1:0] in UCSRC sets the number of data bits
	RXB8 Receive Data Bit 8 (ninth bit of data, if present)
	TXB8 Transmit bit 8 (ninth bit of data)

	UCSRB = 0b00011000

===========================================================================

	UCSRC Control STatus Register C
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|UMSEL1	|UMSEL0	| UPM1	| UPM0	| USBS	| UCSZ1	| UCSZ0	| UCPOL	|
	Set	|	0	|	0	|	0	|	0	|	0	|	1	|	1	|	0	|
	---------------------------------------------------------------------

	UMSEL[1:0] is the USART mode select. We want 0b00, Asynchronous UART
	UPM[1:0] is Parity mode. 0b00 is disabled, 0b01 reserved, 0b10 Enabled,
	even parity, 0b11 Enabled, odd parity.
	USBS is Stop Bit Select (how many stop bits). 0->1-bit, 1->2-bits.
	UCSZ[1:0] is the rest of the character size description. An 8-bit
	situation says both of these bits should be high.
	UCPOL is clock polarity. Changes fall/rising edges.

	UCSRC = 0b00000110

===========================================================================

	TCCR0A (Timer/Counter Control Register A for Timer0) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| COM0A1| COM0A0| COM0B1| COM0B0|	-	|	-	| WGM01	| WGM00	|
	Set	|	1	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|
	---------------------------------------------------------------------

	For phase-correct PWM with TOP=OCR0A, WGM0[2:0] should be 0b001 (WGM0[2]
	is in TCCR0B). In phase correct PWM mode the timer counts up, then down.

	We're only using one compare register here so COM0B[1:0] is 0b00 and
	COM0A[1:0] is 0b10, which sets OC0A on the way up and clears it on the
	way down.

	TCCR0A = 0b10000001

===========================================================================

	TCCR0B (Timer/Counter Control Register B for Timer0) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| FOC0A	| FOC0B	|	-	|	-	| WGM02	|  CS02	|  CS01	|  CS00	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	CS0[2:0] are the clock select bits:

		0b000	No clock source (timer stopped)
		0b001	clk/1 (no prescaling)
		0b010	clk/8
		0b011	clk/64
		0b100	clk/256
		0b101	clk/1024
		0b110	External clock on T0 pin, falling edge
		0b111	External clock on T0 pin, rising edge

	The PWM frequency is clk/(N*510) where N is the prescale factor. If
	our nominal CPU clock frequency is 14.745600 MHz, the frequency and
	period for the prescaled timer are are:

		======================================
		CS0[2:0]	Frequency		Period
		--------------------------------------
		0b001		28.913 kHz		 34.6 us
		0b010		 3.614 kHz		276.7 us
		0b011	   452.765 Hz		  2.214 ms
		0b100	   112.941 Hz		  8.854 ms
		0b101	    28.235 Hz		 35.417 ms
		======================================

	The Faulhaber 2622 B-SC motor takes a PWM frequency input in the range
	of 500 Hz to 18 kHz. Let's choose 3.614 kHz. The motor stops turning at
	duty cycle < 2.0% (5/255) and starts turning at duty cycle > 3.0% (8/255).

	For phase-correct PWM, WGM0[2:0] (Wavform Generation Mode)
	should be 0b001 if we're using OC0A (setting WGM0[2] to "1" means that
	OC0A becomes TOP and OC0B is the compared register). WGM0[1:0] are in
	TCCR0A.

	The Force Output Compare (FOC0A and FOC0B bits) must be set to zero (0)
	when using the timer in PWM mode.

	TCCR0B = 0b00000010

===========================================================================

	TCCR1A (Timer/Counter Control Register A for Timer1) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| COM1A1| COM1A0| Com1B1| COM1B0|   -	|   -	| WGM11	| WGM10	|
	Set	|	1	|	0	|	1	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	COM1A1/COM1B1 = 1 with COM1A0/COM1B0 = 0 clears OC1A/OC1B on compare
											 match, set OC1A/OC1B at TOP
	COM1A1/COM1B1 = 1 with COM1A0/COM1B0 = 1 set OC1A/OC1B on compare match
											 clear OC1A/OC1B at TOP
	We'll use WGM mode 14, which is fast PWM, TOP=ICR1. That
	means we set WGM13, WGM12, WGM11 high and keep WGM10 low.
	
	TCCR1A = 0b10100010 sets up both COM1A and COM1B

===========================================================================

	TCCR1B (Timer/Counter Control Register B for Timer1) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| ICNC1	| ICES1	|	-	| WGM13	| WGM12	|  CS12	|  CS11	|  CS10	|
	Set	|	0	|	0	|	0	|	1	|	1	|	0	|	1	|	1	|
	---------------------------------------------------------------------

	========================================================|
	| CS12	| CS11	| CS10	|	Note						|
	|-------------------------------------------------------|
	|	0	|	0	|	0	|	No clock source (stopped)	|
	|	0	|	0	|	1	|	/1, no prescaling			|
	|	0	|	1	|	0	|	/8							|
	|	0	|	1	|	1	|	/64							|
	|	1	|	0	|	0	|	/256						|
	|	1	|	1	|	0	|	External clk on T1, fall	|
	|	1	|	1	|	1	|	External clk on T1, rising	|
	=========================================================

	TCCR1B = 0b00011011


	First tell Timer1 to repeat every 20ms (50Hz). To calculate TOP:
	TOP = (Clock_Speed / (Prescaler * Output_PWM_Frequency)) - 1
	TOP = (14745600 / (Prescaler * 50)) - 1
	We want a /64 prescaler (4607 counts to TOP) at 14.7456 MHz.



===========================================================================

	MCUSR MCU Status Register (what caused the MCU Reset?)
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|   -	|   -	|	-	|	-	| WDRF	|  BORF	| EXTRF	|  PORF	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	WDRF: Watchdog Reset Flag is set if a WD Reset occurs. Reset by power
	on or writing zero to the flag.
	
	BORF: Brown-out Reset flag is set if a brown-out Reset happens.
	
	EXTRF: External Reset Flag
	
	PORF: Power-on Reset Flag

	NB: THe WDRF *MUST* be reset or the system will immediately reset and
	loop forever.

===========================================================================

	WDTCR Watchdog Timer Control Register (aka WDTCSR on ATtiny2313)
	128kHz oscillator
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| WDIF	| WDIE	| WDP3	| WDCE	|  WDE	| WDP2	| WDP1	|  WDP0	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	WDIF: Watchdog Timeout Interrupt Flag is set when a timeout occurs
	and the WDT is configured for interrupt.
	WDIE: Watchdog Timeout Interrupt Enable.
	WDP3: One of three prescaler bits (see below)
	WDCE: Watchdog Change Enable (must be set to change WDE)
	WDE: Watchdog enable
	WDP[3:0]: Timer prescaler bits

		=========================================
		WDP3	WDP2	WDP1	WDP0	Appx time
		-----------------------------------------
		 0		 0		 0		 0		16 ms
		 0		 0		 0		 1		32 ms
		 0		 0		 1		 0		64 ms
		 0		 0		 1		 1		0.125 s
		 0		 1		 0		 0		0.25 s
		 0		 1		 0		 1		0.5 s
		 0		 1		 1		 0		1.0 s
		 0		 1		 1		 1		2.0 s
		 1		 0		 0		 0		4.0 s
		 1		 0		 0		 1		8.0 s
		=========================================

	Recovering from a WDT reset is a little tricky. The steps:
		1. Clear the MCUSR watchdog reset flag (WDRF).
		2. In one operation, write logic one to both WDCE and WDE.
		3. In the very next line, write a logic 0 to WDE.


===========================================================================

*/

#define BAUDRATE 19200
#define MYUBRR ((F_CPU / 16 / BAUDRATE) -1)
#define WATCHDOG	1
#define POWERON		2
#define PUSHBUTTON	3
#define BROWNOUT	4

void init_ATtiny4313(void)
{

	char strBuf[25];
	uint8_t junk, resetType = 0;

	if (MCUSR & _BV(WDRF)) {	// If there was a watchdog reset...
		MCUSR = 0x00;			// Clear the WDRF flag (and everything else)
		WDTCR = 0x18;			// WDCE must be high to change WDE (WDEnable)
		WDTCR = 0x00;			// Now clear WDE
		resetType = WATCHDOG;
	} else if (MCUSR & _BV(PORF)) {
		resetType = POWERON;
	} else if (MCUSR & _BV(EXTRF)) {
		resetType = PUSHBUTTON;
	} else if (MCUSR & _BV(BORF)) {
		resetType = BROWNOUT;
	}
	MCUSR = 0x00;

	UBRRH = (uint8_t) (MYUBRR >> 8);	// Set baud rate
	UBRRL = (uint8_t) MYUBRR;
	UCSRB = 0b00011000;			// Enable transmit & receive
	UCSRC = 0b00000110;			// 8 data bits, no parity, one stop bit

	TCCR1A = 0;
			// 20 ms period (50 MHz) with 14.745600 MHz Xtal
	
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)
	ICR1 = 4607;		// 50 MHz

	DDRB  |= _BV(PWMPINA);		// PWMPIN is an output
								// Should also make a PWMPINB

	while (COMMANDSENT) {		// Clear the serial port
		junk = UDR;
	}

	switch (resetType) {
		case (WATCHDOG):
			sendString("S");
			break;
		case (BROWNOUT):
			sendString("BO");
			break;
		case (POWERON):
			sendString("UP");
			break;
		case (PUSHBUTTON):
			sendString("PB");
			break;
		default:
			break;
		}

	eeprom_read_block((void*) &strBuf, (const void*) &eeVer, 24);
	sendCRLF();
	sendString(strBuf);

}
