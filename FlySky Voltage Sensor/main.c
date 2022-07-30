/*
 * FlySky Voltage Sensor.c
 *
 * Created: 26/06/2022 8:48:27 AM
 * Author : Frank Tkalcevic
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#if defined(__AVR_ATmega328P__)

#define LED_BUILTIN_DDR		DDRB
#define LED_BUILTIN_PORT	PORTB
#define LED_BUILTIN_PIN		PINB
#define LED_BUILTIN			5

#define UCSRA				UCSR0A
#define UDRE				UDRE0
#define UDR					UDR0
#define UCSRB				UCSR0B
#define TXEN				TXEN0
#define RXEN				RXEN0
#define TXC					RXC0
#define RXC					TXC0
#define UBRRL				UBRR0L
#define U2X					U2X0
#define WGM00				WGM10
#define WGM01				WGM11
#define WGM02				WGM12
#define WGM03				WGM13
#define ICF0				ICF1
#define ICES0				ICES1
#define ICR0				ICR1

#elif defined(__AVR_ATtiny102__)

// Not a real LED, just an output for testing the clock
#define LED_BUILTIN_DDR		DDRB
#define LED_BUILTIN_PORT	PORTB
#define LED_BUILTIN_PIN		PINB
#define LED_BUILTIN			1

// 8	GND
// 7	RxD0		USART RX	
// 6	TxD0/ICP0	USART TX		ICP0 for measuring timer pulses
// 5	ADC5		ADC input
// 

// 4	Reset		TDI-Reset
// 3	TPI_Data	
// 2	TPI_Clk		
// 1	Vcc
#endif

#define CMD_DISCOVER_SENSOR		0x80
#define CMD_REQUEST_SENSOR_TYPE	0x90
#define CMD_REQUEST_MEASUREMENT	0xA0

#define ADDRESS_1				1

#define SENSOR_TYPE_EXTERNAL_VOLATAGE	0x03 
#define REPLY_DELAY				2	// us

enum EReceiveState
{
	EIdle,
	EReadData,
	ESkipData
};

//static uint16_t checksum  __attribute(( section(".noinit") ));	// this is broken.
static uint16_t checksum;
static uint16_t adc_sum;
static uint16_t last_voltage;
static uint16_t last_input_capture;
static uint8_t cmd;
static uint8_t len;
static uint8_t adc_count;
static uint8_t overflowed;
static uint8_t got_falling_edge;
static enum EReceiveState receive_state;


//void __do_clear_bss(void)		// don't init bss
//{
//}

static void test_clock( void )
{
	#ifdef LED_BUILTIN
		LED_BUILTIN_DDR |= _BV(LED_BUILTIN);
		while (1)
		{
			LED_BUILTIN_PIN |= _BV(LED_BUILTIN);
			_delay_ms(1000);
		}
	#endif
}

static void _SEND( uint8_t c)
{
	while ( !(UCSRA & _BV(UDRE)) )
	continue;
	UDR = c;
}

static void SEND( uint8_t c)
{
	_SEND(c);
	checksum -= c;
}

static void send_checksum(void)
{
	_SEND( *(((uint8_t *)&checksum)) );
	_SEND( *(((uint8_t *)&checksum)+1) );
}


static void replay_delay(void)
{
	_delay_us(REPLY_DELAY);
}


static void switch_to_transmit(void)
{
	UCSRB = 0;
	UCSRB = _BV(TXEN);
	checksum = 0xFFFF;
}

static void switch_to_receive(void)
{
	while ( (UCSRA & _BV(TXC)) == 0 )
		continue;
									
	UCSRA |= _BV(RXC);
	UCSRB = _BV(RXEN);
}

static void calibrate_clock(void)
{
	// The first byte we receive on the RX line is 0x04, 0b00000100 in binary.  
	// That is 2 low bits, plus 1 low start bit.  So at 115200 baud, the first
	// low pulse will be 3/115200 sec = 26.041666us.
	// At 1MHz clock, that's 26.041666 clock cycles. 208.3333 at 8MHz.
	
	// If we aim for 7.3728MHz, we get 115.2 baud with 0 error, which is 192.000 ticks
	
	// program timer
	// Wait for idle
	// interrupt timer resets at falling edge
	// timer measures at rising edge.
}

void main(void)
{
     // set up clock - turn off div8 scale
	 CCP = CCP_IOREG_gc;
	 CLKPSR = 0;
	 
	 //test_clock(); 	 return;
	 
	 // calibrate clock
	 calibrate_clock();
	 
	 // set up usart.  2 options, depending on the calibaration-ability of the internal oscillator.
	 // BRR=7, F_CPU=7372800, 64 timer ticks per bit
	 // BRR=8, F_CPU=8294400, 72 timer ticks per bit
#define USART_TIME_TICKS		72
	 UBRRL = 8;
	 UCSRA = _BV(U2X);
	 UCSRB =  _BV(RXEN); 
	 //UCSR0C =
	 //UCSR1D = 
	 
	 // set up adc
	ADMUX = _BV(REFS0) | _BV(MUX2) | _BV(MUX0); // 2.2v reference.  ADC5 input
	ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADATE);	// enable, start, prescale 128
	
	// timer (atmega TC1 16bit attiny102 TC0 16bit)
	// 8MHz, fast PWM (mode 15), TOP=OCR1A=0x1FFF ~ 2ms
	OCR0A = 0x1FFF;
	TCCR0A = _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(WGM03) | _BV(WGM02) | _BV(CS00);	// Clock div 1 - 8MHz.  Overflow 8.192ms

DDRA |= _BV(0);
	
	receive_state = EIdle; 
    while (1) 
    {
		if ( TIFR0 & _BV(TOV0) )
		{
			// Overflow
			TIFR0 |= _BV(TOV0) | _BV(ICF0);
			TCCR0B &= ~_BV(ICES0);	// next trigger on falling edge
			receive_state = EIdle;
			last_input_capture = 0;
			overflowed = 1;
			got_falling_edge = 0;
		}
		
		if ( overflowed && (TIFR0 & _BV(ICF0)) )
		{
			register uint16_t capture = ICR0;
			TIFR0 |= _BV(ICF0);
			
			{
				if ( !got_falling_edge )
				{
					TCCR0B |= _BV(ICES0);	// next trigger on rising edge
					last_input_capture = capture;
					got_falling_edge = 1;
				}
				else
				{
					// input capture
					register uint16_t diff = capture - last_input_capture;
					
					// Correct oscillator.  192 clicks gives 7.3728MHz, which is an even baud rate multiplier for 115.2kHz
					// TODO The assumption here is the first bits are start-0-0-1.  192 assumes 64ticks per bit.  We could look for multiples
					if ( diff > 3*USART_TIME_TICKS )
					{
						if ( OSCCAL > 0 )
							OSCCAL = OSCCAL - 1;
					}
					else if ( diff < 3*USART_TIME_TICKS )
					{
						if ( OSCCAL < 255 )
							OSCCAL = OSCCAL + 1;
					}
					// TODO - if we aren't adjusting anymore, we can power off the timers to save power.
					overflowed = 0;

#ifdef SHOW_10us_PULSE					
					// Debug the OSCCAL auto calibration against the 115200 baud uart.
					PINA |= _BV(0);
					_delay_us(10);
					PINA |= _BV(0);
#endif
				}
			}
		}
		
		
		if ( ADCSRA & _BV(ADIF) )
		{
			// Divider is 39k / 4k7 -> 1023n = 10.23v
			register uint16_t sample = ADCL;
			sample |= ADCH<<8;
			adc_sum += sample;
			adc_count++;
			if (adc_count == 32)
			{
				last_voltage = adc_sum >> 5;
				adc_sum = 0;
				adc_count = 0;
			}
		}
		
		if ( UCSRA & _BV(RXC) )
		{
			register uint8_t c = UDR;
			
			switch ( receive_state )
			{
				case EIdle:
					len = c;
					if ( len == 4 )	// We only support 4 byte commands
					{
						checksum = 0xFFFF;
						checksum -= c;
						receive_state = EReadData;
					}
					else
					{
						receive_state = ESkipData;
					}
					len--;
					break;
						
				case ESkipData:
					len--;
					if ( len == 0 )
						receive_state = EIdle;
					break;
						
				case EReadData:
						
					if ( len == 3 )
					{
						cmd = c;
						checksum -= c;
						len--;
					}
					else if ( len == 2 )
					{
						*((uint8_t *)&checksum) ^= c;
						len--;
					}
					else if ( len == 1 )
					{
						*(((uint8_t *)&checksum)+1) ^= c;
							
						if ( checksum == 0 )
						{
							// process cmd
							if ( cmd == (CMD_DISCOVER_SENSOR | ADDRESS_1 ) )
							{
								// reply
								switch_to_transmit();

								replay_delay();
								SEND(4);
								SEND(cmd);
								send_checksum();
									
								switch_to_receive();									
							}
							else if ( cmd == (CMD_REQUEST_SENSOR_TYPE | ADDRESS_1 ) )
							{
								switch_to_transmit();
									
								replay_delay();
								SEND(6);
								SEND(cmd);
								SEND(SENSOR_TYPE_EXTERNAL_VOLATAGE);
								SEND(2);
								send_checksum();
									
								switch_to_receive();
							}
							else if ( cmd == (CMD_REQUEST_MEASUREMENT | ADDRESS_1 ) )
							{
								switch_to_transmit();

								replay_delay();
								SEND(6);
								SEND(cmd);
								uint8_t t1 = last_voltage & 0xff;
								SEND(t1);
								t1 = last_voltage >> 8;
								SEND(t1);
								send_checksum();

								switch_to_receive();
							}
						}
						receive_state = EIdle;
					}
					break;
			}
		}
    }
}

