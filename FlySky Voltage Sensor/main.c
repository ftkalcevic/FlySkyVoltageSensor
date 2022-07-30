/*
 * FlySky Voltage Sensor.c
 *
 * Created: 26/06/2022 8:48:27 AM
 * Author : Frank Tkalcevic
 */ 

#include <avr/io.h>
#include <util/delay.h>

#if defined(__AVR_ATmega328P__)

#define LED_BUILTIN_DDR		DDRB
#define LED_BUILTIN_PORT	PORTB
#define LED_BUILTIN_PIN		PINB
#define LED_BUILTIN			5


#elif defined(__AVR_ATTINY102__)

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

static void SEND( uint8_t c)
{
	while ( !(UCSR0A & _BV(UDRE0)) )
		continue;
	UDR0 = c;
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

int main(void)
{
     // set up clock
	 //test_clock();
	 
	 // calibrate clock
	 calibrate_clock();
	 
	 // set up usart
	 UBRR0L = 7;
	 UCSR0A = _BV(U2X0);
	 UCSR0B =  _BV(RXEN0); 
	 //UCSR0C =
	 //UCSR1D = 
	 // set up adc
	ADMUX = _BV(REFS0) | _BV(REFS1); // 1.1v reference.  ADC0 input
	ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADATE);	// enable, start, prescale 128
	
	// timer (atmega TC1 16bit attiny102 TC0 16bit)
	// 8MHz, fast PWM (mode 15), TOP=OCR1A=0x1FFF ~ 2ms
	OCR1A = 0x1FFF;
	TCCR1A = _BV(WGM11) | _BV(WGM10);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);	// Clock div 1 - 8MHz.  Overflow 8.192ms

	
	 
	enum EReceiveState receive_state;
	int receiving = 1;
	uint8_t cmd;
	uint8_t len;
	uint16_t checksum;
	uint8_t adc_count = 0;
	uint16_t adc_sum = 0;
	uint16_t last_voltage = 0;
	uint16_t last_input_capture = 0;
	uint8_t overflowed = 0;
	uint8_t got_falling_edge = 0;
    while (1) 
    {
		if ( TIFR1 & _BV(TOV1) )
		{
			// Overflow
			TIFR1 |= _BV(TOV1) | _BV(ICF1);
			TCCR1B &= ~_BV(ICES1);	// next trigger on falling edge
			receive_state = EIdle;
			last_input_capture = 0;
			overflowed = 1;
			got_falling_edge = 0;
		}
		
		if ( overflowed && (TIFR1 & _BV(ICF1)) )
		{
			uint16_t capture = ICR1;
			TIFR1 |= _BV(ICF1);
			
			{
				if ( !got_falling_edge )
				{
					TCCR1B |= _BV(ICES1);	// next trigger on rising edge
					last_input_capture = capture;
					got_falling_edge = 1;
				}
				else
				{
					// input capture
					uint16_t diff = capture - last_input_capture;
					
					// Correct oscillator.  192 clicks gives 7.3728MHz, which is an even baud rate multiplier for 115.2kHz
					// TODO The assumption here is the first bits are start-0-0-1.  192 assumes 64ticks per bit.  We could look for multiples
					if ( diff > 192 )
					{
						if ( OSCCAL > 0 )
							OSCCAL = OSCCAL - 1;
					}
					else if ( diff < 192 )
					{
						if ( OSCCAL < 255 )
							OSCCAL = OSCCAL + 1;
					}
					
					overflowed = 0;
				}
			}
		}
		
		
		if ( ADCSRA & _BV(ADIF) )
		{
			// Divider is 39k / 4k7 -> 1023n = 10.23v
			adc_sum += ADC;
			adc_count++;
			if (adc_count == 32)
			{
				last_voltage = adc_sum >> 5;
				adc_sum = 0;
				adc_count = 0;
			}
		}
		
		if ( UCSR0A & _BV(RXC0) )
		{
			uint8_t c = UDR0;
			
			if ( receiving )
			{
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
									checksum = 0xFFFF;
									UCSR0B = 0;
									UCSR0B = _BV(TXEN0);
// TODO these delays aren't right
									_delay_us(REPLY_DELAY);
									SEND(4);
									checksum -= 4;
									SEND(cmd);
									checksum -= cmd;
									SEND(checksum & 0xFF);
									SEND(checksum >> 8);
									while ( (UCSR0A & _BV(TXC0)) == 0 )
										continue;
									
									UCSR0A |= _BV(RXC0);
									UCSR0B = _BV(RXEN0);
								}
								else if ( cmd == (CMD_REQUEST_SENSOR_TYPE | ADDRESS_1 ) )
								{
									checksum = 0xFFFF;
									
									UCSR0B = 0;
									_delay_us(REPLY_DELAY);
									UCSR0B = _BV(TXEN0);
									SEND(6);
									checksum -= 6;
									SEND(cmd);
									checksum -= cmd;
									SEND(SENSOR_TYPE_EXTERNAL_VOLATAGE);
									checksum -= SENSOR_TYPE_EXTERNAL_VOLATAGE;
									SEND(2);
									checksum -= 2;
									SEND(checksum & 0xFF);
									SEND(checksum >> 8);
									while ( (UCSR0A & _BV(TXC0)) == 0 )
										continue;

									UCSR0A |= _BV(RXC0);									
									UCSR0B = _BV(RXEN0);
								}
								else if ( cmd == (CMD_REQUEST_MEASUREMENT | ADDRESS_1 ) )
								{
									uint16_t last_scaled_voltage = last_voltage;
									checksum = 0xFFFF;
									
									UCSR0B = 0;
									_delay_us(REPLY_DELAY);
									UCSR0B = _BV(TXEN0);
									SEND(6);
									checksum -= 6;
									SEND(cmd);
									checksum -= cmd;
									uint8_t t1 = last_voltage & 0xff;
									SEND(t1);
									checksum -= t1;
									t1 = last_voltage >> 8;
									SEND(t1);
									checksum -= t1;
									SEND(checksum & 0xFF);
									SEND(checksum >> 8);
									while ( (UCSR0A & _BV(TXC0)) == 0 )
										continue;

									UCSR0A |= _BV(RXC0);									
									UCSR0B = _BV(RXEN0);
								}
							}
							receive_state = EIdle;
						}
						
						break;
				}
			}
		}

    }
}

