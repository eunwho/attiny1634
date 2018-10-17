

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "stdio.h"
#include "string.h"


// select only 1 below 
//#define MODE_MEASURE_CURRENT
//#define MODE_MEASURE_PRESSURE

//#define	READ_ADC	1

//#define ID_0x51		1
#define ID_0x61		1
//#define ID_0x71		1

#if ID_0x51
#define ID_ADDR		0x51
#define ADC_CHAN	0
#define SENS_HIGH	100.0
#define SENS_LOW	0.0
#define ADC_HIGH	255
#define ADC_LOW		0
#endif

#if ID_0x61
#define ID_ADDR		0x61
#define ADC_CHAN	1
#define SENS_HIGH	5.0
#define SENS_LOW	0.0
#define ADC_HIGH	255
#define ADC_LOW		0
#endif

#if ID_0x71
#define ID_ADDR		0x71
#endif

#define SET		1
#define RESET	0
#define HIGH	1
#define LOW		0

//============== PORT A
// PA0 : AREF
// PA3 : ADC0 : CURR_3
// PA4 : ADC1 : DC_IN
#define UART_RXD0	0x80	// PA7, input

//--- PORT B
#define UART_TXD0	0x01	// PB0, output
#define DEBUG_LED	0x08	// PB3, output

//--- PORT C
#define RS485_EN		0x01	// PC0, output
#define IN_NORMAL_OPEN	0x04	// PC2, input

//=============================================
#define LED_ON			PORTB &= ~DEBUG_LED;
#define LED_OFF			PORTB |= DEBUG_LED;

#define RS485_TX_EN		DDRC |= RS485_EN;
#define RS485_RX_EN		DDRC &= ~RS485_EN;

//--- sensor calc 
float sensScale;
float sensOffset;

unsigned int	adcResult;
unsigned long adcCount = 0;
unsigned long adcCheck = 0;
unsigned int adcCh = 0;

unsigned char gState;
unsigned char gflagConnect;
unsigned char gValue_Current;
unsigned char gValue_DCVoltage;
unsigned char gValue_DigitalPressure;
unsigned char gUartRxDone;
unsigned char gUartRxBuffer[20]={0};
unsigned char gUartTxBuffer[20]={0};

void Initial_GPIO(void){
	
	DDRA = 0x00;	
	DDRB = 0x00; DDRB |= DEBUG_LED;	DDRB |= UART_TXD0;
	PUEC = 0x40; DDRC = 0x00;	DDRC |= RS485_EN;
	LED_OFF;
}

void Initial_uart(void){
	UBRR0H = 0;
	UBRR0L = 51;	// 9600bps
	UCSR0A = (0 << RXC0)  | (1 << UDRE0);
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);
	
	gUartRxDone = RESET;
}

void send_uart(uint8_t c) 
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void uart_puts(const char *s) {
	while (*s) {
		send_uart(*s);
		s++;
	}
}


unsigned long adCheck;

#if	(ID_0x51 | ID_0x61)
void calcSensScaleOffset(){
	float x1,x2,y2,y1;
	
	y2 = SENS_HIGH; y1 = SENS_LOW ; x2 = ADC_HIGH ; x1 = ADC_LOW;
	sensScale = ( y2 - y1 ) / ( x2 - x1 );
	sensOffset = ( y1 * x2 - y2 * x1 ) / ( x2 - x1 );
}

float calcSens(unsigned int arg1){
	return sensScale * arg1 + sensOffset;
}

void Initial_ADC(void)
{
	// ADMUX = (1 << REFS0);
	//	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);      // set prescaler to 64
	//ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0) | (1<<ADIE ) | (1 << ADSC) | (1 << ADEN) | (1<< ADATE);		// irq enabled
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0) | (1<<ADIE ) | (1 << ADSC) | (1 << ADEN) | (1<< ADATE);		// irq enabled
	ADCSRB = (1<<ADLAR );	// ADCH, ADCL are left adjusted
	DIDR0 = 0x38;
	ADMUX = ADC_CHAN;	// AREF use, select adc1
}

ISR(ADC_vect)
{
	adcResult = ADCH ;
	ADCSRA |= (1<<ADSC);
}
#endif

ISR(USART0_RX_vect)
{
	static int i,flagLed;
	unsigned char inData;
	static unsigned char gUartIndex=0;
	static unsigned char flagStx = RESET;
	
	inData = UDR0;
	if(flagStx == RESET){
		if(inData == 0x02){
			gUartIndex = 0;
			gUartRxBuffer[gUartIndex++] = inData;
			flagStx = SET;
		}
	}
	else if(flagStx == SET){
		gUartRxBuffer[gUartIndex++] = inData;
		if(inData == 0x03){
			flagLed = ( flagLed ) ? 0 : 1;
			if( flagLed ){ LED_ON; }
			else {			LED_OFF;}

			gflagConnect = SET;
			flagStx = RESET;
			for( i = 0 ; i < 6 ; i++) gUartTxBuffer[i] = gUartRxBuffer[i];
			gUartIndex = 0;
		}
	}
}


void Initial_Timer(void)
{
	TCCR1B = 0x0C; 
	OCR1A = 30;	// 1ms
	TIMSK = 0x40; 
}

ISR(TIMER1_COMPA_vect)	//1msec
{
	static  unsigned int gCounter = 0;
	static unsigned char test = 0;

	if(gCounter >= 500){	// 500msec
		if(gflagConnect == RESET){
			if(test == 0){
				//LED_ON;
				test = 1;
			}
			else{
				//LED_OFF;
				test = 0;				
			}
		}
		gCounter = 0;
	}
	else gCounter++;
}

void rs485Send( ){

	int i;
	unsigned long input;

	RS485_RX_EN;

#if ID_0x71	

	char tmp;
	
	i = ( PINC & ( 1 << PINC2 )) ? 1 : 0;
	
	tmp = ( i ) ? 'C' : 'O' ;
	
	for( i = 3 ; i < 7 ; i++) gUartTxBuffer[i] = tmp;
	
	gUartTxBuffer[0] = 0x02;
	gUartTxBuffer[1] = ID_ADDR;
	gUartTxBuffer[2] = 'A';
	gUartTxBuffer[7] = 0x03;
#endif

#if READ_ADC
	input = (unsigned long)(adcResult * 10);
#else
	if (ID_ADDR == 0x51)		input = (unsigned long)(calcSens( adcResult) / 100);
	else if(ID_ADDR == 0x61 )   input = (unsigned long)(calcSens(adcResult ) * 10);
	else                        input = 1;	
#endif

#if (ID_0x51 | ID_0x61 )
	gUartTxBuffer[0] = 0x02;
	gUartTxBuffer[1] = ID_ADDR;
	gUartTxBuffer[2] = 'A';	
	gUartTxBuffer[3] = ((unsigned char)(input / 1000))			+ 0x30;
	gUartTxBuffer[4] = ((unsigned char)((input % 1000)/100))	+ 0x30;
	gUartTxBuffer[5] = ((unsigned char)((input % 100 ) /10))	+ 0x30;
	gUartTxBuffer[6] = (unsigned char)( (input % 100)%10 + 0x30);
	gUartTxBuffer[7] = 0x03;
#endif

	_delay_ms(7);
	for( i = 0 ; i < 8 ; i++){
		send_uart( gUartTxBuffer[i]);
	}
	_delay_ms(10);
}

int main(void)
{

	//_delay_ms(500);
	Initial_GPIO();
	Initial_uart();
	Initial_Timer();

#if (ID_0x51 | ID_0x61)
	Initial_ADC();
	calcSensScaleOffset();   // get scale and offset for calc sensor value
#endif
	
	RS485_RX_EN;
	gflagConnect = RESET;
	sei();
	RS485_TX_EN;
	while(1){
		if( gflagConnect ){
			gflagConnect = 0;
			if(( gUartTxBuffer[2] == 'E') && (gUartTxBuffer[1] == ID_ADDR)){
				rs485Send( );
			}
		} else {
			RS485_TX_EN;
		}
	}
}


