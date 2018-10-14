

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "stdio.h"
#include "string.h"


// select only 1 below 
//#define MODE_MEASURE_CURRENT
//#define MODE_MEASURE_PRESSURE

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

//--- by skj 2018.10.13
#define MODE_MEASURE_DC		'A'
#define ID_CURRENT			'B'
#define ID_PRESSURE			'C'
#define ID_DC				'D'

//--- sensor calc 
#define I_HIGH			200.0
#define I_LOW			0.0
#define I_ADC_HIGH		1024
#define I_ADC_LOW		0

#define VOLT_HIGH		500.0
#define VOLT_LOW		0.0
#define VOLT_ADC_HIGH	1024
#define VOLT_ADC_LOW	0

#define PRESS_HIGH		500.0
#define PRESS_LOW		0.0
#define PRESS_ADC_HIGH	1024
#define PRESS_ADC_LOW	0

float currentScale;
float currentOffset;

float voltScale;
float voltOffset;

float pressScale;
float pressOffset;

float sensCurrent;
float sensVolt;
float sensPress;

unsigned long adcResult[3];
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

void calcSensScaleOffset(){
	float x1,x2,y2,y1;
	
	y2 = I_HIGH; y1 = I_LOW ; x2 = I_ADC_HIGH ; x1 = I_ADC_LOW;
	currentScale = ( y2 - y1 ) / ( x2 - x1 );
	currentOffset = ( y1 * x2 - y2 * x1 ) / ( x2 - x1 );
	
	y2 = VOLT_HIGH; y1 = VOLT_LOW ; x2 = VOLT_ADC_HIGH ; x1 = VOLT_ADC_LOW;
	voltScale = ( y2 - y1 ) / ( x2 - x1 );
	voltOffset = ( y1 * x2 - y2 * x1 ) / ( x2 - x1 );
	
	y2 = PRESS_HIGH; y1 = PRESS_LOW ; x2 = PRESS_ADC_HIGH ; x1 = PRESS_ADC_LOW;
	pressScale = ( y2 - y1 ) / ( x2 - x1 );
	pressOffset = ( y1 * x2 - y2 * x1 ) / ( x2 - x1 );
}

float calcSens(unsigned input, float scale, float offset){
	float returnValue;
	
	returnValue = scale * input + offset;
	return returnValue;
}

unsigned long adCheck;


void Initial_ADC(void)
{
	// ADMUX = (1 << REFS0);
	//	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);      // set prescaler to 64
	//ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0) | (1<<ADIE ) | (1 << ADSC) | (1 << ADEN) | (1<< ADATE);		// irq enabled
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0) | (1<<ADIE ) | (1 << ADSC) | (1 << ADEN) | (1<< ADATE);		// irq enabled
	ADCSRB = (1<<ADLAR );	// ADCH, ADCL are left adjusted
	DIDR0 = 0x38;
	//DIDR0 = 0x08;
}

ISR(ADC_vect)
{
	adcResult[adcCh] = ADCH ;

	if(adcCh < 2 ) adcCh ++;
	else			adcCh = 0;

	ADMUX = adcCh;
	ADCSRA |= (1<<ADSC);
}

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
			for( i = 0 ; i < 6 ; i++){
				gUartTxBuffer[i] = gUartRxBuffer[i];
			}
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



void readDCVoltage(void)
{
	// 방화문 전압(1~5V) 읽기
	// 30V -> 5V (1~5V)
	// 10bit : 1024
	float fAdc;
	float fDustVolt;

	ADMUX = 0;	// AREF use, select adc1
	
	ADCSRA |= (1 << ADSC);         // start ADC measurement
	while (ADCSRA & (1 << ADSC) ); // wait till conversion complete
	_delay_us(40); // delta time
	
	_delay_us(9680);	// sleep time
	
	fAdc = ADC;
	fDustVolt = fAdc * (5.0 / 1024.0);
	
	gValue_DCVoltage = (unsigned char)(fDustVolt * 6.0); 
	if(gValue_DCVoltage < 1) gValue_DCVoltage = 0;
}

void readDigitalPressure(void)
{
	// 디지털압력계(1~5V) 읽기

	float fAdc;
	float fDustVolt;
	
	ADMUX = 1;	// AREF use, select adc1
	
	ADCSRA |= (1 << ADSC);         // start ADC measurement
	while (ADCSRA & (1 << ADSC) ); // wait till conversion complete
	_delay_us(40); // delta time
	
	_delay_us(9680);	// sleep time
	
	fAdc = ADC;
	fDustVolt = fAdc * (5.0 / 1024.0);
	
	gValue_DigitalPressure = (fDustVolt - 2.5) * 40.0;
	
	gValue_DigitalPressure *= 10;
	if(gValue_DigitalPressure <= 0) gValue_DigitalPressure = 0;
}

void readCurrent(void)
{
	// 전류측정

	float fAdc;
	float fDustVolt;
	
	ADMUX = 0;	// AREF use, select adc0 : PA3
	
	ADCSRA |= (1 << ADSC);         // start ADC measurement
	while (ADCSRA & (1 << ADSC) ); // wait till conversion complete
	_delay_us(40); // delta time
	
	_delay_us(9680);	// sleep time
	
	fAdc = ADC;
	fDustVolt = fAdc * (5.0 / 1024.0);
	
	gValue_Current = fDustVolt * 6.0;
	
}

void txReply(){
	int i;
	RS485_RX_EN;
	_delay_ms(10);

	
	for( i = 0 ; i < 6 ; i++){  
		send_uart( gUartTxBuffer[i]);
	}
	_delay_ms(20);
}
void rs485SendInput(){
	int i, input;
	RS485_RX_EN;

	input = ( PINC & ( 1 << PINC2 )) ? 2 : 0; 	
	
	gUartTxBuffer[0] = 0x02;
	gUartTxBuffer[1] = 'D';
	gUartTxBuffer[2] = input + 0x30;
	gUartTxBuffer[3] = 0x30;
	gUartTxBuffer[4] = 0x30;
	gUartTxBuffer[5] = 0x30;
	gUartTxBuffer[6] = 0x03;

	_delay_ms(5);
	for( i = 0 ; i < 7 ; i++){
		send_uart( gUartTxBuffer[i]);
	}
	_delay_ms(10);
}

void rs485Send(char sensId, float fInput){
	int i;
	unsigned long input;
	
	input = (unsigned long)(fInput * 10);
	//	if(gUartRxBuffer[1] == ID_CURRENT && gUartRxBuffer[2] == 'R'){
	RS485_RX_EN;

	gUartTxBuffer[0] = 0x02;
	gUartTxBuffer[1] = sensId;
	gUartTxBuffer[2] = ((unsigned char)(input / 1000))			+ 0x30;
	gUartTxBuffer[3] = ((unsigned char)((input % 1000)/100))	+ 0x30;
	gUartTxBuffer[4] = ((unsigned char)((input % 100 ) /10))	+ 0x30;
	gUartTxBuffer[5] = (unsigned char)( (input % 100)%10 + 0x30);
	gUartTxBuffer[6] = 0x03;

	_delay_ms(5);
	for( i = 0 ; i < 7 ; i++){
		send_uart( gUartTxBuffer[i]);
	}
	_delay_ms(10);
}

int main(void)
{
	char arg1;
	float arg2;

	//_delay_ms(500);
	Initial_GPIO();
	Initial_uart();
	Initial_Timer();
	Initial_ADC();
	
	calcSensScaleOffset();   // get scale and offset for calc sensor value
	
	RS485_RX_EN;
	gflagConnect = RESET;
	sei();
	RS485_TX_EN;
	while(1){
		if( gflagConnect ){
			gflagConnect = 0;

			switch(gUartTxBuffer[1]){
				case 'A':	// current
					arg1 = 'A'; 
					arg2 = adcResult[0];
					rs485Send(arg1,arg2);
					break;
				case 'B':  // volt
					arg1 = 'B'; 
					arg2 = adcResult[1];
					rs485Send(arg1,arg2);
					break;
				case 'C':  // press
					arg1 = 'C'; arg2 = adcResult[2];
					rs485Send(arg1,arg2);
					break;
				case 'D':	// digital input
					rs485SendInput( );
				break;
			}
		} else {
			RS485_TX_EN;
		}
		sensCurrent = calcSens(adcResult[0],currentScale,currentOffset);
		sensVolt	= calcSens(adcResult[1],voltScale,voltOffset);
		sensPress	= calcSens(adcResult[2],pressScale,pressOffset);
	}
}


