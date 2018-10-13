

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


// select only 1 below 
//#define MODE_MEASURE_CURRENT
//#define MODE_MEASURE_PRESSURE
// #define MODE_MEASURE_DC

#define ID_CURRENT	0x50
#define ID_PRESSURE	0x60
#define ID_DC		0x70

unsigned char gState;

unsigned char gflagConnect;

unsigned char gValue_Current;
unsigned char gValue_DCVoltage;
unsigned char gValue_DigitalPressure;

unsigned char gUartRxDone;

unsigned char gUartRxBuffer[20];
unsigned char gUartTxBuffer[50];


#define SET		1
#define RESET	0
#define HIGH	1
#define LOW		0


//============== PORT A
// PA0 : AREF 
// PA3 : ADC0 : CURR_3
// PA4 : ADC1 : DC_IN
#define UART_RXD0	0x80	// PA7, input

//============= PORT B
#define UART_TXD0	0x01	// PB0, output
#define DEBUG_LED	0x08	// PB3, output

//============= PORT C
#define RS485_EN		0x01	// PC0, output
#define IN_NORMAL_OPEN	0x04	// PC2, input
#define LED_ON			PORTB &= ~DEBUG_LED;
#define LED_OFF			PORTB |= DEBUG_LED;

#define RS485_TX_EN		DDRC |= RS485_EN;
#define RS485_RX_EN		DDRC &= ~RS485_EN;

void Initial_GPIO(void)
{
	DDRA = 0x00;
	
	DDRB = 0x00;
	DDRB |= DEBUG_LED;
	DDRB |= UART_TXD0;
	
	DDRC = 0x00;
	DDRC |= RS485_EN;
	
	LED_OFF;
}

void Initial_uart(void) 
{
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

ISR(USART0_RX_vect)
{
	unsigned char inData;
	unsigned char gUartIndex;
	static unsigned char flagStx = RESET;
	
	inData = UDR0;
	
	if(flagStx == RESET){
		if(inData == 0x02){
			LED_ON;
			gUartIndex = 0;
			gUartRxBuffer[gUartIndex++] = inData;
			flagStx = SET;
		}
	}
	else if(flagStx == SET){
		if(inData == 0x03){
			LED_OFF;
			gflagConnect = SET;
			flagStx = RESET;
		}
		gUartRxBuffer[gUartIndex++] = inData;
	}

	if(gUartRxDone == SET){
#ifdef MODE_MEASURE_CURRENT
		if(gUartRxBuffer[1] == ID_CURRENT && gUartRxBuffer[2] == 'R'){
			gUartRxDone = SET;	
		}
#endif
#ifdef MODE_MEASURE_PRESSURE
		if(gUartRxBuffer[1] == ID_PRESSURE && gUartRxBuffer[2] == 'R'){
			gUartRxDone = SET;	
		}
#endif
#ifdef MODE_MEASURE_DC
		if(gUartRxBuffer[1] == ID_DC && gUartRxBuffer[2] == 'R'){
			gUartRxDone = SET;	
		}
#endif
		
	}
}
//===================================================================================

//===================================================================================
void Initial_ADC(void)
{

	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);      // set prescaler to 64	

}

//===================================================================================

//===================================================================================
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
				LED_ON;
				test = 1;
			}
			else{
				LED_OFF;
				test = 0;
			}
		}
		gCounter = 0;
	}
	
	else gCounter++;
}


void read_DCVoltage(void)
{
	// 방화문 전압(1~5V) 읽기
	// 30V -> 5V (1~5V)
	// 10bit : 1024
	float fAdc;
	float fDustVolt;
	
	ADMUX = 1;	// AREF use, select adc1
	
	ADCSRA |= (1 << ADSC);         // start ADC measurement
	while (ADCSRA & (1 << ADSC) ); // wait till conversion complete
	_delay_us(40); // delta time
	
	_delay_us(9680);	// sleep time
	
	fAdc = ADC;
	fDustVolt = fAdc * (5.0 / 1024.0);
	
	gValue_DCVoltage = (unsigned char)(fDustVolt * 6.0); 
	if(gValue_DCVoltage < 1) gValue_DCVoltage = 0;
}

void read_DigitalPressure(void)
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

void read_Current(void)
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

int main(void)
{
	unsigned char tmp1, tmp2;
	
	//_delay_ms(500);
	Initial_GPIO();
	Initial_uart();
	Initial_Timer();
	Initial_ADC();
	
	// RS485_RX_EN;
	RS485_TX_EN;

	gflagConnect = RESET;
	
	sei();

	while(1)
	{
		if(gUartRxDone == SET){
			gState = 1;
		}		
	}


	while (1)	
    {
		switch(gState){
			case 0 :
				if(gUartRxDone == SET){
					gState = 1;
				}
				break;
				
			case 1 :
#ifdef MODE_MEASURE_CURRENT
				if(gUartRxBuffer[1] == ID_CURRENT && gUartRxBuffer[2] == 'R'){
					tmp1 = 0;
					gUartTxBuffer[tmp1++] = 0x02;
					gUartTxBuffer[tmp1++] = ID_CURRENT;
					gUartTxBuffer[tmp1++] = 'A';
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current / 1000))+0x30;
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current / 100))+0x30;
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current / 10))+0x30;
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current % 10))+0x30;
					gUartTxBuffer[tmp1++] = 0x03;
				}
#endif
#ifdef MODE_MEASURE_PRESSURE
				if(gUartRxBuffer[1] == ID_PRESSURE && gUartRxBuffer[2] == 'R'){
					tmp1 = 0;
					gUartTxBuffer[tmp1++] = 0x02;
					gUartTxBuffer[tmp1++] = ID_PRESSURE;
					gUartTxBuffer[tmp1++] = 'A';
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current / 1000))+0x30;
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current / 100))+0x30;
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current / 10))+0x30;
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current % 10))+0x30;
					gUartTxBuffer[tmp1++] = 0x03;
					
				}
#endif
#ifdef MODE_MEASURE_DC
				if(gUartRxBuffer[1] == ID_DC && gUartRxBuffer[2] == 'R'){
					tmp1 = 0;
					gUartTxBuffer[tmp1++] = 0x02;
					gUartTxBuffer[tmp1++] = ID_DC;
					gUartTxBuffer[tmp1++] = 'A';
					gUartTxBuffer[tmp1++] = '0';
					gUartTxBuffer[tmp1++] = '0';
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current / 10))+0x30;
					gUartTxBuffer[tmp1++] = ((unsigned char)(gValue_Current % 10))+0x30;
					gUartTxBuffer[tmp1++] = 0x03;
					
				}
#endif
				RS485_TX_EN;
				_delay_ms(1);
				for(unsigned char tmp2 = 0; tmp2 < tmp1; tmp2++){
					send_uart(gUartTxBuffer[tmp2]);
				}
				RS485_RX_EN;
				gUartRxDone = RESET;
				gState = 0;
				break;
				

		}
	
    }
}


