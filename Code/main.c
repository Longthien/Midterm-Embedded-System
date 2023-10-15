#define F_CPU 2000000UL
#define UART_BAUD 19200
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define BAUDRATE ((F_CPU + (UART_BAUD * 4UL))/(UART_BAUD * 8UL)-1)
void define();

//Module dieu khien dong co
void DC_motor_init();
void CLK();
void ShiftData(uint8_t data);
void DC_motor(uint8_t mode);
void DC_motor_active(char t);

//Module Bluetooth
void uart_init (unsigned int ubrr);
void uart_send_char(unsigned char data);
void uart_send_string(char* str);
uint8_t uart_receive(void);

//Module MQ2 - Cam bien khi gas
void setup_mq2();
void mq2_value();

//Module HCSR04 - Dung dong co khi gap vat can
void HCSR04Init();
uint16_t GetPulseWidth();
void HCSR04Trigger();
uint16_t ultrasonic();

int main(void)
{
    define();
    TCCR0 =(1<< CS00) | (1<<CS01);
    TCNT0 = 130;
    TIMSK = (1<<TOIE0); 
    //sei();
    uart_init(BAUDRATE); // Initialize UART module       
    HCSR04Init();  

    DC_motor_init();
    DC_motor(0);                                           	
    uint8_t t;    
             
    while(1)
    {
        if(ultrasonic() < 6)
            PORTB ^= (1 << PB0);  // Toggle PB0 pin

        mq2_value();
        
        while(UCSRA & (1<<RXC)) // Wait for data to be received
        {
            t = UDR; // Get and return received data from buffer
            DC_motor_active(t);
        }
        t = 0;
        _delay_ms(5);
    }
}

void define(){
    #define C0 PC0  // Chân kết nối với cảm biến MQ5
    #define US_ERROR        -1      
    #define US_NO_OBSTACLE  -2
    #define rs PD0                 
    #define en PD1                 
    #define US_PORT PORTC         
    #define US_PIN  PINC            
    #define US_DDR  DDRC         
    #define US_TRIG_POS PC2        
    #define US_ECHO_POS PC1 

    #define PD_ALL 0xff
    #define PATTERN_STOP 0b00000000
    #define PATTERN_FORWARD  0b10000000
    #define PATTERN_BACKWARD 0b00100000
    #define PATTERN_LEFT 0b00001000
    #define PATTERN_RIGHT 0b00000010
    #define PATTERN_CIRCULAR 0b10101000

}

uint16_t ultrasonic()
{
    HCSR04Trigger(); 
    uint16_t r=GetPulseWidth();                         
    uint16_t distance=(r*0.034/2.0)*8;
    return distance;
}

uint16_t GetPulseWidth()
{
    uint32_t i,result;
    for(i=0;i<600000;i++)
    {
        if(!(US_PIN & (1<<US_ECHO_POS)))
          continue; 
        else
          break;        
    }
    if(i==600000)
       return US_ERROR; 
    TCCR1A=0X00;
    TCCR1B=(1<<CS11);   
    TCNT1=0x00;         
    for(i=0;i<600000;i++)              
    {
        if(US_PIN & (1<<US_ECHO_POS))
        {
            if(TCNT1 > 60000) 
                break;
            else
                continue;  
        }
        else
            break;
    }
    if(i==600000)
       return US_NO_OBSTACLE;   
    result=TCNT1;         
    TCCR1B=0x00;
    if(result > 60000)
       return US_NO_OBSTACLE;   
    else
       return (result>>1);
}
void HCSR04Init()
{
    US_DDR|=(1<<US_TRIG_POS);
}
void HCSR04Trigger()
{   
    US_PORT|=(1<<US_TRIG_POS);      
    _delay_us(15);                  
    US_PORT&=~(1<<US_TRIG_POS); 
}

void mq2_value()
{
    uint16_t adc_value;
    setup_mq2();
  
    // Đọc giá trị ADC từ chân kết nối với MQ5
    ADMUX = (ADMUX & 0xF0) | (C0 & 0x0F);  // Chọn kênh ADC
    ADCSRA |= (1 << ADSC);  // Bắt đầu chuyển đổi ADC

    // Chờ cho quá trình chuyển đổi ADC hoàn thành
    while (ADCSRA & (1 << ADSC));

    // Đọc giá trị ADC
    adc_value = ADC ;
    uart_send_char('G');
    
    uint8_t digit1, digit2, digit3, digit4;
    digit1 = (int)adc_value / 1000;
    digit2 = ((int)adc_value / 100) % 10;
    digit3 = ((int)adc_value / 10) % 10;
    digit4 = (int)adc_value % 10;
    uart_send_char(digit1+48);
    uart_send_char(digit2+48);
    uart_send_char(digit3+48);
    uart_send_char(digit4+48);
    
    uart_send_char('-');
    uart_send_char('-');
    _delay_ms(100); 
}

void setup_mq2() 
{
  // Cấu hình ADC
  ADMUX |= (1 << REFS0);  // Chọn độ chính xác của ADC (AVCC with external capacitor at AREF pin)
  ADCSRA |= (1 << ADEN);  // Bật ADC
}

uint8_t uart_receive(void)
{
    while (!(UCSRA & (1<<RXC))); // Wait for data to be received
    return UDR; // Get and return received data from buffer
}
void uart_send_string(char* str)
{
    while (*str)
        uart_send_char(*str++);
}

void uart_send_char(unsigned char data)
{
    while (!(UCSRA & (1<<UDRE))); // Wait for empty transmit buffer
    UDR = data; // Send data
}

void uart_init (unsigned int ubrr)
{
    // the gcc compiler is able to update UBRRH and UBRRL on its own
    UBRRH = (unsigned char)(ubrr>>8);
    UBRRL = (unsigned char)ubrr;                        //
    UCSRA = (1 << U2X);                         // enable u2x mode
    UCSRB = (1<<TXEN)|(1<<RXEN);                // enable receiver and transmitter
    UCSRC = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);   // 8bit data format
}

void DC_motor(uint8_t mode)
{
    PORTD&= ~(1<<PD7);
    //_delay_ms(5);
    uint8_t data=0x00;
    switch (mode)
    {
        case 0:
        data = PATTERN_STOP;
        break;
        case 1:
        data = PATTERN_FORWARD;
        break;
        case 2:
        data = PATTERN_BACKWARD;
        break;
        case 3:
        data = PATTERN_LEFT;
        break;
        case 4:
        data = PATTERN_RIGHT;
        break;
        case 5:
        data = PATTERN_CIRCULAR;
        break;
    }
    ShiftData(data);
    PORTD|=(1<<PD7);
   // _delay_ms(1000);        

}

void ShiftData(uint8_t data) 
{  
    
    uint8_t i;
    for (i = 0; i < 8; i++) 
    {

        PORTD &= ~(1 << PD5); 
        _delay_us(1);   
        PORTD |= ((data >> i) & 1) << PD5;
        _delay_us(1);
        CLK();
    }
}
void DC_motor_init()
{
    DDRB |= (1 << PB0);  // Set PB0 pin as output  
    DDRD |=  (1 << PB1)|(1 << PB2) | (1 << PB3)|(1 << PB4); 
    DDRD |= (1 << PD5)|(1 << PD6) | (1 << PD7);
}
void CLK()
{
    PORTD |= (1 << PD6);
    _delay_us(1);
    PORTD &= ~(1 << PD6);
}
void DC_motor_active(char t){
    if(t == 'u')
            {
                DC_motor(1);
                //_delay_ms(1000);
                //DC_motor(0);
            }
            else if(t == 'r')
            {
                DC_motor(2);
                //_delay_ms(1000);
                //DC_motor(0);
            }
            else if(t == 'l')
            {
                DC_motor(3);
                //_delay_ms(1000);
                //DC_motor(0);
            }
            else if(t == 'd')
            {
                DC_motor(4);
                //_delay_ms(1000);
                //DC_motor(0);
            }
            else
            {
                DC_motor(0);
            }
}
