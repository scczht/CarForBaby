#include "delay.h"
#include "stm8s.h"

void simple_delay_us(unsigned short n) 
{
    for(;n>0;n--) 
    { 
        //_asm("nop"); //在STM8里面，16M晶振，_nop_() 延时了 333ns
        //_asm("nop"); 
        //_asm("nop"); 
       // _asm("nop");
        nop();
        nop();
        nop();
    }
}

//---- 毫秒级延时程序----------------------- 
void simple_delay_ms(unsigned short time) 
{ 
    unsigned int i; 
    while(time--)
    {
        for(i=900;i>0;i--) 
            simple_delay_us(1);
    }
 
}

//使用内部16M时，1一个while延时0.8us
void delay_us(unsigned short us)
{
    while(us--)
    {
        nop();
        nop();
    }
}

//使用内部16M时，1ms
void delay_ms(unsigned short ms)
{
	int i;
	while(ms--)
	{
		for(i=0;i<1250;i++);
	}
}


void _delay_us(unsigned short i)
{
    i *= 3; 
    while(--i);
}

void _delay_ms(unsigned short i)
{
    while(i--)
    {
        _delay_us(1000);
    }
}



