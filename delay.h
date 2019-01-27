#ifndef __DELAY__
#define __DELAY__
typedef struct
{
    unsigned short Timer4OverTimes;
    unsigned char  Timer4Cnt;
    unsigned long   Distance;
}HcSrc4Info_s;

void simple_delay_us(unsigned short n);
void simple_delay_ms(unsigned short time);
void _delay_us(unsigned short i);
void _delay_ms(unsigned short i);
void delay_ms(unsigned short ms);
void delay_us(unsigned short us);


#endif
