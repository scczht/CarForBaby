#include "stdio.h"
#include "stm8s_gpio.h"
#include "nrf24l01.h"
#include "stm8s.h"
#include "delay.h"

#define     Cycle_50us      500
#define     HIGH_TTL 0
#define     LOW_TTL  1000

unsigned char RxBuf[5]={0x01,0x03,0x00,0x07,0x09};
HcSrc4Info_s HcSr4Cnt;
/*Only test 20220301*/

void Init_Uart2(void)
{
    UART2_DeInit();
    UART2_Init((u32)9600, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, 
               UART2_PARITY_NO, UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);  
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
}

void  Time1_PWM_Init(void)
{
    GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST);
   
    /*******IO����Ϊ���**********/
    CLK->PCKENR1 |= 0X80;  //ʹ��TIME1ʱ��Դ
    TIM1->EGR |= 0X01;     //��ʼ�������������������¼�
    TIM1->RCR = 0X00;      //�رռ�����

    /********����ʱ�ӡ��رռ�����*********/
    TIM1->SMCR = 0X00;
    TIM1->ETR = 0X00;    //ʹ���ڲ�Fmasterʱ�ӣ�16M  
    TIM1->PSCRH = 0X00;
    TIM1->PSCRL = 0X07;  //fCK_PSC/( PSCR[15:0]+1)=2Mhz  0.5us
    TIM1->ARRH = 0x03;
    TIM1->ARRL = 0xE8;   //��װֵ 1000  -->2Khz  
    TIM1->RCR = 0X00;    //�ظ�����0
    TIM1->CR1 = 0X00;    //���ϼ���

    /*******ͨ��1 CH1��� CH1N��ֹ���*******/
    TIM1->CCMR1 = 0x70;    //PWMģʽ2,CC1ͨ������Ϊ���
    TIM1->CCR1H = 0X00;
    TIM1->CCR1L = 0x64;    //ռ�ձ�
    TIM1->CCER1 |= 0X03;   //CC1����Ϊ������͵�ƽ��Ч  
    
    /*******ͨ��2*************/
    TIM1->CCMR2 = 0X70;
    TIM1->CCR2H = 0X00;
    TIM1->CCR2L = 0xFA;
    TIM1->CCER1 |= (1<<5)|(1<<4); 
    /*******ͨ��3*************/
    TIM1->CCMR3 = 0X70;
    TIM1->CCR3H = 0X00;
    TIM1->CCR3L = Cycle_50us>>1;
    TIM1->CCER2 |= (1<<1)|(1<<0);
    /*******ͨ��4*************/
    TIM1->CCMR4 = 0X70;
    TIM1->CCR4H = Cycle_50us>>8;
    TIM1->CCR4L = (0xf0&Cycle_50us);
    TIM1->CCER2 |= (1<<5)|(1<<4);

    TIM1->BKR = 0X80;        //��ֹɲ��    
    TIM1->CR1 |= 0X01;       //ʹ��ʱ��Դ
}

void Time1_PWM_Out(unsigned int pwm1,unsigned int pwm2,unsigned int pwm3,unsigned int pwm4)
{
    //ͨ��1 ռ�ձ�
    //pwm1=1000-pwm1;
    TIM1->CCR1H  =(pwm1>>8);
    TIM1->CCR1L  =(0xFF & pwm1); //ռ�ձ�
    //ͨ��2 ռ��
    //pwm2=1000-pwm2;
    TIM1->CCR2H  =(pwm2>>8);
    TIM1->CCR2L  =(0xFF & pwm2); //ռ�ձ�
    //ͨ��3 ռ�ձ�
    //pwm3=1000-pwm3;
    TIM1->CCR3H  =(pwm3>>8);
    TIM1->CCR3L  =(0xFF & pwm3); //ռ�ձ�
    //ͨ��4 ռ�ձ�
    //pwm4=1000-pwm4;
    TIM1->CCR4H  =(pwm4>>8);
    TIM1->CCR4L  =(0xFF & pwm4);//ռ�ձ�
}

void Timer2_Init(void) 
{ 
    GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST);
	//CLK_ICKR|=0x01;         //�����ڲ�HSI 
	//while(!(CLK_ICKR&0x02));//HSI׼������ 
	//CLK_SWR=0xe1;           //HSIΪ��ʱ��Դ 
	//CLK_CKDIVR=0x00;   //HSI��8��Ƶ=16M 
	#if 1
	CLK->PCKENR1 |= 0X20;  //ʹ��TIME2ʱ��Դ
	//CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2,ENABLE);
	TIM2->PSCR=0x03;       //8��Ƶ 1/2us 
	TIM2->ARRH=0x9c;       //�Զ���װ��ֵ40000=0x9c40  ����20ms���ڶ������
	TIM2->ARRL=0x40; 
	TIM2->CCER1=0x03; //low level,OC1

	TIM2->CCMR1=0x78;  //MODE 
	TIM2->CCR1H=0x0b; 
	TIM2->CCR1L=0xb8; 
	//TIM2->IER=0x00;        //�����ж�ʹ�� 

	TIM2->CR1=0x01;        //enable counter 
	#endif
    //TIM2_TimeBaseInit(8, 40000);
    //TIM2_OC1Init(TIM2_OCMODE_PWM2,TIM2_OUTPUTSTATE_ENABLE,3000,TIM2_OCPOLARITY_LOW);
    //TIM2_Cmd(ENABLE);//����ʱ��

}

void Time2_PWM_Out(unsigned short pwm1)
{
    TIM2->CCR1H=(unsigned char)(pwm1>>8); 
	TIM2->CCR1L=(unsigned char)(0xFF & pwm1); //ռ�ձ�
}

void Send_Byte(uint8_t dat)
{
    while(( UART2_GetFlagStatus(UART2_FLAG_TXE)==RESET));
    UART2_SendData8(dat);   
}
void Send_String(unsigned char *str)
{
    while('\0'!=*str)
    {
        while(UART2_GetFlagStatus(UART2_FLAG_TXE)==RESET);
        UART2_SendData8(*str++);
    }
}
void HcSr4_Start(void)
{
    GPIO_WriteHigh(GPIOE, GPIO_PIN_5);
    //delay_us(1000);
    //delay_ms(20);
    simple_delay_us(20);
    GPIO_WriteLow(GPIOE, GPIO_PIN_5);
    //delay_us(100000);
    //delay_ms(1000);
}
void Init_Timer4(void)
{
    TIM4_TimeBaseInit(TIM4_PRESCALER_64, 250);//4us���� 1ms���
    TIM4_ClearFlag(TIM4_FLAG_UPDATE);
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
    TIM4_Cmd(DISABLE);
}

void main(void)
{
    u8 uStatus;
    uint16_t i=0; 
    unsigned char dd[10];
    
    CLK_HSICmd(ENABLE);//�����ڲ�����ʱ��
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); //�ڲ�ʱ��  16M
    Init_Uart2();
    Time1_PWM_Init();
    Timer2_Init();
    Init_Timer4();
    NRF24Init();
    if (1 == NRF24L01_Check())
    {
        Send_String("NO NRF !!!");
    }
    else
    {
        Send_String("NRF OK!!!");
    }
    Time1_PWM_Out(LOW_TTL, LOW_TTL, LOW_TTL, LOW_TTL); 
    disableInterrupts();
    Init_Timer4();
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);//Trig
    GPIO_WriteLow(GPIOE, GPIO_PIN_5);
    GPIO_Init(GPIOB, GPIO_PIN_1, GPIO_MODE_IN_PU_IT);//Echo
    EXTI_DeInit();
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB,  EXTI_SENSITIVITY_RISE_FALL);
    //ITC_SetSoftwarePriority(ITC_IRQ_PORTB, ITC_PRIORITYLEVEL_2);
    //ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF, ITC_PRIORITYLEVEL_3);
    enableInterrupts();
    delay_ms(1000);

#if 0    
    //Time1_PWM_Out(HIGH_TTL, LOW_TTL, HIGH_TTL, LOW_TTL); 
    //delay_ms(1000);//5s
    //Time1_PWM_Out(LOW_TTL, HIGH_TTL, LOW_TTL, HIGH_TTL);
    //delay_ms(1000);//5s
    
    Time1_PWM_Out(600, LOW_TTL, 600, LOW_TTL);  //����
    delay_ms(3000);//10s
    Time1_PWM_Out(LOW_TTL, 600, LOW_TTL, 600);//ǰ��
    delay_ms(3000);//10s    

    Time1_PWM_Out(LOW_TTL, 600, 600, LOW_TTL);//��ת
    delay_ms(3000);//10s    
    Time1_PWM_Out(600, LOW_TTL, LOW_TTL, 600);//��ת
    delay_ms(3000);//10s  
    Time1_PWM_Out(LOW_TTL, LOW_TTL, LOW_TTL, LOW_TTL); 

    delay_ms(2000);
		
    Time2_PWM_Out(1000);
    delay_ms(2000);
    Time2_PWM_Out(2000);
    delay_ms(2000);
    Time2_PWM_Out(3000);
    delay_ms(2000); 
    Time2_PWM_Out(4000);
    delay_ms(2000);
    Time2_PWM_Out(4800);
    delay_ms(2000);
		
    for (i = 4800; i > 1000; i--)
    {
        Time2_PWM_Out(i);
        delay_ms(1);
    }
    for (i = 1000; i < 4800; i++)
    {
        Time2_PWM_Out(i);
        delay_ms(1);
    }

	delay_ms(10000);	
    Time2_PWM_Out(3000);
    delay_ms(2000);
#endif

    while (1)
    { 
#if 1
        TX_Mode();//TX
        uStatus = nRF24L01_TxPacket(RxBuf);
        RxBuf[3] = RxBuf[0];
        RxBuf[0] = RxBuf[1];
        RxBuf[1] = RxBuf[3];
        RxBuf[3] = 0x00;
        
        if(uStatus ==TX_OK)
        { 
            //Send_String("Send OK!\r\n");
        }else if(uStatus == MAX_TX)
        {                                           
            //Send_String("Send MAX_TX Faild!\r\n");
        }  
        else
        {
            //Send_String("Send  Faild!\r\n");
        }
#endif

#if 1
        HcSr4_Start();
        
        delay_ms(20);
        HcSr4Cnt.Distance = 68UL*(HcSr4Cnt.Timer4OverTimes * 250 + HcSr4Cnt.Timer4Cnt)/1000;
        sprintf(dd,"%ld\r\n",HcSr4Cnt.Distance); 
        Send_String(dd);
        
        if (15 > HcSr4Cnt.Distance)
        {
            //����
            Time1_PWM_Out(600, LOW_TTL, 600, LOW_TTL);  //����
        }
        else if (40 < HcSr4Cnt.Distance)
        {
            Time1_PWM_Out(LOW_TTL, 600, LOW_TTL, 600);//ǰ��
        }
        else
        {
            //Time1_PWM_Out(LOW_TTL, LOW_TTL, LOW_TTL, LOW_TTL); 
        }
        HcSr4Cnt.Timer4Cnt = 0;
        HcSr4Cnt.Timer4OverTimes = 0;
        HcSr4Cnt.Distance = 0;

#endif
    }  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
    while (1)
    {
    }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
