
//#include "STM8S105K.h"


#include "stm8s.h"
#include "nrf24l01.h"
#include "delay.h"



unsigned char  TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10,0x01}; // Define a static TX address
unsigned char RX_ADDRESS[RX_ADR_WIDTH]  = {0x34,0x43,0x10,0x10,0x01}; // Define a static RX address
unsigned char stat;
unsigned char SPI_RW(unsigned char byte)
{   
    //unsigned char  retry;
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    SPI_SendData(byte);
    /*!< Wait until a data is received */
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
    {
        //retry++;
        //if(retry>200)return 0;
    }
    /*!< Get the received data */
    byte = SPI_ReceiveData();
    return(byte);  
}
unsigned char  SPI_RW_Reg(BYTE reg, BYTE value)
{
    unsigned char status;
    
    GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                // CSN low, init SPI transaction
    status = SPI_RW(reg);      // select register
    simple_delay_us(10);
    SPI_RW(value);             // ..and write value to it..
    GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                   // CSN high again
    return(status);            // return nRF24L01 status byte
}
unsigned char SPI_Read(unsigned char reg)
{
    BYTE reg_val;
    GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                 // CSN low, initialize SPI communication...
    //NRF24L01_CS_LOW() ;
    SPI_RW(reg);            // Select register to read from..
    simple_delay_us(10);
    reg_val = SPI_RW(0);    // ..then read registervalue
    GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);           // CSN high, terminate SPI communication
    //NRF24L01_CS_HIGH() ;
    return(reg_val);        // return register value
}
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
    unsigned char status,byte_ctr;
           
    GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);
    status = SPI_RW(reg);        
    for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) 
        SPI_RW(*pBuf++);
    GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);
    return(status);         
}

void NRF24_GPIOInit(void)
{
    //无线模块的IO口初始化 
    //MISO 读取数据  浮动输入，没有外部中断
    GPIO_Init(NRF24L01_CE_PORT, (GPIO_Pin_TypeDef)(NRF24L01_CE_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//ce 
    GPIO_Init(NRF24L01_CS_PORT, (GPIO_Pin_TypeDef)(NRF24L01_CS_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//cs
    GPIO_Init(NRF24L01_SCK_PORT, (GPIO_Pin_TypeDef)(NRF24L01_SCK_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//sck
    GPIO_Init(NRF24L01_MOSI_PORT, (GPIO_Pin_TypeDef)(NRF24L01_MOSI_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//MOSI
    GPIO_Init(NRF24L01_MISO_PORT, (GPIO_Pin_TypeDef)(NRF24L01_MISO_PIN), GPIO_MODE_IN_FL_NO_IT);//MISO
    GPIO_Init(NRF24L01_IRQ_PORT, (GPIO_Pin_TypeDef)(NRF24L01_IRQ_PIN ), GPIO_MODE_IN_FL_NO_IT); //IRQ
    simple_delay_us(125);
    GPIO_WriteHigh(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
    GPIO_WriteHigh(NRF24L01_CS_PORT, NRF24L01_CS_PIN);
    GPIO_WriteLow(NRF24L01_SCK_PORT, NRF24L01_SCK_PIN);
    simple_delay_us(125);
}
void NRF24Init(void)
{
    NRF24_GPIOInit();
    SPI_DeInit();
    /* Initialize SPI in Slave mode  */
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW,
                    SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT,(uint8_t)0x07);  
    /* Enable the SPI*/
    SPI_Cmd(ENABLE);
    simple_delay_us(125);
    //RX_Mode();
} 
void RX_Mode(void)
{  
    GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
    SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, RX_ADR_WIDTH); // Use the same address on the RX device as the TX device
    
    SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      //频道0自动   ACK应答允许  
    SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  //允许接收地址只有频道0， 
    SPI_RW_Reg(WRITE_REG + RF_CH, 40);        //   设置信道工作为2.4GHZ，收发必须一致
    //SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); 
    SPI_RW_Reg(WRITE_REG + RX_PW_P0, 5); //设置接收数据长度，本次设置为32字节
    SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07); // 设置发射速率为1MHZ，发射功率为最大值0dB 
    /*设置发射速率250kbps，位5和位3配合使用 位5位3=00:1Mbps 01:2Mbps 10:250kbps 11:不使用*/
    //SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x27); // 设置发射速率为250，发射功率为最大值0dB  该型号不支持
    SPI_RW_Reg(WRITE_REG + CONFIG2, 0x0f);    //power up  1: PRX
    //CE = 1; 
    GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
    simple_delay_us(100);
}



unsigned char SPI_Read_Buf(BYTE reg, BYTE *pBuf, BYTE bytes)
{
    unsigned char status,byte_ctr;
    
    GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);// Set CSN low, init SPI tranaction
    status = SPI_RW(reg);               // Select register to write to and read status byte
    //printf("\n\r %d \n\r",status);
    for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
        pBuf[byte_ctr] = SPI_RW(0);     
    
    GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);

    return(status);                    // return nRF24L01 status byte
}

/**************************************************/
//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，相同时返回值:0，表示在位;否则返回1，表示不在位 
unsigned char NRF24L01_Check(void)
{
    unsigned char buf[5]={0xa5,0xa5,0xa5,0xa5,0xa5};
    unsigned char i;       
    SPI_Write_Buf(WRITE_REG+TX_ADDR,buf,5);   //写入5个字节的地址.   
    SPI_Read_Buf(TX_ADDR,buf,5);              //读出写入的地址    
    for(i=0;i<5;i++)
      if(buf[i]!=0xa5)
          break;                     
    if(i!=5)
      return 1;                               //NRF24L01不在位 
    return 0;                                 //NRF24L01在位
    
#if 0    
    unsigned char buf1[5]={0xff,0xff,0xff,0xff,0xff};
    unsigned char i; 
    //SPI_Write_Buf(WRITE_REG+RF_SETUP,buf1,5);//写入5个字节的地址.
    SPI_Read_Buf(RF_SETUP,buf1,5); 


    SPI_Read_Buf(TX_ADDR,buf1,5); 
    if(0x34==buf1[0]&&0x01==buf1[4])
        return 0;
    else
        return 1;
#endif    
}       
/*************************************************************/

/*************************************************************/
//  接收数据包
/**************************************************************/
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
    unsigned char revale=0;
    //unsigned char buf[1]={0xaa};
    //RX_Mode();
    //SPI_Write_Buf(WRITE_REG+STATUS,buf,1);
    stat=SPI_Read(STATUS);  // read register STATUS's value
    SPI_RW_Reg(WRITE_REG+STATUS,stat);
    if(stat&0x40)               // if receive data ready (RX_DR) interrupt
    { 
        SPI_Read_Buf(RD_RX_PLOAD,rx_buf,5);// read receive payload from RX_FIFO buffer      
        SPI_RW_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
        revale =1;//we have receive data    
    }
    return revale;
}

//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void TX_Mode(void)
{														 
	GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);    
  	SPI_Write_Buf(WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	SPI_Write_Buf(WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	SPI_RW_Reg(WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	SPI_RW_Reg(WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	SPI_RW_Reg(WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	SPI_RW_Reg(WRITE_REG+RF_CH,40);       //设置RF通道为40
    SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07); // 设置发射速率为1MHZ，发射功率为最大值0dB 
    /*设置发射速率250kbps，位5和位3配合使用 位5位3=00:1Mbps 01:2Mbps 10:250kbps 11:不使用*/
    //SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x27); // 设置发射速率为250，发射功率为最大值0dB  该型号不支持

    SPI_RW_Reg(WRITE_REG+CONFIG2,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);//CE为高,10us后启动发送
	simple_delay_us(10);
}


//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);          //使能SPI传输
  	status = SPI_RW(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
        SPI_RW(*pBuf++); //写入数据  
  	GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);       //关闭SPI传输
  	return status;          //返回读到的状态值
}	

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
u8 nRF24L01_TxPacket(u8 *txbuf)
{
	unsigned char sta;
  
	GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  
 	GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);//启动发送	   
	//while(NRF24L01_IRQ!=0);//等待发送完成
	while(GPIO_ReadInputPin( GPIOD, GPIO_PIN_4) != 0);//等待发送完成
	sta=SPI_Read(STATUS);  //读取状态寄存器的值	   
	SPI_RW_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		SPI_RW_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
























