#ifndef NRF24L01_H
#define NRF24L01_H

typedef unsigned char BYTE;
//typedef unsigned char uchar;
/*--------------------------------------------------------*/
/*--------------------发送端引脚定义表--------------------*/
#define NRF24L01_IRQ_PORT  GPIOD 
#define NRF24L01_IRQ_PIN   GPIO_PIN_4     //IRq    PD4

#define NRF24L01_CE_PORT   GPIOD   
#define NRF24L01_CE_PIN    GPIO_PIN_2      //CE   PD2 

#define NRF24L01_CS_PORT   GPIOD
#define NRF24L01_CS_PIN   GPIO_PIN_3			 //CS  D3

#define NRF24L01_SCK_PORT  GPIOC
#define NRF24L01_SCK_PIN   GPIO_PIN_5      //SCK  PC5

#define NRF24L01_MOSI_PORT  GPIOC
#define NRF24L01_MOSI_PIN   GPIO_PIN_6     //PC6   MOSI

#define NRF24L01_MISO_PORT  GPIOC
#define NRF24L01_MISO_PIN   GPIO_PIN_7     //MISO  PC7


#define TX_ADR_WIDTH    5   // 5 bytes TX address width
#define RX_ADR_WIDTH    5   // 5 bytes RX address width
#define TX_PLOAD_WIDTH  5  // 20 bytes TX payload
#define RX_PLOAD_WIDTH  5  // 20 bytes TX payload

//****************************************************************//
// SPI(nRF24L01) commands
#define READ_REG        0x00  // Define read command to register
#define WRITE_REG       0x20  // Define write command to register
#define RD_RX_PLOAD     0x61  // Define RX payload register address
#define WR_TX_PLOAD     0xA0  // Define TX payload register address
#define FLUSH_TX        0xE1  // Define flush TX register command
#define FLUSH_RX        0xE2  // Define flush RX register command
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NOP             0xFF  // Define No Operation, might be used to read status register


//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG2          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address

//************************一下是自行添加的*****************************//
#define MAX_TX  	0x10  //达到最大发送次数中断
#define TX_OK   	0x20  //TX发送完成中断
#define RX_OK   	0x40  //接收到数据中断

//***************************************************************//
//                   FUNCTION's PROTOTYPES  //
/****************************************************************/
// void SPI_Init(BYTE Mode);     // Init HW or SW SPI
 BYTE SPI_RW(BYTE byte);                                // Single SPI read/write
 BYTE SPI_Read(BYTE reg);                               // Read one byte from nRF24L01
 BYTE SPI_RW_Reg(BYTE reg, BYTE byte);                  // Write one byte to register 'reg'
 BYTE SPI_Write_Buf(BYTE reg, BYTE *pBuf, BYTE bytes);  // Writes multiply bytes to one register
 BYTE SPI_Read_Buf(BYTE reg, BYTE *pBuf, BYTE bytes);   // Read multiply bytes from one register
//*****************************************************************/
void inerDelay_us(unsigned char n);
void init_io(void) ;
void NRF24Init(void);
void RX_Mode(void);
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf);
//unsigned char nRF24L01_RxPacket(unsigned char* rx_buf,unsigned char len);
unsigned char nRF24L01_TxPacket(unsigned char * tx_buf);
unsigned char NRF24L01_Check(void);
void TX_Mode(void);

//extern uchar const TX_ADDRESS[TX_ADR_WIDTH];//TX address
//extern uchar const RX_ADDRESS[RX_ADR_WIDTH];//;RX address

#endif   

