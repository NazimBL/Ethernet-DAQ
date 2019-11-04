/*
 Copyright:
 DeadLine, Bellabaci Nazim.
*/
 
#include "Wizchip_Conf.h"
#include "W5500.h"
#include "Socket.h"

// Socket & Port number definition
#define SOCK_ID_TCP       0
#define SOCK_ID_UDP       1
#define PORT_UDP          10001
#define DATA_BUF_SIZE     2048
#define spi_T 100


sbit Spi_CLK at GPIOA_ODR.B6;
sbit Spi_SDI at GPIOB_ODR.B0;
sbit Spi_CS at GPIOB_ODR.B1;

sbit SoftSpi_CLK at GPIOA_ODR.B6;
sbit SoftSpi_SDI at GPIOB_ODR.B0;
sbit SoftSpi_SDO at GPIOB_ODR.B5;

uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t sendDATABUF[DATA_BUF_SIZE];
uint8_t destip[4];
uint8_t i=0,j=0;
uint8_t tag=0;
uint8_t connected=0;

uint16_t destport;

void UDP_Send(uint8_t sendBUF[],uint16_t size);
void writeDAC(unsigned dac,unsigned char channel);
void outputDAC();
void DAC_Init();
void ADCSetup();
void TimerSetup();
uint8_t fetchInput();
void fetchOutput();
char SPI_transfer(char data0);

unsigned char lowB,highB;
unsigned adc_input;

volatile wiz_NetInfo gWIZNETINFO =
{
  {0x00, 0x14, 0xA3, 0x72, 0x17, 0x3F},    // Source Mac Address
  {192, 168, 197, 52},                    // LOCAL RECEIVER ADDRESS
  {255, 255, 0, 0},                      // Subnet Mask
  {0, 0,  0, 0},                       // Gateway IP Address
  {0, 0,  0, 0},                      // DNS server IP Address
  NETINFO_STATIC
  
 };

volatile wiz_PhyConf phyConf =
{
  PHY_CONFBY_HW,       // PHY_CONFBY_SW
  PHY_MODE_MANUAL,     // PHY_MODE_AUTONEGO
  PHY_SPEED_10,        // PHY_SPEED_100
  PHY_DUPLEX_FULL,     // PHY_DUPLEX_HALF
};

volatile wiz_NetInfo pnetinfo;

// brief Call back function for WIZCHIP select.
void CB_ChipSelect(void)
{
GPIOB_ODRbits.ODR12= 0;
}

// brief Call back function for WIZCHIP deselect.
void CB_ChipDeselect(void)
{
 GPIOB_ODRbits.ODR12= 1;
}

// brief Callback function to read byte usig SPI.
uint8_t CB_SpiRead(void)
{
    return SPI_Read(0xAA);
}

// brief Callback function to write byte usig SPI.
void CB_SpiWrite(uint8_t wb)
{
    SPI_Write(wb);
}
// brief Handle UDP socket state.
void UDP_Server(void)
{
    int32_t  ret;
    uint16_t size, sentsize;

    // Get status of socket
    switch(getSn_SR(SOCK_ID_UDP))
    {
        // Socket is opened in UDP mode
        case SOCK_UDP:
        {
             connected=1;
            // Get received data size
            if((size = getSn_RX_RSR(SOCK_ID_UDP)) > 0)
            {
                // Cut data to size of data buffer
                if(size > DATA_BUF_SIZE)size = DATA_BUF_SIZE;
                // Get received data
                ret = recvfrom(SOCK_ID_UDP, gDATABUF, size, destip, (uint16_t*)&destport);
                // Check for error
                if(ret <= 0) return;
                /*
                // Send echo to remote
                 size=(uint16_t)ret;
                 size=(uint16_t)2;
                 sentsize = 0;
                while(sentsize != size)
                {
                    ret = sendto(SOCK_ID_UDP, sendDATABUF + sentsize, size - sentsize, destip, destport);
                    if(ret < 0)
                    {
                        return;
                    }
                    // Update number of sent bytes
                    sentsize += ret;
                } */
            }
            break;
        }
        // Socket is not opened
        case SOCK_CLOSED:
        {
            // Open UDP socket
            connected=0;
            if((ret=socket(SOCK_ID_UDP, Sn_MR_UDP, PORT_UDP, 0x00)) != SOCK_ID_UDP)
            {
                return;
            }
            break;
        }
        default :
        {
           connected=0;
           break;
        }
    }
}
// brief Initialize modules
static void W5500_Init(void)
{
    // Set Tx and Rx buffer size to 2KB
    uint8_t buffsize[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    
    GPIO_Digital_Output(&GPIOB_BASE, _GPIO_PINMASK_13| _GPIO_PINMASK_15); //SCK|Mosi
    GPIO_Digital_Input(&GPIOB_BASE, _GPIO_PINMASK_14);//Miso
    GPIO_Digital_Output(&GPIOB_BASE, _GPIO_PINMASK_12);// Set CS# pin as Output
    GPIO_Digital_Output(&GPIOA_BASE, _GPIO_PINMASK_8);//W5500 Rst pin
    
    CB_ChipDeselect();
      
    //SPI2RST_bit=1;   
    SPI2EN_bit=1;
    //SPE_SPI2_CR1_bit=1;  
    SPI2_Init_Advanced(_SPI_FPCLK_DIV4, _SPI_MASTER | _SPI_8_BIT |
                       _SPI_CLK_IDLE_LOW | _SPI_FIRST_CLK_EDGE_TRANSITION |
                        _SPI_MSB_FIRST | _SPI_SS_DISABLE | _SPI_SSM_ENABLE | _SPI_SSI_1,
                       &_GPIO_MODULE_SPI2_PB13_14_15);
                               
    // Reset module
    GPIOA_ODRbits.ODR8= 0;
    Delay_ms(1);
    GPIOA_ODRbits.ODR8= 1;
    Delay_ms(1);
                            
    // Wizchip initialize
    wizchip_init(buffsize, buffsize, 0, 0, CB_ChipSelect, CB_ChipDeselect, 0, 0, CB_SpiRead, CB_SpiWrite);

    // Wizchip netconf
    ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
    ctlwizchip(CW_SET_PHYCONF, (void*) &phyConf);    
}

void Timer3_interrupt() iv IVT_INT_TIM3
{
  TIM3_SR.UIF = 0;
  /*
  // test dummy values
  sendDATABUF[0]=0xF5;
  sendDATABUF[1]=0x10;
  if(connected==1)UDP_Send(sendDATABUF,(uint16_t)2);
  */
  //GPIOC_ODRbits.ODR13=~GPIOC_ODRbits.ODR13;
  tag=1;     
}

void main()
{
    ADCSetup();
    //set digital inputs
    GPIO_Digital_Input(&GPIOA_BASE, _GPIO_PINMASK_12 | _GPIO_PINMASK_15);
    GPIO_Digital_Input(&GPIOB_BASE, _GPIO_PINMASK_3 | _GPIO_PINMASK_4 |_GPIO_PINMASK_5| _GPIO_PINMASK_6);    
     //digital outputs
    GPIO_Digital_Output(&GPIOC_BASE,_GPIO_PINMASK_13 |_GPIO_PINMASK_14 | _GPIO_PINMASK_15);
    GPIO_Digital_Output(&GPIOB_BASE,_GPIO_PINMASK_7 |_GPIO_PINMASK_8 | _GPIO_PINMASK_9);
    
    DAC_Init();
    W5500_Init();
    TimerSetup();
   
    wizchip_getnetinfo(&pnetinfo);

    while(1)
    {          
          
           UDP_Server();
          
          writeDAC(500,0);
          writeDAC(0,1);
          
          if(connected==1){
            //outputDAC();
            fetchOutput();
            connected=0;
           }
           
          //read adc ports
          for(i=0;i<6;i++){
          
          adc_input=ADC1_Get_Sample(i);
          lowB=adc_input & 0xFF;
          highB=adc_input >> 8;

          sendDATABUF[j++]=lowB;
          sendDATABUF[j++]=highB;  
          }
          //read digital input
          sendDATABUF[12]=fetchInput();
          UDP_Send(sendDATABUF,13);
          //13 = 6 ADC (12bytes) + 1 digital byte = 13, size of the data to be sent
     }                            
}

void DAC_Init(){

GPIO_Digital_Output(&GPIOB_BASE, _GPIO_PINMASK_1| _GPIO_PINMASK_0); //SDI  CS
GPIO_Digital_Output(&GPIOA_BASE, _GPIO_PINMASK_6);// SCK
 
 //Soft_SPI_Init();
 Spi_CLK = 1;
 Spi_CS=1;
 
}
void outputDAC(){

unsigned adc1,adc2;
adc1=gDATABUF[0]+(256*gDATABUF[1]);
adc2=gDATABUF[2]+(256*gDATABUF[3]);

writeDAC(adc1,0);
writeDAC(adc2,1);
}

//DAC interface function
void writeDAC(unsigned dac,unsigned char channel){
    
        char lowB,highB;
        lowB=dac & 0xFF;
        highB=dac >> 8;
        //add command word
        highB+=16;
        //highB+=128;
        //if(channel==1)highB+=128;
       highB+=(channel*128);
        
       Spi_CS=0;
        SPI_transfer(highB);
        SPI_transfer(lowB);
       //Soft_SPI_Write(highB);
       //Soft_SPI_Write(lowB);
       Spi_CS=1;
}

uint8_t fetchInput(){

    uint8_t output=0;
    //A12 is LSB
    output+=GPIOA_IDRbits.IDR12;
    output+=(2*GPIOA_IDRbits.IDR15);
    output+=(4*GPIOB_IDRbits.IDR3);
    output+=(8*GPIOB_IDRbits.IDR4);
    output+=(16*GPIOB_IDRbits.IDR5);
    output+=(32*GPIOB_IDRbits.IDR6);   
    return output;

}

void fetchOutput(){

uint8_t output=gDATABUF[4];
//B9 is LSB
GPIOB_ODRbits.ODR7= output & 0x01;
GPIOB_ODRbits.ODR8= (output & 0x02) >> 1;
GPIOB_ODRbits.ODR9= (output & 0x04) >> 2;

GPIOC_ODRbits.ODR15= (output & 0x08) >> 3;
GPIOC_ODRbits.ODR14= (output & 0x10) >> 4;
GPIOC_ODRbits.ODR13= (output & 0x20) >> 5;
}

void TimerSetup(){

RCC_APB1ENR.TIM3EN = 1;       // Enable clock gating for timer module 2
TIM3_CR1.CEN = 0;             // Disable timer
  // 1 hz == one sec
TIM3_PSC = 1098;              // Set timer prescaler.
TIM3_ARR = 65514;
NVIC_IntEnable(IVT_INT_TIM3); // Enable timer interrupt
TIM3_DIER.UIE = 1;            // Update interrupt enable
TIM3_CR1.CEN = 1;             // Enable timer
}
void UDP_Send(uint8_t sendBUF[],uint16_t size){

uint16_t sentsize=0;
uint8_t ret=0;
uint8_t ip[4]={255,255,255,255};
uint16_t port=10000;
           while(sentsize <= size)
                {
                    //ret = sendto(SOCK_ID_UDP, sendBUF + sentsize, size - sentsize, destip, destport);
                    ret = sendto(SOCK_ID_UDP, sendBUF + sentsize, size - sentsize, ip, port);
                    
                    if(ret < 0)return;                   
                    // Update number of sent bytes
                    sentsize += ret;
                }
}
void ADCSetup(){

    ADC_Set_Input_Channel(_ADC_CHANNEL_0);
    ADC_Set_Input_Channel(_ADC_CHANNEL_1);
    ADC_Set_Input_Channel(_ADC_CHANNEL_2);
    ADC_Set_Input_Channel(_ADC_CHANNEL_3);
    ADC_Set_Input_Channel(_ADC_CHANNEL_4);
    ADC_Set_Input_Channel(_ADC_CHANNEL_5);
   
    ADC1_Init(); 
}
char SPI_transfer(char data0)
{
char counter;
  for(counter = 8; counter>0; counter--)
  {
    if (data0 & 0x80)Spi_SDI = 1;
    else Spi_SDI = 0;
    data0 <<= 1;
  
    Spi_CLK = 1;     
    Delay_us(spi_T);
    Spi_CLK = 0;     
    Delay_us(spi_T);
  }
  return(data0);
}