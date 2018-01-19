#include "C1101.h"
#include "stm32f1xx_hal.h"
#include "Delay.h"
const uint8_t paTable[8] = {0x60, 0, 0, 0, 0, 0, 0, 0};
uint8_t val;

PACKET packet;  
uint8_t InitError;

extern SPI_HandleTypeDef hspi1;




void radioSendByte(unsigned char Data)
{
  //if(hspi1.Init.DataSize != SPI_DATASIZE_8BIT)
  //{
  //  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  //  HAL_SPI_Init(&hspi1);
 // }
  cc1101_Select();                    // Deselect CC1101
  HAL_SPI_Transmit(&hspi1, &Data,1,0x1);
  //cc1101_Deselect();                    // Deselect CC1101
 // SPI1->DR = (uint8_t)Data;
  //while(!(SPI1->SR & SPI_SR_TXE));
  //while((SPI1->SR & SPI_SR_BSY));
}

uint8_t getByteFromRadio(void)
{
  uint8_t Data;
  HAL_SPI_Receive(&hspi1,&Data,1,0x1);
  return Data;
}

void writeBurstReg(uint8_t regAddr, uint8_t* buffer, uint8_t len)
{
  uint8_t addr, i;
  
  addr = regAddr | WRITE_BURST;         // Enable burst transfer
  cc1101_Select();                      // Select CC1101
  wait_miso_cc1101();                   // Wait until MISO goes low
  radioSendByte(addr);                  // Send register address
  
  for(i=0 ; i<len ; i++)
  radioSendByte(buffer[i]);           // Send value

}



void setCarrierFreq(uint8_t freq)
{
  write_register_cc1101(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2);
  write_register_cc1101(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1);
  write_register_cc1101(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0);
}



void setChannel(uint8_t chnl) 
{
    write_register_cc1101(CC1101_CHANNR,  chnl);
}

void setDevAddress(uint8_t addr) 
{
    write_register_cc1101(CC1101_ADDR, addr);
}


void setSyncWord(uint8_t *sync)                    // ?????????
{
    write_register_cc1101(CC1101_SYNC1, sync[0]);
    write_register_cc1101(CC1101_SYNC0, sync[1]);
}


void setDefaultRegs(void)
{
  uint8_t defSyncWrd[] = {CC1101_DEFVAL_SYNC1, CC1101_DEFVAL_SYNC0};

   write_register_cc1101(CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2);
   

   write_register_cc1101(CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1);
  

   write_register_cc1101(CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0);

   
   write_register_cc1101(CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR);

  
   write_register_cc1101(CC1101_PKTLEN,   CC1101_DEFVAL_PKTLEN);
 
  
   write_register_cc1101(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);

  
   write_register_cc1101(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);


  // Set default synchronization word
   setSyncWord(defSyncWrd);

  // Set default device address
   setDevAddress(CC1101_DEFVAL_ADDR);
  // Set default frequency channel
   setChannel(CC1101_DEFVAL_CHANNR);
  
   write_register_cc1101(CC1101_FSCTRL1,  CC1101_DEFVAL_FSCTRL1);

    
   write_register_cc1101(CC1101_FSCTRL0,  CC1101_DEFVAL_FSCTRL0);


  // Set default carrier frequency = 868 MHz
   setCarrierFreq(CFREQ_868);
  

   write_register_cc1101(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4);

   
   write_register_cc1101(CC1101_MDMCFG3,  CC1101_DEFVAL_MDMCFG3);

   
   write_register_cc1101(CC1101_MDMCFG2,  CC1101_DEFVAL_MDMCFG2);

   
   write_register_cc1101(CC1101_MDMCFG1,  CC1101_DEFVAL_MDMCFG1);

   
   write_register_cc1101(CC1101_MDMCFG0,  CC1101_DEFVAL_MDMCFG0);

   
   write_register_cc1101(CC1101_DEVIATN,  CC1101_DEFVAL_DEVIATN);

   
   write_register_cc1101(CC1101_MCSM2,    CC1101_DEFVAL_MCSM2);

   
   write_register_cc1101(CC1101_MCSM1,    CC1101_DEFVAL_MCSM1);
 
   
   write_register_cc1101(CC1101_MCSM0,    CC1101_DEFVAL_MCSM0);

   
   write_register_cc1101(CC1101_FOCCFG,   CC1101_DEFVAL_FOCCFG);

   
   write_register_cc1101(CC1101_BSCFG,    CC1101_DEFVAL_BSCFG);
 
   
   write_register_cc1101(CC1101_AGCCTRL2,  CC1101_DEFVAL_AGCCTRL2);

   
   write_register_cc1101(CC1101_AGCCTRL1,  CC1101_DEFVAL_AGCCTRL1);

   
   write_register_cc1101(CC1101_AGCCTRL0,  CC1101_DEFVAL_AGCCTRL0);

   
   write_register_cc1101(CC1101_WOREVT1,  CC1101_DEFVAL_WOREVT1);
 
   
   write_register_cc1101(CC1101_WOREVT0,  CC1101_DEFVAL_WOREVT0);
 
   
   write_register_cc1101(CC1101_WORCTRL,  CC1101_DEFVAL_WORCTRL);
  
   
   write_register_cc1101(CC1101_FREND1,  CC1101_DEFVAL_FREND1);

   
   write_register_cc1101(CC1101_FREND0,  CC1101_DEFVAL_FREND0);
   
   
   write_register_cc1101(CC1101_FSCAL3,  CC1101_DEFVAL_FSCAL3);

   
   write_register_cc1101(CC1101_FSCAL2,  CC1101_DEFVAL_FSCAL2);

   
   write_register_cc1101(CC1101_FSCAL1,  CC1101_DEFVAL_FSCAL1);

   
   write_register_cc1101(CC1101_FSCAL0,  CC1101_DEFVAL_FSCAL0);

   
   write_register_cc1101(CC1101_RCCTRL1,  CC1101_DEFVAL_RCCTRL1);

   
   write_register_cc1101(CC1101_RCCTRL0,  CC1101_DEFVAL_RCCTRL0);

   
   write_register_cc1101(CC1101_FSTEST,  CC1101_DEFVAL_FSTEST);

   
   write_register_cc1101(CC1101_PTEST,  CC1101_DEFVAL_PTEST);

   
   write_register_cc1101(CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST);

   
   write_register_cc1101(CC1101_TEST2,  CC1101_DEFVAL_TEST2);

   
   write_register_cc1101(CC1101_TEST1,  CC1101_DEFVAL_TEST1);

   
   write_register_cc1101(CC1101_TEST0,  CC1101_DEFVAL_TEST0); 

   
}


void write_register_cc1101(uint8_t adr, uint8_t data)
{
  cc1101_Select();
  wait_miso_cc1101();
  radioSendByte(adr);

  radioSendByte(data);
  cc1101_Deselect();
  
 // delay_us(10);
    
  uint8_t val;
  val=readReg_C1101(adr, READ_SINGLE);
  if(!(data==val))
  { InitError=1; }
  
}
    

/*
void send_spi_byte(uint8_t data)
{
  SPI1->DR = (uint8_t)data;
  while(!(SPI1->SR & SPI_SR_TXE));
  while((SPI1->SR & SPI_SR_BSY));
 // delay_us(6);
}
*/


uint8_t CC1101_init(void)
{
  InitError = 0;
  
  reset_cc1101();
  writeBurstReg(CC1101_PATABLE, (uint8_t*)paTable, 8);
  
  
   uint8_t test_patable_table[8];
   uint8_t addr;
  addr = CC1101_PATABLE | READ_BURST;
  cc1101_Select();                             // Select CC1101
  wait_miso_cc1101();                          // Wait until MISO goes low
  radioSendByte(addr);                       // Send register address 
  test_patable_table[0] = SPI1->DR;
  for(int i=0;i<8;i++)
  {
    //radioSendByte(0x00); 
    //test_patable_table[i]=SPI1->DR;
    test_patable_table[i] = getByteFromRadio();

  }

  cc1101_Deselect();                       // Deselect CC1101
  
  for(int i=0;i<8;i++)
  { 
    
    if(!(test_patable_table[i]==paTable[i]))
    {
      InitError=1;
    }
  }
  
  
  
   
  return InitError;
}



void reset_cc1101(void)
{
  cc1101_Deselect();                    // Deselect CC1101
  delay_us(5);
  cc1101_Select();                      // Select CC1101
  delay_us(10);
  cc1101_Deselect();                    // Deselect CC1101
  delay_ms(10);
 
 
  cc1101_Select();                      // Select CC1101
 
  
  wait_miso_cc1101();                   // Wait until MISO goes low
  radioSendByte(CC1101_SRES);           // Send reset command strobe
  wait_miso_cc1101();                          // Wait until MISO goes low

  cc1101_Deselect();                    // Deselect CC1101

  setDefaultRegs();                     // Reconfigure CC1101
  
  //setRegsFromEeprom();                // Пользовательские настройки 
  
}



void cmdStrobe_C1101(uint8_t cmd) 
{
 cc1101_Select();                              // Select CC1101
  wait_miso_cc1101();                          // Wait until MISO goes low
  radioSendByte(cmd);                // Send strobe command
   cc1101_Deselect();                            // Deselect CC1101 
}

uint8_t readReg_C1101(uint8_t regAddr, uint8_t regType) 
{
  uint8_t addr,val;
  
  addr = regAddr | regType;
  cc1101_Select();                       // Select CC1101
  wait_miso_cc1101();                          // Wait until MISO goes low
  radioSendByte(addr);                       // Send register address 
  val=SPI1->DR;
  radioSendByte(0x00); 
  val=SPI1->DR;

   cc1101_Deselect();                       // Deselect CC1101

  return val;
}


void reset_C1101(void) 
{
  cc1101_Deselect();                            // Deselect CC1101
   delay_us(5);
  cc1101_Select();                     // Select CC1101
   delay_us(10);
  cc1101_Deselect();                    // Deselect CC1101
  delay_us(41);
 cc1101_Select();                     // Select CC1101

  wait_miso_cc1101();                          // Wait until MISO goes low
  radioSendByte(CC1101_SRES);                // Send reset command strobe
  wait_miso_cc1101();;                          // Wait until MISO goes low

 cc1101_Deselect();                   // Deselect CC1101

 setDefaultRegs_C1101();                     // Reconfigure CC1101
  //setRegsFromEeprom();                  // Take user settings from EEPROM
}




void writeBurstReg_C1101(uint8_t regAddr, uint8_t* buffer, uint8_t len)
{
  uint8_t addr, i;
  
  addr = regAddr | WRITE_BURST;         // Enable burst transfer
  cc1101_Select();                      // Select CC1101
  wait_miso_cc1101();                          // Wait until MISO goes low
  radioSendByte(addr);                       // Send register address
  
  for(i=0 ; i<len ; i++)
    radioSendByte(buffer[i]);                // Send value

  cc1101_Deselect();                    // Deselect CC1101 
 
}

    
  void disableAddressCheck(void)
  {
    write_register_cc1101(CC1101_PKTCTRL1, 0x04); 
  }

  void wait_miso_cc1101(void) 
  { 
    while((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))); 
  }


  void cc1101_Deselect(void) 
  { 
    HAL_GPIO_WritePin(RADIO_CS_GPIO_Port, RADIO_CS_Pin,GPIO_PIN_SET);
    asm("NOP");
   // delay_us(5);
  }
  
  
  void cc1101_Select(void) 
  {
    HAL_GPIO_WritePin(RADIO_CS_GPIO_Port, RADIO_CS_Pin,GPIO_PIN_RESET);
    asm("NOP");
    //delay_us(5);
  }




/*
uint8_t CC1101_sendData(PACKET packet)
{  
  uint8_t marcState;

  // Enter RX state
  setRxState();
  
  while ((readStatusReg(CC1101_MARCSTATE) & 0x1F) != 0x0D)
  { delay_us(10); } 
 
  writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);
 
  setTxState();
  delay_us(4);
  
   marcState = readStatusReg(CC1101_MARCSTATE) & 0x1F;
  if((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15))
  {
    setIdleState();       // Enter IDLE state
    flushTxFifo();        // Flush Tx FIFO
    setRxState();         // Back to RX state
    return 0;
  }
  
   wait_GDO0_high();
   wait_GDO0_low();
   
  // Check that the TX FIFO is empty
  if((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0)
  {
    flushTxFifo();
    return 1;
  }
  return 0;
  
}
*/

uint8_t CC1101_sendData(PACKET packet)
{  

  writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);
  setIdleState();
  setTxState();
  
  
   wait_GDO0_high();
   wait_GDO0_low();

  // Check that the TX FIFO is empty
  if((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0)
  {
     
     
    return 1;
  }
  else
  {
     flushTxFifo();
     return 0;
  }
  
}








void setRxState(void)  
{ 
  cmdStrobe_C1101(CC1101_SRX);
}

uint8_t readStatusReg(uint8_t regAddr)  
{
  return readReg_C1101(regAddr, CC1101_STATUS_REGISTER);
}

void setTxState(void)
{
cmdStrobe_C1101(CC1101_STX);
}

void setIdleState(void)
{
  cmdStrobe_C1101(CC1101_SIDLE); 
}

void flushRxFifo(void)
{
   cmdStrobe_C1101(CC1101_SFRX);
}

void flushTxFifo(void) 
{
  cmdStrobe_C1101(CC1101_SFTX);
}

void wait_GDO0_high(void)
{
  while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12)))
  {
  }
}

void wait_GDO0_low(void)
 {
    while((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12)))
    {
    }  
 }
 




  
