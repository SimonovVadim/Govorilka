#include "App.h"
#include "Delay.h"

#define B     3950.0f          //4485.0  коэфициент
#define C     13.255f
#define R 1

FIL file;
UINT nRead, nWritten;
FRESULT result;
FATFS FATFS_Obj;
uint8_t Buff[1024];														//	????? ??? SD ?????
uint8_t Sound_buff[1024];	
uint8_t flag_1;
uint8_t flag;
uint8_t DMA_TRANSMMIT_COMPLETE = 0 ;
uint8_t DMA_HALF_TRASMMIT_COMPLETE = 0;

uint8_t Buff[1024];														//	????? ??? SD ?????

uint8_t DMA_HALF_TRASMMIT_COMPLETE;
uint8_t DMA_TRANSMMIT_COMPLETE;

uint8_t FLAG_PLAY;

uint16_t Max;

  uint16_t Res_1;
  uint16_t Res_2;
  uint16_t Res_3;
  uint16_t Res_4;
  uint16_t Res_5;
  uint16_t Res_6;
  uint16_t Res_7;
  uint16_t Res_8;

  
  extern PACKET packet;  
extern uint8_t InitError;
  
const uint32_t sinus_12bit[] =
{ 3968, 3840 , 3712 , 3584 , 3456 ,  3328 ,  3200,  3072,
		 2944 , 2816 , 2688,  2560
				,  2432,  2304, 2176,
		2048, 1920,1792,1664
				, 1408,1280,
		1152,1024,896,768
				,64,  512, 384,
		256, 128,0 };

void Play_1_file(void)
{
  static uint8_t flag = 0;
  
  if(!flag)
  {
    flag = 1;
    
   result = f_open(&file, "fire.wav", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
   if (result != FR_OK)
   {
     //lcd7735_putstr(60,0,"Ошибка открытия файла ",0x0F00,0x0000);
     flag = 0;					 
   }
  // lcd7735_putstr(60,0,"Файл открыт ",0x0F00,0x0000);
   
   result = f_read(&file, &Buff, 512, &nRead);  	
   if(	result != FR_OK)	
   {
    //lcd7735_putstr(40,0,"Ошибка чтения файла ",0x0F00,0x0000);
    flag = 0;
   }
   
   for(int i=0;i<512;i++)
   {
      Sound_buff[i] = Buff[i];
   } 
			 
   result = f_read(&file, &Buff, 512, &nRead);
   if(result !=	FR_OK )	
   {
    //lcd7735_putstr(40,0,"Ошибка чтения файла ",0x0F00,0x0000);
    flag = 0;
   }
   
   // lcd7735_putstr(40,0,"Воспроизведение ",0x0F00,0x0000);
   if(flag)
   {
     HAL_TIM_Base_Start(&htim6); 
     HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t *)Sound_buff,512,DAC_ALIGN_8B_R);
   }
   
  }
   

   if(DMA_TRANSMMIT_COMPLETE)
   {
     DMA_TRANSMMIT_COMPLETE = 0;					 
     for(uint32_t y = 256;y <512 ; y ++)
     {
        Sound_buff[y] = Buff[y];
     }
					
     result = f_read(&file, &Buff, 512, &nRead);
     if(  result != FR_OK )	
     {
       flag = 0;
       interrupt_play();
     }
							
     if(nRead < 511)
     {
       flag = 0;
       interrupt_play();
      }
     }
						
    if(DMA_HALF_TRASMMIT_COMPLETE)
   {
     DMA_HALF_TRASMMIT_COMPLETE =0;
						
     for(int i=0;i<256;i++)
     {
	Sound_buff[i] = Buff[i];
     }
   }



   
}

void interrupt_play(void)
{
  HAL_TIM_Base_Stop(&htim6); 
  HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);
  f_close(&file);
  FLAG_PLAY = 0;
}

uint16_t ADC_GET(void)
{
  uint16_t adcResult_C;
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  adcResult_C = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  
  return adcResult_C; 
}

uint16_t dempGetData(void)
{
  //Max = 0;
  DEMP_TX_1();
  delay_us(1000);
  DEMP_TX_0();
  
  DEMP_SI_1();
  delay_us(100);
  
  Res_1  = ADC_GET();
 // Res_2  = ADC_GET(); 
 // Res_3  = ADC_GET(); 
 // Res_4  = ADC_GET(); 
 // Res_5  = ADC_GET(); 
  //Res_6  = ADC_GET(); 
 // Res_7  = ADC_GET(); 
//  Res_8  = ADC_GET(); 
  
  
 // uint16_t Result = (Res_1 + Res_2 + Res_3 + Res_4 + Res_5 + Res_6) / 6;
/*
  if ( Max < Res_1 ) Max = Res_1;
  Delay_us(50);
  if ( Max < Res_2 ) Max = Res_2;
  Delay_us(50);
  if ( Max < Res_3 ) Max = Res_3;
  Delay_us(50);
  if ( Max < Res_4 ) Max = Res_4; 
  Delay_us(50);
  if ( Max < Res_5 ) Max = Res_5;
  Delay_us(50);
  if ( Max < Res_6 ) Max = Res_6;
  */
 // if ( Max < Res_7 ) Max = Res_7;
 // if ( Max < Res_8 ) Max = Res_8; 

  //Delay_us(5);
  DEMP_SI_0();
  
  return (int)Res_1;
}




float tempGetData(void)
{
  float Res;
  
  for(int i = 0 ; i < 10 ; i++)
  {
    Res += ADC_GET(); 
  }
  
  Res = (float) Res / 10;
  
  
  float Vol = ( (float) 6.6 / 4.095) * Res;
  
  float Rt = (float)145000 / ( (float)( 5 / Vol) - 1 );
  
  float Tem = ((float)1336530 / (4485+298*log((float)Rt/100))) - 273;
  
 //  float temperature = B/(C+log( (double)(Res/(4095.0-Res))*R ) ) - 273;       // вычисляем значение температуры - в градусах
   
   
  return  Tem;
}


void SendResultViaRadio(float Data)
{
   packet.length=4;
   packet.data[0]=3;
   packet.data[1]=0xAA;
   packet.data[2]=0xCA;
   packet.data[3]=0xAC;
   packet.data[4]=0xCC;
  
  if(!(CC1101_sendData(packet)))
  {
    SetXY(0,80);
    printf("Успешная отправка   \n");
      
   }
   else
   {
     printf("Ошибка передачи   \n"); 
   }
  
}