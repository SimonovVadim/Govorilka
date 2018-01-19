#include "ff.h"
#include "integer.h"
#include "diskio.h"
#include "stm32f1xx_hal.h"
#include "C1101.h"

extern FIL file;
extern UINT nRead, nWritten;
extern FRESULT result;
extern FATFS FATFS_Obj;
extern uint8_t Buff[1024];
extern uint8_t Sound_buff[1024];
extern const uint32_t sinus_12bit[];

extern TIM_HandleTypeDef htim6;
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac_ch1;

extern uint8_t DMA_HALF_TRASMMIT_COMPLETE;
extern uint8_t DMA_TRANSMMIT_COMPLETE;
extern ADC_HandleTypeDef hadc1;

void Play_1_file(void);
void interrupt_play(void);
void SendResultViaRadio(float Data);

uint16_t ADC_GET(void);

extern uint8_t FLAG_PLAY;
uint16_t dempGetData(void);
float tempGetData(void);

extern uint16_t Max;

void Delay_us(uint16_t Delay);

 extern  uint16_t Res_1;
 extern uint16_t Res_2;
 extern uint16_t Res_3;
 extern uint16_t Res_4;
 extern uint16_t Res_5;
 extern uint16_t Res_6;
 extern uint16_t Res_7;
 extern uint16_t Res_8;

#define DEMP_TX_1()   HAL_GPIO_WritePin(DEMP_TX_GPIO_Port, DEMP_TX_Pin,GPIO_PIN_SET);
#define DEMP_TX_0()   HAL_GPIO_WritePin(DEMP_TX_GPIO_Port, DEMP_TX_Pin,GPIO_PIN_RESET);

#define DEMP_SI_1()   HAL_GPIO_WritePin(DEMP_SI_GPIO_Port, DEMP_SI_Pin,GPIO_PIN_SET);
#define DEMP_SI_0()   HAL_GPIO_WritePin(DEMP_SI_GPIO_Port, DEMP_SI_Pin,GPIO_PIN_RESET);



