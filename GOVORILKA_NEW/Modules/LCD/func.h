#ifndef FUNC_H
#define FUNC_H
//инициализация дисплея
void lcd7735_ini(void);
//отправка данных
void lcd7735_sendData(unsigned char data);
//отправка команды
void lcd7735_sendCmd(unsigned char cmd); 
// процедура заполнения прямоугольной области экрана заданным цветом
void lcd7735_fillrect(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY, unsigned int color);
// вывод пиксела
void lcd7735_putpix(unsigned char x, unsigned char y, unsigned int Color);
// отрисовка линии 
void lcd7735_line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned int color);
// рисование прямоугольника (не заполненного)	 
void lcd7735_rect(char x1,char y1,char x2,char y2, unsigned int color);
//вывод символа на экран по координатам	 
void lcd7735_putchar( unsigned char chr);
//вывод строки		 
void lcd7735_putstr(unsigned char x, unsigned char y, const unsigned char str[], unsigned int charColor, unsigned int bkgColor);
//большие цифры
void printDigit (unsigned char posX, unsigned char posY,unsigned char number,unsigned int numberColor,unsigned int bacgroundColor);
//печать десятичного числа		 
void LCD7735_dec(unsigned int numb, unsigned char dcount, unsigned char x, unsigned char y,unsigned int fntColor, unsigned int bkgColor);
// определение области экрана для заполнения
void lcd7735_at(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY) ;
#endif



#define LCD_RST1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);
#define LCD_RST0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
//   LCD_DC
#define LCD_DC1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);//a0//GPIOB->BSRR=GPIO_BSRR_BS_12
#define LCD_DC0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);//GPIOB->BSRR=GPIO_BSRR_BR_12//

//  LCD_CS
#define LCD_CS1  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,GPIO_PIN_SET);// GPIOB->BSRR=GPIO_BSRR_BS_10//
#define LCD_CS0  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,GPIO_PIN_RESET);// GPIOB->BSRR=GPIO_BSRR_BR_10//



void lcd7735_senddata(unsigned char data);
void lcd7735_send16bData(unsigned char msb,unsigned char lsb);
void spi2_8b_init(void);
void spi2_16b_init(void);
void SetXY(int i,int y );