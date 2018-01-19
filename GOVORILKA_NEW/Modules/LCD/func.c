#include "stm32f1xx_hal.h"
#include "func.h"
#include "font7x15.h"
#include "main.h"

int LCD_X;
int LCD_Y;
extern SPI_HandleTypeDef hspi1;

/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------инициализация дисплея -----------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void spi2_8b_init(void){//переходим в 8 бит SPI
 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
 HAL_SPI_Init(&hspi1);

}
void spi2_16b_init(void){//переходим в 16 бит SPI
 hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
 HAL_SPI_Init(&hspi1);
}	
	
void lcd7735_senddata(unsigned char data) {//передаем в режиме 8 бит
  
  if(hspi1.Init.DataSize != SPI_DATASIZE_8BIT)
  {
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&hspi1);
  }
  
  LCD_CS0;
  HAL_SPI_Transmit(&hspi1, &data,1,0x1);
  LCD_CS1;
}
void lcd7735_send16bData(uint8_t msb,uint8_t lsb) {//передаем в режиме 16 бит
  
   if(hspi1.Init.DataSize != SPI_DATASIZE_16BIT)
  {
    hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
    HAL_SPI_Init(&hspi1);
  }
  
  
	uint8_t masData[]={lsb,msb};
        LCD_CS0;
	HAL_SPI_Transmit(&hspi1,masData,1,0x1);
        LCD_CS1;
}

void SetXY(int i, int y)
{
    LCD_X = i;    
    LCD_Y = y;   

}

void lcd7735_ini(void) {
   LCD_CS0;            // CS=0   
   LCD_RST0;           // RST=0 

   HAL_Delay(10);      

   LCD_RST1;           // RST=1
   HAL_Delay(10);      // 

   
   lcd7735_sendCmd(0x11); // 

   HAL_Delay(120);      // 

   lcd7735_sendCmd (0x3A); //    режим цвета:
   lcd7735_sendData(0x05); //     16  бит
	 lcd7735_sendCmd (0x36);// направление вывода изображения:
	 lcd7735_sendData(0x14); //0x1C-снизу вверх, справа на лево, порядок цветов RGB;0x14-снизу вверх, справа на лево, порядок цветов BGR
	 lcd7735_sendCmd (0x29);//Display on
}  


/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------Отправка команды ----------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void lcd7735_sendCmd(unsigned char cmd) {
   LCD_DC0;//ставим в ноль линию DC
   lcd7735_senddata(cmd);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------Отправка данных------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_sendData(unsigned char data) {
   LCD_DC1;//ставим вывод DC в единицу т.к. передаем данные
   lcd7735_senddata(data);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------определение области экрана для заполнения----------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_at(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY) {
	lcd7735_sendCmd(0x2A);//КОМАНДА адресс колонки 
	LCD_DC1;
	lcd7735_senddata(0x00); 
	lcd7735_senddata(startX);
	lcd7735_senddata(0x00); 
	lcd7735_senddata(stopX);

	lcd7735_sendCmd(0x2B);//КОМАНДА адресс ряда
	LCD_DC1;
	lcd7735_senddata(0x00); 
	lcd7735_senddata(startY);
	lcd7735_senddata(0x00); 
	lcd7735_senddata(stopY);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//---------- процедура заполнения прямоугольной области экрана заданным цветом---------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_fillrect(unsigned char startX, unsigned char startY, unsigned char stopX, unsigned char stopY, unsigned int color) {
	unsigned char y;
	unsigned char x;
  unsigned char msb =((unsigned char)((color & 0xFF00)>>8));//старшие 8 бит
  unsigned char lsb =((unsigned char)(color & 0x00FF));	//младшие 8 бит
  LCD_CS0;//ставим чипселект в 0 
  lcd7735_at(startX, startY, stopX, stopY);
	lcd7735_sendCmd(0x2C);//Memory write
  
	spi2_16b_init();// переходим в 16 бит
	LCD_DC1;//закоментить для 8 бит
	for (y=startY;y<stopY+1;y++)
	for (x=startX;x<stopX+1;x++) {	
   	lcd7735_send16bData(msb,lsb);//надо переключаться в 16 бит прередачу
//  lcd7735_sendData(msb);//для 8 бит
//  lcd7735_sendData(lsb);//для 8 бит
		}
		while((SPI2->SR) & SPI_SR_BSY); // ожидаем окончания передачи данных SPI2
		spi2_8b_init();
 		LCD_CS1;

}

// вывод пиксела
void lcd7735_putpix(unsigned char x, unsigned char y, unsigned int Color) {

	LCD_CS0;
	lcd7735_at(x, y, x, y);
	lcd7735_sendCmd(0x2C);//запись в память
	LCD_DC1;
	lcd7735_senddata((unsigned char)((Color & 0xFF00)>>8));//меняем старший и младший байты согласно протоколу прередачи
	lcd7735_senddata((unsigned char) (Color & 0x00FF));//меняем старший и младший байты согласно протоколу прередачи
  LCD_CS1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------отрисовка линии-------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned int color) {
signed char   dx, dy, sx, sy;
unsigned char  x,  y, mdx, mdy, l;

  if (x1==x2) { // 
	  lcd7735_fillrect(x1,y1, x1,y2, color);//если линия прямая то просто чертим
	  return;
  }

  if (y1==y2) { // 
	  lcd7735_fillrect(x1,y1, x2,y1, color);//если линия прямая то просто чертим
	  return;
  }
//здесь расчет черчения косой линии 
  dx=x2-x1; dy=y2-y1;

  if (dx>=0) { mdx=dx; sx=1; } else { mdx=x1-x2; sx=-1; }
  if (dy>=0) { mdy=dy; sy=1; } else { mdy=y1-y2; sy=-1; }

  x=x1; y=y1;

  if (mdx>=mdy) {//если длинна линии  по х меньше чем по у
     l=mdx;
     while (l>0) {
         if (dy>0) { y=y1+mdy*(x-x1)/mdx; }
            else { y=y1-mdy*(x-x1)/mdx; }
         lcd7735_putpix(x,y,color);//расчитали текущий х,у и ресуем попиксельно задавая цвет
         x=x+sx;
         l--;
     }
  } else {
     l=mdy;//если длинна линии  по у меньше чем по х
     while (l>0) {
        if (dy>0) { x=x1+((mdx*(y-y1))/mdy); }
          else { x=x1+((mdx*(y1-y))/mdy); }
        lcd7735_putpix(x,y,color);//расчитали текущий х,у и ресуем попиксельно задавая цвет
        y=y+sy;
        l--;
     }
  }
  lcd7735_putpix(x2, y2, color);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------рисование прямоугольника (не заполненного)---------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_rect(char x1,char y1,char x2,char y2, unsigned int color) {
	lcd7735_fillrect(x1,y1, x2,y1, color);
	lcd7735_fillrect(x1,y2, x2,y2, color);
	lcd7735_fillrect(x1,y1, x1,y2, color);
	lcd7735_fillrect(x2,y1, x2,y2, color);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------//печать десятичного числа//-----------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void LCD7735_dec(unsigned int numb, unsigned char dcount, unsigned char x, unsigned char y,unsigned int fntColor, unsigned int bkgColor) {
	unsigned int divid=10000;
	unsigned char i;

	for (i=5; i!=0; i--) {

		unsigned char res=numb/divid;

		if (i<=dcount) {
			lcd7735_putchar(res);
			y=y+6;
		}

		numb%=divid;
		divid/=10;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------большие цифры--------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////////////
void printDigit (unsigned char posX, unsigned char posY, unsigned char number,unsigned int numberColor,unsigned int bacgroundColor){
switch (number) {
    case 0:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,numberColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,numberColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,numberColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,bacgroundColor);//7
      break;
	case 1:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,bacgroundColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,bacgroundColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,bacgroundColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,bacgroundColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,bacgroundColor);//7
      break;
    case 2:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,bacgroundColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,numberColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,numberColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,bacgroundColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,numberColor);//7
      break;
    case 3:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,numberColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,bacgroundColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,bacgroundColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,numberColor);//7
      break;
     case 4:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,bacgroundColor );//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,bacgroundColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,bacgroundColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,numberColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,numberColor);//7
      break; 
		 case 5:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,bacgroundColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,numberColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,bacgroundColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,numberColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,numberColor);//7       
      break;
		 case 6:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,bacgroundColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,numberColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,numberColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,numberColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,numberColor);//7     
      break;
		 case 7:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,bacgroundColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,bacgroundColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,bacgroundColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,bacgroundColor);//7     
      break;
		 case 8:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,numberColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,numberColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,numberColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,numberColor);//7     
      break;
	case 9:
lcd7735_fillrect(posX+48,posY+4,posX+52,posY+24,numberColor);//1
lcd7735_fillrect(posX+28,posY+24,posX+48,posY+28,numberColor);//2
lcd7735_fillrect(posX+4,posY+24,posX+24,posY+28,numberColor);//3
lcd7735_fillrect(posX+0,posY+4,posX+4,posY+24,numberColor);//4 
lcd7735_fillrect(posX+4,posY+0,posX+24,posY+4,bacgroundColor);//5 
lcd7735_fillrect(posX+28,posY+0,posX+48,posY+4,numberColor);//6
lcd7735_fillrect(posX+24,posY+4,posX+28,posY+24,numberColor);//7     
      break;	 
		 
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------//вывод символа на экран по координатам//--------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_putchar( unsigned char chr) {
  
  LCD_X +=8;
  if ('\r'== chr )
  {

  }
  else if ('\t'== chr )
  {

  }
  else if ('\n'== chr )
  {

  }
  else
  {
  
  uint8_t y = LCD_X;
  uint8_t x = LCD_Y;
  uint16_t charColor = 0xFFFF;
  uint16_t bkgColor =  0x0;
  
	unsigned char i;
	unsigned char j;
	
LCD_CS0;

	lcd7735_at(x, y, x+12, y+8);
	lcd7735_sendCmd(0x2C);//запись в память

spi2_16b_init();// переходим в 16 бит
	LCD_DC1;//закоментить для 8 бит
  unsigned char k;
	for (i=0;i<7;i++)
		for (k=2;k>0;k--) {//к=1 верхняя половина символа,2 нижняя
		   unsigned char chl=NewBFontLAT[ ( (chr-0x20)*14 + i+ 7*(k-1)) ];
		   chl=chl<<2*(k-1); // нижнюю половину символа сдвигаем на 1 позицию влево (убираем одну линию снизу)
		   unsigned char h;
		   if (k==2) h=6; else h=7; // у нижней половины выведем только 6 точек вместо 7
		   for (j=0;j<h;j++) {
			unsigned int color;
			//HAL_Delay(100);
			if (chl & 0x80) color=charColor; else color=bkgColor;//если MSBit==0 тогда оставляем цвет бэграунда,если нет тогда цвет символа
			chl = chl<<1;//сдвигаем обработанный бит влево
			unsigned char msb =((unsigned char)((color & 0xFF00)>>8));//старшие 8 бит меняем местами MSB и LSB
      unsigned char lsb =((unsigned char)(color & 0x00FF));	//младшие 8 бит	 
     
	   	lcd7735_send16bData(msb,lsb);//надо переключаться в 16 бит прередачу
		// lcd7735_sendData(msb);//нужен 8 бит режим
		//lcd7735_sendData(lsb);//нужен 8 бит режим


		}
	}
	// рисуем справо от символа пустую вертикальную линию для бокового интервала
	
	  unsigned char msb =((unsigned char)((bkgColor & 0xFF00)>>8));//старшие 8 бит
      unsigned char lsb =((unsigned char)(bkgColor & 0x00FF));	//младшие 8 бит	 
	for (j=0;j<13;j++) {
     
	   	  lcd7735_send16bData(msb,lsb);//надо переключаться в 16 бит прередачу
		//	lcd7735_sendData(msb);//для 8 бит
		//	lcd7735_sendData(lsb);//старший байт первым

	}
	while((SPI2->SR) & SPI_SR_BSY); // ожидаем окончания передачи данных SPI2
	spi2_8b_init();
LCD_CS1;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------//вывод строки	//-----------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////
void lcd7735_putstr(unsigned char x, unsigned char y, const unsigned char str[], unsigned int charColor, unsigned int bkgColor) {

	while (*str!=0) {
		lcd7735_putchar(*str);
		y=y+8;
		str++;
	}
}


