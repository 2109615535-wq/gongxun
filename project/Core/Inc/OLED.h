#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
#include "OLED_Data.h"

/*参数宏定义*********************/

/*字体像素宽度*/
#define OLED_6X8				6
#define OLED_8X16				8

/*IsFilled参数数值*/
#define OLED_UNFILLED			0
#define OLED_FILLED				1

/*********************参数宏定义*/


/*函数声明*********************/

/*初始化函数*/
void OLED_Init(void);

/*更新函数*/
void OLED_Update(void);

/*显存控制函数*/
void OLED_Clear(void);
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);
void OLED_Reverse(void);
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

/*显示函数*/
/* 后缀1：小字号6x8；后缀2：大字号8x16。 */
/* 数字长度：十进制0~10位，十六进制0~8位，二进制0~32位；小数0~9位。 */
void OLED_ShowChar1(int16_t X, int16_t Y, char Char);
void OLED_ShowChar2(int16_t X, int16_t Y, char Char);
void OLED_ShowString1(int16_t X, int16_t Y, char *String);
void OLED_ShowString2(int16_t X, int16_t Y, char *String);
void OLED_ShowNum1(int16_t X, int16_t Y, uint32_t Number, uint8_t Length);
void OLED_ShowNum2(int16_t X, int16_t Y, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum1(int16_t X, int16_t Y, int32_t Number, uint8_t Length);
void OLED_ShowSignedNum2(int16_t X, int16_t Y, int32_t Number, uint8_t Length);
void OLED_ShowHexNum1(int16_t X, int16_t Y, uint32_t Number, uint8_t Length);
void OLED_ShowHexNum2(int16_t X, int16_t Y, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum1(int16_t X, int16_t Y, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum2(int16_t X, int16_t Y, uint32_t Number, uint8_t Length);
void OLED_ShowFloatNum1(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength);
void OLED_ShowFloatNum2(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength);
void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);
void OLED_Printf1(int16_t X, int16_t Y, char *format, ...);
void OLED_Printf2(int16_t X, int16_t Y, char *format, ...);

/*绘图函数*/
void OLED_DrawPoint(int16_t X, int16_t Y);
uint8_t OLED_GetPoint(int16_t X, int16_t Y);
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1);
void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled);
void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled);
void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled);
void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled);
void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled);

/*********************函数声明*/

#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
