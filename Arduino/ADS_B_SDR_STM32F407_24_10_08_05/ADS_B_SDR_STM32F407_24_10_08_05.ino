// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       ADS_B_SDR_STM32F407_24_10_08_01.ino
    Created:	08.10.2024 7:41:34
    Author:     MASTER\Alex
*/

#include <Arduino.h>              // define I/O functions
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"


#include "stdint.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>

#include "XPT2046_touch.h"   // Тачскрин дисплея
#include "ili9341.h"         // Настройки дисплея


GPIO_TypeDef* GPIO_PORT[LEDn] = { LED1_GREEN_GPIO_Port,           // Светодиоды на плате
                                 LED2_RED_GPIO_Port };

const uint16_t GPIO_PIN[LEDn] = { LED1_GREEN_Pin,
                                   LED2_RED_Pin };

void BSP_LED_On(Led_TypeDef Led)                                  // Включить светодиод
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

void BSP_LED_Off(Led_TypeDef Led)                                 // Выключить светодиод                                
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

void BSP_LED_Toggle(Led_TypeDef Led)                              // Переключить светодиод
{
    HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = { BUTTON_WK_AP_GPIO_Port,     // Кнопки на плате   
                                      BUTTON_K0_GPIO_Port,
                                      BUTTON_K1_GPIO_Port };

const uint16_t BUTTON_PIN[BUTTONn] = { BUTTON_WK_AP_Pin,           // Кнопки на плате   
                                      BUTTON_K0_Pin,
                                      BUTTON_K1_Pin };

const uint16_t BUTTON_IRQn[BUTTONn] = { BUTTON_WK_AP_EXTI_IRQn,    // Кнопки на плате   
                                       BUTTON_K0_EXTI_IRQn,
                                       BUTTON_K1_EXTI_IRQn };


uint32_t BSP_PB_GetState(Button_TypeDef Button)                   // Получить соотояние кнопки на плате   
{
    return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

static void Device_InitApplication(void);                    // Инициализация  устройств на плате 
void Start_TFT(void);

uint16_t x = 0, y = 0;
uint16_t xx = 0, y2 = 0;

char str1[20];
char str2[20];
char str3[20];
char str4[20];
char str[20];
bool flag1;
uint16_t strw;


void setup() {
    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    delay(500);
    Serial.println("Start");

    MX_USB_HOST_Init();

	Device_InitApplication();    // 
	//Start_TFT();
	//BSP_LED_Off(LED1);
}


void loop()
{
    MX_USB_HOST_Process();


}


static void Device_InitApplication(void)  // Инициализация  устройств на плате 
{
	BSP_LED_On(LED1);
	BSP_LED_On(LED2);
	HAL_Delay(200);
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	HAL_Delay(200);
	BSP_LED_On(LED1);
	BSP_LED_On(LED2);
	HAL_Delay(200);
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);

	lcdBacklightOn();
	lcdInit();
	lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);
	lcdFillRGB(COLOR_WHITE);

	lcdSetTextFont(&Font16);
	lcdSetTextColor(COLOR_WHITE, COLOR_BLUE);
	lcdFillRect(0, 0, 319, 20, COLOR_BLUE);
	lcdSetCursor(2, 5);   // x,y	

//  
//  /* Enable the display */
//  BSP_LCD_DisplayOn();
//  
//  /* Initialize the LCD Log module */
//  LCD_LOG_Init();

  /* Configure Button pin as input with External interrupt */


//#ifdef USE_USB_HS 
//  //LCD_LOG_SetHeader((uint8_t *)" USB OTG HS RTLSDR Host");
//  LCD_LOG_SetHeader((uint8_t *)" STM32F7 USB HS RTL-SDR Host");
//#else
//  LCD_LOG_SetHeader((uint8_t *)" STM32F7 USB FS RTL-SDR Host");
//#endif
//  
//  LCD_UsrLog("USB Host library started.\n"); 
//  
//  /* Start RTLSDR Interface */
//  USBH_UsrLog("Starting RTLSDR Demo");
//  



#ifdef USE_USB_HS 
	//LCD_LOG_SetHeader((uint8_t *)" USB OTG HS RTLSDR Host");
	lcdPrintf(" STM32F4 USB HS RTL-SDR Host");
#else
	lcdPrintf(" STM32F4 USB FS RTL-SDR Host");
#endif



}






void Start_TFT(void)
{
	Serial.println("Start_TFT");

	lcdBacklightOn();
	lcdInit();
	lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);
	lcdFillRGB(COLOR_WHITE);
	Serial.println("lcdInit Ok");
		// Пишем текст сверху экрана
	lcdSetTextFont(&Font20);
	lcdSetTextColor(COLOR_BLACK, COLOR_WHITE);
	lcdSetCursor(50, 5);   // xy
	lcdPrintf("www.stm32res.ru");
	// Начинаем рисовать рамки
	lcdDrawRect(50, 70, 230, 30, COLOR_BLUE);

	lcdDrawRect(30, 120, 40, 30, COLOR_BLUE);  //lcdDrawRect(x, y, w, h, color)
	lcdDrawRect(75, 120, 40, 30, COLOR_BLUE);
	lcdDrawRect(120, 120, 40, 30, COLOR_BLUE);
	lcdDrawRect(165, 120, 40, 30, COLOR_BLUE);
	lcdDrawRect(210, 120, 40, 30, COLOR_BLUE);
	lcdDrawRect(255, 120, 50, 30, COLOR_BLUE);

	lcdDrawRect(30, 155, 40, 30, COLOR_BLUE);  //lcdDrawRect(x, y, w, h, color)
	lcdDrawRect(75, 155, 40, 30, COLOR_BLUE);
	lcdDrawRect(120, 155, 40, 30, COLOR_BLUE);
	lcdDrawRect(165, 155, 40, 30, COLOR_BLUE);
	lcdDrawRect(210, 155, 40, 30, COLOR_BLUE);
	lcdDrawRect(255, 155, 50, 30, COLOR_BLUE);
	// Начинаем заполнять рамки цифрами    
	lcdSetTextFont(&Font24);
	lcdSetTextColor(COLOR_BLACK, COLOR_WHITE);

	lcdSetCursor(45, 125);   // x
	lcdPrintf("1");
	lcdSetCursor(90, 125);   // x
	lcdPrintf("2");
	lcdSetCursor(135, 125);   // x
	lcdPrintf("3");
	lcdSetCursor(180, 125);   // x
	lcdPrintf("4");
	lcdSetCursor(225, 125);   // x
	lcdPrintf("5");

	lcdSetCursor(45, 160);   // x
	lcdPrintf("6");
	lcdSetCursor(90, 160);   // x
	lcdPrintf("7");
	lcdSetCursor(135, 160);   // x
	lcdPrintf("8");
	lcdSetCursor(180, 160);   // x
	lcdPrintf("9");
	lcdSetCursor(225, 160);   // x
	lcdPrintf("0");

	lcdSetCursor(263, 125);   // x
	lcdPrintf("<-");

	lcdSetTextFont(&Font20);
	lcdSetCursor(265, 160);   // x
	lcdPrintf("Ok");

}

//void Test_TFT(void)
//{
//	// проверяем в какую область нажали на экране. Нас интересует только 
//	// внутренняя площадь рамок    
//
//	if (x >= 31 && x <= 65)   // Координаты по х
//	{
//		if (y >= 119 && y <= 146)  // Координаты по y
//		{
//			xx = 1;                 // Если попали то передаем переменной - номер 1  
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);      // Добавляем символ к общей строчке
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 80 && x <= 109)
//	{
//		if (y >= 111 && y <= 148)
//		{
//			xx = 2;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 131 && x <= 158)
//	{
//		if (y >= 109 && y <= 151)
//		{
//			xx = 3;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 166 && x <= 203)
//	{
//		if (y >= 120 && y <= 151)
//		{
//			xx = 4;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 201 && x <= 250)
//	{
//		if (y >= 110 && y <= 151)
//		{
//			xx = 5;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 32 && x <= 80)
//	{
//		if (y >= 156 && y <= 181)
//		{
//			xx = 6;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 78 && x <= 110)
//	{
//		if (y >= 150 && y <= 181)
//		{
//			xx = 7;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 121 && x <= 165)
//	{
//		if (y >= 145 && y <= 181)
//		{
//			xx = 8;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 168 && x <= 203)
//	{
//		if (y >= 156 && y <= 181)
//		{
//			xx = 9;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//	if (x >= 212 && x <= 247)
//	{
//		if (y >= 156 && y <= 181)
//		{
//			xx = 0;
//			sprintf(str3, "%u", xx);
//			strcat(str, str3);
//			strw = strlen(str);
//			sprintf(str4, "%u", strw);
//		}
//	}
//
//
//	if (x >= 255 && x <= 280)
//	{
//		if (y >= 119 && y <= 150)
//		{
//			// Эту рамку назначьте самостоятельно                  
//		}
//		//
//	}
//
//
//	// Выводим то что врамках 
//	lcdSetTextFont(&Font20);
//	lcdSetCursor(60, 78);   // x,y
//	lcdPrintf(str);
//
//	lcdSetTextFont(&Font20);
//	lcdSetCursor(60, 200);   // x,y
//	lcdPrintf(str1); //"x= "
//
//	lcdSetTextFont(&Font20);
//	lcdSetCursor(180, 200);   // x,y
//	lcdPrintf(str2); //"y= "
//
//	lcdSetTextFont(&Font20);
//	lcdSetCursor(285, 75);   // x,y
//	lcdPrintf(str4);
//
//}







/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
//void Error_Handler(void)
//{
//	/* USER CODE BEGIN Error_Handler_Debug */
//	/* User can add his own implementation to report the HAL error return state */
//
//	BSP_LED_On(LED2);
//	lcdSetTextFont(&Font20);
//	lcdSetCursor(10, 200);   // 
//	lcdPrintf("ERROR");
//
//	__disable_irq();
//	while (1)
//	{
//	}
//	/* USER CODE END Error_Handler_Debug */
//}


