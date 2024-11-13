// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       ADS_B_SDR_STM32F407_24_10_08_01.ino
    Created:	08.10.2024 7:41:34
    Author:     MASTER\Alex
*/


#include "main.h"
#include "fatfs.h"
#include "usb_host.h"


#include "stdint.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>

#include "XPT2046_touch.h"   // Тачскрин дисплея
#include "ili9341.h"         // Настройки дисплея

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

void setup() {
    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    delay(500);
    Serial.println("Start");

    MX_USB_HOST_Init();

	Device_InitApplication();    // 


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


