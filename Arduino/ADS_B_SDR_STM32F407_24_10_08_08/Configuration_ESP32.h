#pragma once
#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
//#include "usbh_rtlsdr.h"
//#include "libusb.h"
//#include "rtl-sdr.h"


#define LED_RED        PA7
#define LED_GREEN      PA6
#define BUTTON_WAKEUP  PA0
#define BUTTON_KEY0    PE4
#define BUTTON_KEY1    PE3



	/* Exported constants --------------------------------------------------------*/
	/* USER CODE BEGIN EC */
//#define USB_PIPE_NUMBER 0x81                   //
//#define KILOBYTES 1024                         //
//#define RAW_BUFFER_BYTES (25*KILOBYTES)        //
//#define SIZEOF_DEMOD_BUF_EL 2                  //
//#define DEMOD_BUFF_BYTES (RAW_BUFFER_BYTES/SIZEOF_DEMOD_BUF_EL) //
//#define DOWNSAMPLE 15                          //
//#define RTL_SAMPLERATE 240000                  //
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
	//void Error_Handler(void);

	/* USER CODE BEGIN EFP */

	/* USER CODE END EFP */

	/* Private defines -----------------------------------------------------------*/

//#define T_PEN_Pin GPIO_PIN_5
//#define T_PEN_GPIO_Port GPIOC
//#define T_PEN_EXTI_IRQn EXTI9_5_IRQn
//#define LCD_BL_Pin GPIO_PIN_1
//#define LCD_BL_GPIO_Port GPIOB
//#define T_CS_Pin GPIO_PIN_12
//#define T_CS_GPIO_Port GPIOB


	////extern USBH_HandleTypeDef hUSBHost;
	//extern USBH_HandleTypeDef hUsbHostFS;

	//void Start_TFT(void);
	//void Test_TFT(void);

	//extern volatile uint8_t raw_bufA[RAW_BUFFER_BYTES];
	//extern volatile uint8_t raw_bufB[RAW_BUFFER_BYTES];
	//extern volatile uint8_t* raw_buf_filling;

	//extern volatile int16_t demod_bufferA[DEMOD_BUFF_BYTES];
	//extern volatile int16_t demod_bufferB[DEMOD_BUFF_BYTES];
	//extern volatile int16_t* curr_demod_buff;

	//extern uint8_t usb_device_ready;
	//extern uint8_t OutPipe, InPipe;

	///* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif
