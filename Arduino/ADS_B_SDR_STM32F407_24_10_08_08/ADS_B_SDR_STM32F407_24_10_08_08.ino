// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       ADS_B_SDR_STM32F407_24_10_08_01.ino
    Created:	08.10.2024 7:41:34
    Author:     MASTER\Alex
*/

#include "Arduino.h"
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"
#include "Configuration_ESP32.h"
#include "usbh_rtlsdr.h"


#include "stdint.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>



void setup() 
{
    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    delay(500);
    Serial.println("Start");

    pinMode(BUTTON_WAKEUP, INPUT);
    pinMode(BUTTON_KEY0, INPUT);
    pinMode(BUTTON_KEY1, INPUT);

    pinMode(LED_RED, OUTPUT);                // Подсветка дисплея
    digitalWrite(LED_RED, LOW);             // 
    delay(500);
    digitalWrite(LED_RED, HIGH);             // 
    pinMode(LED_GREEN, OUTPUT);                // Подсветка дисплея
    digitalWrite(LED_GREEN, LOW);             // 
    delay(500);
    digitalWrite(LED_GREEN, HIGH);             // 

    MX_USB_HOST_Init();

}


void loop()
{
    MX_USB_HOST_Process();


}


