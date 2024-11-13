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


void setup() {
    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    delay(500);
    Serial.println("Start");

    MX_USB_HOST_Init();


}


void loop()
{
    MX_USB_HOST_Process();


}




