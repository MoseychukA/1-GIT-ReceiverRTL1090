#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include <esp_task_wdt.h>
#include "src/esp_libusb.h"
#include "src/Configuration_ESP32.h"
#include "src/DeviceUSB.h"


//#include "rtl-sdr.h"


//#define DAEMON_TASK_PRIORITY 2
//#define CLASS_TASK_PRIORITY  3
//
//static const char* TAG_DAEMON = "DAEMON";
//static const char* TAG_CLASS = "CLASS";





void setup()
{ 
    Serial.begin(115200);
    while (!Serial && millis() < 1000);

    esp_log_level_set("*", ESP_LOG_VERBOSE);  // Выводим отладочные сообщения

    String ver_soft = __FILE__;
    int val_srt = ver_soft.lastIndexOf('\\');
    ver_soft.remove(0, val_srt + 1);
    val_srt = ver_soft.lastIndexOf('.');
    ver_soft.remove(val_srt);
    Serial.println("\n*** Version " + ver_soft + "\n");

    esp_task_wdt_init(8, false);

    usbh_init();
    
   
    
}


void loop()
{
 

}

