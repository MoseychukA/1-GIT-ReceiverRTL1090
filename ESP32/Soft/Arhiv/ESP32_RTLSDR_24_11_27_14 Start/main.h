#pragma once
#include "usb/usb_host.h"
#include <esp_task_wdt.h>
#include "Configuration_ESP32.h"

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;
