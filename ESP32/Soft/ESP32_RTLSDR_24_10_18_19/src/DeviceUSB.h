#pragma once
#include "Configuration_ESP32.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include <esp_task_wdt.h>
#include "esp_libusb.h"
#include <elapsedMillis.h>

const TickType_t HOST_EVENT_TIMEOUT = 1;
const TickType_t CLIENT_EVENT_TIMEOUT = 1;

//usb_host_client_handle_t Client_Handle;
//usb_device_handle_t Device_Handle;
typedef void (*usb_host_enum_cb_t)(const usb_config_desc_t* config_desc);
static usb_host_enum_cb_t _USB_host_enumerate;

//bool isRTLSDR = false;
//bool isRTLSDRReady = false;
//uint8_t RTLSDRInterval;
//bool isRTLSDRPolling = false;
//elapsedMillis RTLSDRTimer;

const size_t RTLSDR_IN_BUFFER_SIZE = 8;
usb_transfer_t* RTLSDRIn = NULL;


void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
void RTLSDR_transfer_cb(usb_transfer_t* transfer);
void check_interface_desc_boot_RTLSDR(const void* p);
void prepare_endpoint(const void* p);

void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
void usbh_setup(usb_host_enum_cb_t enumeration_cb);
void usbh_init();