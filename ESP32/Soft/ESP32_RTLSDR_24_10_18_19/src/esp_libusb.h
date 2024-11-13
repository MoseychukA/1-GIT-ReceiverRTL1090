#pragma once

#include "usb/usb_host.h"
#include "Configuration_ESP32.h"
#include "DeviceUSB.h"

#ifndef portMAX_DELAY
#define portMAX_DELAY (TickType_t)0xffffffffUL
#endif

#define CTRL_OUT (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_OUT)
#define CTRL_IN (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_IN)

#define USB_SETUP_PACKET_INIT_CONTROL(setup_pkt_ptr, bm_reqtype, b_request, w_value, w_index, w_length) ({ \
    (setup_pkt_ptr)->bmRequestType = bm_reqtype;                                                           \
    (setup_pkt_ptr)->bRequest = b_request;                                                                 \
    (setup_pkt_ptr)->wValue = w_value;                                                                     \
    (setup_pkt_ptr)->wIndex = w_index;                                                                     \
    (setup_pkt_ptr)->wLength = w_length;                                                                   \
})


//static const char* TAG_ADSB = "ADSB";
void init_adsb_dev();
void bulk_transfer_read_cb(usb_transfer_t* transfer);
void transfer_read_cb(usb_transfer_t* transfer);
int esp_libusb_bulk_transfer(class_driver_t* driver_obj, unsigned char endpoint, unsigned char* data, int length, int* transferred, unsigned int timeout);
int esp_libusb_control_transfer(class_driver_t* driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char* data, uint16_t wLength, unsigned int timeout);
void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t* str_desc, char* str);
