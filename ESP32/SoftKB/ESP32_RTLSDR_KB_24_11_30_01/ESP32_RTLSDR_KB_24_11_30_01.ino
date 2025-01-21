/*
 * MIT License
 *
 * Copyright (c) 2021 touchgadgetdev@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <elapsedMillis.h>
#include <usb/usb_host.h>
#include "show_desc.hpp"

#include <Arduino.h>              // define I/O functions
#include <stdint.h>
#include <stdio.h>                // define I/O functions
#include <stdlib.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 8

static const char* TAG_MAIN = "MAIN";

const TickType_t HOST_EVENT_TIMEOUT = 1;
const TickType_t CLIENT_EVENT_TIMEOUT = 1;

usb_host_client_handle_t Client_Handle;
usb_device_handle_t Device_Handle;
typedef void (*usb_host_enum_cb_t)(const usb_config_desc_t* config_desc);
static usb_host_enum_cb_t _USB_host_enumerate;

bool isKeyboard = false;
bool isKeyboardReady = false;
uint8_t KeyboardInterval;
bool isKeyboardPolling = false;
elapsedMillis KeyboardTimer;

const size_t KEYBOARD_IN_BUFFER_SIZE = 8;
usb_transfer_t *KeyboardIn = NULL;

bool isRTLSDR = false;
bool isRTLSDR_Ready = false;
uint8_t RTLSDRInterval;
bool isRTLSDRPolling = false;
elapsedMillis RTLSDRTimer;

const size_t RTLSDR_IN_BUFFER_SIZE = 64;
usb_transfer_t* RTLSDRIn = NULL;

//=============================================================
void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
void usbh_setup(usb_host_enum_cb_t enumeration_cb);
void usbh_task(void);
//=============================================================

void keyboard_transfer_cb(usb_transfer_t* transfer);
void rtlsdr_transfer_cb(usb_transfer_t* transfer);
void check_interface_desc_boot_keyboard(const void* p);
void check_interface_desc_boot_rtlsdr(const void* p);
void prepare_endpoint(const void* p);
void prepare_endpoint_rtlsdr(const void* p);
void show_config_desc_full(const usb_config_desc_t* config_desc);
//==================================================================

void setup()
{

    Serial.begin(115200);
    while (!Serial && millis() < 1000);

    esp_log_level_set("*", ESP_LOG_VERBOSE);  //  

    String ver_soft = __FILE__;
    int val_srt = ver_soft.lastIndexOf('\\');
    ver_soft.remove(0, val_srt + 1);
    val_srt = ver_soft.lastIndexOf('.');
    ver_soft.remove(val_srt);
    Serial.println(ver_soft);
    //esp_task_wdt_reset();

  usbh_setup(show_config_desc_full);
  ESP_LOGI(TAG_MAIN, "*** End setup");
}

void loop()
{
  usbh_task();

  if (isKeyboardReady && !isKeyboardPolling && (KeyboardTimer > KeyboardInterval)) 
  {
    KeyboardIn->num_bytes = 8;
    esp_err_t err = usb_host_transfer_submit(KeyboardIn);
    if (err != ESP_OK) 
    {
      ESP_LOGI("", "usb_host_transfer_submit In fail: %x", err);
    }
    isKeyboardPolling = true;
    KeyboardTimer = 0;
  }

  if (isRTLSDR_Ready && !isRTLSDRPolling && (RTLSDRTimer > RTLSDRInterval))
  {
      RTLSDRIn->num_bytes = 64;
      esp_err_t err = usb_host_transfer_submit(RTLSDRIn);
      if (err != ESP_OK)
      {
          ESP_LOGI("", "usb_host_transfer_submit In fail: %x", err);
      }
      isRTLSDRPolling = true;
      RTLSDRTimer = 0;
  }


}

//==================================================================================================
void keyboard_transfer_cb(usb_transfer_t* transfer)
{
    if (Device_Handle == transfer->device_handle)
    {
        isKeyboardPolling = false;
        if (transfer->status == 0)
        {
            if (transfer->actual_num_bytes == 8)
            {
                uint8_t* const p = transfer->data_buffer;
                ESP_LOGI("", "HID report: %02x %02x %02x %02x %02x %02x %02x %02x",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
            }
            else
            {
                ESP_LOGI("", "Keyboard boot hid transfer too short or long");
            }
        }
        else {
            ESP_LOGI("", "transfer->status %d", transfer->status);
        }
    }
}

void rtlsdr_transfer_cb(usb_transfer_t* transfer)
{
    if (Device_Handle == transfer->device_handle)
    {
        isRTLSDRPolling = false;
        if (transfer->status == 0)
        {
            if (transfer->actual_num_bytes == 64)
            {
                uint8_t* const p = transfer->data_buffer;
                ESP_LOGI("", "HID report: %02x %02x %02x %02x %02x %02x %02x %02x",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
            }
            else
            {
                ESP_LOGI("", "Keyboard boot hid transfer too short or long");
            }
        }
        else {
            ESP_LOGI("", "transfer->status %d", transfer->status);
        }
    }
}

void check_interface_desc_boot_keyboard(const void* p)
{
    const usb_intf_desc_t* intf = (const usb_intf_desc_t*)p;

    if ((intf->bInterfaceClass == USB_CLASS_HID) &&
        (intf->bInterfaceSubClass == 1) &&
        (intf->bInterfaceProtocol == 1)) {
        isKeyboard = true;
        ESP_LOGI("", "Claiming a boot keyboard!");
        esp_err_t err = usb_host_interface_claim(Client_Handle, Device_Handle,
            intf->bInterfaceNumber, intf->bAlternateSetting);
        if (err != ESP_OK) ESP_LOGI("", "usb_host_interface_claim failed: %x", err);
    }
}

void check_interface_desc_boot_rtlsdr(const void* p)
{
    const usb_intf_desc_t* intf = (const usb_intf_desc_t*)p;

    if ((intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC) && (intf->bInterfaceSubClass == 0xff) && (intf->bInterfaceProtocol == 0xff))
    {
        isRTLSDR = true;
        ESP_LOGI("", "Claiming a boot RTLSDR!");
        esp_err_t err = usb_host_interface_claim(Client_Handle, Device_Handle,
            intf->bInterfaceNumber, intf->bAlternateSetting);
        if (err != ESP_OK) ESP_LOGI("", "usb_host_interface_claim failed: %x", err);
    }
}

void prepare_endpoint(const void* p)
{
    const usb_ep_desc_t* endpoint = (const usb_ep_desc_t*)p;
    esp_err_t err;

    // must be interrupt for HID
    if ((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_INT) {
        ESP_LOGI("", "Not interrupt endpoint: 0x%02x", endpoint->bmAttributes);
        return;
    }
    if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) {
        err = usb_host_transfer_alloc(KEYBOARD_IN_BUFFER_SIZE, 0, &KeyboardIn);
        if (err != ESP_OK) {
            KeyboardIn = NULL;
            ESP_LOGI("", "usb_host_transfer_alloc In fail: %x", err);
            return;
        }
        KeyboardIn->device_handle = Device_Handle;
        KeyboardIn->bEndpointAddress = endpoint->bEndpointAddress;
        KeyboardIn->callback = keyboard_transfer_cb;
        KeyboardIn->context = NULL;
        isKeyboardReady = true;
        KeyboardInterval = endpoint->bInterval;
        ESP_LOGI("", "USB boot keyboard ready");
    }
    else {
        ESP_LOGI("", "Ignoring interrupt Out endpoint");
    }
}

void prepare_endpoint_rtlsdr(const void* p)
{
    const usb_ep_desc_t* endpoint = (const usb_ep_desc_t*)p;
    esp_err_t err;

    // must be interrupt for HID
    if ((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_BULK)
    {
        ESP_LOGI("", "Not interrupt endpoint: 0x%02x", endpoint->bmAttributes);
        return;
    }
    if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK)
    {
        err = usb_host_transfer_alloc(RTLSDR_IN_BUFFER_SIZE, 0, &RTLSDRIn);
        if (err != ESP_OK)
        {
            RTLSDRIn = NULL;
            ESP_LOGI("", "usb_host_transfer_alloc In fail: %x", err);
            return;
        }
        RTLSDRIn->device_handle = Device_Handle;
        RTLSDRIn->bEndpointAddress = endpoint->bEndpointAddress;
        RTLSDRIn->callback = keyboard_transfer_cb;
        RTLSDRIn->context = NULL;
        isRTLSDR_Ready = true;
        RTLSDRInterval = endpoint->bInterval;
        ESP_LOGI("", "USB boot keyboard ready");
    }
    else {
        ESP_LOGI("", "Ignoring interrupt Out endpoint");
    }
}

void show_config_desc_full(const usb_config_desc_t* config_desc)
{
    // Full decode of config desc.
    const uint8_t* p = &config_desc->val[0];
    static uint8_t USB_Class = 0;
    uint8_t bLength;
    for (int i = 0; i < config_desc->wTotalLength; i += bLength, p += bLength) {
        bLength = *p;
        if ((i + bLength) <= config_desc->wTotalLength) {
            const uint8_t bDescriptorType = *(p + 1);
            switch (bDescriptorType) {
            case USB_B_DESCRIPTOR_TYPE_DEVICE:
                ESP_LOGI("", "USB Device Descriptor should not appear in config");
                break;
            case USB_B_DESCRIPTOR_TYPE_CONFIGURATION:
                show_config_desc(p);
                break;
            case USB_B_DESCRIPTOR_TYPE_STRING:
                ESP_LOGI("", "USB string desc TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_INTERFACE:
                USB_Class = show_interface_desc(p);
                if (USB_Class == USB_CLASS_HID)
                {
                    check_interface_desc_boot_keyboard(p);
                }
                else if (USB_Class == USB_CLASS_VENDOR_SPEC)
                {
                    check_interface_desc_boot_rtlsdr(p);
                }
                break;
            case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
                show_endpoint_desc(p);
                if (isKeyboard && KeyboardIn == NULL) prepare_endpoint(p);
                else if (isRTLSDR && RTLSDRIn == NULL) prepare_endpoint_rtlsdr(p);
                break;
            case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
                // Should not be config config?
                ESP_LOGI("", "USB device qual desc TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
                // Should not be config config?
                ESP_LOGI("", "USB Other Speed TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER:
                // Should not be config config?
                ESP_LOGI("", "USB Interface Power TBD");
                break;
            case 0x21:
                if (USB_Class == USB_CLASS_HID) {
                    show_hid_desc(p);
                }
                break;
            default:
                ESP_LOGI("", "Unknown USB Descriptor Type: 0x%x", bDescriptorType);
                break;
            }
        }
        else {
            ESP_LOGI("", "USB Descriptor invalid");
            return;
        }
    }
}

//==================================================================================================
void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    esp_err_t err;
    switch (event_msg->event)
    {
        /**< A new device has been enumerated and added to the USB Host Library */
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI("", "New device address: %d", event_msg->new_dev.address);
        err = usb_host_device_open(Client_Handle, event_msg->new_dev.address, &Device_Handle);
        if (err != ESP_OK) ESP_LOGI("", "usb_host_device_open: %x", err);

        usb_device_info_t dev_info;
        err = usb_host_device_info(Device_Handle, &dev_info);
        if (err != ESP_OK) ESP_LOGI("", "usb_host_device_info: %x", err);
        ESP_LOGI("", "speed: %d dev_addr %d vMaxPacketSize0 %d bConfigurationValue %d",
            dev_info.speed, dev_info.dev_addr, dev_info.bMaxPacketSize0,
            dev_info.bConfigurationValue);

        const usb_device_desc_t* dev_desc;
        err = usb_host_get_device_descriptor(Device_Handle, &dev_desc);
        if (err != ESP_OK) ESP_LOGI("", "usb_host_get_device_desc: %x", err);
        show_dev_desc(dev_desc);

        const usb_config_desc_t* config_desc;
        err = usb_host_get_active_config_descriptor(Device_Handle, &config_desc);
        if (err != ESP_OK) ESP_LOGI("", "usb_host_get_config_desc: %x", err);
        (*_USB_host_enumerate)(config_desc);
        break;
        /**< A device opened by the client is now gone */
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI("", "Device Gone handle: %x", event_msg->dev_gone.dev_hdl);
        break;
    default:
        ESP_LOGI("", "Unknown value %d", event_msg->event);
        break;
    }
}

void usbh_setup(usb_host_enum_cb_t enumeration_cb)
{
    const usb_host_config_t config = {
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&config);
    ESP_LOGI("", "usb_host_install: %x", err);

    const usb_host_client_config_t client_config = {
      .is_synchronous = false,
      .max_num_event_msg = 5,
      .async = {
          .client_event_callback = _client_event_callback,
          .callback_arg = Client_Handle
      }
    };
    err = usb_host_client_register(&client_config, &Client_Handle);
    ESP_LOGI("", "usb_host_client_register: %x", err);

    _USB_host_enumerate = enumeration_cb;
}

void usbh_task(void)
{
    uint32_t event_flags;
    static bool all_clients_gone = false;
    static bool all_dev_free = false;

    esp_err_t err = usb_host_lib_handle_events(HOST_EVENT_TIMEOUT, &event_flags);
    if (err == ESP_OK) {
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI("", "No more clients");
            all_clients_gone = true;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI("", "No more devices");
            all_dev_free = true;
        }
    }
    else {
        if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGI("", "usb_host_lib_handle_events: %x flags: %x", err, event_flags);
        }
    }

    err = usb_host_client_handle_events(Client_Handle, CLIENT_EVENT_TIMEOUT);
    if ((err != ESP_OK) && (err != ESP_ERR_TIMEOUT)) {
        ESP_LOGI("", "usb_host_client_handle_events: %x", err);
    }
}
//==================================================================================================