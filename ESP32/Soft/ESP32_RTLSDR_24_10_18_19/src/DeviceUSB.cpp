#include "DeviceUSB.h"
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
#include "show_desc.hpp"

//const TickType_t HOST_EVENT_TIMEOUT = 1;
//const TickType_t CLIENT_EVENT_TIMEOUT = 1;
//
//usb_host_client_handle_t Client_Handle;
//usb_device_handle_t Device_Handle;
//typedef void (*usb_host_enum_cb_t)(const usb_config_desc_t* config_desc);
//static usb_host_enum_cb_t _USB_host_enumerate;

bool isRTLSDR = false;
bool isRTLSDRReady = false;
uint8_t RTLSDRInterval;
bool isRTLSDRPolling = false;
elapsedMillis RTLSDRTimer;



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

void RTLSDR_transfer_cb(usb_transfer_t* transfer)
{
    if (Device_Handle == transfer->device_handle)
    {
        isRTLSDRPolling = false;
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
                ESP_LOGI("", "RTLSDR boot hid transfer too short or long");
            }
        }
        else {
            ESP_LOGI("", "transfer->status %d", transfer->status);
        }
    }
}

void check_interface_desc_boot_RTLSDR(const void* p)
{
    const usb_intf_desc_t* intf = (const usb_intf_desc_t*)p;

    if ((intf->bInterfaceClass == USB_CLASS_HID) && (intf->bInterfaceSubClass == 1) && (intf->bInterfaceProtocol == 1))
    {
        isRTLSDR = true;
        ESP_LOGI("", "Claiming a boot RTLSDR!");
        esp_err_t err = usb_host_interface_claim(Client_Handle, Device_Handle, intf->bInterfaceNumber, intf->bAlternateSetting);
        if (err != ESP_OK) ESP_LOGI("", "usb_host_interface_claim failed: %x", err);
    }
}

void prepare_endpoint(const void* p)
{
    const usb_ep_desc_t* endpoint = (const usb_ep_desc_t*)p;
    esp_err_t err;

    // must be interrupt for HID
    if ((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_INT)
    {
        ESP_LOGI("", "Not interrupt endpoint: 0x%02x", endpoint->bmAttributes);
        return;
    }
    if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK)
    {
        err = usb_host_transfer_alloc(RTLSDR_IN_BUFFER_SIZE, 0, &RTLSDRIn);
        if (err != ESP_OK) {
            RTLSDRIn = NULL;
            ESP_LOGI("", "usb_host_transfer_alloc In fail: %x", err);
            return;
        }
        RTLSDRIn->device_handle = Device_Handle;
        RTLSDRIn->bEndpointAddress = endpoint->bEndpointAddress;
        RTLSDRIn->callback = RTLSDR_transfer_cb;
        RTLSDRIn->context = NULL;
        isRTLSDRReady = true;
        RTLSDRInterval = endpoint->bInterval;
        ESP_LOGI("", "USB boot RTLSDR ready");
    }
    else
    {
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
                check_interface_desc_boot_RTLSDR(p);
                break;
            case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
                show_endpoint_desc(p);
                if (isRTLSDR && RTLSDRIn == NULL) prepare_endpoint(p);
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
//===============================================================================

void usbh_setup(usb_host_enum_cb_t enumeration_cb)
{
    ESP_LOGI(TAG_DAEMON, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    vTaskDelay(10); // Short delay to let client task spin up

    class_driver_t driver_obj = { 0 };

    const usb_host_client_config_t client_config = {
     .is_synchronous = false,
     .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
     .async = {
         .client_event_callback = _client_event_callback,
         .callback_arg = (void*)&driver_obj
     }
    };
    int err = usb_host_client_register(&client_config, &driver_obj.client_hdl);
    ESP_LOGI(TAG_DAEMON, "usb_host_client_register: %x", err);
    //_USB_host_enumerate = enumeration_cb;
}

void usbh_init()
{
    usbh_setup(show_config_desc_full);


}