#pragma once
#include "usb/usb_host.h"
#include <esp_task_wdt.h>

//#include "rtl-sdr.h"


static const char* TAG_DAEMON = "DAEMON";
static const char* TAG_CLASS = "CLASS";

#define CLIENT_NUM_EVENT_MSG        5

#define ACTION_OPEN_DEV             0x01
#define ACTION_GET_DEV_INFO         0x02
#define ACTION_GET_DEV_DESC         0x04
#define ACTION_GET_CONFIG_DESC      0x08
#define ACTION_GET_STR_DESC         0x10
#define ACTION_CLOSE_DEV            0x20
#define ACTION_OPEN_RTLSDR          0x30
#define ACTION_EXIT                 0x40

//#define DEFAULT_BUF_LENGTH (14 * 16384)
#define DEFAULT_BUF_LENGTH (16 * 32 * 512)// 262144


//typedef struct rtlsdr_dev rtlsdr_dev_t;
//
//static rtlsdr_dev_t* rtldev = NULL;

typedef struct
{
    usb_host_client_handle_t Client_Handle;
    uint8_t dev_addr;
    usb_device_handle_t Device_Handle;
    uint32_t actions;
} class_driver_t;

const size_t USB_HID_DESC_SIZE = 9;

typedef union {
    struct {
        uint8_t bLength;                    /**< Размер дескриптора в байтах */
        uint8_t bDescriptorType;            /**< Постоянное имя, указывающее тип дескриптора HID. */
        uint16_t bcdHID;                    /**< Номер версии спецификации USB HID в двоично-десятичном коде (например, 2.10 — это 210H) */
        uint8_t bCountryCode;               /**< Числовое выражение, идентифицирующее код страны локализованного оборудования. */
        uint8_t bNumDescriptor;             /**< Числовое выражение, указывающее количество дескрипторов класса. */
        uint8_t bHIDDescriptorType;         /**< Имя константы, идентифицирующее тип дескриптора класса. См. раздел 7.1.2: Set_Descriptor Запрос таблицы констант дескриптора класса. */
        uint16_t wHIDDescriptorLength;      /**< Числовое выражение, представляющее собой общий размер дескриптора отчета. */
        uint8_t bHIDDescriptorTypeOpt;      /**< Необязательное имя константы, идентифицирующее тип дескриптора класса. См. раздел 7.1.2: Запрос Set_Descriptor для таблицы констант дескриптора класса. */
        uint16_t wHIDDescriptorLengthOpt;   /**< Необязательное числовое выражение, представляющее собой общий размер дескриптора отчета. */
    } USB_DESC_ATTR;
    uint8_t val[USB_HID_DESC_SIZE];
} usb_hid_desc_t;

const TickType_t HOST_EVENT_TIMEOUT = 1;
const TickType_t CLIENT_EVENT_TIMEOUT = 1;