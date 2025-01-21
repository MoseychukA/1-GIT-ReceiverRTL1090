#pragma once
#include "usb/usb_host.h"
#include <esp_task_wdt.h>

//#include "rtl-sdr.h"


#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3


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


typedef struct rtlsdr_dev rtlsdr_dev_t;

static rtlsdr_dev_t* rtldev = NULL;

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;

