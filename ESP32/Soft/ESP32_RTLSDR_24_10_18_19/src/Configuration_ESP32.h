#pragma once

#include "usb/usb_host.h"


#define CLIENT_NUM_EVENT_MSG      5     // Максимальное количество сообщений о событиях, которые можно сохранить

#define ACTION_OPEN_DEV        0x01
#define ACTION_GET_DEV_INFO    0x02
#define ACTION_GET_DEV_DESC    0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC    0x10
#define ACTION_CLOSE_DEV       0x20
#define ACTION_EXIT            0x40

static const char* TAG_DAEMON = "DAEMON";
static const char* TAG_CLASS = "CLASS";
static const char* TAG_ADSB = "ADSB";


typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t device_hdl;
    uint32_t actions;
} class_driver_t;

typedef struct
{
    bool is_adsb;
    uint8_t* response_buf;
    bool is_done;
    bool is_success;
    int bytes_transferred;
    usb_transfer_t* transfer;
} class_adsb_dev;