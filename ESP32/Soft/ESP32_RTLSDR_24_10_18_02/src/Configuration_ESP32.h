#pragma once

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
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