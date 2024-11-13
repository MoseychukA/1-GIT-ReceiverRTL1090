#include "usb/usb_host.h"
#include "rtl-sdr.h"

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
    uint8_t *response_buf;
    bool is_done;
    bool is_success;
    int bytes_transferred;
    usb_transfer_t *transfer;
} class_adsb_dev;

const char *TAG_ADSB = "ADSB";
void init_adsb_dev();
void bulk_transfer_read_cb(usb_transfer_t *transfer);
void transfer_read_cb(usb_transfer_t *transfer);
int esp_libusb_bulk_transfer(class_driver_t *driver_obj, unsigned char endpoint, unsigned char *data, int length, int *transferred, unsigned int timeout);
int esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout);
void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str);


//===============================================================================
#define CLIENT_NUM_EVENT_MSG 5

#define ACTION_OPEN_DEV 0x01
#define ACTION_GET_DEV_INFO 0x02
#define ACTION_GET_DEV_DESC 0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC 0x10
#define ACTION_CLOSE_DEV 0x20
#define ACTION_EXIT 0x40

#define CTRL_OUT (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_OUT)
#define CTRL_IN (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_IN)

#define R820T_I2C_ADDR 0x34
#define R82XX_CHECK_ADDR 0x00
#define R82XX_CHECK_VAL 0x69

#define CTRL_TIMEOUT 300

#define R82XX_IF_FREQ 3570000


static const char* TAG = "CLASS";

static class_adsb_dev *adsbdev = NULL;
static rtlsdr_dev_t* rtldev = NULL;

void client_event_cb(const usb_host_client_event_msg_t* event_msg, void* arg);
void action_open_dev(class_driver_t* driver_obj);
void action_get_info(class_driver_t* driver_obj);
void action_get_dev_desc(class_driver_t* driver_obj);
void action_get_config_desc(class_driver_t* driver_obj);
void transfer_cb(usb_transfer_t* transfer);
void transfer_read_cb(usb_transfer_t* transfer);
#define USB_SETUP_PACKET_INIT_CONTROL(setup_pkt_ptr, bm_reqtype, b_request, w_value, w_index, w_length) ({ \
    (setup_pkt_ptr)->bmRequestType = bm_reqtype;                                                           \
    (setup_pkt_ptr)->bRequest = b_request;                                                                 \
    (setup_pkt_ptr)->wValue = w_value;                                                                     \
    (setup_pkt_ptr)->wIndex = w_index;                                                                     \
    (setup_pkt_ptr)->wLength = w_length;                                                                   \
})
enum usb_reg
{
    USB_SYSCTL = 0x2000,
    USB_CTRL = 0x2010,
    USB_STAT = 0x2014,
    USB_EPA_CFG = 0x2144,
    USB_EPA_CTL = 0x2148,
    USB_EPA_MAXPKT = 0x2158,
    USB_EPA_MAXPKT_2 = 0x215a,
    USB_EPA_FIFO_CFG = 0x2160,
};
enum sys_reg
{
    DEMOD_CTL = 0x3000,
    GPO = 0x3001,
    GPI = 0x3002,
    GPOE = 0x3003,
    GPD = 0x3004,
    SYSINTE = 0x3005,
    SYSINTS = 0x3006,
    GP_CFG0 = 0x3007,
    GP_CFG1 = 0x3008,
    SYSINTE_1 = 0x3009,
    SYSINTS_1 = 0x300a,
    DEMOD_CTL_1 = 0x300b,
    IR_SUSPEND = 0x300c,
};

bool libusb_control_transfer(class_driver_t* driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char* data, uint16_t wLength, unsigned int timeout);
bool rtlsdr_read_array(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);
bool rtlsdr_write_array(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);

enum blocks
{
    DEMODB = 0,
    USBB = 1,
    SYSB = 2,
    TUNB = 3,
    ROMB = 4,
    IRB = 5,
    IICB = 6,
};

uint8_t rtlsdr_i2c_read_reg(class_driver_t* driver_obj, uint8_t i2c_addr, uint8_t reg);
bool rtlsdr_write_reg(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint16_t val, uint8_t len);
uint16_t rtlsdr_demod_read_reg(class_driver_t* driver_obj, uint8_t page, uint16_t addr, uint8_t len);
int rtlsdr_demod_write_reg(class_driver_t* driver_obj, uint8_t page, uint16_t addr, uint16_t val, uint8_t len);
static const int fir_default[16] = {
    -54, -36, -41, -40, -32, -14, 14, 53,  /* 8 bit signed */
    101, 156, 215, 273, 327, 372, 404, 421 /* 12 bit signed */
};

int rtlsdr_set_fir(class_driver_t* driver_obj);
void rtlsdr_init_baseband(class_driver_t* driver_obj);
void rtlsdr_set_i2c_repeater(class_driver_t* driver_obj, int on);

#define TWO_POW(n) ((double)(1ULL << (n)))
#define DEF_RTL_XTAL_FREQ 28800000

int rtlsdr_set_if_freq(class_driver_t* driver_obj, uint32_t freq);
void action_get_str_desc(class_driver_t* driver_obj);
void aciton_close_dev(class_driver_t* driver_obj);
void class_driver_task(void* arg);
