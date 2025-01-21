#pragma once


#include "main.h"
#include "tuner_r82xx.h"

/* two raised to the power of n */
#define TWO_POW(n) ((double)(1ULL << (n)))

static const char* TAG_LIBRTLSDR = "LIBRTLSDR";

enum rtlsdr_tuner
{
    RTLSDR_TUNER_UNKNOWN = 0,
    RTLSDR_TUNER_R820T,
    RTLSDR_TUNER_R828D
};


typedef struct rtlsdr_tuner_iface
{
    /* tuner interface */
    int (*init)(void*);
    int (*exit)(void*);
    int (*set_freq)(void*, uint32_t freq /* Hz */);
    int (*set_bw)(void*, int bw /* Hz */);
    int (*set_gain)(void*, int gain /* tenth dB */);
    int (*set_if_gain)(void*, int stage, int gain /* tenth dB */);
    int (*set_gain_mode)(void*, int manual);
} rtlsdr_tuner_iface_t;

enum rtlsdr_async_status
{
    RTLSDR_INACTIVE = 0,
    RTLSDR_CANCELING,
    RTLSDR_RUNNING
};



#define FIR_LEN 16


static const int fir_default[FIR_LEN] = {
    -54, -36, -41, -40, -32, -14, 14, 53,  /* 8 bit signed */
    101, 156, 215, 273, 327, 372, 404, 421 /* 12 bit signed */
};

struct rtlsdr_dev
{
    class_driver_t* driver_obj;
    uint32_t xfer_buf_num;
    uint32_t xfer_buf_len;
    struct libusb_transfer** xfer;
    unsigned char** xfer_buf;
   //!! rtlsdr_read_async_cb_t cb;
    void* cb_ctx;
    enum rtlsdr_async_status async_status;
    int async_cancel;
    int use_zerocopy;
    /* rtl demod context */
    uint32_t rate;     /* Hz */
    uint32_t rtl_xtal; /* Hz */
    int fir[FIR_LEN];
    int direct_sampling;
    /* tuner context */
    enum rtlsdr_tuner tuner_type;
    rtlsdr_tuner_iface_t* tuner;
    uint32_t tun_xtal; /* Hz */
    uint32_t freq;     /* Hz */
    uint32_t bw;
    uint32_t offs_freq; /* Hz */
    int corr;           /* ppm */
    int gain;           /* tenth dB */
    struct r82xx_config r82xx_c;
    struct r82xx_priv r82xx_p;
    /* status */
    int dev_lost;
    int driver_active;
    unsigned int xfer_errors;
};

int r820t_init(void* dev);
int r820t_exit(void* dev);
int r820t_set_freq(void* dev, uint32_t freq);
int r820t_set_bw(void* dev, int bw);
int r820t_set_gain(void* dev, int gain);
int r820t_set_gain_mode(void* dev, int manual);

/* definition order must match enum rtlsdr_tuner */
static rtlsdr_tuner_iface_t tuners[] = {
    {
        NULL, NULL, NULL, NULL, NULL, NULL, NULL /* dummy for unknown tuners */
    },
    {r820t_init, r820t_exit,
     r820t_set_freq, r820t_set_bw, r820t_set_gain, NULL,
     r820t_set_gain_mode},
    {r820t_init, r820t_exit,
     r820t_set_freq, r820t_set_bw, r820t_set_gain, NULL,
     r820t_set_gain_mode},
};



typedef struct rtlsdr_dongle
{
    uint16_t vid;
    uint16_t pid;
    const char* name;
} rtlsdr_dongle_t;


/*
 * Please add your device here and send a patch to osmocom-sdr@lists.osmocom.org
 */
static rtlsdr_dongle_t known_devices[] = {
    {0x0bda, 0x2832, "Generic RTL2832U"},
    {0x0bda, 0x2838, "Generic RTL2832U OEM"},
    {0x0413, 0x6680, "DigitalNow Quad DVB-T PCI-E card"},
    {0x0413, 0x6f0f, "Leadtek WinFast DTV Dongle mini D"},
    {0x0458, 0x707f, "Genius TVGo DVB-T03 USB dongle (Ver. B)"},
    {0x0ccd, 0x00a9, "Terratec Cinergy T Stick Black (rev 1)"},
    {0x0ccd, 0x00b3, "Terratec NOXON DAB/DAB+ USB dongle (rev 1)"},
    {0x0ccd, 0x00b4, "Terratec Deutschlandradio DAB Stick"},
    {0x0ccd, 0x00b5, "Terratec NOXON DAB Stick - Radio Energy"},
    {0x0ccd, 0x00b7, "Terratec Media Broadcast DAB Stick"},
    {0x0ccd, 0x00b8, "Terratec BR DAB Stick"},
    {0x0ccd, 0x00b9, "Terratec WDR DAB Stick"},
    {0x0ccd, 0x00c0, "Terratec MuellerVerlag DAB Stick"},
    {0x0ccd, 0x00c6, "Terratec Fraunhofer DAB Stick"},
    {0x0ccd, 0x00d3, "Terratec Cinergy T Stick RC (Rev.3)"},
    {0x0ccd, 0x00d7, "Terratec T Stick PLUS"},
    {0x0ccd, 0x00e0, "Terratec NOXON DAB/DAB+ USB dongle (rev 2)"},
    {0x1554, 0x5020, "PixelView PV-DT235U(RN)"},
    {0x15f4, 0x0131, "Astrometa DVB-T/DVB-T2"},
    {0x15f4, 0x0133, "HanfTek DAB+FM+DVB-T"},
    {0x185b, 0x0620, "Compro Videomate U620F"},
    {0x185b, 0x0650, "Compro Videomate U650F"},
    {0x185b, 0x0680, "Compro Videomate U680F"},
    {0x1b80, 0xd393, "GIGABYTE GT-U7300"},
    {0x1b80, 0xd394, "DIKOM USB-DVBT HD"},
    {0x1b80, 0xd395, "Peak 102569AGPK"},
    {0x1b80, 0xd397, "KWorld KW-UB450-T USB DVB-T Pico TV"},
    {0x1b80, 0xd398, "Zaapa ZT-MINDVBZP"},
    {0x1b80, 0xd39d, "SVEON STV20 DVB-T USB & FM"},
    {0x1b80, 0xd3a4, "Twintech UT-40"},
    {0x1b80, 0xd3a8, "ASUS U3100MINI_PLUS_V2"},
    {0x1b80, 0xd3af, "SVEON STV27 DVB-T USB & FM"},
    {0x1b80, 0xd3b0, "SVEON STV21 DVB-T USB & FM"},
    {0x1d19, 0x1101, "Dexatek DK DVB-T Dongle (Logilink VG0002A)"},
    {0x1d19, 0x1102, "Dexatek DK DVB-T Dongle (MSI DigiVox mini II V3.0)"},
    {0x1d19, 0x1103, "Dexatek Technology Ltd. DK 5217 DVB-T Dongle"},
    {0x1d19, 0x1104, "MSI DigiVox Micro HD"},
    {0x1f4d, 0xa803, "Sweex DVB-T USB"},
    {0x1f4d, 0xb803, "GTek T803"},
    {0x1f4d, 0xc803, "Lifeview LV5TDeluxe"},
    {0x1f4d, 0xd286, "MyGica TD312"},
    {0x1f4d, 0xd803, "PROlectrix DV107669"},
};

#define DEFAULT_BUF_NUMBER 15
//#define DEFAULT_BUF_LENGTH (16 * 32 * 512)

#define DEF_RTL_XTAL_FREQ 28800000
#define MIN_RTL_XTAL_FREQ (DEF_RTL_XTAL_FREQ - 1000)
#define MAX_RTL_XTAL_FREQ (DEF_RTL_XTAL_FREQ + 1000)

#define CTRL_TIMEOUT 300
#define BULK_TIMEOUT 0

#define EEPROM_ADDR 0xa0

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


//void esp_action_get_dev_desc(rtlsdr_dev_t* dev);
//int rtlsdr_open(rtlsdr_dev_t** out_dev, uint8_t index, usb_host_client_handle_t client_hdl);
//int rtlsdr_close(rtlsdr_dev_t* dev);
//
//int rtlsdr_read_array(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);
//int rtlsdr_write_array(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);
//int rtlsdr_i2c_write_reg(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t reg, uint8_t val);
//uint8_t rtlsdr_i2c_read_reg(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t reg);
//int rtlsdr_i2c_write(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t* buffer, int len);
//int rtlsdr_i2c_read(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t* buffer, int len);
//uint16_t rtlsdr_read_reg(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint8_t len);
//int rtlsdr_write_reg(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint16_t val, uint8_t len);
//uint16_t rtlsdr_demod_read_reg(rtlsdr_dev_t* dev, uint8_t page, uint16_t addr, uint8_t len);
//int rtlsdr_demod_write_reg(rtlsdr_dev_t* dev, uint8_t page, uint16_t addr, uint16_t val, uint8_t len);
//void rtlsdr_set_gpio_bit(rtlsdr_dev_t* dev, uint8_t gpio, int val);
//void rtlsdr_set_gpio_output(rtlsdr_dev_t* dev, uint8_t gpio);
//void rtlsdr_set_i2c_repeater(rtlsdr_dev_t* dev, int on);
//int rtlsdr_set_fir(rtlsdr_dev_t* dev);
//void rtlsdr_init_baseband(rtlsdr_dev_t* dev);
//int rtlsdr_deinit_baseband(rtlsdr_dev_t* dev);
//static int rtlsdr_set_if_freq(rtlsdr_dev_t* dev, uint32_t freq);
//int rtlsdr_set_sample_freq_correction(rtlsdr_dev_t* dev, int ppm);
//int rtlsdr_set_xtal_freq(rtlsdr_dev_t* dev, uint32_t rtl_freq, uint32_t tuner_freq);
//int rtlsdr_get_xtal_freq(rtlsdr_dev_t* dev, uint32_t* rtl_freq, uint32_t* tuner_freq);
//int rtlsdr_get_usb_strings(rtlsdr_dev_t* dev, char* manufact, char* product, char* serial);
//int rtlsdr_write_eeprom(rtlsdr_dev_t* dev, uint8_t* data, uint8_t offset, uint16_t len);
//int rtlsdr_read_eeprom(rtlsdr_dev_t* dev, uint8_t* data, uint8_t offset, uint16_t len);
//int rtlsdr_set_center_freq(rtlsdr_dev_t* dev, uint32_t freq);
//uint32_t rtlsdr_get_center_freq(rtlsdr_dev_t* dev);
//int rtlsdr_set_freq_correction(rtlsdr_dev_t* dev, int ppm);
//int rtlsdr_get_freq_correction(rtlsdr_dev_t* dev);
//enum rtlsdr_tuner rtlsdr_get_tuner_type(rtlsdr_dev_t* dev);
//int rtlsdr_get_tuner_gains(rtlsdr_dev_t* dev, int* gains);
//int rtlsdr_set_tuner_bandwidth(rtlsdr_dev_t* dev, uint32_t bw);
//int rtlsdr_set_tuner_gain(rtlsdr_dev_t* dev, int gain);
//int rtlsdr_get_tuner_gain(rtlsdr_dev_t* dev);
//int rtlsdr_set_tuner_if_gain(rtlsdr_dev_t* dev, int stage, int gain);
//int rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t* dev, int mode);
//int rtlsdr_set_sample_rate(rtlsdr_dev_t* dev, uint32_t samp_rate);
//uint32_t rtlsdr_get_sample_rate(rtlsdr_dev_t* dev);
//int rtlsdr_set_testmode(rtlsdr_dev_t* dev, int on);
//int rtlsdr_set_agc_mode(rtlsdr_dev_t* dev, int on);
//int rtlsdr_set_direct_sampling(rtlsdr_dev_t* dev, int on);
//int rtlsdr_get_direct_sampling(rtlsdr_dev_t* dev);
//int rtlsdr_set_offset_tuning(rtlsdr_dev_t* dev, int on);
//int rtlsdr_get_offset_tuning(rtlsdr_dev_t* dev);
////!!static rtlsdr_dongle_t* find_known_device(uint16_t vid, uint16_t pid);
//void esp_action_get_dev_desc(rtlsdr_dev_t* dev);
//int rtlsdr_open(rtlsdr_dev_t** out_dev, uint8_t index, usb_host_client_handle_t client_hdl);
//int rtlsdr_close(rtlsdr_dev_t* dev);
//int rtlsdr_reset_buffer(rtlsdr_dev_t* dev);
//int rtlsdr_read_sync(rtlsdr_dev_t* dev, void* buf, int len, int* n_read);
//int rtlsdr_cancel_async(rtlsdr_dev_t* dev);
//uint32_t rtlsdr_get_tuner_clock(void* dev);
//int rtlsdr_i2c_write_fn(void* dev, uint8_t addr, uint8_t* buf, int len);
//int rtlsdr_i2c_read_fn(void* dev, uint8_t addr, uint8_t* buf, int len);
//int rtlsdr_set_bias_tee_gpio(rtlsdr_dev_t* dev, int gpio, int on);
//int rtlsdr_set_bias_tee(rtlsdr_dev_t* dev, int on);

//==================================================================================