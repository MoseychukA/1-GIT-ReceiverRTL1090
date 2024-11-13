#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "src/rtl-sdr.h"
#include <esp_task_wdt.h>
#include "src/esp_libusb.h"
#include "src/tuner_r82xx.h"

#include "show_desc.hpp"
#include "usbhhelp.hpp"


#define DAEMON_TASK_PRIORITY 2
#define CLASS_TASK_PRIORITY  3

static const char* TAG_DAEMON = "DAEMON";
static const char* TAG_CLASS = "CLASS";
static const char* TAG_ADSB = "ADSB";

static class_adsb_dev* adsbdev = NULL;
static rtlsdr_dev_t* rtldev = NULL;


#define CLIENT_NUM_EVENT_MSG      5     // Максимальное количество сообщений о событиях, которые можно сохранить

#define ACTION_OPEN_DEV        0x01
#define ACTION_GET_DEV_INFO    0x02
#define ACTION_GET_DEV_DESC    0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC    0x10
#define ACTION_CLOSE_DEV       0x20
#define ACTION_EXIT            0x40

#define DEFAULT_BUF_LENGTH (14 * 16384) 

#define TWO_POW(n) ((double)(1ULL << (n)))
#define DEF_RTL_XTAL_FREQ 28800000
#define CTRL_TIMEOUT           300
#define BULK_TIMEOUT             0


#define USB_SETUP_PACKET_INIT_CONTROL(setup_pkt_ptr, bm_reqtype, b_request, w_value, w_index, w_length) ({ \
    (setup_pkt_ptr)->bmRequestType = bm_reqtype;                                                           \
    (setup_pkt_ptr)->bRequest = b_request;                                                                 \
    (setup_pkt_ptr)->wValue = w_value;                                                                     \
    (setup_pkt_ptr)->wIndex = w_index;                                                                     \
    (setup_pkt_ptr)->wLength = w_length;                                                                   \
})
enum usb_reg
{
    USB_SYSCTL       = 0x2000,
    USB_CTRL         = 0x2010,
    USB_STAT         = 0x2014,
    USB_EPA_CFG      = 0x2144,
    USB_EPA_CTL      = 0x2148,
    USB_EPA_MAXPKT   = 0x2158,
    USB_EPA_MAXPKT_2 = 0x215a,
    USB_EPA_FIFO_CFG = 0x2160,
};
enum sys_reg
{
    DEMOD_CTL   = 0x3000,
    GPO         = 0x3001,
    GPI         = 0x3002,
    GPOE        = 0x3003,
    GPD         = 0x3004,
    SYSINTE     = 0x3005,
    SYSINTS     = 0x3006,
    GP_CFG0     = 0x3007,
    GP_CFG1     = 0x3008,
    SYSINTE_1   = 0x3009,
    SYSINTS_1   = 0x300a,
    DEMOD_CTL_1 = 0x300b,
    IR_SUSPEND  = 0x300c,
};

enum blocks
{
    DEMODB = 0,
    USBB   = 1,
    SYSB   = 2,
    TUNB   = 3,
    ROMB   = 4,
    IRB    = 5,
    IICB   = 6,
};


static const int fir_default[16] = {
    -54, -36, -41, -40, -32, -14, 14, 53,  /* 8 bit signed */
    101, 156, 215, 273, 327, 372, 404, 421 /* 12 bit signed */
};


static void host_lib_daemon_task(void* arg);

void client_event_cb(const usb_host_client_event_msg_t* event_msg, void* arg);
void action_open_dev(class_driver_t* driver_obj);
void action_get_info(class_driver_t* driver_obj);
void action_get_dev_desc(class_driver_t* driver_obj);
void action_get_config_desc(class_driver_t* driver_obj);
void transfer_cb(usb_transfer_t* transfer);
void transfer_read_cb(usb_transfer_t* transfer);

bool libusb_control_transfer(class_driver_t* driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char* data, uint16_t wLength, unsigned int timeout);
bool rtlsdr_read_array(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);
bool rtlsdr_write_array(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);

uint8_t rtlsdr_i2c_read_reg(class_driver_t* driver_obj, uint8_t i2c_addr, uint8_t reg);
bool rtlsdr_write_reg(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint16_t val, uint8_t len);
uint16_t rtlsdr_demod_read_reg(class_driver_t* driver_obj, uint8_t page, uint16_t addr, uint8_t len);
int rtlsdr_demod_write_reg(class_driver_t* driver_obj, uint8_t page, uint16_t addr, uint16_t val, uint8_t len);

int rtlsdr_set_fir(class_driver_t* driver_obj);
void rtlsdr_init_baseband(class_driver_t* driver_obj); 
void rtlsdr_set_i2c_repeater(class_driver_t* driver_obj, int on);

int rtlsdr_set_if_freq(class_driver_t* driver_obj, uint32_t freq);
void action_get_str_desc(class_driver_t* driver_obj);
void aciton_close_dev(class_driver_t* driver_obj);
void class_driver_task(void* arg);

//===============================================================================
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
                break;
            case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
                show_endpoint_desc(p);
                break;
            case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
                // Should not be in config?
                ESP_LOGI("", "USB device qual desc TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
                // Should not be in config?
                ESP_LOGI("", "USB Other Speed TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER:
                // Should not be in config?
                ESP_LOGI("", "USB Interface Power TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_OTG:
                // Should not be in config?
                ESP_LOGI("", "USB OTG TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_DEBUG:
                // Should not be in config?
                ESP_LOGI("", "USB DEBUG TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION:
                show_interface_assoc(p);
                break;
                // Class specific descriptors have overlapping values.
            case 0x21:
                switch (USB_Class) {
                case USB_CLASS_HID:
                    show_hid_desc(p);
                    break;
                case USB_CLASS_APP_SPEC:
                    ESP_LOGI("", "App Spec Class descriptor TBD");
                    break;
                default:
                    ESP_LOGI("", "Unknown USB Descriptor Type: 0x%02x Class: 0x%02x", bDescriptorType, USB_Class);
                    break;
                }
                break;
            case 0x22:
                switch (USB_Class) {
                default:
                    ESP_LOGI("", "Unknown USB Descriptor Type: 0x%02x Class: 0x%02x", bDescriptorType, USB_Class);
                    break;
                }
                break;
            case 0x23:
                switch (USB_Class) {
                default:
                    ESP_LOGI("", "Unknown USB Descriptor Type: 0x%02x Class: 0x%02x", bDescriptorType, USB_Class);
                    break;
                }
                break;
            case 0x24:
                switch (USB_Class) {
                case USB_CLASS_AUDIO:
                    ESP_LOGI("", "Audio Class Descriptor 0x24 TBD");
                    break;
                case USB_CLASS_COMM:
                    ESP_LOGI("", "Comm Class CS_INTERFACE 0x24 TBD");
                    break;
                default:
                    ESP_LOGI("", "Unknown USB Descriptor Type: 0x%02x Class: 0x%02x", bDescriptorType, USB_Class);
                    break;
                }
                break;
            case 0x25:
                switch (USB_Class) {
                case USB_CLASS_AUDIO:
                    ESP_LOGI("", "Audio Class Descriptor 0x25 TBD");
                    break;
                case USB_CLASS_COMM:
                    ESP_LOGI("", "Comm Class CS_ENDPOINT 0x25 TBD");
                    break;
                default:
                    ESP_LOGI("", "Unknown USB Descriptor Type: 0x%02x Class: 0x%02x", bDescriptorType, USB_Class);
                    break;
                }
                break;
            default:
                ESP_LOGI("", "Unknown USB Descriptor Type: 0x%02x Class: 0x%02x", bDescriptorType, USB_Class);
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

// The setup() function runs once each time the micro-controller starts
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 1000);

    esp_log_level_set("*", ESP_LOG_VERBOSE);  // Выводим отладочные сообщения

    String ver_soft = __FILE__;
    int val_srt = ver_soft.lastIndexOf('\\');
    ver_soft.remove(0, val_srt + 1);
    val_srt = ver_soft.lastIndexOf('.');
    ver_soft.remove(val_srt);
    Serial.println("\n*** Version " + ver_soft + "\n");

    esp_task_wdt_init(8, false);

    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    TaskHandle_t daemon_task_hdl;
    TaskHandle_t class_driver_task_hdl;
    // Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
        "daemon",
        4096,
        (void*)signaling_sem,
        DAEMON_TASK_PRIORITY,
        &daemon_task_hdl,
        0);
    // Create the class driver task
    xTaskCreatePinnedToCore(class_driver_task,
        "class",
        4096,
        (void*)signaling_sem,
        CLASS_TASK_PRIORITY,
        &class_driver_task_hdl,
        0);

    vTaskDelay(20); // Add a short delay to let the tasks run

    // Дождитесь завершения задач.
    for (int i = 0; i < 2; i++)
    {
        xSemaphoreTake(signaling_sem, portMAX_DELAY);
    }

    // Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);

}

// Add the main program code into the continuous loop() function
void loop()
{


}

static void host_lib_daemon_task(void* arg) // Настройка USB порта
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    vTaskDelay(10); // Short delay to let client task spin up
    ESP_LOGI(TAG_DAEMON, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG_DAEMON, "usb_host_install");

   // usbh_setup(show_config_desc_full);
    
    xSemaphoreGive(signaling_sem);      // Сигнал задаче драйвера класса о том, что хост-библиотека установлена
    vTaskDelay(20);                     // Короткая задержка, позволяющая клиентской задаче разогнаться

    bool has_clients = true;
    bool has_devices = true;

    while (has_clients || has_devices)  // Постоянно контролируем подключенное устройство
    {
       // usbh_task();
        uint32_t event_flags;
        /*  Обработка событий USB Host Library
        *
        * -Эта функция обрабатывает всю обработку USB Host Library и должна вызываться многократно в цикле
        * -Проверьте event_flags_ret, чтобы увидеть, установлены ли флаги, указывающие на определенные события USB Host Library
        * -Эта функция никогда не должна вызываться несколькими потоками одновременно
        *
        *@note Эта функция может блокировать
        * @param[in] timeout_ticks Тайм - аут в тиках для ожидания события
        * @param[out] event_flags_ret Флаги событий, указывающие, какое событие USB Host Library произошло.
        * @return esp_err_t
        */

        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));  // Эта функция обрабатывает всю обработку USB Host Library и должна вызываться многократно в цикле

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            has_devices = false;
        }
        esp_task_wdt_reset();
    }
    ESP_LOGI(TAG_DAEMON, "No more clients and devices");  // Больше никаких клиентов и устройств

    ESP_ERROR_CHECK(usb_host_uninstall());  // Удалите библиотеку USB Host.
    // Wait to be deleted
    xSemaphoreGive(signaling_sem);          // 
    vTaskSuspend(NULL);                     // Остановить задачу
}
//================================================================

void client_event_cb(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    class_driver_t* driver_obj = (class_driver_t*)arg;

    switch (event_msg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        if (driver_obj->dev_addr == 0)
        {
            driver_obj->dev_addr = event_msg->new_dev.address;
            rtlsdr_open(&rtldev, event_msg->new_dev.address, driver_obj->client_hdl);
            int r;
            r = rtlsdr_set_sample_rate(rtldev, 2000000);
            if (r < 0)
            {
                fprintf(stderr, "WARNING: Failed to set sample rate.\n");
            }
            else
            {
                fprintf(stderr, "Sampling at %u S/s.\n", 2000000);
            }
            r = rtlsdr_set_center_freq(rtldev, 1090000000);
            if (r < 0)
            {
                fprintf(stderr, "WARNING: Failed to set center freq.\n");
            }
            else
            {
                fprintf(stderr, "Tuned to %u Hz.\n", 1090000000);
            }
            r = rtlsdr_set_tuner_gain_mode(rtldev, 0);
            if (r != 0)
            {
                fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
            }
            else
            {
                fprintf(stderr, "Tuner gain set to automatic.\n");
            }
            // r = rtlsdr_set_freq_correction(rtldev, 0);
            // if (r < 0)
            // {
            //     fprintf(stderr, "WARNING: Failed to set ppm error.\n");
            // }
            // else
            // {
            //     fprintf(stderr, "Tuner error set to %i ppm.\n", 0);
            // }
            r = rtlsdr_reset_buffer(rtldev);
            if (r < 0)
            {
                fprintf(stderr, "WARNING: Failed to reset buffers.\n");
            }
            vTaskDelay(100); // Short delay to let client task spin up

            uint32_t out_block_size = DEFAULT_BUF_LENGTH;  //16384;
            uint8_t* buffer = (uint8_t*)malloc(out_block_size * sizeof(uint8_t));
            int n_read = 2;
            ESP_LOGI(TAG_CLASS, "Free memory: %ld bytes", esp_get_free_heap_size());
            // uint32_t bytes_to_read = 0;
            while (true)
            {
                r = rtlsdr_read_sync(rtldev, buffer, out_block_size, &n_read);
                ESP_LOGI(TAG_CLASS, "Free memory: %ld bytes", esp_get_free_heap_size());
                if (r < 0)
                {
                    fprintf(stderr, "WARNING: sync read failed.\n");
                    break;
                }

                // if ((bytes_to_read > 0) && (bytes_to_read < (uint32_t)n_read))
                // {
                //     n_read = bytes_to_read;
                //     do_exit = 1;
                // }
                for (int i = 0; i < out_block_size; i++)
                    fprintf(stdout, "%02X", buffer[i]);
                // if (fwrite(buffer, 1, n_read, stdout) != (size_t)n_read)
                // {
                //     fprintf(stderr, "Short write, samples lost, exiting!\n");
                //     break;
                // }

                if ((uint32_t)n_read < out_block_size)
                {
                    fprintf(stderr, "Short read, samples lost, exiting!\n");
                    break;
                }

                // if (bytes_to_read > 0)
                //     bytes_to_read -= n_read;
            }
            // esp_action_get_dev_desc(rtldev);
            // driver_obj->dev_hdl = *(usb_device_handle_t *)get_driver_obj(rtldev);
            // Open the device next
            // driver_obj->actions |= ACTION_OPEN_DEV;
            // ESP_LOGI(TAG_CLASS, "here");
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        if (driver_obj->dev_hdl != NULL)
        {
            // Cancel any other actions and close the device next
            driver_obj->actions = ACTION_CLOSE_DEV;
        }
        break;
    default:
        // Should never occur
        abort();
    }
}

void action_open_dev(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(TAG_CLASS, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    // Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

void action_get_info(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    ESP_LOGI(TAG_CLASS, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    ESP_LOGI(TAG_CLASS, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    // Todo: Print string descriptors

    // Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}

void action_get_dev_desc(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device descriptor");
    const usb_device_desc_t* dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
    if (dev_desc->idProduct == 0x2838 && dev_desc->idVendor == 0x0bda)
    {
        ESP_LOGI(TAG_ADSB, "Found ADSB Device");
        //!!adsbdev = calloc(1, sizeof(class_adsb_dev));
        adsbdev->is_adsb = true;
    }
    usb_print_device_descriptor(dev_desc);
    // Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;
}

void action_get_config_desc(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting config descriptor");
    const usb_config_desc_t* config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    usb_print_config_descriptor(config_desc, NULL);
    // Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}
void transfer_cb(usb_transfer_t* transfer)
{
    // This is function is called from within usb_host_client_handle_events(). Don't block and try to keep it short
    //  struct class_driver_control *class_driver_obj = (struct class_driver_control *)transfer->context;
    // printf("Transfer type %d %ld \n", transfer->actual_num_bytes, transfer->flags);
    // printf("Transfer status %d, actual number of bytes transferred %d, databuffer size %d\n", transfer->status, transfer->actual_num_bytes, transfer->data_buffer_size);
}

void transfer_read_cb(usb_transfer_t* transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done = true;
    //   printf("Transfer:Read type %d %ld \n", transfer->actual_num_bytes, transfer->flags);
    //  printf("Transfer:Read status %d, actual number of bytes transferred %d, databuffer size %d, %d\n", transfer->status, transfer->actual_num_bytes, transfer->data_buffer[8], adsbdev->response_buf[8]);
}


bool libusb_control_transfer(class_driver_t* driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char* data, uint16_t wLength, unsigned int timeout)
{
    usb_host_transfer_free(adsbdev->transfer);
    free(adsbdev->response_buf);
    size_t sizePacket = sizeof(usb_setup_packet_t) + wLength;
    usb_host_transfer_alloc(sizePacket, 0, &adsbdev->transfer);
    USB_SETUP_PACKET_INIT_CONTROL((usb_setup_packet_t*)adsbdev->transfer->data_buffer, bm_req_type, b_request, wValue, wIndex, wLength);
    adsbdev->transfer->num_bytes = sizePacket;
    adsbdev->transfer->device_handle = driver_obj->dev_hdl;
    adsbdev->transfer->timeout_ms = timeout;
    adsbdev->transfer->context = (void*)&driver_obj;
    adsbdev->transfer->callback = transfer_read_cb;
    adsbdev->is_done = false;
    adsbdev->response_buf = (uint8_t*)calloc(sizePacket, sizeof(uint8_t));

    if (bm_req_type == CTRL_OUT)
    {
        for (uint8_t i = 0; i < wLength; i++)
        {
            adsbdev->transfer->data_buffer[sizeof(usb_setup_packet_t) + i] = data[i];
        }
    }
    esp_err_t r = usb_host_transfer_submit_control(driver_obj->client_hdl, adsbdev->transfer);
    if (r != ESP_OK)
    {
        ESP_LOGI(TAG_ADSB, "libusb_control_transfer failed with %d", r);
        return false;
    }

    while (!adsbdev->is_done)
    {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }
    for (uint8_t i = 0; i < wLength; i++)
    {
        data[i] = adsbdev->response_buf[sizeof(usb_setup_packet_t) + i];
    }
    return true;
}
bool rtlsdr_read_array(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len)
{
    uint16_t index = (block << 8);
    unsigned char* data = (unsigned char*)calloc(len, sizeof(char));
    bool ret = libusb_control_transfer(driver_obj, CTRL_IN, 0, addr, index, data, len, CTRL_TIMEOUT);
    *array = data[0];
    return ret;
}

bool rtlsdr_write_array(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len)
{
    uint16_t index = (block << 8) | 0x10;
    unsigned char* data = (unsigned char*)calloc(len, sizeof(char));
    data[0] = *array;
    bool ret = libusb_control_transfer(driver_obj, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);
    free(data);
    return ret;
}


uint8_t rtlsdr_i2c_read_reg(class_driver_t* driver_obj, uint8_t i2c_addr, uint8_t reg)
{
    uint16_t addr = i2c_addr;
    uint8_t data = 0;

    rtlsdr_write_array(driver_obj, IICB, addr, &reg, 1);
    rtlsdr_read_array(driver_obj, IICB, addr, &data, 1);

    return data;
}

bool rtlsdr_write_reg(class_driver_t* driver_obj, uint8_t block, uint16_t addr, uint16_t val, uint8_t len)
{
    uint16_t index = (block << 8) | 0x10;
    unsigned char data[2];
    if (len == 1)
        data[0] = val & 0xff;
    else
        data[0] = val >> 8;
    data[1] = val & 0xff;
    return libusb_control_transfer(driver_obj, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);
}

uint16_t rtlsdr_demod_read_reg(class_driver_t* driver_obj, uint8_t page, uint16_t addr, uint8_t len)
{
    unsigned char data[2] = { 0, 0 };
    uint16_t index = page;
    uint16_t reg;
    addr = (addr << 8) | 0x20;
    libusb_control_transfer(driver_obj, CTRL_IN, 0, addr, index, data, len, CTRL_TIMEOUT);
    ESP_LOGI(TAG_ADSB, "rtlsdr_demod_read_reg val %d %d", data[0], data[1]);
    reg = (data[1] << 8) | data[0];
    return reg;
}

int rtlsdr_demod_write_reg(class_driver_t* driver_obj, uint8_t page, uint16_t addr, uint16_t val, uint8_t len)
{
    unsigned char data[2];
    uint16_t index = 0x10 | page;
    addr = (addr << 8) | 0x20;

    if (len == 1)
        data[0] = val & 0xff;
    else
        data[0] = val >> 8;

    data[1] = val & 0xff;

    libusb_control_transfer(driver_obj, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

    // size_t sizePacket = sizeof(usb_setup_packet_t) + len;
    // usb_transfer_t *transfer;
    // usb_host_transfer_alloc(sizePacket, 0, &transfer);
    // USB_SETUP_PACKET_INIT_CONTROL((usb_setup_packet_t *)transfer->data_buffer, CTRL_OUT, 0, addr, index, len);
    // transfer->data_buffer[8] = data[0];
    // if (len == 2)
    //     transfer->data_buffer[9] = data[1];
    // transfer->num_bytes = sizePacket;
    // transfer->device_handle = driver_obj->dev_hdl;
    // transfer->callback = transfer_cb;
    // transfer->context = (void *)&driver_obj;
    // int r = usb_host_transfer_submit_control(driver_obj->client_hdl, transfer);
    // if (r != ESP_OK)
    // {
    //     ESP_LOGI(TAG_ADSB, "rtlsdr_write_reg failed with %d", r);
    //     return false;
    // }
    int r = rtlsdr_demod_read_reg(driver_obj, 0x0a, 0x01, 1);
    ESP_LOGI(TAG_ADSB, "rtlsdr_demod_write_reg val %d", r);
    return (r == len) ? 0 : -1;
}

//static const int fir_default[16] = {
//    -54, -36, -41, -40, -32, -14, 14, 53,  /* 8 bit signed */
//    101, 156, 215, 273, 327, 372, 404, 421 /* 12 bit signed */
//};

int rtlsdr_set_fir(class_driver_t* driver_obj)
{
    uint8_t fir[20];

    int i;
    /* format: int8_t[8] */
    for (i = 0; i < 8; ++i)
    {
        const int val = fir_default[i];
        if (val < -128 || val > 127)
        {
            return -1;
        }
        fir[i] = val;
    }
    /* format: int12_t[8] */
    for (i = 0; i < 8; i += 2)
    {
        const int val0 = fir_default[8 + i];
        const int val1 = fir_default[8 + i + 1];
        if (val0 < -2048 || val0 > 2047 || val1 < -2048 || val1 > 2047)
        {
            return -1;
        }
        fir[8 + i * 3 / 2] = val0 >> 4;
        fir[8 + i * 3 / 2 + 1] = (val0 << 4) | ((val1 >> 8) & 0x0f);
        fir[8 + i * 3 / 2 + 2] = val1;
    }

    for (i = 0; i < (int)sizeof(fir); i++)
    {
        if (rtlsdr_demod_write_reg(driver_obj, 1, 0x1c + i, fir[i], 1))
            return -1;
    }

    return 0;
}

void rtlsdr_init_baseband(class_driver_t* driver_obj)
{
    /* initialize USB */
    rtlsdr_write_reg(driver_obj, USBB, USB_SYSCTL, 0x09, 1);
    rtlsdr_write_reg(driver_obj, USBB, USB_EPA_MAXPKT, 0x0002, 2);
    rtlsdr_write_reg(driver_obj, USBB, USB_EPA_CTL, 0x1002, 2);

    /* poweron demod */
    rtlsdr_write_reg(driver_obj, SYSB, DEMOD_CTL_1, 0x22, 1);
    rtlsdr_write_reg(driver_obj, SYSB, DEMOD_CTL, 0xe8, 1);

    /* reset demod (bit 3, soft_rst) */
    rtlsdr_demod_write_reg(driver_obj, 1, 0x01, 0x14, 1);
    rtlsdr_demod_write_reg(driver_obj, 1, 0x01, 0x10, 1);

    /* disable spectrum inversion and adjacent channel rejection */
    rtlsdr_demod_write_reg(driver_obj, 1, 0x15, 0x00, 1);
    rtlsdr_demod_write_reg(driver_obj, 1, 0x16, 0x0000, 2);

    /* clear both DDC shift and IF frequency registers  */
    for (int i = 0; i < 6; i++)
        rtlsdr_demod_write_reg(driver_obj, 1, 0x16 + i, 0x00, 1);

    rtlsdr_set_fir(driver_obj);

    /* enable SDR mode, disable DAGC (bit 5) */
    rtlsdr_demod_write_reg(driver_obj, 0, 0x19, 0x05, 1);

    /* init FSM state-holding register */
    rtlsdr_demod_write_reg(driver_obj, 1, 0x93, 0xf0, 1);
    rtlsdr_demod_write_reg(driver_obj, 1, 0x94, 0x0f, 1);

    /* disable AGC (en_dagc, bit 0) (this seems to have no effect) */
    rtlsdr_demod_write_reg(driver_obj, 1, 0x11, 0x00, 1);

    /* disable RF and IF AGC loop */
    rtlsdr_demod_write_reg(driver_obj, 1, 0x04, 0x00, 1);

    /* disable PID filter (enable_PID = 0) */
    rtlsdr_demod_write_reg(driver_obj, 0, 0x61, 0x60, 1);

    /* opt_adc_iq = 0, default ADC_I/ADC_Q datapath */
    rtlsdr_demod_write_reg(driver_obj, 0, 0x06, 0x80, 1);

    /* Enable Zero-IF mode (en_bbin bit), DC cancellation (en_dc_est),
     * IQ estimation/compensation (en_iq_comp, en_iq_est) */
    rtlsdr_demod_write_reg(driver_obj, 1, 0xb1, 0x1b, 1);

    /* disable 4.096 MHz clock output on pin TP_CK0 */
    rtlsdr_demod_write_reg(driver_obj, 0, 0x0d, 0x83, 1);
}

void rtlsdr_set_i2c_repeater(class_driver_t* driver_obj, int on)
{
    rtlsdr_demod_write_reg(driver_obj, 1, 0x01, on ? 0x18 : 0x10, 1);
}


int rtlsdr_set_if_freq(class_driver_t* driver_obj, uint32_t freq)
{
    uint32_t rtl_xtal;
    int32_t if_freq;
    uint8_t tmp;
    int r;

    if (!driver_obj)
        return -1;

    // /* read corrected clock value */
    // if (rtlsdr_get_xtal_freq(driver_obj, &rtl_xtal, NULL))
    //     return -2;

    if_freq = ((freq * TWO_POW(22)) / DEF_RTL_XTAL_FREQ) * (-1);

    tmp = (if_freq >> 16) & 0x3f;
    r = rtlsdr_demod_write_reg(driver_obj, 1, 0x19, tmp, 1);
    tmp = (if_freq >> 8) & 0xff;
    r |= rtlsdr_demod_write_reg(driver_obj, 1, 0x1a, tmp, 1);
    tmp = if_freq & 0xff;
    r |= rtlsdr_demod_write_reg(driver_obj, 1, 0x1b, tmp, 1);

    return r;
}

void action_get_str_desc(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));

    if (dev_info.str_desc_manufacturer)
    {
        ESP_LOGI(TAG_CLASS, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product)
    {
        ESP_LOGI(TAG_CLASS, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num)
    {
        ESP_LOGI(TAG_CLASS, "Getting Serial Number string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_serial_num);
    }
    if (adsbdev != NULL && adsbdev->is_adsb)
    {
        ESP_LOGI(TAG_ADSB, "Doing ADSB Things");
        ESP_ERROR_CHECK(usb_host_interface_claim(driver_obj->client_hdl, driver_obj->dev_hdl, 1, 0));
        // dummy request
        rtlsdr_write_reg(driver_obj, 1, USB_SYSCTL, 0x09, 1);
        rtlsdr_init_baseband(driver_obj);
        rtlsdr_set_i2c_repeater(driver_obj, 1);
        uint8_t reg = rtlsdr_i2c_read_reg(driver_obj, R820T_I2C_ADDR, R82XX_CHECK_ADDR);
        if (reg == R82XX_CHECK_VAL)
        {
            fprintf(stderr, "Found Rafael Micro R820T tuner\n");
            rtlsdr_demod_write_reg(driver_obj, 1, 0xb1, 0x1a, 1);
            rtlsdr_demod_write_reg(driver_obj, 0, 0x08, 0x4d, 1);
            rtlsdr_set_if_freq(driver_obj, R82XX_IF_FREQ);
            rtlsdr_demod_write_reg(driver_obj, 1, 0x15, 0x01, 1);
        }
    }
    // Nothing to do until the device disconnects
    driver_obj->actions &= ~ACTION_GET_STR_DESC;
}

void aciton_close_dev(class_driver_t* driver_obj)
{
    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    // We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}


void class_driver_task(void* arg)                    // Работа со свистком RTLSDR
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;  
    class_driver_t driver_obj = { 0 };

    xSemaphoreTake(signaling_sem, portMAX_DELAY);    // Подождите, пока задача демона установит библиотеку USB Host

    ESP_LOGI(TAG_CLASS, "Registering Client"); 
    usb_host_client_config_t client_config = {
        .is_synchronous = false,                       // Синхронные клиенты в настоящее время не поддерживаются. Установите значение false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,     // Максимальное количество сообщений о событиях, которые можно сохранить
        .async = {
            .client_event_callback = client_event_cb,  //
            .callback_arg = (void*)&driver_obj,        //
        },
    };

    /*@brief Регистрация клиента USB Host Library
    * -Эта функция регистрирует клиента USB Host Library
    * -После регистрации клиента его функция обработки usb_host_client_handle_events() должна вызываться повторно
    *@param[in] client_config Конфигурация клиента
    * @param[out] client_hdl_ret Дескриптор клиента
    * @return esp_err_t
    */
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));
    ESP_LOGI(TAG_CLASS, "driver_obj.client_hdl  %d", driver_obj.client_hdl);

    while (1)
    {
        if (driver_obj.actions == 0)
        {
            ESP_LOGI(TAG_CLASS, "** here looking for events");  // здесь ищу события
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        }
        else
        {
            if (driver_obj.actions & ACTION_OPEN_DEV)
            {
                ESP_LOGI(TAG_CLASS, "**action_open_dev");
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO)
            {
                ESP_LOGI(TAG_CLASS, "**action_get_info");
                action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC)
            {
                action_get_dev_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC)
            {
                action_get_config_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC)
            {
                action_get_str_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV)
            {
                aciton_close_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_EXIT)
            {
                break;
            }
        }
    }

    ESP_LOGI(TAG_CLASS, "Deregistering Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    // Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}