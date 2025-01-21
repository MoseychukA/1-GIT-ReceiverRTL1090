/*
 
 */
#include <elapsedMillis.h>
#include <usb/usb_host.h>
#include "show_desc.hpp"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h" 
#include <esp_task_wdt.h>

#include "main.h"
#include "librtlsdr.h"

//============================= usbhhelp ================================================
const TickType_t HOST_EVENT_TIMEOUT = 1;
const TickType_t CLIENT_EVENT_TIMEOUT = 1;

typedef void (*usb_host_enum_cb_t)(const usb_config_desc_t* config_desc);
static usb_host_enum_cb_t _USB_host_enumerate;
void show_config_desc_full(const usb_config_desc_t* config_desc);
//=======================================================================================

static void host_lib_daemon_task(void* arg);
void class_driver_task(void* arg);
void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);

void usbh_setup(usb_host_enum_cb_t enumeration_cb);

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

    //SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    //TaskHandle_t daemon_task_hdl;
    //TaskHandle_t class_driver_task_hdl;
    ////Create daemon task
    //xTaskCreatePinnedToCore(host_lib_daemon_task,
    //    "daemon",
    //    4096,
    //    (void*)signaling_sem,
    //    DAEMON_TASK_PRIORITY,
    //    &daemon_task_hdl,
    //    0);
    ////Create the class driver task
    //xTaskCreatePinnedToCore(class_driver_task,
    //    "class",
    //    4096,
    //    (void*)signaling_sem,
    //    CLASS_TASK_PRIORITY,
    //    &class_driver_task_hdl,
    //    0);

    //vTaskDelay(10);     //Add a short delay to let the tasks run

    ////Wait for the tasks to complete
    //for (int i = 0; i < 2; i++) {
    //    xSemaphoreTake(signaling_sem, portMAX_DELAY);
    //}

    ////Delete the tasks
    //vTaskDelete(class_driver_task_hdl);
    //vTaskDelete(daemon_task_hdl);





  /*usbh_setup(show_config_desc_full);*/
}

void loop()
{
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    TaskHandle_t daemon_task_hdl;
    TaskHandle_t class_driver_task_hdl;
    //Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
        "daemon",
        4096,
        (void*)signaling_sem,
        DAEMON_TASK_PRIORITY,
        &daemon_task_hdl,
        0);
    //Create the class driver task
    xTaskCreatePinnedToCore(class_driver_task,
        "class",
        4096,
        (void*)signaling_sem,
        CLASS_TASK_PRIORITY,
        &class_driver_task_hdl,
        0);

    vTaskDelay(10);     //Add a short delay to let the tasks run

    //Wait for the tasks to complete
    for (int i = 0; i < 2; i++) {
        xSemaphoreTake(signaling_sem, portMAX_DELAY);
    }

    //Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);

}

static void host_lib_daemon_task(void* arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG_DAEMON, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    esp_err_t err = usb_host_install(&host_config);
    ESP_LOGI(TAG_DAEMON, "usb_host_install: %x", err);

   // ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Сигнал задаче драйвера класса о том, что хост-библиотека установлена
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); // Короткая задержка, позволяющая клиентской задаче разогнаться

    bool has_clients = true;
    bool has_devices = true;
    while (has_clients || has_devices)
    {
        uint32_t event_flags;

        /**
        * @brief Обработка событий USB Host Library
        *
        * - Эта функция обрабатывает всю обработку USB Host Library и должна вызываться многократно в цикле
        * - Проверьте event_flags_ret, чтобы увидеть, установлены ли флаги, указывающие на определенные события USB Host Library
        * - Эта функция никогда не должна вызываться несколькими потоками одновременно
        *
        * @note Эта функция может блокировать
        * @param[in] timeout_ticks Тайм-аут в тиках для ожидания события
        * @param[out] event_flags_ret Флаги событий, указывающие, какое событие USB Host Library произошло.
        * @return esp_err_t
        */
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            has_devices = false;
        }
    }
    ESP_LOGI(TAG_DAEMON, "No more clients and devices");

    // Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    // Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

//====================================================================================================

void class_driver_task(void* arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    class_driver_t driver_obj = { 0 };

    // Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    ESP_LOGI(TAG_CLASS, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false, // Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = _client_event_callback,//client_event_cb,
            .callback_arg = (void*)&driver_obj,
        },
    };
   // ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));
    esp_err_t err = usb_host_client_register(&client_config, &driver_obj.client_hdl);
    ESP_LOGI(TAG_DAEMON, "usb_host_client_register: %x", err);

    

    uint32_t event_flags;
    static bool all_clients_gone = false;
    static bool all_dev_free = false;

    
    while (1)
    {
        esp_err_t err = usb_host_lib_handle_events(HOST_EVENT_TIMEOUT, &event_flags);

        if (err == ESP_OK)
        {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            {
                ESP_LOGI(TAG_DAEMON, "No more clients");
                all_clients_gone = true;
            }
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
            {
                ESP_LOGI(TAG_DAEMON, "No more devices");
                all_dev_free = true;
            }
        }
        else
        {
            if (err != ESP_ERR_TIMEOUT)
            {
                ESP_LOGI(TAG_DAEMON, "usb_host_lib_handle_events: %x flags: %x", err, event_flags);
            }
        }

        err = usb_host_client_handle_events(driver_obj.client_hdl, CLIENT_EVENT_TIMEOUT);
        if ((err != ESP_OK) && (err != ESP_ERR_TIMEOUT))
        {
            ESP_LOGI(TAG_DAEMON, "usb_host_client_handle_events: %x", err);
        }

        ESP_LOGI(TAG_DAEMON, "*** driver_obj.actions: %d", driver_obj.actions);

        if (driver_obj.actions == 0)
        {

            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        }
        else
        {
           // ESP_LOGI(TAG_DAEMON, "driver_obj.actions: %d", driver_obj.actions);
            if (driver_obj.actions & ACTION_OPEN_DEV)
            {
                //action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO)
            {
               // action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC)
            {
               // action_get_dev_desc1(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC)
            {
               // action_get_config_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC)
            {
               // action_get_str_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV)
            {
               // aciton_close_dev(&driver_obj);
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

//=======================================================================================
void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    esp_err_t err;
    class_driver_t* driver_obj = (class_driver_t*)arg;
    switch (event_msg->event)
    {
        /**< A new device has been enumerated and added to the USB Host Library */
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG_DAEMON, "New device address: %d", event_msg->new_dev.address);
        /**
        * @brief Открыть устройство
        *
        * - Эта функция позволяет клиенту открыть устройство
        * - Клиент должен сначала открыть устройство, прежде чем пытаться его использовать (например, отправлять передачи, запросы устройств и т. д.)
        *
        * @param[in] client_hdl Дескриптор клиента
        * @param[in] dev_addr Адрес устройства
        * @param[out] dev_hdl_ret Дескриптор устройства
        * @return esp_err_t
        */
        err = usb_host_device_open(driver_obj->client_hdl, event_msg->new_dev.address, &driver_obj->dev_hdl);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_device_open: %x", err);

        
       // rtlsdr_open(&rtldev, event_msg->new_dev.address, Client_Handle);


        /* int r;
         r = rtlsdr_set_sample_rate(rtldev, 2000000);
         fprintf(stderr, "r1 %d\n", r);
         if (r < 0)
         {
             fprintf(stderr, "WARNING: Failed to set sample rate.\n");
         }
         else
         {
             fprintf(stderr, "Sampling at %u S/s.\n", 2000000);
         }*/



        usb_device_info_t dev_info;
        err = usb_host_device_info(driver_obj->dev_hdl, &dev_info);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_device_info: %x", err);
        ESP_LOGI(TAG_DAEMON, "speed: %d dev_addr %d vMaxPacketSize0 %d bConfigurationValue %d",
            dev_info.speed, dev_info.dev_addr, dev_info.bMaxPacketSize0,
            dev_info.bConfigurationValue);

        const usb_device_desc_t* dev_desc;
        err = usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_get_device_desc: %x", err);
        show_dev_desc(dev_desc);

        const usb_config_desc_t* config_desc;
        err = usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_get_config_desc: %x", err);
       // (*_USB_host_enumerate)(config_desc);
        break;
        /**< A device opened by the client is now gone */
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI(TAG_DAEMON, "Device Gone handle: %x", event_msg->dev_gone.dev_hdl);
        break;
    default:
        ESP_LOGI(TAG_DAEMON, "Unknown value %d", event_msg->event);
        break;
    }
}


void usbh_setup(usb_host_enum_cb_t enumeration_cb)
{
 /*   ESP_LOGI(TAG_DAEMON, "Installing USB Host Library");
    const usb_host_config_t config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&config);
    ESP_LOGI(TAG_DAEMON, "usb_host_install: %x", err);

    const usb_host_client_config_t client_config = {
      .is_synchronous = false,
      .max_num_event_msg = 5,
      .async = {
          .client_event_callback = _client_event_callback,
          .callback_arg = Client_Handle
      }
    };
    err = usb_host_client_register(&client_config, &Client_Handle);
    ESP_LOGI(TAG_DAEMON, "usb_host_client_register: %x", err);*/

    _USB_host_enumerate = enumeration_cb;
}


void show_config_desc_full(const usb_config_desc_t* config_desc)
{
    // Full decode of config desc.
    const uint8_t* p = &config_desc->val[0];
    static uint8_t USB_Class = 0;
    uint8_t bLength;
    for (int i = 0; i < config_desc->wTotalLength; i += bLength, p += bLength)
    {
        bLength = *p;
        if ((i + bLength) <= config_desc->wTotalLength)
        {
            const uint8_t bDescriptorType = *(p + 1);
            switch (bDescriptorType)
            {
            case USB_B_DESCRIPTOR_TYPE_DEVICE:
                ESP_LOGI(TAG_DAEMON, "USB Device Descriptor should not appear in config");
                break;
            case USB_B_DESCRIPTOR_TYPE_CONFIGURATION:
                show_config_desc(p);
                break;
            case USB_B_DESCRIPTOR_TYPE_STRING:
                ESP_LOGI(TAG_DAEMON, "USB string desc TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_INTERFACE:
                USB_Class = show_interface_desc(p);
                //check_interface_desc_boot_RTLSDR(p);
                break;
            case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
                show_endpoint_desc(p);
                //if (isRTLSDR && RTLSDRIn == NULL) prepare_endpoint(p);
                break;
            case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
                // Should not be config config?
                ESP_LOGI(TAG_DAEMON, "USB device qual desc TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
                // Should not be config config?
                ESP_LOGI(TAG_DAEMON, "USB Other Speed TBD");
                break;
            case USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER:
                // Should not be config config?
                ESP_LOGI(TAG_DAEMON, "USB Interface Power TBD");
                break;
            case 0x21:
                if (USB_Class == USB_CLASS_HID)
                {
                    show_hid_desc(p);
                }
                break;
            default:
                ESP_LOGI(TAG_DAEMON, "Unknown USB Descriptor Type: 0x%x", bDescriptorType);
                break;
            }
        }
        else
        {
            ESP_LOGI(TAG_DAEMON, "USB Descriptor invalid");
            return;
        }
    }
}

//==================================================================================================