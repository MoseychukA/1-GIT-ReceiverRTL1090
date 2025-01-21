#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#ifndef _WIN32
#include <unistd.h>
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include "esp_libusb.h"

#include "Configuration_ESP32.h"




#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

#ifdef CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
#define ENABLE_ENUM_FILTER_CALLBACK
#endif // CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK



static const char* TAG_DAEMON = "DAEMON";
static const char* TAG_CLASS = "CLASS";

#define CLIENT_NUM_EVENT_MSG        5

#define ACTION_OPEN_DEV             0x01
#define ACTION_GET_DEV_INFO         0x02
#define ACTION_GET_DEV_DESC         0x04
#define ACTION_GET_CONFIG_DESC      0x08
#define ACTION_GET_STR_DESC         0x10
#define ACTION_CLOSE_DEV            0x20
#define ACTION_EXIT                 0x40

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;

static void client_event_cb(const usb_host_client_event_msg_t* event_msg, void* arg);

static void action_open_dev(class_driver_t* driver_obj);

static void action_get_info(class_driver_t* driver_obj);

static void action_get_dev_desc(class_driver_t* driver_obj);

static void action_get_config_desc(class_driver_t* driver_obj);

static void action_get_str_desc(class_driver_t* driver_obj);

static void aciton_close_dev(class_driver_t* driver_obj);

void class_driver_task(void* arg);



#ifdef ENABLE_ENUM_FILTER_CALLBACK
static bool set_config_cb(const usb_device_desc_t* dev_desc, uint8_t* bConfigurationValue)
{
    // If the USB device has more than one configuration, set the second configuration
    if (dev_desc->bNumConfigurations > 1) {
        *bConfigurationValue = 2;
    }
    else {
        *bConfigurationValue = 1;
    }

    // Return true to enumerate the USB device
    return true;
}
#endif // ENABLE_ENUM_FILTER_CALLBACK

static void host_lib_daemon_task(void* arg);

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
    for (int i = 0; i < 2; i++)
    {
        xSemaphoreTake(signaling_sem, portMAX_DELAY);
    }

    //Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);

}

//================================= host_lib_daemon_task =========================================================
static void host_lib_daemon_task(void* arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG_DAEMON, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
#ifdef ENABLE_ENUM_FILTER_CALLBACK
        .enum_filter_cb = set_config_cb,
#endif // ENABLE_ENUM_FILTER_CALLBACK
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    //Signal to the class driver task that the host library is installed
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); //Short delay to let client task spin up

    bool has_clients = true;
    bool has_devices = true;
    while (has_clients || has_devices) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            has_devices = false;
        }
    }
    ESP_LOGI(TAG_DAEMON, "No more clients and devices");

    //Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}
//========================== class_driver.c ================================================
static void client_event_cb(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    class_driver_t* driver_obj = (class_driver_t*)arg;
    switch (event_msg->event) 
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        if (driver_obj->dev_addr == 0)
        {
            driver_obj->dev_addr = event_msg->new_dev.address;
           /* rtlsdr_open(&rtldev, event_msg->new_dev.address, driver_obj->client_hdl);
            int r;
            r = rtlsdr_set_sample_rate(rtldev, 2000000);
            if (r < 0)
            {
                fprintf(stderr, "WARNING: Failed to set sample rate.\n");
            }
            else
            {
                fprintf(stderr, "Sampling at %u S/s.\n", 2000000);
            }*/




            //Open the device next
            //driver_obj->actions |= ACTION_OPEN_DEV;
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        if (driver_obj->dev_hdl != NULL) 
        {
            //Cancel any other actions and close the device next
            driver_obj->actions = ACTION_CLOSE_DEV;
        }
        break;
    default:
        //Should never occur
        abort();
    }
}

static void action_open_dev(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(TAG_CLASS, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    //Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    ESP_LOGI(TAG_CLASS, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    ESP_LOGI(TAG_CLASS, "\tbConfigurationValue %d", dev_info.bConfigurationValue);

    //Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device descriptor");
    const usb_device_desc_t* dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
    usb_print_device_descriptor(dev_desc);
    //Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting config descriptor");
    const usb_config_desc_t* config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    usb_print_config_descriptor(config_desc, NULL);
    //Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t* driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    if (dev_info.str_desc_manufacturer) {
        ESP_LOGI(TAG_CLASS, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product) {
        ESP_LOGI(TAG_CLASS, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num) {
        ESP_LOGI(TAG_CLASS, "Getting Serial Number string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_serial_num);
    }
    //Nothing to do until the device disconnects
    driver_obj->actions &= ~ACTION_GET_STR_DESC;
}

static void aciton_close_dev(class_driver_t* driver_obj)
{
    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    //We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}

void class_driver_task(void* arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    class_driver_t driver_obj = { 0 };

    //Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    ESP_LOGI(TAG_CLASS, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,    //Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void*)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));

    while (1) {
        if (driver_obj.actions == 0) {
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        }
        else {
            if (driver_obj.actions & ACTION_OPEN_DEV) {
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO) {
                action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC) {
                action_get_dev_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC) {
                action_get_config_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC) {
                action_get_str_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV) {
                aciton_close_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_EXIT) {
                break;
            }
        }
    }

    ESP_LOGI(TAG_CLASS, "Deregistering Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}
//========================================================================================