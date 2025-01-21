/*
 
 */
#include <elapsedMillis.h>
#include <usb/usb_host.h>
#include "show_desc.hpp"

static const char* TAG_DAEMON = "DAEMON";
static const char* TAG_CLASS = "CLASS";


//============================= usbhhelp ================================================
const TickType_t HOST_EVENT_TIMEOUT = 1;
const TickType_t CLIENT_EVENT_TIMEOUT = 1;

usb_host_client_handle_t Client_Handle;
usb_device_handle_t Device_Handle;
typedef void (*usb_host_enum_cb_t)(const usb_config_desc_t* config_desc);
static usb_host_enum_cb_t _USB_host_enumerate;

void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
void usbh_setup(usb_host_enum_cb_t enumeration_cb);
void usbh_task(void);


//=======================================================================================

bool isRTLSDR = false;
bool isRTLSDRReady = false;
uint8_t RTLSDRInterval;
bool isRTLSDRPolling = false;
elapsedMillis RTLSDRTimer;

const size_t RTLSDR_IN_BUFFER_SIZE = 64;
usb_transfer_t *RTLSDRIn = NULL;


void RTLSDR_transfer_cb(usb_transfer_t* transfer);
void check_interface_desc_boot_RTLSDR(const void* p);
void prepare_endpoint(const void* p);
void show_config_desc_full(const usb_config_desc_t* config_desc);

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

  usbh_setup(show_config_desc_full);
  ESP_LOGI(TAG_DAEMON, "*** End setup");
}

void loop()
{
  usbh_task();

  if (isRTLSDRReady && !isRTLSDRPolling && (RTLSDRTimer > RTLSDRInterval)) 
  {
    RTLSDRIn->num_bytes = 64;

    /**
    * @brief Отправить неконтролируемый перевод
    *
    * - Отправить перевод на определенную конечную точку. Устройство и номер конечной точки указываются внутри перевода
    * - Перед отправкой перевод должен быть правильно инициализирован
    * - По завершении обратный вызов перевода будет вызван из функции usb_host_client_handle_events() клиента.
    *
    * @param[in] перевод Инициализированный объект перевода
    * @return esp_err_t
    */
    esp_err_t err = usb_host_transfer_submit(RTLSDRIn);
    if (err != ESP_OK) 
    {
      ESP_LOGI(TAG_DAEMON, "usb_host_transfer_submit In fail: %x", err);
    }
    isRTLSDRPolling = true;
    RTLSDRTimer = 0;
  }
}


//==========================================================================================

void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    esp_err_t err;
    switch (event_msg->event)
    {
        /**< A new device has been enumerated and added to the USB Host Library */
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG_DAEMON, "New device address: %d", event_msg->new_dev.address);
        err = usb_host_device_open(Client_Handle, event_msg->new_dev.address, &Device_Handle);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_device_open: %x", err);

        usb_device_info_t dev_info;
        err = usb_host_device_info(Device_Handle, &dev_info);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_device_info: %x", err);
        ESP_LOGI(TAG_DAEMON, "speed: %d dev_addr %d vMaxPacketSize0 %d bConfigurationValue %d",
            dev_info.speed, dev_info.dev_addr, dev_info.bMaxPacketSize0,
            dev_info.bConfigurationValue);

        const usb_device_desc_t* dev_desc;
        err = usb_host_get_device_descriptor(Device_Handle, &dev_desc);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_get_device_desc: %x", err);
        show_dev_desc(dev_desc);

        const usb_config_desc_t* config_desc;
        err = usb_host_get_active_config_descriptor(Device_Handle, &config_desc);
        if (err != ESP_OK) ESP_LOGI(TAG_DAEMON, "usb_host_get_config_desc: %x", err);
        (*_USB_host_enumerate)(config_desc);
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

// Reference: esp-idf/examples/peripherals/usb/host/usb_host_lib/main/usb_host_lib_main.c

void usbh_setup(usb_host_enum_cb_t enumeration_cb)
{
    const usb_host_config_t config = {
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
    ESP_LOGI(TAG_DAEMON, "usb_host_client_register: %x", err);

    _USB_host_enumerate = enumeration_cb;
}

void usbh_task(void)
{
    uint32_t event_flags;
    static bool all_clients_gone = false;
    static bool all_dev_free = false;

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

    err = usb_host_client_handle_events(Client_Handle, CLIENT_EVENT_TIMEOUT);
    if ((err != ESP_OK) && (err != ESP_ERR_TIMEOUT))
    {
        ESP_LOGI(TAG_DAEMON, "usb_host_client_handle_events: %x", err);
    }
}

//========================================================================================

void RTLSDR_transfer_cb(usb_transfer_t* transfer)
{
    if (Device_Handle == transfer->device_handle)
    {
        isRTLSDRPolling = false;
        if (transfer->status == 0)
        {
            if (transfer->actual_num_bytes == 64)
            {
                uint8_t* const p = transfer->data_buffer;
                ESP_LOGI(TAG_DAEMON, "HID report: %02x %02x %02x %02x %02x %02x %02x %02x",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
            }
            else
            {
                ESP_LOGI(TAG_DAEMON, "RTLSDR boot hid transfer too short or long");
            }
        }
        else 
        {
            ESP_LOGI(TAG_DAEMON, "transfer->status %d", transfer->status);
        }
    }
}

void check_interface_desc_boot_RTLSDR(const void* p)
{
    const usb_intf_desc_t* intf = (const usb_intf_desc_t*)p;

    if ((intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC) && (intf->bInterfaceSubClass == 0xff) && (intf->bInterfaceProtocol == 0xff))
    {
        isRTLSDR = true;
        ESP_LOGI(TAG_DAEMON, "Claiming a boot RTLSDR!");
        /**
        * @brief Функция для клиента, чтобы заявить интерфейс устройства
        *
        * - Клиент должен заявить интерфейс устройства, прежде чем пытаться связаться с любой из его конечных точек
        * - После того, как интерфейс заявляется клиентом, он не может быть заявлён никаким другим клиентом.
        *
        * @note Эта функция может блокировать
        * @param[in] client_hdl Дескриптор клиента
        * @param[in] dev_hdl Дескриптор устройства
        * @param[in] bInterfaceNumber Номер интерфейса
        * @param[in] bAlternateSetting Номер альтернативной настройки интерфейса
        * @return esp_err_t
        */
        esp_err_t err = usb_host_interface_claim(Client_Handle, Device_Handle,intf->bInterfaceNumber, intf->bAlternateSetting);
        if (err != ESP_OK)
        {
            ESP_LOGI(TAG_DAEMON, "!!!****usb_host_interface_claim failed: %x", err);
        }
        else
        {
            ESP_LOGI(TAG_DAEMON, "!!!****usb_host_interface_claim OK!: %x", err);
        }
    }
}

void prepare_endpoint(const void* p)
{
    const usb_ep_desc_t* endpoint = (const usb_ep_desc_t*)p;
    esp_err_t err;

    // должно быть прерывание для HID
    if ((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_BULK)
    {
        ESP_LOGI(TAG_DAEMON, "Not interrupt endpoint: 0x%02x", endpoint->bmAttributes);
        return;
    }
    if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) 
    {
        err = usb_host_transfer_alloc(RTLSDR_IN_BUFFER_SIZE, 0, &RTLSDRIn);
        if (err != ESP_OK) 
        {
            RTLSDRIn = NULL;
            ESP_LOGI(TAG_DAEMON, "usb_host_transfer_alloc In fail: %x", err);
            return;
        }
        RTLSDRIn->device_handle = Device_Handle;
        RTLSDRIn->bEndpointAddress = endpoint->bEndpointAddress;
        RTLSDRIn->callback = RTLSDR_transfer_cb;
        RTLSDRIn->context = NULL;
        isRTLSDRReady = true;
        RTLSDRInterval = endpoint->bInterval;
        ESP_LOGI(TAG_DAEMON, "USB boot RTLSDR ready");
    }
    else 
    {
        ESP_LOGI(TAG_DAEMON, "Ignoring interrupt Out endpoint");
    }
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
                if (USB_Class == USB_CLASS_VENDOR_SPEC)
                {
                    check_interface_desc_boot_RTLSDR(p);
                }
                break;
            case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
                show_endpoint_desc(p);
                if (isRTLSDR && RTLSDRIn == NULL) prepare_endpoint(p);
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