/*
 
 */
#include <elapsedMillis.h>
#include <usb/usb_host.h>
#include "main.h"

static const char* TAG_DAEMON = "DAEMON";
static const char* TAG_CLASS = "CLASS";

bool isRtlsdr            = false;
bool isRtlsdrReady       = false;
uint8_t RtlsdrInterval;
bool isRtlsdrPolling     = false;
elapsedMillis RtlsdrTimer;

const size_t RTLSDR_IN_BUFFER_SIZE = 8;
usb_transfer_t *RtlsdrIn           = NULL;

typedef void (*usb_host_enum_cb_t)(const usb_config_desc_t* config_desc);
static usb_host_enum_cb_t _USB_host_enumerate;

class_driver_t driver_obj = { 0 };

void show_dev_desc(const usb_device_desc_t* dev_desc);
void show_config_desc(const void* p);
uint8_t show_interface_desc(const void* p);
void show_endpoint_desc(const void* p);
void show_hid_desc(const void* p);
void show_interface_assoc(const void* p);

//====================================================================================

void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg);
void usbh_setup(usb_host_enum_cb_t enumeration_cb);
void usbh_task(void);

//=====================================================================================

int rtlsdr_open(rtlsdr_dev_t** out_dev, uint8_t index, usb_host_client_handle_t client_hdl);

void Rtlsdr_transfer_cb(usb_transfer_t *transfer) 
{
  if (driver_obj.Device_Handle == transfer->device_handle) 
  {
    isRtlsdrPolling = false;
    if (transfer->status == 0) 
    {
      if (transfer->actual_num_bytes == 8) 
      {
        uint8_t *const p = transfer->data_buffer;
        ESP_LOGI("", "HID report: %02x %02x %02x %02x %02x %02x %02x %02x",
            p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
      }
      else 
       {
        ESP_LOGI("", "Rtlsdr boot hid transfer too short or long");
      }
    }
    else {
      ESP_LOGI("", "transfer->status %d", transfer->status);
    }
  }
}

void check_interface_desc_boot_Rtlsdr(const void *p)
{
  const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;

  if ((intf->bInterfaceClass == USB_CLASS_HID) && (intf->bInterfaceSubClass == 1) && (intf->bInterfaceProtocol == 1)) 
  {
    isRtlsdr = true;
    ESP_LOGI("", "Claiming a boot Rtlsdr!");
    esp_err_t err = usb_host_interface_claim(driver_obj.Client_Handle, driver_obj.Device_Handle,intf->bInterfaceNumber, intf->bAlternateSetting);
    if (err != ESP_OK)
    {
        ESP_LOGI("", "usb_host_interface_claim failed: %x", err);
    }
  }
}

//подготовить конечную точку
void prepare_endpoint(const void *p)
{
  const usb_ep_desc_t *endpoint = (const usb_ep_desc_t *)p;
  esp_err_t err;

  // должно быть прерывание для HID
  if ((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_INT) 
  {
    ESP_LOGI("", "Not interrupt endpoint: 0x%02x", endpoint->bmAttributes);
    return;
  }

  if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) 
  {
    err = usb_host_transfer_alloc(RTLSDR_IN_BUFFER_SIZE, 0, &RtlsdrIn);
    if (err != ESP_OK) 
    {
      RtlsdrIn = NULL;
      ESP_LOGI("", "usb_host_transfer_alloc In fail: %x", err);
      return;
    }
    RtlsdrIn->device_handle = driver_obj.Device_Handle;
    RtlsdrIn->bEndpointAddress = endpoint->bEndpointAddress;
    RtlsdrIn->callback = Rtlsdr_transfer_cb;
    RtlsdrIn->context = NULL;
    isRtlsdrReady = true;
    RtlsdrInterval = endpoint->bInterval;
    ESP_LOGI("", "USB boot Rtlsdr ready");

    fprintf(stderr, "transfer->num_bytes %d\n", RTLSDR_IN_BUFFER_SIZE);
    fprintf(stderr, "transfer->device_handle %d\n", RtlsdrIn->device_handle);
    fprintf(stderr, "transfer->bEndpointAddress %x\n", RtlsdrIn->bEndpointAddress);
    fprintf(stderr, "transfer->callback %d\n", RtlsdrIn->callback);
    fprintf(stderr, "transfer->context %d\n", RtlsdrIn->context);
    fprintf(stderr, "transfer->timeout_ms %d\n", RtlsdrInterval);
  }
  else 
  {
    ESP_LOGI("", "Ignoring interrupt Out endpoint");
  }
}

void show_config_desc_full(const usb_config_desc_t *config_desc)
{
  // Full decode of config desc.
  const uint8_t *p = &config_desc->val[0];
  static uint8_t USB_Class = 0;
  uint8_t bLength;
  for (int i = 0; i < config_desc->wTotalLength; i+=bLength, p+=bLength) {
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
          check_interface_desc_boot_Rtlsdr(p);
          break;
        case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
          show_endpoint_desc(p);
          if (isRtlsdr && RtlsdrIn == NULL) prepare_endpoint(p);
          break;
        case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
          // Should not be config config?
          ESP_LOGI("", "USB device qual desc TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
          // Should not be config config?
          ESP_LOGI("", "USB Other Speed TBD");
          break;
        case USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER:
          // Should not be config config?
          ESP_LOGI("", "USB Interface Power TBD");
          break;
        case 0x21:
          if (USB_Class == USB_CLASS_HID) {
            show_hid_desc(p);
          }
          break;
        default:
          ESP_LOGI("", "Unknown USB Descriptor Type: 0x%x", bDescriptorType);
          break;
      }
    }
    else {
      ESP_LOGI("", "USB Descriptor invalid");
      return;
    }
  }
}

void setup()
{

    Serial.begin(115200);
    while (!Serial && millis() < 1000);

    esp_log_level_set("*", ESP_LOG_VERBOSE);  //  

    String ver_soft = __FILE__;
    int val_srt = ver_soft.lastIndexOf('\\');
    ver_soft.remove(0, val_srt + 1);
    val_srt = ver_soft.lastIndexOf('.');
    ver_soft.remove(val_srt);
    Serial.println(ver_soft);

  usbh_setup(show_config_desc_full);
}

void loop()
{
  usbh_task();

  if (isRtlsdrReady && !isRtlsdrPolling && (RtlsdrTimer > RtlsdrInterval)) 
  {
    RtlsdrIn->num_bytes = 8;
    esp_err_t err = usb_host_transfer_submit(RtlsdrIn);
    if (err != ESP_OK) 
    {
      ESP_LOGI("", "usb_host_transfer_submit In fail: %x", err);
    }
    isRtlsdrPolling = true;
    RtlsdrTimer = 0;
  }
}

//=========================================================================

void show_dev_desc(const usb_device_desc_t* dev_desc)
{
    ESP_LOGI("", "bLength: %d", dev_desc->bLength);
    ESP_LOGI("", "bDescriptorType(device): %d", dev_desc->bDescriptorType);
    ESP_LOGI("", "bcdUSB: 0x%x", dev_desc->bcdUSB);
    ESP_LOGI("", "bDeviceClass: 0x%02x", dev_desc->bDeviceClass);
    ESP_LOGI("", "bDeviceSubClass: 0x%02x", dev_desc->bDeviceSubClass);
    ESP_LOGI("", "bDeviceProtocol: 0x%02x", dev_desc->bDeviceProtocol);
    ESP_LOGI("", "bMaxPacketSize0: %d", dev_desc->bMaxPacketSize0);
    ESP_LOGI("", "idVendor: 0x%x", dev_desc->idVendor);
    ESP_LOGI("", "idProduct: 0x%x", dev_desc->idProduct);
    ESP_LOGI("", "bcdDevice: 0x%x", dev_desc->bcdDevice);
    ESP_LOGI("", "iManufacturer: %d", dev_desc->iManufacturer);
    ESP_LOGI("", "iProduct: %d", dev_desc->iProduct);
    ESP_LOGI("", "iSerialNumber: %d", dev_desc->iSerialNumber);
    ESP_LOGI("", "bNumConfigurations: %d", dev_desc->bNumConfigurations);
}

void show_config_desc(const void* p)
{
    const usb_config_desc_t* config_desc = (const usb_config_desc_t*)p;

    ESP_LOGI("", "bLength: %d", config_desc->bLength);
    ESP_LOGI("", "bDescriptorType(config): %d", config_desc->bDescriptorType);
    ESP_LOGI("", "wTotalLength: %d", config_desc->wTotalLength);
    ESP_LOGI("", "bNumInterfaces: %d", config_desc->bNumInterfaces);
    ESP_LOGI("", "bConfigurationValue: %d", config_desc->bConfigurationValue);
    ESP_LOGI("", "iConfiguration: %d", config_desc->iConfiguration);
    ESP_LOGI("", "bmAttributes(%s%s%s): 0x%02x",
        (config_desc->bmAttributes & USB_BM_ATTRIBUTES_SELFPOWER) ? "Self Powered" : "",
        (config_desc->bmAttributes & USB_BM_ATTRIBUTES_WAKEUP) ? ", Remote Wakeup" : "",
        (config_desc->bmAttributes & USB_BM_ATTRIBUTES_BATTERY) ? ", Battery Powered" : "",
        config_desc->bmAttributes);
    ESP_LOGI("", "bMaxPower: %d = %d mA", config_desc->bMaxPower, config_desc->bMaxPower * 2);
}

uint8_t show_interface_desc(const void* p)
{
    const usb_intf_desc_t* intf = (const usb_intf_desc_t*)p;

    ESP_LOGI("", "bLength: %d", intf->bLength);
    ESP_LOGI("", "bDescriptorType (interface): %d", intf->bDescriptorType);
    ESP_LOGI("", "bInterfaceNumber: %d", intf->bInterfaceNumber);
    ESP_LOGI("", "bAlternateSetting: %d", intf->bAlternateSetting);
    ESP_LOGI("", "bNumEndpoints: %d", intf->bNumEndpoints);
    ESP_LOGI("", "bInterfaceClass: 0x%02x", intf->bInterfaceClass);
    ESP_LOGI("", "bInterfaceSubClass: 0x%02x", intf->bInterfaceSubClass);
    ESP_LOGI("", "bInterfaceProtocol: 0x%02x", intf->bInterfaceProtocol);
    ESP_LOGI("", "iInterface: %d", intf->iInterface);
    return intf->bInterfaceClass;
}

void show_endpoint_desc(const void* p)
{
    const usb_ep_desc_t* endpoint = (const usb_ep_desc_t*)p;
    const char* XFER_TYPE_NAMES[] = {
      "Control", "Isochronous", "Bulk", "Interrupt"
    };
    ESP_LOGI("", "bLength: %d", endpoint->bLength);
    ESP_LOGI("", "bDescriptorType (endpoint): %d", endpoint->bDescriptorType);
    ESP_LOGI("", "bEndpointAddress(%s): 0x%02x",
        (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) ? "In" : "Out",
        endpoint->bEndpointAddress);
    ESP_LOGI("", "bmAttributes(%s): 0x%02x",
        XFER_TYPE_NAMES[endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK],
        endpoint->bmAttributes);
    ESP_LOGI("", "wMaxPacketSize: %d", endpoint->wMaxPacketSize);
    ESP_LOGI("", "bInterval: %d", endpoint->bInterval);
}

void show_hid_desc(const void* p)
{
    usb_hid_desc_t* hid = (usb_hid_desc_t*)p;
    ESP_LOGI("", "bLength: %d", hid->bLength);
    ESP_LOGI("", "bDescriptorType (HID): %d", hid->bDescriptorType);
    ESP_LOGI("", "bcdHID: 0x%04x", hid->bcdHID);
    ESP_LOGI("", "bCountryCode: %d", hid->bCountryCode);
    ESP_LOGI("", "bNumDescriptor: %d", hid->bNumDescriptor);
    ESP_LOGI("", "bDescriptorType: %d", hid->bHIDDescriptorType);
    ESP_LOGI("", "wDescriptorLength: %d", hid->wHIDDescriptorLength);
    if (hid->bNumDescriptor > 1) {
        ESP_LOGI("", "bDescriptorTypeOpt: %d", hid->bHIDDescriptorTypeOpt);
        ESP_LOGI("", "wDescriptorLengthOpt: %d", hid->wHIDDescriptorLengthOpt);
    }
}

void show_interface_assoc(const void* p)
{
    usb_iad_desc_t* iad = (usb_iad_desc_t*)p;
    ESP_LOGI("", "bLength: %d", iad->bLength);
    ESP_LOGI("", "bDescriptorType: %d", iad->bDescriptorType);
    ESP_LOGI("", "bFirstInterface: %d", iad->bFirstInterface);
    ESP_LOGI("", "bInterfaceCount: %d", iad->bInterfaceCount);
    ESP_LOGI("", "bFunctionClass: 0x%02x", iad->bFunctionClass);
    ESP_LOGI("", "bFunctionSubClass: 0x%02x", iad->bFunctionSubClass);
    ESP_LOGI("", "bFunctionProtocol: 0x%02x", iad->bFunctionProtocol);
    ESP_LOGI("", "iFunction: %d", iad->iFunction);
}

//====================================================================================

void _client_event_callback(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    esp_err_t err;
    switch (event_msg->event)
    {
        /**< A new device has been enumerated and added to the USB Host Library */
    case USB_HOST_CLIENT_EVENT_NEW_DEV:

        ESP_LOGI(TAG_DAEMON, "**driver_obj->dev_addr0: %d", driver_obj.dev_addr);
        if (driver_obj.dev_addr == 0)  // при старте адрес не определен
        {
            driver_obj.dev_addr = event_msg->new_dev.address; // Определили адрес устройства
            ESP_LOGI(TAG_DAEMON, "**driver_obj->dev_addr1: %x", driver_obj.dev_addr);
            ESP_LOGI(TAG_DAEMON, "**driver_obj->client_hdl: %d", driver_obj.Client_Handle);

           // rtlsdr_open(&rtldev, event_msg->new_dev.address, driver_obj.Client_Handle);

            int r;
            rtlsdr_dev_t* dev = NULL;
            uint8_t reg;

            dev = (rtlsdr_dev_t*)ps_malloc(sizeof(rtlsdr_dev_t*));



            ESP_LOGI(TAG_DAEMON, "New device address: %d", event_msg->new_dev.address);
            err = usb_host_device_open(driver_obj.Client_Handle, event_msg->new_dev.address, &driver_obj.Device_Handle);
            if (err != ESP_OK) ESP_LOGI("", "usb_host_device_open: %x", err);

            usb_device_info_t dev_info;
            err = usb_host_device_info(driver_obj.Device_Handle, &dev_info);
            if (err != ESP_OK) ESP_LOGI("", "usb_host_device_info: %x", err);
            ESP_LOGI("", "speed: %d dev_addr %d vMaxPacketSize0 %d bConfigurationValue %d",
                dev_info.speed, dev_info.dev_addr, dev_info.bMaxPacketSize0,
                dev_info.bConfigurationValue);

            const usb_device_desc_t* dev_desc;
            err = usb_host_get_device_descriptor(driver_obj.Device_Handle, &dev_desc);
            if (err != ESP_OK) ESP_LOGI("", "usb_host_get_device_desc: %x", err);
            show_dev_desc(dev_desc);

            const usb_config_desc_t* config_desc;
            err = usb_host_get_active_config_descriptor(driver_obj.Device_Handle, &config_desc);
            if (err != ESP_OK) ESP_LOGI("", "usb_host_get_config_desc: %x", err);
            (*_USB_host_enumerate)(config_desc);
        }
        break;
        /**< A device opened by the client is now gone */
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI("", "Device Gone handle: %x", event_msg->dev_gone.dev_hdl);
        break;
    default:
        ESP_LOGI("", "Unknown value %d", event_msg->event);
        break;
    }
}

void usbh_setup(usb_host_enum_cb_t enumeration_cb)
{
    const usb_host_config_t config = {
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&config);
    ESP_LOGI("", "usb_host_install: %x", err);

    const usb_host_client_config_t client_config = {
      .is_synchronous = false,
      .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
      .async = {
          .client_event_callback = _client_event_callback,
          .callback_arg = driver_obj.Client_Handle
      }
    };
    err = usb_host_client_register(&client_config, &driver_obj.Client_Handle);
    ESP_LOGI("", "usb_host_client_register: %x", err);

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
            ESP_LOGI("", "No more clients");
            all_clients_gone = true;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) 
        {
            ESP_LOGI("", "No more devices");
            all_dev_free = true;
        }
    }
    else 
    {
        if (err != ESP_ERR_TIMEOUT) 
        {
            ESP_LOGI("", "usb_host_lib_handle_events: %x flags: %x", err, event_flags);
        }
    }

    err = usb_host_client_handle_events(driver_obj.Client_Handle, CLIENT_EVENT_TIMEOUT);
    if ((err != ESP_OK) && (err != ESP_ERR_TIMEOUT))
    {
        ESP_LOGI("", "usb_host_client_handle_events: %x", err);
    }
}


//===================================================================================================

int rtlsdr_open(rtlsdr_dev_t* out_dev, uint8_t index, usb_host_client_handle_t client_hdl)
{
    //int r;
    //rtlsdr_dev_t* dev = NULL;
    //uint8_t reg;

   // dev = (rtlsdr_dev_t*)ps_malloc(sizeof(rtlsdr_dev_t*));
    //class_driver_t* driver_obj = (class_driver_t*)ps_calloc(1, sizeof(class_driver_t));
    //if (NULL == dev)
    //    return -ENOMEM;

   // memset(dev, 0, sizeof(rtlsdr_dev_t*));
   //!! memcpy(dev->fir, fir_default, sizeof(fir_default));

    //dev->dev_lost = 1;

    //driver_obj->Client_Handle = client_hdl;
    //ESP_ERROR_CHECK(usb_host_device_open(driver_obj->Client_Handle, index, &driver_obj->Device_Handle));
    //fprintf(stderr, "driver_obj->client_hdl: %d\n", driver_obj->Client_Handle);
    //fprintf(stderr, "driver_obj->dev_hdl: %d\n", driver_obj->Device_Handle);
    //!!dev->driver_obj = driver_obj;

    /**
    * Функция для клиента, чтобы заявить интерфейс устройства
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
//!!
    //ESP_ERROR_CHECK(usb_host_interface_claim(dev->driver_obj->Client_Handle, dev->driver_obj->Device_Handle, 0, 0));
    //dev->rtl_xtal = DEF_RTL_XTAL_FREQ;
    //init_adsb_dev();
    ///* perform a dummy write, if it fails, reset the device */
    //if (rtlsdr_write_reg(dev, USBB, USB_SYSCTL, 0x09, 1) < 0)
    //{
    //    fprintf(stderr, "Resetting device...\n");
    //    // libusb_reset_device(dev->devh);
    //}
    //else
    //{
    //    fprintf(stderr, "!!*** rtlsdr_write_reg Ok!...\n");
    //}

    //rtlsdr_init_baseband(dev);
    //dev->dev_lost = 0;

    ///* Probe tuners */
    //rtlsdr_set_i2c_repeater(dev, 1);

    //reg = rtlsdr_i2c_read_reg(dev, R820T_I2C_ADDR, R82XX_CHECK_ADDR);
    ////fprintf(stderr, "rtl device number %d\n", reg);
    ////fprintf(stderr, "rtlsdr_i2c_read_reg R82XX_CHECK_ADDR setting done\n");
    //if (reg == R82XX_CHECK_VAL)
    //{
    //    fprintf(stderr, "Found Rafael Micro R820T tuner\n");
    //    dev->tuner_type = RTLSDR_TUNER_R820T;
    //    goto found;
    //}


found:
    //!!
    ///* use the rtl clock value by default */
    //dev->tun_xtal = dev->rtl_xtal;
    //dev->tuner = &tuners[dev->tuner_type];

    //switch (dev->tuner_type)
    //{
    //case RTLSDR_TUNER_R828D:
    //    dev->tun_xtal = R828D_XTAL_FREQ;
    //    /* fall-through */
    //case RTLSDR_TUNER_R820T:
    //    /* disable Zero-IF mode */
    //    rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1a, 1);

    //    /* only enable In-phase ADC input */
    //    rtlsdr_demod_write_reg(dev, 0, 0x08, 0x4d, 1);

    //    /* the R82XX use 3.57 MHz IF for the DVB-T 6 MHz mode, and
    //     * 4.57 MHz for the 8 MHz mode */
    //    rtlsdr_set_if_freq(dev, R82XX_IF_FREQ);

    //    /* enable spectrum inversion */
    //    rtlsdr_demod_write_reg(dev, 1, 0x15, 0x01, 1);
    //    break;
    //case RTLSDR_TUNER_UNKNOWN:
    //    fprintf(stderr, "No supported tuner found\n");
    //    rtlsdr_set_direct_sampling(dev, 1);
    //    break;
    //default:
    //    break;
    //}


    //!!
    //if (dev->tuner->init)
    //    r = dev->tuner->init(dev);

    //rtlsdr_set_i2c_repeater(dev, 0);

    //*out_dev = dev;

    return 0;
}
