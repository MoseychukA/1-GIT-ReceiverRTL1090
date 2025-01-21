#include "esp_libusb.h"
#include "usb/usb_host.h"
#include "esp_intr_alloc.h"
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp32-hal-psram.h"

static class_adsb_dev* adsbdev;

// Для информации
//typedef enum {
//USB_TRANSFER_STATUS_COMPLETED,      /**< Перевод прошел успешно (но может быть коротким) */
//USB_TRANSFER_STATUS_ERROR,          /**< Передача не удалась из-за большого количества ошибок (например, отсутствие ответа или ошибка CRC) */
//USB_TRANSFER_STATUS_TIMED_OUT,      /**< Передача не удалась из-за тайм-аута */
//USB_TRANSFER_STATUS_CANCELED,       /**< Перевод был отменен. */
//USB_TRANSFER_STATUS_STALL,          /**< Передача была остановлена. */
//USB_TRANSFER_STATUS_OVERFLOW,       /**< !! Здесь проблема!! Передача, так как было отправлено больше данных, чем было запрошено */
//USB_TRANSFER_STATUS_SKIPPED,        /**< Только пакеты ISOC. Пакет был пропущен из-за задержки системы или перегрузки шины */
//USB_TRANSFER_STATUS_NO_DEVICE,      /**< Передача не удалась, так как целевое устройство исчезло. */
//} usb_transfer_status_t;
//


void init_adsb_dev()
{
    adsbdev = (class_adsb_dev*)ps_calloc(1, sizeof(class_adsb_dev));
    adsbdev->is_adsb = true;
}

void bulk_transfer_read_cb(usb_transfer_t* transfer)
{
  // printf("BULK: actual_num_bytes %d\n", transfer->actual_num_bytes);
  
    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done = true;
    adsbdev->is_success = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->num_bytes;
    printf("BULK: Transfer:Read type %d\n", transfer->num_bytes);
    printf("BULK: Transfer:Read status %d, actual number of bytes transferred %d, databuffer size %d, %d\n", transfer->status, transfer->actual_num_bytes, transfer->data_buffer_size, adsbdev->response_buf[8]);
    //!! Здесь проблема!! Передача, так как было отправлено больше данных, чем было запрошено
}
void transfer_read_cb(usb_transfer_t* transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done = true;
    adsbdev->is_success = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->actual_num_bytes - sizeof(usb_setup_packet_t);
   // printf("Transfer:Read type %d %ld \n", transfer->actual_num_bytes, transfer->flags);
   // printf("Transfer:Read status %d, actual number of bytes transferred %d, databuffer size %d, %d\n", transfer->status, transfer->actual_num_bytes, transfer->data_buffer[8], adsbdev->response_buf[8]);
}

/*
* @brief Структура передачи USB
*
* Эта структура используется для представления передачи от программного клиента к конечной точке по шине USB. Некоторые из
* полей намеренно сделаны константными, поскольку они фиксированы при выделении. Пользователи должны вызывать соответствующую функцию библиотеки USB Host
* для выделения структуры передачи USB вместо того, чтобы выделять эту структуру самостоятельно.
*
* Тип передачи выводится из конечной точки, на которую отправляется эта передача. В зависимости от типа передачи пользователи
* должны обратить внимание на следующее:
*
* - Bulk: эта структура представляет собой одну массовую передачу. Если количество байтов превышает MPS конечной точки,
* передача будет разделена на несколько пакетов размером MPS, за которыми следует короткий пакет.
* - Control: эта структура представляет собой одну передачу управления. Эти первые 8 байтов data_buffer должны быть заполнены
* пакетом настройки (см. usb_setup_packet_t). Поле num_bytes должно быть общим размером
* передачи (т. е. размером установочного пакета + wLength).
* - Прерывание: представляет собой прерывание передачи. Если num_bytes превышает MPS конечной точки, передача будет
* разделена на несколько пакетов, и каждый пакет будет передан с указанным интервалом конечной точки.
* - Изохронный: представляет собой поток байтов, который должен быть передан на конечную точку с фиксированной скоростью. Передача
* разделяется на пакеты в соответствии с каждым isoc_packet_desc. Пакет передается с каждым интервалом
* конечной точки. Если весь ISOC URB был передан без ошибок (пропущенные пакеты не считаются
* ошибками), общее состояние URB и состояние каждого дескриптора пакета будут обновлены, а
* actual_num_bytes отражает общее количество байтов, переданных по всем пакетам. Если ISOC URB обнаруживает
* ошибку, весь URB считается ошибочным, поэтому будет обновлен только общий статус.
*
* @note Для массовых/контрольных/прерывных IN-передач num_bytes должен быть целым числом, кратным MPS конечной точки
* @note Эта структура должна быть выделена через usb_host_transfer_alloc()
* @note После отправки передачи пользователи не должны изменять структуру, пока передача не будет завершена
*/
int esp_libusb_bulk_transfer(class_driver_t* driver_obj, unsigned char endpoint, unsigned char* data, int length, int* transferred, unsigned int timeout)
{
    fprintf(stderr, "**!!** esp_libusb_bulk_transfer info %X %d %d %d\n", endpoint, length, timeout, *transferred);
    assert(driver_obj->dev_hdl != NULL);
    size_t sizePacket = usb_round_up_to_mps(length, 64);
    usb_transfer_t* transfer = NULL;
    esp_err_t err;
    err = usb_host_transfer_alloc(sizePacket, 0, &transfer);
    if (err != ESP_OK) 
    {
        transfer = NULL;
        fprintf(stderr, "!!**usb_host_transfer_alloc In fail: %x\n", err);
    }
    else
    {
        fprintf(stderr, "**!!**usb_host_transfer_alloc In Ok!: %x\n", err);
    }

    //fprintf(stderr, "esp_libusb_bulk_transfer usb_host_transfer_alloc %d\n", sizePacket);
    transfer->num_bytes = sizePacket;
    transfer->device_handle = driver_obj->dev_hdl;
    transfer->bEndpointAddress = endpoint;
    transfer->callback = bulk_transfer_read_cb; //массовая передача чтения
    transfer->context = (void*)&driver_obj;
    transfer->timeout_ms = timeout;
    adsbdev->is_done = false;
    adsbdev->response_buf = (uint8_t*)ps_calloc(sizePacket, sizeof(uint8_t));

    fprintf(stderr, "transfer->num_bytes %d\n", transfer->num_bytes);
    fprintf(stderr, "transfer->device_handle %d\n", transfer->device_handle);
    fprintf(stderr, "transfer->bEndpointAddress %x\n", transfer->bEndpointAddress);
    fprintf(stderr, "transfer->callback %d\n", transfer->callback);
    fprintf(stderr, "transfer->context %d\n", transfer->context);
    fprintf(stderr, "transfer->timeout_ms %d\n", transfer->timeout_ms);

    /**
    * - Отправить неконтролируемый перевод
    * - Отправить перевод на определенную конечную точку. Устройство и номер конечной точки указываются внутри перевода
    * - Перед отправкой перевод должен быть правильно инициализирован
    * - По завершении обратный вызов перевода будет вызван из функции usb_host_client_handle_events() клиента.
    *
    * @param[in] перевод Инициализированный объект перевода
    * @return esp_err_t
    */
    esp_err_t r = usb_host_transfer_submit(transfer);

    if (r != ESP_OK)
    {
        fprintf(stderr, "!!!!!esp_libusb_bulk_transfer failed with % d\n", r);
        return -1;
    }
    else
    {
        fprintf(stderr, "!!!!!esp_libusb_bulk_transfer OK!! with % d\n", r);
    }
 

    //fprintf(stderr, "!!!!!!adsbdev->is_done % d\n", !adsbdev->is_done);
    while (!adsbdev->is_done)
    {
        /**
        * @brief Функция обработки клиента USB Host Library
        *
        * - Эта функция обрабатывает всю обработку клиента и должна вызываться многократно в цикле
        * - Для конкретного клиента эта функция никогда не должна вызываться несколькими потоками одновременно
        *
        * @note Эта функция может блокировать
        * @param[in] client_hdl Дескриптор клиента
        * @param[in] timeout_ticks Тайм-аут в тиках для ожидания события
        * @return esp_err_t
        */

        fprintf(stderr, "!!!!!!adsbdev->is_done % d\n", !adsbdev->is_done);
        fprintf(stderr, "!!!!!client_hdl % d\n", driver_obj->client_hdl);
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);  // 

        err = usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
        if ((err != ESP_OK) && (err != ESP_ERR_TIMEOUT))
        {
            fprintf(stderr, "*!!!!* usb_host_client_handle_events: %x\n", err);
        }
        else
        {
            fprintf(stderr, "*!!!!* usb_host_client_handle_events Ok!!: %x\n", err);
        }
    }

   // adsbdev->is_success = 1;
   // fprintf(stderr, "*!!!!* adsbdev->is_success %d\n", adsbdev->is_success);

   // int tst_is_success = adsbdev->is_success;

    if (!adsbdev->is_success)
    {
        fprintf(stderr, "!!!!esp_libusb_bulk_transfer failed\n");
        return -1;
    }

    ESP_ERROR_CHECK(usb_host_endpoint_clear(driver_obj->dev_hdl, endpoint));
    for (int i = 0; i < length; i++)
    {
        data[i] = adsbdev->response_buf[i];
    }
    *transferred = adsbdev->bytes_transferred;
    usb_host_transfer_free(transfer);
    return 0;
}
int esp_libusb_control_transfer(class_driver_t* driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char* data, uint16_t wLength, unsigned int timeout)
{
    ESP_ERROR_CHECK(usb_host_transfer_free(adsbdev->transfer));
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
    adsbdev->response_buf = (uint8_t*)ps_calloc(sizePacket, sizeof(uint8_t));

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
        fprintf(stderr, "libusb_control_transfer failed with %d\n", r);
        return -1;
    }

    while (!adsbdev->is_done)
    {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }

    if (!adsbdev->is_success)
    {
        fprintf(stderr, "*!!!!* libusb_control_transfer failed\n");
        return -1;
    }
    else
    {
       // fprintf(stderr, "*!!!!* libusb_control_transfer OK!\n");
    }

    for (uint8_t i = 0; i < wLength; i++)
    {
        data[i] = adsbdev->response_buf[sizeof(usb_setup_packet_t) + i];
    }
    return adsbdev->bytes_transferred;
}
void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t* str_desc, char* str)
{
    if (str_desc == NULL)
    {
        return;
    }

    for (int i = 0; i < str_desc->bLength / 2; i++)
    {
        str[i] = (char)str_desc->wData[i];
    }
}


