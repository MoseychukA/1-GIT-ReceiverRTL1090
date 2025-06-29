/*
 * SoCHelper.h
 * Copyright (C) 2018-2023 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SOCHELPER_H
#define SOCHELPER_H

#define SOC_UNUSED_PIN 255

#include "SoftRF.h"
#include "ESP32RF.h"


typedef struct SoC_ops_struct {
  uint8_t id;
  const char name[16];
  void (*setup)();
  void (*post_init)();
  void (*loop)();
  void (*fini)(int);
  void (*reset)();
  uint32_t (*getChipId)();
  void* (*getResetInfoPtr)();
  String (*getResetInfo)();
  String (*getResetReason)();
  uint32_t (*getFreeHeap)();
  long (*random)(long, long);
  uint32_t (*maxSketchSpace)();
  void (*WiFi_set_param)(int, int);
  void (*WiFi_transmit_UDP)(int, byte *, size_t);
  void (*WiFiUDP_stopAll)();
  bool (*WiFi_hostname)(String);
  int  (*WiFi_clients_count)();
  bool (*EEPROM_begin)(size_t);
  void (*EEPROM_extension)(int);
  void (*SPI_begin)();
  void (*swSer_begin)(unsigned long);
  void (*swSer_enableRx)(boolean);
  IODev_ops_t *Bluetooth_ops;
  IODev_ops_t *USB_ops;
  IODev_ops_t *UART_ops;
  byte (*Display_setup)();
  void (*Display_loop)();
  void (*Display_fini)(int);
#if 0
  bool (*Display_lock)();
  bool (*Display_unlock)();
#endif
  bool (*Baro_setup)();
  void (*WDT_setup)();
  void (*WDT_fini)();
} SoC_ops_t;

enum
{
	SOC_NONE,
	SOC_ESP32,
	SOC_ESP32S2,
	SOC_ESP32S3,
	SOC_ESP32C3,
};

extern const SoC_ops_t *SoC;

#if defined(ESP32)
extern const SoC_ops_t ESP32_ops;
#endif

byte SoC_setup(void);
void SoC_fini(int);
uint32_t DevID_Mapper(uint32_t);

#endif /* SOCHELPER_H */
