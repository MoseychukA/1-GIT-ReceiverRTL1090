/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Scott Shawcroft for Adafruit Industries LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_ATMEL_SAMD_EXTERNAL_FLASH_DEVICES_H
#define MICROPY_INCLUDED_ATMEL_SAMD_EXTERNAL_FLASH_DEVICES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t total_size;
  uint16_t start_up_time_us;

  // Three response bytes to 0x9f JEDEC ID command.
  uint8_t manufacturer_id;
  uint8_t memory_type;
  uint8_t capacity;

  // Max clock speed for all operations and the fastest read mode.
  uint8_t max_clock_speed_mhz;

  // Bitmask for Quad Enable bit if present. 0x00 otherwise. This is for the
  // highest byte in the status register.
  uint8_t quad_enable_bit_mask;

  bool has_sector_protection : 1;

  // Supports the 0x0b fast read command with 8 dummy cycles.
  bool supports_fast_read : 1;

  // Supports the fast read, quad output command 0x6b with 8 dummy cycles.
  bool supports_qspi : 1;

  // Supports the quad input page program command 0x32. This is known as 1-1-4
  // because it only uses all four lines for data.
  bool supports_qspi_writes : 1;

  // Requires a separate command 0x31 to write to the second byte of the status
  // register. Otherwise two byte are written via 0x01.
  bool write_status_register_split : 1;

  // True when the status register is a single byte. This implies the Quad
  // Enable bit is in the first byte and the Read Status Register 2 command
  // (0x35) is unsupported.
  bool single_status_byte : 1;

  // Fram does not need/support erase and has much simpler WRITE operation
  bool is_fram : 1;

} SPIFlash_Device_t;

// Settings for the Adesto Tech AT25DF081A 1MiB SPI flash. Its on the SAMD21
// Xplained board.
// Datasheet: https://www.adestotech.com/wp-content/uploads/doc8715.pdf
#define AT25DF081A                                                             \
  {                                                                            \
    .total_size = (1UL << 20), /* 1 MiB */                                     \
        .start_up_time_us = 10000, .manufacturer_id = 0x1f,                    \
    .memory_type = 0x84, .capacity = 0x01, .max_clock_speed_mhz = 85,          \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = true,               \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Adesto Tech AT25SF041 4MiB SPI flash used in AS7262 sensor
#define AT25SF041                                                              \
  {                                                                            \
    .total_size = (4UL << 20), /* 4 MiB */                                     \
        .start_up_time_us = 10000, .manufacturer_id = 0x1f,                    \
    .memory_type = 0x45, .capacity = 0x01, .max_clock_speed_mhz = 85,          \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = true,               \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Gigadevice GD25Q16C 2MiB SPI flash.
// Datasheet: http://www.gigadevice.com/datasheet/gd25q16c/
#define GD25Q16C                                                               \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc8,                     \
    .memory_type = 0x40, .capacity = 0x15,                                     \
    .max_clock_speed_mhz =                                                     \
        104, /* if we need 120 then we can turn on high performance mode */    \
        .quad_enable_bit_mask = 0x02, .has_sector_protection = false,          \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Gigadevice GD25Q32C 4MiB SPI flash.
// Datasheet: http://www.gigadevice.com/datasheet/gd25q32c/
#define GD25Q32C                                                               \
  {                                                                            \
    .total_size = (1UL << 22), /* 4 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc8,                     \
    .memory_type = 0x40, .capacity = 0x16,                                     \
    .max_clock_speed_mhz =                                                     \
        104, /* if we need 120 then we can turn on high performance mode */    \
        .quad_enable_bit_mask = 0x02, .has_sector_protection = false,          \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = true,         \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Gigadevice GD25Q64C 8MiB SPI flash.
// Datasheet:
// http://www.elm-tech.com/en/products/spi-flash-memory/gd25q64/gd25q64.pdf
#define GD25Q64C                                                               \
  {                                                                            \
    .total_size = (1UL << 23), /* 8 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc8,                     \
    .memory_type = 0x40, .capacity = 0x17,                                     \
    .max_clock_speed_mhz =                                                     \
        104, /* if we need 120 then we can turn on high performance mode */    \
        .quad_enable_bit_mask = 0x02, .has_sector_protection = false,          \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = true,         \
    .single_status_byte = false, .is_fram = false,                             \
  }

// https://www.fujitsu.com/uk/Images/MB85RS64V.pdf
// RDID has continuation code: 04-7F-03-02
#define MB85RS64V                                                              \
  {                                                                            \
    .total_size = 8UL * 1024, .start_up_time_us = 5000,                        \
    .manufacturer_id = 0x04, .memory_type = 0x03, .capacity = 0x02,            \
    .max_clock_speed_mhz = 20, .quad_enable_bit_mask = 0x00,                   \
    .has_sector_protection = false, .supports_fast_read = false,               \
    .supports_qspi = false, .supports_qspi_writes = false,                     \
    .write_status_register_split = false, .single_status_byte = true,          \
    .is_fram = true,                                                           \
  }

// https://www.fujitsu.com/uk/Images/MB85RS1MT.pdf
// RDID has continuation code: 04-7F-27-03
#define MB85RS1MT                                                              \
  {                                                                            \
    .total_size = 128UL * 1024, .start_up_time_us = 5000,                      \
    .manufacturer_id = 0x04, .memory_type = 0x27, .capacity = 0x03,            \
    .max_clock_speed_mhz = 40, .quad_enable_bit_mask = 0x00,                   \
    .has_sector_protection = false, .supports_fast_read = true,                \
    .supports_qspi = false, .supports_qspi_writes = false,                     \
    .write_status_register_split = false, .single_status_byte = true,          \
    .is_fram = true,                                                           \
  }

// https://www.fujitsu.com/uk/Images/MB85RS2MTA.pdf
// RDID has continuation code: 04-7F-48-03
#define MB85RS2MTA                                                             \
  {                                                                            \
    .total_size = 256UL * 1024, .start_up_time_us = 5000,                      \
    .manufacturer_id = 0x04, .memory_type = 0x48, .capacity = 0x03,            \
    .max_clock_speed_mhz = 40, .quad_enable_bit_mask = 0x00,                   \
    .has_sector_protection = false, .supports_fast_read = true,                \
    .supports_qspi = false, .supports_qspi_writes = false,                     \
    .write_status_register_split = false, .single_status_byte = true,          \
    .is_fram = true,                                                           \
  }

// https://www.fujitsu.com/uk/Images/MB85RS4MT.pdf
// RDID has continuation code: 04-7F-49-03
#define MB85RS4MT                                                              \
  {                                                                            \
    .total_size = 512UL * 1024, .start_up_time_us = 5000,                      \
    .manufacturer_id = 0x04, .memory_type = 0x49, .capacity = 0x03,            \
    .max_clock_speed_mhz = 40, .quad_enable_bit_mask = 0x00,                   \
    .has_sector_protection = false, .supports_fast_read = true,                \
    .supports_qspi = false, .supports_qspi_writes = true,                      \
    .write_status_register_split = false, .single_status_byte = true,          \
    .is_fram = true,                                                           \
  }

// Settings for the Macronix MX25L1606 2MiB SPI flash.
// Datasheet:
#define MX25L1606                                                              \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x20, .capacity = 0x15, .max_clock_speed_mhz = 8,           \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Macronix MX25R1635F 2MiB SPI flash.
// Datasheet:
// https://www.macronix.com/Lists/Datasheet/Attachments/7595/MX25R1635F,%20Wide%20Range,%2016Mb,%20v1.6.pdf
// By default its in lower power mode which can only do 8mhz. In high power mode
// it can do 80mhz.
#define MX25R1635F                                                             \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x28, .capacity = 0x15, .max_clock_speed_mhz = 8,           \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Macronix MX25L3233F 4MiB SPI flash.
// Datasheet:
// http://www.macronix.com/Lists/Datasheet/Attachments/7426/MX25L3233F,%203V,%2032Mb,%20v1.6.pdf
#define MX25L3233F                                                             \
  {                                                                            \
    .total_size = (1UL << 22), /* 4 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x20, .capacity = 0x16, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Macronix MX25L6433F 8MiB SPI flash.
// Datasheet:
// https://www.macronix.com/Lists/Datasheet/Attachments/7408/MX25L6433F,%203V,%2064Mb,%20v1.6.pdf
#define MX25L6433F                                                             \
  {                                                                            \
    .total_size = (1UL << 23), /* 8 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x20, .capacity = 0x17, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Macronix MX25R6435F 8MiB SPI flash.
// Datasheet:
// http://www.macronix.com/Lists/Datasheet/Attachments/7428/MX25R6435F,%20Wide%20Range,%2064Mb,%20v1.4.pdf
// By default its in lower power mode which can only do 8mhz. In high power mode
// it can do 80mhz.
#define MX25R6435F                                                             \
  {                                                                            \
    .total_size = (1UL << 23), /* 8 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x28, .capacity = 0x17, .max_clock_speed_mhz = 8,           \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Macronix MX25L12833F 16MiB SPI flash.
// Datasheet:
// https://www.macronix.com/Lists/Datasheet/Attachments/7408/MX25L12833F,%203V,%2064Mb,%20v1.6.pdf
#define MX25L12833F                                                            \
  {                                                                            \
    .total_size = (1UL << 24), /* 8 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xc2,                     \
    .memory_type = 0x20, .capacity = 0x18, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x40, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = true, .is_fram = false,                              \
  }

// Settings for the Cypress (was Spansion) S25FL064L 8MiB SPI flash.
// Datasheet: http://www.cypress.com/file/316661/download
#define S25FL064L                                                              \
  {                                                                            \
    .total_size = (1UL << 23), /* 8 MiB */                                     \
        .start_up_time_us = 300, .manufacturer_id = 0x01, .memory_type = 0x60, \
    .capacity = 0x17, .max_clock_speed_mhz = 108,                              \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Cypress (was Spansion) S25FL116K 2MiB SPI flash.
// Datasheet: http://www.cypress.com/file/196886/download
#define S25FL116K                                                              \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 10000, .manufacturer_id = 0x01,                    \
    .memory_type = 0x40, .capacity = 0x15, .max_clock_speed_mhz = 108,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Cypress (was Spansion) S25FL216K 2MiB SPI flash.
// Datasheet: http://www.cypress.com/file/197346/download
#define S25FL216K                                                              \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 10000, .manufacturer_id = 0x01,                    \
    .memory_type = 0x40, .capacity = 0x15, .max_clock_speed_mhz = 65,          \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q80DL 1MiB SPI flash.
// https://www.winbond.com/resource-files/w25q80dv%20dl_revh_10022015.pdf

#define W25Q80DL                                                               \
  {                                                                            \
    .total_size = (1UL << 20), /* 1 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x14, .max_clock_speed_mhz = 80,          \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q80DV 1MiB SPI flash.
// https://www.winbond.com/resource-files/w25q80dv%20dl_revh_10022015.pdf

#define W25Q80DV                                                               \
  {                                                                            \
    .total_size = (1UL << 20), /* 1 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x14, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q16FW 2MiB SPI flash.
// Datasheet:
// https://www.winbond.com/resource-files/w25q16fw%20revj%2005182017%20sfdp.pdf
#define W25Q16FW                                                               \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x60, .capacity = 0x15, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q16JV-IQ 2MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// https://www.winbond.com/resource-files/w25q16jv%20spi%20revf%2005092017.pdf
#define W25Q16JV_IQ                                                            \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x15, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q16JV-IM 2MiB SPI flash. Note that JV-IQ has a
// different .memory_type (0x40) Datasheet:
// https://www.winbond.com/resource-files/w25q16jv%20spi%20revf%2005092017.pdf
#define W25Q16JV_IM                                                            \
  {                                                                            \
    .total_size = (1UL << 21), /* 2 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x15, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q32BV 4MiB SPI flash.
// Datasheet:
// https://www.winbond.com/resource-files/w25q32bv_revi_100413_wo_automotive.pdf
#define W25Q32BV                                                               \
  {                                                                            \
    .total_size = (1UL << 22), /* 4 MiB */                                     \
        .start_up_time_us = 10000, .manufacturer_id = 0xef,                    \
    .memory_type = 0x60, .capacity = 0x16, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q32FV 4MiB SPI flash.
// Datasheet:http://www.winbond.com/resource-files/w25q32fv%20revj%2006032016.pdf?__locale=en
#define W25Q32FV                                                               \
  {                                                                            \
    .total_size = (1UL << 22), /* 4 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x16, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x00, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = false,                        \
    .supports_qspi_writes = false, .write_status_register_split = false,       \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q32JV-IM 4MiB SPI flash.
// https://www.winbond.com/resource-files/w25q32jv%20revg%2003272018%20plus.pdf
#define W25Q32JV_IM                                                            \
  {                                                                            \
    .total_size = (1UL << 22), /* 4 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x16, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
  }

// Settings for the Winbond W25Q32JV-IQ 4MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// https://www.winbond.com/resource-files/w25q32jv%20revh%2001072019%20plus.pdf
#define W25Q32JV_IQ                                                            \
  {                                                                            \
    .total_size = (1UL << 22), /* 4 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x16, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q64JV-IM 8MiB SPI flash. Note that JV-IQ has a
// different .memory_type (0x40) Datasheet:
// http://www.winbond.com/resource-files/w25q64jv%20revj%2003272018%20plus.pdf
#define W25Q64JV_IM                                                            \
  {                                                                            \
    .total_size = (1UL << 23), /* 8 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x17, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q64JV-IQ 8MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// http://www.winbond.com/resource-files/w25q64jv%20revj%2003272018%20plus.pdf
#define W25Q64JV_IQ                                                            \
  {                                                                            \
    .total_size = (1UL << 23), /* 8 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x17, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q128JV-SQ 16MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
#define W25Q128JV_SQ                                                           \
  {                                                                            \
    .total_size = (1UL << 24), /* 16 MiB */                                    \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x18, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q128JV-PM 16MiB SPI flash. Note that JV-IM has a
// different .memory_type (0x70) Datasheet:
// https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf
#define W25Q128JV_PM                                                           \
  {                                                                            \
    .total_size = (1UL << 24), /* 16 MiB */                                    \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x70, .capacity = 0x18, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Winbond W25Q256JV 32MiB SPI flash.
// https://www.winbond.com/resource-files/w25q256jv%20spi%20revg%2008032017.pdf
#define W25Q256JV                                                              \
  {                                                                            \
    .total_size = (1UL << 25), /* 32 MiB */                                    \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x19, .max_clock_speed_mhz = 133,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

// Settings for the Zetta Device ZD25WQ16B 2MiB SPI flash.
// Datasheet: http://www.zettadevice.com/upload/file/pdf/ZD25WQ16B_datasheet.pdf
#define ZD25WQ16B                                                              \
  {                                                                            \
    .total_size = (1 << 21), /* 2 MiB */                                       \
        .start_up_time_us = 12000, .manufacturer_id = 0xba,                    \
    .memory_type = 0x60, .capacity = 0x15, .max_clock_speed_mhz = 85,          \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }

#endif // MICROPY_INCLUDED_ATMEL_SAMD_EXTERNAL_FLASH_DEVICES_H
