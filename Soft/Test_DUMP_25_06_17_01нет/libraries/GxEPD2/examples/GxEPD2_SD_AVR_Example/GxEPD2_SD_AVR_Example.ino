// GxEPD2_SD_AVR_Example : Display Library example for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: the e-paper panels require 3.3V supply AND data lines!
//
// Display Library based on Demo Example from Good Display: http://www.e-paper-display.com/download_list/downloadcategoryid=34&isMode=false.html
//
// BMP handling code extracts taken from: https://github.com/prenticedavid/MCUFRIEND_kbv/tree/master/examples/showBMP_kbv_Uno
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

// Supporting Arduino Forum Topics:
// Waveshare e-paper displays with SPI: http://forum.arduino.cc/index.php?topic=487007.0
// Good Display ePaper for Arduino : https://forum.arduino.cc/index.php?topic=436411.0

// mapping suggestion from Waveshare SPI e-Paper to Wemos D1 mini
// BUSY -> D2, RST -> D4, DC -> D3, CS -> D8, CLK -> D5, DIN -> D7, GND -> GND, 3.3V -> 3.3V

// mapping suggestion from Waveshare SPI e-Paper to generic ESP8266
// BUSY -> GPIO4, RST -> GPIO2, DC -> GPIO0, CS -> GPIO15, CLK -> GPIO14, DIN -> GPIO13, GND -> GND, 3.3V -> 3.3V

// mapping of Waveshare e-Paper ESP8266 Driver Board
// BUSY -> GPIO16, RST -> GPIO5, DC -> GPIO4, CS -> GPIO15, CLK -> GPIO14, DIN -> GPIO13, GND -> GND, 3.3V -> 3.3V

// mapping suggestion for ESP32, e.g. LOLIN32, see .../variants/.../pins_arduino.h for your board
// NOTE: there are variants with different pins for SPI ! CHECK SPI PINS OF YOUR BOARD
// BUSY -> 4, RST -> 16, DC -> 17, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V

// new mapping suggestion for STM32F1, e.g. STM32F103C8T6 "BluePill"
// BUSY -> A1, RST -> A2, DC -> A3, CS-> A4, CLK -> A5, DIN -> A7

// mapping suggestion for AVR, UNO, NANO etc.
// BUSY -> 7, RST -> 9, DC -> 8, CS-> 10, CLK -> 13, DIN -> 11
//
// **** NOTE that the mapping suggestion may need modification depending on SD board used! ****
// ********************************************************************************************
//
//#define SD_CS SS  // e.g. for RobotDyn Wemos D1 mini SD board
//#define EPD_CS D1 // alternative I use with RobotDyn Wemos D1 mini SD board

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#if defined(ESP32)

// has support for FAT32 support with long filenames
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define SdFile File
#define seekSet seek

#else

// include SdFat for FAT32 support with long filenames; available through Library Manager
#include <SdFat.h>
SdFat SD;

#endif

#if defined(__AVR)
#define SD_CS 6  // adapt to your wiring
#define EPD_CS SS // adapt to your wiring
// select one and adapt to your mapping
//GxEPD2_154 display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7); // GDEP015OC1 no longer available
//GxEPD2_154_D67 display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7); // GDEH0154D67
//GxEPD2_213 display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_290 display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_270 display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_420 display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_750 display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
// 3-color e-papers
// !!! can't be used for this //GxEPD2_154c display(GxEPD2_154c(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_213c display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_290c display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_270c display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_420c display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_750c display(/*CS=10*/ EPD_CS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
#endif

// non-AVR board can also be used with GxEPD2 base display classes, e.g. for SD bitmap drawing
// uncomment your display for your board in the following included header file and adapt to your mapping

#include "GxEPD2_SD_AVR_boards_added.h"

// function declaration with default parameter
void drawBitmapFromSD(const char *filename, int16_t x, int16_t y, bool with_color = true);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("setup"));

  display.init(115200);

  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(SD_CS))
  {
    Serial.println(F("SD failed!"));
    return;
  }
  Serial.println(F("SD OK!"));

  drawBitmaps_200x200();
  drawBitmaps_other();

  //drawBitmaps_test();
}

void loop(void)
{
}

void drawBitmaps_200x200()
{
  int16_t x = (int16_t(display.WIDTH) - 200) / 2;
  int16_t y = (int16_t(display.HEIGHT) - 200) / 2;
  drawBitmapFromSD("logo200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("first200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("second200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("third200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("fourth200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("fifth200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("sixth200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("seventh200x200.bmp", x, y);
  delay(2000);
  drawBitmapFromSD("eighth200x200.bmp", x, y);
  delay(2000);
}

void drawBitmaps_other()
{
  int16_t w2 = display.WIDTH / 2;
  int16_t h2 = display.HEIGHT / 2;
  drawBitmapFromSD("parrot.bmp", w2 - 64, h2 - 80);
  delay(2000);
  drawBitmapFromSD("betty_1.bmp", w2 - 100, h2 - 160);
  delay(2000);
  drawBitmapFromSD("betty_4.bmp", w2 - 102, h2 - 126);
  delay(2000);
  drawBitmapFromSD("marilyn_240x240x8.bmp", w2 - 120, h2 - 120);
  delay(2000);
  drawBitmapFromSD("miniwoof.bmp", w2 - 60, h2 - 80);
  delay(2000);
  drawBitmapFromSD("t200x200.bmp", w2 - 100, h2 - 100);
  delay(2000);
  drawBitmapFromSD("test.bmp", w2 - 120, h2 - 160);
  delay(2000);
  drawBitmapFromSD("tiger.bmp", w2 - 160, h2 - 120);
  delay(2000);
  drawBitmapFromSD("tiger_178x160x4.bmp", w2 - 89, h2 - 80);
  delay(2000);
  drawBitmapFromSD("tiger_240x317x4.bmp", w2 - 120, h2 - 160);
  delay(2000);
  drawBitmapFromSD("tiger_320x200x24.bmp", w2 - 160, h2 - 100);
  delay(2000);
  drawBitmapFromSD("tiger16T.bmp", w2 - 160, h2 - 120);
  delay(2000);
  drawBitmapFromSD("woof.bmp", w2 - 120, h2 - 160);
  delay(2000);
  drawBitmapFromSD("bitmap640x384_1.bmp", 0, 0);
  delay(2000);
}

void drawBitmaps_test()
{
  int16_t w2 = display.WIDTH / 2;
  int16_t h2 = display.HEIGHT / 2;
  drawBitmapFromSD("betty_4.bmp", w2 - 102, h2 - 126);
  delay(2000);
}

static const uint16_t input_buffer_pixels = 20; // may affect performance

static const uint16_t max_row_width = 640; // for up to 7.5" display
static const uint16_t max_palette_pixels = 256; // for depth <= 8

uint8_t input_buffer[3 * input_buffer_pixels]; // up to depth 24
uint8_t output_row_mono_buffer[max_row_width / 8]; // buffer for at least one row of b/w bits
uint8_t output_row_color_buffer[max_row_width / 8]; // buffer for at least one row of color bits
uint8_t mono_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 b/w
uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w

void drawBitmapFromSD(const char *filename, int16_t x, int16_t y, bool with_color)
{
  SdFile file;
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top
  uint32_t startTime = millis();
  //Serial.println(); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(filename);
  if ((x >= int16_t(display.WIDTH)) || (y >= int16_t(display.HEIGHT))) return;
  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
#if defined(ESP32)
  file = SD.open(String("/") + filename, FILE_READ);
  if (!file)
  {
    Serial.print(F("File not found"));
    return;
  }
#else
  if (!file.open(filename, FILE_READ))
  {
    Serial.print(F("File not found"));
    return;
  }
#endif
  // Parse BMP header
  if (read16(file) == 0x4D42) // BMP signature
  {
    uint32_t fileSize = read32(file);
    uint32_t creatorBytes = read32(file);
    uint32_t imageOffset = read32(file); // Start of image data
    uint32_t headerSize = read32(file);
    uint32_t width  = read32(file);
    uint32_t height = read32(file);
    uint16_t planes = read16(file);
    uint16_t depth = read16(file); // bits per pixel
    uint32_t format = read32(file);
    if ((planes == 1) && ((format == 0) || (format == 3))) // uncompressed is handled, 565 also
    {
      Serial.print(F("File size: ")); Serial.println(fileSize);
      Serial.print(F("Image Offset: ")); Serial.println(imageOffset);
      Serial.print(F("Header size: ")); Serial.println(headerSize);
      Serial.print(F("Bit Depth: ")); Serial.println(depth);
      Serial.print(F("Image size: "));
      Serial.print(width);
      Serial.print('x');
      Serial.println(height);
      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;
      if (height < 0)
      {
        height = -height;
        flip = false;
      }
      uint16_t w = width;
      uint16_t h = height;
      if ((x + w - 1) >= int16_t(display.WIDTH))  w = int16_t(display.WIDTH)  - x;
      if ((y + h - 1) >= int16_t(display.HEIGHT)) h = int16_t(display.HEIGHT) - y;
      if (w <= max_row_width) // handle with direct drawing
      {
        valid = true;
        uint8_t bitmask = 0xFF;
        uint8_t bitshift = 8 - depth;
        uint16_t red, green, blue;
        bool whitish, colored;
        if (depth == 1) with_color = false;
        if (depth <= 8)
        {
          if (depth < 8) bitmask >>= depth;
          //file.seekSet(54); //palette is always @ 54
          file.seekSet(imageOffset - (4 << depth)); // 54 for regular, diff for colorsimportant
          for (uint16_t pn = 0; pn < (1 << depth); pn++)
          {
            blue  = file.read();
            green = file.read();
            red   = file.read();
            file.read();
            whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
            colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
            if (0 == pn % 8) mono_palette_buffer[pn / 8] = 0;
            mono_palette_buffer[pn / 8] |= whitish << pn % 8;
            if (0 == pn % 8) color_palette_buffer[pn / 8] = 0;
            color_palette_buffer[pn / 8] |= colored << pn % 8;
          }
        }
        display.clearScreen();
        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) // for each line
        {
          uint32_t in_remain = rowSize;
          uint32_t in_idx = 0;
          uint32_t in_bytes = 0;
          uint8_t in_byte = 0; // for depth <= 8
          uint8_t in_bits = 0; // for depth <= 8
          uint8_t out_byte = 0xFF; // white (for w%8!=0 border)
          uint8_t out_color_byte = 0xFF; // white (for w%8!=0 border)
          uint32_t out_idx = 0;
          file.seekSet(rowPosition);
          for (uint16_t col = 0; col < w; col++) // for each pixel
          {
            // Time to read more pixel data?
            if (in_idx >= in_bytes) // ok, exact match for 24bit also (size IS multiple of 3)
            {
              in_bytes = file.read(input_buffer, in_remain > sizeof(input_buffer) ? sizeof(input_buffer) : in_remain);
              in_remain -= in_bytes;
              in_idx = 0;
            }
            switch (depth)
            {
              case 24:
                blue = input_buffer[in_idx++];
                green = input_buffer[in_idx++];
                red = input_buffer[in_idx++];
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                break;
              case 16:
                {
                  uint8_t lsb = input_buffer[in_idx++];
                  uint8_t msb = input_buffer[in_idx++];
                  if (format == 0) // 555
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                    red   = (msb & 0x7C) << 1;
                  }
                  else // 565
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                    red   = (msb & 0xF8);
                  }
                  whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                  colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                }
                break;
              case 1:
              case 4:
              case 8:
                {
                  if (0 == in_bits)
                  {
                    in_byte = input_buffer[in_idx++];
                    in_bits = 8;
                  }
                  uint16_t pn = (in_byte >> bitshift) & bitmask;
                  whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  in_byte <<= depth;
                  in_bits -= depth;
                }
                break;
            }
            if (whitish)
            {
              //out_byte |= 0x80 >> col % 8; // not black
              //out_color_byte |= 0x80 >> col % 8; // not colored
              // keep white
            }
            else if (colored && with_color)
            {
              //out_byte |= 0x80 >> col % 8; // not black
              out_color_byte &= ~(0x80 >> col % 8); // colored
            }
            else
            {
              //out_color_byte |= 0x80 >> col % 8; // not colored
              out_byte &= ~(0x80 >> col % 8); // black
            }
            if ((7 == col % 8) || (col == w - 1)) // write that last byte! (for w%8!=0 border)
            {
              output_row_color_buffer[out_idx] = out_color_byte;
              output_row_mono_buffer[out_idx++] = out_byte;
              out_byte = 0xFF; // white (for w%8!=0 border)
              out_color_byte = 0xFF; // white (for w%8!=0 border)
            }
          } // end pixel
          uint16_t yrow = y + (flip ? h - row - 1 : row);
          display.writeImage(output_row_mono_buffer, output_row_color_buffer, x, yrow, w, 1);
        } // end line
        Serial.print(F("loaded in ")); Serial.print(millis() - startTime); Serial.println(F(" ms"));
        display.refresh();
      }
    }
  }
  //Serial.print(F("end curPosition  ")); Serial.println(file.curPosition());
  file.close();
  if (!valid)
  {
    Serial.println(F("bitmap format not handled."));
  }
}

uint16_t read16(SdFile& f)
{
  // BMP data is stored little-endian, same as Arduino.
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(SdFile& f)
{
  // BMP data is stored little-endian, same as Arduino.
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
