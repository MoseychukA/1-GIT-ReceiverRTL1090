# GxEPD2
## Arduino Display Library for SPI E-Paper Displays

- With full Graphics and Text support using Adafruit_GFX

- For SPI e-paper displays from Dalian Good Display 
- and SPI e-paper boards from Waveshare

### important note :
- the display panels are for 3.3V supply and 3.3V data lines
- never connect data lines directly to 5V Arduino data pins, use e.g. 4k7/10k resistor divider
- series resistor only is not enough for reliable operation (back-feed effect through protection diodes)
- 4k7/10k resistor divider may not work with flat cable extensions or Waveshare 4.2 board, use level converter then
- do not forget to connect GND
- the actual Waveshare display boards now have level converters and series regulator, safe for 5V
- use 4k7 pull-down on SS for ESP8266 for boards with level converters
- note that 7.5" e-paper displays don't work reliable if fed from 3.3V Arduino pin

### Paged Drawing, Picture Loop
 - This library uses paged drawing to limit RAM use and cope with missing single pixel update support
 - buffer size can be selected in the application by template parameter page_height, see GxEPD2_Example
 - Paged drawing is implemented as picture loop, like in U8G2 (Oliver Kraus)
 - see https://github.com/olikraus/u8glib/wiki/tpictureloop
 - Paged drawing is also available using drawPaged() and drawCallback(), like in GxEPD
- ` // GxEPD style paged drawing; drawCallback() is called as many times as needed `
- ` void drawPaged(void (*drawCallback)(const void*), const void* pv) `
- paged drawing is done using Adafruit_GFX methods inside picture loop or drawCallback

### Full Screen Buffer Support
 - full screen buffer is selected by setting template parameter page_height to display height
 - drawing to full screen buffer is done using Adafruit_GFX methods without picture loop or drawCallback
 - and then calling method display()

### Low Level Bitmap Drawing Support
 - bitmap drawing support to the controller memory and screen is available:
 - either through the template class instance methods that forward calls to the base display class
 - or directly using an instance of a base display class and calling its methods directly

### Supporting Arduino Forum Topics:

- Waveshare e-paper displays with SPI: http://forum.arduino.cc/index.php?topic=487007.0
- Good Display ePaper for Arduino : https://forum.arduino.cc/index.php?topic=436411.0

### Note on documentation
- GxEPD2 uses Adafruit_GFX for Graphics and Text support, which is well documented there
- GxEPD2 uses meaningful method names, and has some comments in the header files
- consult the header files GxEPD2_BW.h, GxEPD2_3C.h and GxEPD2_GFX.h
- for the concept of paged drawing and picture loop see: 
- https://github.com/olikraus/u8glib/wiki/tpictureloop

### Supported SPI e-paper panels from Good Display:
- GDEP015OC1     1.54" b/w
- GDEH0154D67    1.54" b/w, replacement for GDEP015OC1
- GDEW0154Z04    1.54" b/w/r 200x200
- GDE0213B1      2.13" b/w
- GDEH0213B72    2.13" b/w, replacement for GDE0213B1
- GDEH0213B73    2.13" b/w, new replacement for GDE0213B1, GDEH0213B72
- GDEW0213I5F    2.13" b/w flexible
- GDEW0213Z16    2.13" b/w/r
- GDEH029A1      2.9" b/w
- GDEW029T5      2.9" b/w
- GDEW029Z10     2.9" b/w/r
- GDEW026T0      2.6" b/w
- GDEW027C44     2.7" b/w/r
- GDEW027W3      2.7" b/w
- GDEW0371W7     3.7" b/w
- GDEW042T2      4.2" b/w
- GDEW042Z15     4.2" b/w/r
- GDEW0583T7     5.83" b/w
- GDEW075T8      7.5" b/w
- GDEW075T7      7.5" b/w 800x480
- GDEW075Z09     7.5" b/w/r
- GDEW075Z08     7.5" b/w/r 800x480
#### Supported SPI e-paper panels & boards from Waveshare: compare with Good Display, same panel
#### other supported panels
- ED060SCT        6" grey levels, on Waveshare e-Paper IT8951 Driver HAT

### Version 1.2.4
- added support for GDEH0154D67 1.54" b/w, replacement for GDEP015OC1
- added GxEPD2_SerialFlash_Loader, WiFi bitmap downloader for SPI-flash
- added GxEPD2_SerialFlash_Example, SPI-flash example, e.g. for Winbond 25Q16BVSIG
- minor fixes and comment cleanups
- fix refresh(false) in upper layer: add powerOff() after full refresh
#### Version 1.2.3
- fixed partial update for 2.13" 3-color and 2.9" 3-color e-paper
- partial update can be disabled with attribute usePartialUpdateWindow = false
- added GxEPD2_GFX_Example to show uses of GxEPD2_GFX base class
- replaced GxEPD2_MultiDisplayExample code, same code as GxEPD2_GFX_MultiDisplayExample
- added extras/examples/GxEPD2_T_MultiDisplayExample, alternate example using template functions
- major and minor fixes, such as typos that survived too long
#### Version 1.2.2
- fixed BMP handling, e.g. for BMPs created by ImageMagick
- see also Arduino Forum Topic https://forum.arduino.cc/index.php?topic=642343.0
#### Version 1.2.1
- added support for GDEW075T7 7.5" b/w 800x480
- GDEW075T7 has differential update (1.6s) using a charge balancing waveform
- added optional SW SPI support, see /extras/sw_spi/README
- added /extras/tests/GxEPD2_RefreshTests/GxEPD2_RefreshTests.ino, for waveform tuning
- minor fixes 
- note that 7.5" e-paper displays don't work reliable if fed from 3.3V Arduino pin
#### Version 1.2.0
- added "fast partial update" (differential update) for GDEW0371W7 3.7" b/w 240x416
- improved differential update waveform for GDEW026T0 2.6" b/w 152x256
- fixed init code & improved differential update for GDEW042T2 4.2" b/w 300x400
- note that all differential refresh waveforms are a compromise (ghosting, big font use)
- parameters for differential waveform for these display can easily be changed for experimenting
- GDEW042T2 would have greyed background without sustain phase
- GDEW042T2 needs multiple full refreshes after extended use of partial updates
#### Version 1.1.10
- added support for GDEH0213B73 2.13" b/w, replacement for GDE0213B1, GDEH0213B72
- added support for GDEW026T0 2.6" b/w 152x256
- added support for GDEW0371W7 3.7" b/w 240x416
- added support for GDEW075Z08 7.5" b/w/r 800x480
- GDEW075Z08 does allow (slow) partial update, set usePartialUpdate = false to disable for better image
- changed 4.2" b/w waveform table, for better result with actual panels
#### Version 1.1.9
- note for ESP8266 when using SS for CS: (wiring suggestion) 
- connect 4.7k pull-down from GPIO15 to GND if your board or shield has level converters
- fixes for large displays (use uint16_t for buffer index)
#### Version 1.1.8
- fix for incomplete download in GxEPD2_WiFi_Example
- added missing method displayWindow() to GxEPD2_GFX base class
- fix and clean up of initial refresh for panels with differential update
- initial refresh needs to be full update, not "fast partial update", for these panels,
- as the screen content may differ from the "previous buffer" content.
- add clean of controller buffer(s) on initial write to controller, for partial update.
#### Version 1.1.7
- enhanced support for full buffered, non-paged use, for processors with enough RAM
- use void display(bool partial_update_mode = false); corresponds to update() in  GxEPD
- use added void displayWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
- use added writeImagePart(...), drawImagePart(...), used by displayWindow()
- added example GxEPD2_NotPagedExample.ino
- several fixes, e.g. parameter range check in setPartialWindow()
#### Version 1.1.6
- added support for GDEH0213B72 2.13" b/w, replacement for GDE0213B1
- changed SPI speed for IT8951 Driver HAT to 24MHz for write, 10MHz for read
- minor fixes, e.g. centering of text bounding box in GxEPD2_Example
#### Version 1.1.5
- added support for 6" ED060SCT on Waveshare e-Paper IT8951 Driver HAT
- uses 3.3V data lines, but 5V supply (~80mA active, ~20mA stand-by, sleep doesn't work)
- note: 5V supply needs to be exact and strong; 5V over diode from USB (e.g. Wemos D1 mini) doesn't work!
- note that the IT8951 Driver HAT is panel specific, with flash parameterized by supplier
- this is an initial version
#### Version 1.1.4+
- added GxEPD2_WS_ESP32_Driver example for Waveshare ESP32 Driver Board
#### Version 1.1.4
- eliminated double refresh for "fast partial update"
- moved wavetables to the driver classes
- added one explicit drawImage(...) and writeImage(...) method (for removed default paramter values for 1.1.3)
- added init method with added parameter  initial, for re-init after processor deep sleep wakeup
- added init parameter pulldown_rst_mode, for special RST handling (not needed for waveshare shield)
#### Version 1.1.3
- fixed wavetables for GDEW029T5 and GDEW0213I5F
- fixed drawImage(...) overloaded methods signature matching ambiguity
#### Version 1.1.2
- added support for GDEW029T5
- fixed (added) clipping for partial window
- fixed (added) powerOff() after full update (partial update keeps power on)
- added hibernate() for minimum power use by displays that support it
#### Version 1.1.1
- 2.7" b/w GDEW027W3 with fast partial update support, based on new demo code wavetable
- mapping suggestion added for Arduino MEGA
- NOTE: use voltage divider resistors for 5V Arduinos, series resistor is not reliable enough
- ConnectingHardware.md updated
#### Version 1.1.0
- added  support for GDEW0213I5F for 2.13" b/w 104x212 flexible display
- updated GxEPD2_WiFi_Example and GxEPD2_Spiffs_Loader to use BearSSL on ESP8266, for large bitmap downloads
#### Version 1.0.9
- add GxEPD2_U8G2_Fonts_Example, e.g. for use of Umlauts ÄÖÜäéöü
- NOTE: you need to SAVE the modified example to a saveable location for UTF-8 characters to work
#### Version 1.0.8
- add GxEPD2_SD_AVR_boards_added.h to GxEPD2_SD_AVR_Example
- with example definitions for non-AVR boards (low level display class use example)
#### Version 1.0.7
- add GxEPD2_boards_added.h to GxEPD2_Example 
- with example definitions for Arduino DUE and MKR1000
#### Version 1.0.6
- add buffered drawing option to GxEPD2_WiFi_Example
- allows use with 1.54" 3-color 200x200 display, may also be useful for small bitmaps
#### Version 1.0.5
- add buffered drawing option to GxEPD2_SD_Example and GxEPD2_Spiffs_Example
- allows use with 1.54" 3-color 200x200 display, may also be useful for small bitmaps
#### Version 1.0.4
- add GxEPD2_GFX base class support (optional, selectable, uses slightly more code)
- base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter
- add GxEPD2_GFX_MultiDisplayExample, uses  GxEPD2_GFX reference parameter
- modify GxEPD2_MultiDisplayExample for ESP8266 (template issue: class expected instead of typename)
#### Version 1.0.3
- fix GxEPD2_SD_Example & GxEPD2_SD_AVR_Example
- add GxEPD2_MultiDisplayExample (preliminary version)
#### Version 1.0.2
- initial release version 1.0.2
- tested with ESP8266, ESP32, STM32F103C8T6, AVR Arduino (Pro Mini 3.3V)
- 1.54" 3-color GxEPD2_154c can be used with paging for AVR
- wave tables in program space,  4.2" b/w can be used with GxEPD2_SD_AVR_Example
- issues enabled and welcome, please use Forum Topic for enhancement suggestions
#### Version 1.0.1
- pre-release test version
- GxEPD2_SD_AVR_Example added, has no graphics buffer to reduce RAM usage (base display class use)
- issues disabled so far, use Arduino Forum Topics instead
- in this version 1.54" 3-color GxEPD2_154c can only be used with full size buffer (or with no buffer base display class)
#### Version 1.0.0
- preliminary version, under construction
