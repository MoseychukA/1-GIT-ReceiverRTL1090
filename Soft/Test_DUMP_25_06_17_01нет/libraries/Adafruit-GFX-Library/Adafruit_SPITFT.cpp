/*!
 * @file Adafruit_SPITFT.cpp
 *
 * @mainpage Adafruit SPI TFT Displays (and some others)
 *
 * @section intro_sec Introduction
 *
 * Part of Adafruit's GFX graphics library. Originally this class was
 * written to handle a range of color TFT displays connected via SPI,
 * but over time this library and some display-specific subclasses have
 * mutated to include some color OLEDs as well as parallel-interfaced
 * displays. The name's been kept for the sake of older code.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!

 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries,
 * with contributions from the open source community.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 */

#if !defined(__AVR_ATtiny85__) // Not for ATtiny, at all

#include "Adafruit_SPITFT.h"

#if defined(PORT_IOBUS)
// On SAMD21, redefine digitalPinToPort() to use the slightly-faster
// PORT_IOBUS rather than PORT (not needed on SAMD51).
#undef  digitalPinToPort
#define digitalPinToPort(P) (&(PORT_IOBUS->Group[g_APinDescription[P].ulPort]))
#endif // end PORT_IOBUS

#if defined(USE_SPI_DMA)
#include <Adafruit_ZeroDMA.h>
#include <malloc.h> // memalign() function

// DMA transfer-in-progress indicator and callback
static volatile bool dma_busy = false;
static void dma_callback(Adafruit_ZeroDMA *dma) {
    dma_busy = false;
}
#endif // end USE_SPI_DMA

// Possible values for Adafruit_SPITFT.connection:
#define TFT_HARD_SPI 0  ///< Display interface = hardware SPI
#define TFT_SOFT_SPI 1  ///< Display interface = software SPI
#define TFT_PARALLEL 2  ///< Display interface = 8- or 16-bit parallel


// CONSTRUCTORS ------------------------------------------------------------

/*!
    @brief   Adafruit_SPITFT constructor for software (bitbang) SPI.
    @param   w     Display width in pixels at default rotation setting (0).
    @param   h     Display height in pixels at default rotation setting (0).
    @param   cs    Arduino pin # for chip-select (-1 if unused, tie CS low).
    @param   dc    Arduino pin # for data/command select (required).
    @param   mosi  Arduino pin # for bitbang SPI MOSI signal (required).
    @param   sck   Arduino pin # for bitbang SPI SCK signal (required).
    @param   rst   Arduino pin # for display reset (optional, display reset
                   can be tied to MCU reset, default of -1 means unused).
    @param   miso  Arduino pin # for bitbang SPI MISO signal (optional,
                   -1 default, many displays don't support SPI read).
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized; application typically will
             need to call subclass' begin() function, which in turn calls
             this library's initSPI() function to initialize pins.
*/
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h,
  int8_t cs, int8_t dc, int8_t mosi, int8_t sck, int8_t rst, int8_t miso) :
  Adafruit_GFX(w, h), connection(TFT_SOFT_SPI), _rst(rst), _cs(cs), _dc(dc) {
    swspi._sck  = sck;
    swspi._mosi = mosi;
    swspi._miso = miso;
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(CORE_TEENSY)
   #if !defined(KINETISK)
    dcPinMask          = digitalPinToBitMask(dc);
   #endif
    dcPortSet          = portSetRegister(dc);
    dcPortClr          = portClearRegister(dc);
    swspi.sckPortSet   = portSetRegister(sck);
    swspi.sckPortClr   = portClearRegister(sck);
    swspi.mosiPortSet  = portSetRegister(mosi);
    swspi.mosiPortClr  = portClearRegister(mosi);
    if(cs >= 0) {
   #if !defined(KINETISK)
        csPinMask      = digitalPinToBitMask(cs);
   #endif
        csPortSet      = portSetRegister(cs);
        csPortClr      = portClearRegister(cs);
    } else {
   #if !defined(KINETISK)
        csPinMask      = 0;
   #endif
        csPortSet      = dcPortSet;
        csPortClr      = dcPortClr;
    }
    if(miso >= 0) {
        swspi.misoPort = portInputRegister(miso);
    } else {
        swspi.misoPort = portInputRegister(dc);
    }
  #else  // !CORE_TEENSY
    dcPinMask        =digitalPinToBitMask(dc);
    swspi.sckPinMask =digitalPinToBitMask(sck);
    swspi.mosiPinMask=digitalPinToBitMask(mosi);
    dcPortSet        =&(PORT->Group[g_APinDescription[dc].ulPort].OUTSET.reg);
    dcPortClr        =&(PORT->Group[g_APinDescription[dc].ulPort].OUTCLR.reg);
    swspi.sckPortSet =&(PORT->Group[g_APinDescription[sck].ulPort].OUTSET.reg);
    swspi.sckPortClr =&(PORT->Group[g_APinDescription[sck].ulPort].OUTCLR.reg);
    swspi.mosiPortSet=&(PORT->Group[g_APinDescription[mosi].ulPort].OUTSET.reg);
    swspi.mosiPortClr=&(PORT->Group[g_APinDescription[mosi].ulPort].OUTCLR.reg);
    if(cs >= 0) {
        csPinMask = digitalPinToBitMask(cs);
        csPortSet = &(PORT->Group[g_APinDescription[cs].ulPort].OUTSET.reg);
        csPortClr = &(PORT->Group[g_APinDescription[cs].ulPort].OUTCLR.reg);
    } else {
        // No chip-select line defined; might be permanently tied to GND.
        // Assign a valid GPIO register (though not used for CS), and an
        // empty pin bitmask...the nonsense bit-twiddling might be faster
        // than checking _cs and possibly branching.
        csPortSet = dcPortSet;
        csPortClr = dcPortClr;
        csPinMask = 0;
    }
    if(miso >= 0) {
        swspi.misoPinMask=digitalPinToBitMask(miso);
        swspi.misoPort   =(PORTreg_t)portInputRegister(digitalPinToPort(miso));
    } else {
        swspi.misoPinMask=0;
        swspi.misoPort   =(PORTreg_t)portInputRegister(digitalPinToPort(dc));
    }
  #endif // end !CORE_TEENSY
 #else  // !HAS_PORT_SET_CLR
    dcPort              =(PORTreg_t)portOutputRegister(digitalPinToPort(dc));
    dcPinMaskSet        =digitalPinToBitMask(dc);
    swspi.sckPort       =(PORTreg_t)portOutputRegister(digitalPinToPort(sck));
    swspi.sckPinMaskSet =digitalPinToBitMask(sck);
    swspi.mosiPort      =(PORTreg_t)portOutputRegister(digitalPinToPort(mosi));
    swspi.mosiPinMaskSet=digitalPinToBitMask(mosi);
    if(cs >= 0) {
        csPort       = (PORTreg_t)portOutputRegister(digitalPinToPort(cs));
        csPinMaskSet = digitalPinToBitMask(cs);
    } else {
        // No chip-select line defined; might be permanently tied to GND.
        // Assign a valid GPIO register (though not used for CS), and an
        // empty pin bitmask...the nonsense bit-twiddling might be faster
        // than checking _cs and possibly branching.
        csPort       = dcPort;
        csPinMaskSet = 0;
    }
    if(miso >= 0) {
        swspi.misoPort   =(PORTreg_t)portInputRegister(digitalPinToPort(miso));
        swspi.misoPinMask=digitalPinToBitMask(miso);
    } else {
        swspi.misoPort   =(PORTreg_t)portInputRegister(digitalPinToPort(dc));
        swspi.misoPinMask=0;
    }
    csPinMaskClr         = ~csPinMaskSet;
    dcPinMaskClr         = ~dcPinMaskSet;
    swspi.sckPinMaskClr  = ~swspi.sckPinMaskSet;
    swspi.mosiPinMaskClr = ~swspi.mosiPinMaskSet;
 #endif // !end HAS_PORT_SET_CLR
#endif // end USE_FAST_PINIO
}

/*!
    @brief   Adafruit_SPITFT constructor for hardware SPI using the board's
             default SPI peripheral.
    @param   w     Display width in pixels at default rotation setting (0).
    @param   h     Display height in pixels at default rotation setting (0).
    @param   cs    Arduino pin # for chip-select (-1 if unused, tie CS low).
    @param   dc    Arduino pin # for data/command select (required).
    @param   rst   Arduino pin # for display reset (optional, display reset
                   can be tied to MCU reset, default of -1 means unused).
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized; application typically will
             need to call subclass' begin() function, which in turn calls
             this library's initSPI() function to initialize pins.
*/
#if defined(ESP8266) // See notes below
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h, int8_t cs,
  int8_t dc, int8_t rst) : Adafruit_GFX(w, h),
  connection(TFT_HARD_SPI), _rst(rst), _cs(cs), _dc(dc) {
    hwspi._spi = &SPI;
}
#else  // !ESP8266
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h, int8_t cs,
  int8_t dc, int8_t rst) : Adafruit_SPITFT(w, h, &SPI, cs, dc, rst) {
    // This just invokes the hardware SPI constructor below,
    // passing the default SPI device (&SPI).
}
#endif // end !ESP8266

#if !defined(ESP8266)
// ESP8266 compiler freaks out at this constructor -- it can't disambiguate
// beteween the SPIClass pointer (argument #3) and a regular integer.
// Solution here it to just not offer this variant on the ESP8266. You can
// use the default hardware SPI peripheral, or you can use software SPI,
// but if there's any library out there that creates a 'virtual' SPIClass
// peripheral and drives it with software bitbanging, that's not supported.
/*!
    @brief   Adafruit_SPITFT constructor for hardware SPI using a specific
             SPI peripheral.
    @param   w         Display width in pixels at default rotation (0).
    @param   h         Display height in pixels at default rotation (0).
    @param   spiClass  Pointer to SPIClass type (e.g. &SPI or &SPI1).
    @param   cs        Arduino pin # for chip-select (-1 if unused, tie CS low).
    @param   dc        Arduino pin # for data/command select (required).
    @param   rst       Arduino pin # for display reset (optional, display reset
                       can be tied to MCU reset, default of -1 means unused).
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized; application typically will
             need to call subclass' begin() function, which in turn calls
             this library's initSPI() function to initialize pins.
*/
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h, SPIClass *spiClass,
  int8_t cs, int8_t dc, int8_t rst) : Adafruit_GFX(w, h),
  connection(TFT_HARD_SPI), _rst(rst), _cs(cs), _dc(dc) {
    hwspi._spi = spiClass;
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(CORE_TEENSY)
   #if !defined(KINETISK)
    dcPinMask     = digitalPinToBitMask(dc);
   #endif
    dcPortSet     = portSetRegister(dc);
    dcPortClr     = portClearRegister(dc);
    if(cs >= 0) {
   #if !defined(KINETISK)
        csPinMask = digitalPinToBitMask(cs);
   #endif
        csPortSet = portSetRegister(cs);
        csPortClr = portClearRegister(cs);
    } else { // see comments below
   #if !defined(KINETISK)
        csPinMask = 0;
   #endif
        csPortSet = dcPortSet;
        csPortClr = dcPortClr;
    }
  #else  // !CORE_TEENSY
    dcPinMask     = digitalPinToBitMask(dc);
    dcPortSet     = &(PORT->Group[g_APinDescription[dc].ulPort].OUTSET.reg);
    dcPortClr     = &(PORT->Group[g_APinDescription[dc].ulPort].OUTCLR.reg);
    if(cs >= 0) {
        csPinMask = digitalPinToBitMask(cs);
        csPortSet = &(PORT->Group[g_APinDescription[cs].ulPort].OUTSET.reg);
        csPortClr = &(PORT->Group[g_APinDescription[cs].ulPort].OUTCLR.reg);
    } else {
        // No chip-select line defined; might be permanently tied to GND.
        // Assign a valid GPIO register (though not used for CS), and an
        // empty pin bitmask...the nonsense bit-twiddling might be faster
        // than checking _cs and possibly branching.
        csPortSet = dcPortSet;
        csPortClr = dcPortClr;
        csPinMask = 0;
    }
  #endif // end !CORE_TEENSY
 #else  // !HAS_PORT_SET_CLR
    dcPort           = (PORTreg_t)portOutputRegister(digitalPinToPort(dc));
    dcPinMaskSet     = digitalPinToBitMask(dc);
    if(cs >= 0) {
        csPort       = (PORTreg_t)portOutputRegister(digitalPinToPort(cs));
        csPinMaskSet = digitalPinToBitMask(cs);
    } else {
        // No chip-select line defined; might be permanently tied to GND.
        // Assign a valid GPIO register (though not used for CS), and an
        // empty pin bitmask...the nonsense bit-twiddling might be faster
        // than checking _cs and possibly branching.
        csPort       = dcPort;
        csPinMaskSet = 0;
    }
    csPinMaskClr = ~csPinMaskSet;
    dcPinMaskClr = ~dcPinMaskSet;
 #endif // end !HAS_PORT_SET_CLR
#endif // end USE_FAST_PINIO
}
#endif // end !ESP8266

/*!
    @brief   Adafruit_SPITFT constructor for parallel display connection.
    @param   w         Display width in pixels at default rotation (0).
    @param   h         Display height in pixels at default rotation (0).
    @param   busWidth  If tft16 (enumeration in header file), is a 16-bit
                       parallel connection, else 8-bit.
                       16-bit isn't fully implemented or tested yet so
                       applications should pass "tft8" for now...needed to
                       stick a required enum argument in there to
                       disambiguate this constructor from the soft-SPI case.
                       Argument is ignored on 8-bit architectures (no 'wide'
                       support there since PORTs are 8 bits anyway).
    @param   d0        Arduino pin # for data bit 0 (1+ are extrapolated).
                       The 8 (or 16) data bits MUST be contiguous and byte-
                       aligned (or word-aligned for wide interface) within
                       the same PORT register (might not correspond to
                       Arduino pin sequence).
    @param   wr        Arduino pin # for write strobe (required).
    @param   dc        Arduino pin # for data/command select (required).
    @param   cs        Arduino pin # for chip-select (optional, -1 if unused,
                       tie CS low).
    @param   rst       Arduino pin # for display reset (optional, display reset
                       can be tied to MCU reset, default of -1 means unused).
    @param   rd        Arduino pin # for read strobe (optional, -1 if unused).
    @return  Adafruit_SPITFT object.
    @note    Output pins are not initialized; application typically will need
             to call subclass' begin() function, which in turn calls this
             library's initSPI() function to initialize pins.
             Yes, the name is a misnomer...this library originally handled
             only SPI displays, parallel being a recent addition (but not
             wanting to break existing code).
*/
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h, tftBusWidth busWidth,
  int8_t d0, int8_t wr, int8_t dc, int8_t cs, int8_t rst, int8_t rd) :
  Adafruit_GFX(w, h), connection(TFT_PARALLEL), _rst(rst), _cs(cs), _dc(dc) {
    tft8._d0  = d0;
    tft8._wr  = wr;
    tft8._rd  = rd;
    tft8.wide = (busWidth == tft16);
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(CORE_TEENSY)
    tft8.wrPortSet = portSetRegister(wr);
    tft8.wrPortClr = portClearRegister(wr);
   #if !defined(KINETISK)
    dcPinMask      = digitalPinToBitMask(dc);
   #endif
    dcPortSet      = portSetRegister(dc);
    dcPortClr      = portClearRegister(dc);
    if(cs >= 0) {
   #if !defined(KINETISK)
        csPinMask = digitalPinToBitMask(cs);
   #endif
        csPortSet = portSetRegister(cs);
        csPortClr = portClearRegister(cs);
    } else { // see comments below
   #if !defined(KINETISK)
        csPinMask = 0;
   #endif
        csPortSet = dcPortSet;
        csPortClr = dcPortClr;
    }
    if(rd >= 0) { // if read-strobe pin specified...
   #if defined(KINETISK)
        tft8.rdPinMask = 1;
   #else  // !KINETISK
        tft8.rdPinMask = digitalPinToBitMask(rd);
   #endif
        tft8.rdPortSet = portSetRegister(rd);
        tft8.rdPortClr = portClearRegister(rd);
    } else {
        tft8.rdPinMask = 0;
        tft8.rdPortSet = dcPortSet;
        tft8.rdPortClr = dcPortClr;
    }
    // These are all uint8_t* pointers -- elsewhere they're recast
    // as necessary if a 'wide' 16-bit interface is in use.
    tft8.writePort = portOutputRegister(d0);
    tft8.readPort  = portInputRegister(d0);
    tft8.dirSet    = portModeRegister(d0);
    tft8.dirClr    = portModeRegister(d0);
  #else  // !CORE_TEENSY
    tft8.wrPinMask = digitalPinToBitMask(wr);
    tft8.wrPortSet = &(PORT->Group[g_APinDescription[wr].ulPort].OUTSET.reg);
    tft8.wrPortClr = &(PORT->Group[g_APinDescription[wr].ulPort].OUTCLR.reg);
    dcPinMask      = digitalPinToBitMask(dc);
    dcPortSet      = &(PORT->Group[g_APinDescription[dc].ulPort].OUTSET.reg);
    dcPortClr      = &(PORT->Group[g_APinDescription[dc].ulPort].OUTCLR.reg);
    if(cs >= 0) {
        csPinMask = digitalPinToBitMask(cs);
        csPortSet = &(PORT->Group[g_APinDescription[cs].ulPort].OUTSET.reg);
        csPortClr = &(PORT->Group[g_APinDescription[cs].ulPort].OUTCLR.reg);
    } else {
        // No chip-select line defined; might be permanently tied to GND.
        // Assign a valid GPIO register (though not used for CS), and an
        // empty pin bitmask...the nonsense bit-twiddling might be faster
        // than checking _cs and possibly branching.
        csPortSet = dcPortSet;
        csPortClr = dcPortClr;
        csPinMask = 0;
    }
    if(rd >= 0) { // if read-strobe pin specified...
        tft8.rdPinMask =digitalPinToBitMask(rd);
        tft8.rdPortSet =&(PORT->Group[g_APinDescription[rd].ulPort].OUTSET.reg);
        tft8.rdPortClr =&(PORT->Group[g_APinDescription[rd].ulPort].OUTCLR.reg);
    } else {
        tft8.rdPinMask = 0;
        tft8.rdPortSet = dcPortSet;
        tft8.rdPortClr = dcPortClr;
    }
    // Get pointers to PORT write/read/dir bytes within 32-bit PORT
    uint8_t       dBit    = g_APinDescription[d0].ulPin; // d0 bit # in PORT
    PortGroup    *p       = (&(PORT->Group[g_APinDescription[d0].ulPort]));
    uint8_t       offset  = dBit / 8; // d[7:0] byte # within PORT
    if(tft8.wide) offset &= ~1; // d[15:8] byte # within PORT
    // These are all uint8_t* pointers -- elsewhere they're recast
    // as necessary if a 'wide' 16-bit interface is in use.
    tft8.writePort    = (volatile uint8_t *)&(p->OUT.reg)    + offset;
    tft8.readPort     = (volatile uint8_t *)&(p->IN.reg)     + offset;
    tft8.dirSet       = (volatile uint8_t *)&(p->DIRSET.reg) + offset;
    tft8.dirClr       = (volatile uint8_t *)&(p->DIRCLR.reg) + offset;
  #endif // end !CORE_TEENSY
 #else  // !HAS_PORT_SET_CLR
    tft8.wrPort       = (PORTreg_t)portOutputRegister(digitalPinToPort(wr));
    tft8.wrPinMaskSet = digitalPinToBitMask(wr);
    dcPort            = (PORTreg_t)portOutputRegister(digitalPinToPort(dc));
    dcPinMaskSet      = digitalPinToBitMask(dc);
    if(cs >= 0) {
        csPort        = (PORTreg_t)portOutputRegister(digitalPinToPort(cs));
        csPinMaskSet  = digitalPinToBitMask(cs);
    } else {
        // No chip-select line defined; might be permanently tied to GND.
        // Assign a valid GPIO register (though not used for CS), and an
        // empty pin bitmask...the nonsense bit-twiddling might be faster
        // than checking _cs and possibly branching.
        csPort        = dcPort;
        csPinMaskSet  = 0;
    }
    if(rd >= 0) { // if read-strobe pin specified...
        tft8.rdPort       =(PORTreg_t)portOutputRegister(digitalPinToPort(rd));
        tft8.rdPinMaskSet =digitalPinToBitMask(rd);
    } else {
        tft8.rdPort       = dcPort;
        tft8.rdPinMaskSet = 0;
    }
    csPinMaskClr      = ~csPinMaskSet;
    dcPinMaskClr      = ~dcPinMaskSet;
    tft8.wrPinMaskClr = ~tft8.wrPinMaskSet;
    tft8.rdPinMaskClr = ~tft8.rdPinMaskSet;
    tft8.writePort    = (PORTreg_t)portOutputRegister(digitalPinToPort(d0));
    tft8.readPort     = (PORTreg_t)portInputRegister(digitalPinToPort(d0));
    tft8.portDir      = (PORTreg_t)portModeRegister(digitalPinToPort(d0));
 #endif // end !HAS_PORT_SET_CLR
#endif // end USE_FAST_PINIO
}

// end constructors -------


// CLASS MEMBER FUNCTIONS --------------------------------------------------

// begin() and setAddrWindow() MUST be declared by any subclass.

/*!
    @brief  Configure microcontroller pins for TFT interfacing. Typically
            called by a subclass' begin() function.
    @param  freq  SPI frequency when using hardware SPI. If default (0)
                  is passed, will fall back on a device-specific value.
                  Value is ignored when using software SPI or parallel
                  connection.
    @note   Another anachronistically-named function; this is called even
            when the display connection is parallel (not SPI). Also, this
            could probably be made private...quite a few class functions
            were generously put in the public section.
*/
void Adafruit_SPITFT::initSPI(uint32_t freq) {

    if(!freq) freq = DEFAULT_SPI_FREQ; // If no freq specified, use default

    // Init basic control pins common to all connection types
    if(_cs >= 0) {
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH); // Deselect
    }
    pinMode(_dc, OUTPUT);
    digitalWrite(_dc, HIGH); // Data mode

    switch(connection) {
      case TFT_HARD_SPI:
#if defined(SPI_HAS_TRANSACTION)
        hwspi.settings = SPISettings(freq, MSBFIRST, SPI_MODE0);
#else
        hwspi._freq    = freq; // Save freq value for later
#endif
        hwspi._spi->begin();
        break;
      case TFT_SOFT_SPI:
        pinMode(swspi._mosi, OUTPUT);
        digitalWrite(swspi._mosi, LOW);
        pinMode(swspi._sck, OUTPUT);
        digitalWrite(swspi._sck, LOW);
        if(swspi._miso >= 0) {
            pinMode(swspi._miso, INPUT);
        }
        break;
      case TFT_PARALLEL:
        // Initialize data pins.  We were only passed d0, so scan
        // the pin description list looking for the other pins.
        // They'll be on the same PORT, and within the next 7 (or 15) bits
        // (because we need to write to a contiguous PORT byte or word).
#if defined(__AVR__)
        // PORT registers are 8 bits wide, so just need a register match...
        for(uint8_t i=0; i<NUM_DIGITAL_PINS; i++) {
            if((PORTreg_t)portOutputRegister(digitalPinToPort(i)) ==
              tft8.writePort) {
                pinMode(i, OUTPUT);
                digitalWrite(i, LOW);
            }
        }
#elif defined(USE_FAST_PINIO)
 #if defined(CORE_TEENSY)
        if(!tft8.wide) {
            *tft8.dirSet    = 0xFF; // Set port to output
            *tft8.writePort = 0x00; // Write all 0s
        } else {
            *(volatile uint16_t *)tft8.dirSet    = 0xFFFF;
            *(volatile uint16_t *)tft8.writePort = 0x0000;
        }
 #else  // !CORE_TEENSY
        uint8_t portNum = g_APinDescription[tft8._d0].ulPort, // d0 PORT #
                dBit    = g_APinDescription[tft8._d0].ulPin,  // d0 bit in PORT
                lastBit = dBit + (tft8.wide ? 15 : 7);
        for(uint8_t i=0; i<PINS_COUNT; i++) {
            if((g_APinDescription[i].ulPort == portNum ) &&
               (g_APinDescription[i].ulPin  >= dBit    ) &&
               (g_APinDescription[i].ulPin  <= (uint32_t)lastBit)) {
                pinMode(i, OUTPUT);
                digitalWrite(i, LOW);
            }
        }
 #endif // end !CORE_TEENSY
#endif
        pinMode(tft8._wr, OUTPUT);
        digitalWrite(tft8._wr, HIGH);
        if(tft8._rd >= 0) {
            pinMode(tft8._rd, OUTPUT);
            digitalWrite(tft8._rd, HIGH);
        }
        break;
    }

    if(_rst >= 0) {
        // Toggle _rst low to reset
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, HIGH);
        delay(100);
        digitalWrite(_rst, LOW);
        delay(100);
        digitalWrite(_rst, HIGH);
        delay(200);
    }

#if defined(USE_SPI_DMA)

    // INITIALIZE DMA

    if(dma.allocate() == DMA_STATUS_OK) { // Allocate channel
        // The DMA library needs to allocate at least one valid descriptor,
        // so we do that here. It's not used in the usual sense though,
        // just before a transfer we copy descriptor[0] to this address.
        if(dptr = dma.addDescriptor(NULL, NULL, 42, DMA_BEAT_SIZE_BYTE,
          false, false)) {
            // Allocate 2 scanlines worth of pixels on display's major axis,
            // whichever that is, rounding each up to 2-pixel boundary.
            int  major = (WIDTH > HEIGHT) ? WIDTH : HEIGHT;
            major     += (major & 1); // -> next 2-pixel bound, if needed.
            maxFillLen = major * 2;   // 2 scanlines
            // Note to future self: if you decide to make the pixel buffer
            // much larger, remember that DMA transfer descriptors can't
            // exceed 65,535 bytes (not 65,536), meaning 32,767 pixels tops.
            // Not that we have that kind of RAM to throw around right now.
            if((pixelBuf[0] =
              (uint16_t *)malloc(maxFillLen * sizeof(uint16_t)))) {
                // Alloc OK. Get pointer to start of second scanline.
                pixelBuf[1] = &pixelBuf[0][major];
                // Determine number of DMA descriptors needed to cover
                // entire screen when entire 2-line pixelBuf is used
                // (round up for fractional last descriptor).
                int numDescriptors = (WIDTH * HEIGHT + (maxFillLen - 1)) /
                      maxFillLen;
                // DMA descriptors MUST be 128-bit (16 byte) aligned.
                // memalign() is considered 'obsolete' but it's replacements
                // (aligned_alloc() or posix_memalign()) are not currently
                // available in the version of ARM GCC in use, but this is,
                // so here we are.
                if((descriptor = (DmacDescriptor *)memalign(16,
                  numDescriptors * sizeof(DmacDescriptor)))) {
                    int                dmac_id;
                    volatile uint32_t *data_reg;

                    // THIS IS AN AFFRONT TO NATURE, but I don't know
                    // any "clean" way to get the sercom number from the
                    // SPIClass pointer (e.g. &SPI or &SPI1), which is
                    // all we have to work with. SPIClass does contain
                    // a SERCOM pointer but it is a PRIVATE member!
                    // Doing an UNSPEAKABLY HORRIBLE THING here, directly
                    // accessing the first 32-bit value in the SPIClass
                    // structure, knowing that's (currently) where the
                    // SERCOM pointer lives, but this ENTIRELY DEPENDS
                    // on that structure not changing nor the compiler
                    // rearranging things. Oh the humanity!

                    if(*(SERCOM **)hwspi._spi == &sercom0) {
                        dmac_id  = SERCOM0_DMAC_ID_TX;
                        data_reg = &SERCOM0->SPI.DATA.reg;
#if defined SERCOM1
                    } else if(*(SERCOM **)hwspi._spi == &sercom1) {
                        dmac_id  = SERCOM1_DMAC_ID_TX;
                        data_reg = &SERCOM1->SPI.DATA.reg;
#endif
#if defined SERCOM2
                    } else if(*(SERCOM **)hwspi._spi == &sercom2) {
                        dmac_id  = SERCOM2_DMAC_ID_TX;
                        data_reg = &SERCOM2->SPI.DATA.reg;
#endif
#if defined SERCOM3
                    } else if(*(SERCOM **)hwspi._spi == &sercom3) {
                        dmac_id  = SERCOM3_DMAC_ID_TX;
                        data_reg = &SERCOM3->SPI.DATA.reg;
#endif
#if defined SERCOM4
                    } else if(*(SERCOM **)hwspi._spi == &sercom4) {
                        dmac_id  = SERCOM4_DMAC_ID_TX;
                        data_reg = &SERCOM4->SPI.DATA.reg;
#endif
#if defined SERCOM5
                    } else if(*(SERCOM **)hwspi._spi == &sercom5) {
                        dmac_id  = SERCOM5_DMAC_ID_TX;
                        data_reg = &SERCOM5->SPI.DATA.reg;
#endif
                    }

                    dma.setPriority(DMA_PRIORITY_3);
                    dma.setTrigger(dmac_id);
                    dma.setAction(DMA_TRIGGER_ACTON_BEAT);

                    // Initialize descriptor list.
                    for(int d=0; d<numDescriptors; d++) {
                        // No need to set SRCADDR, DESCADDR or BTCNT --
                        // those are done in the pixel-writing functions.
                        descriptor[d].BTCTRL.bit.VALID    = true;
                        descriptor[d].BTCTRL.bit.EVOSEL   =
                          DMA_EVENT_OUTPUT_DISABLE;
                        descriptor[d].BTCTRL.bit.BLOCKACT =
                           DMA_BLOCK_ACTION_NOACT;
                        descriptor[d].BTCTRL.bit.BEATSIZE = DMA_BEAT_SIZE_BYTE;
                        descriptor[d].BTCTRL.bit.DSTINC   = 0;
                        descriptor[d].BTCTRL.bit.STEPSEL  = DMA_STEPSEL_SRC;
                        descriptor[d].BTCTRL.bit.STEPSIZE =
                          DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
                        descriptor[d].DSTADDR.reg         = (uint32_t)data_reg;
                    }
                    lastFillColor = 0x0000;
                    lastFillLen   = 0;
                    dma.setCallback(dma_callback);
                    return; // Success!
                }
                // Else some alloc/init error along the way...clean up...
                free(pixelBuf[0]);
                pixelBuf[0] = pixelBuf[1] = NULL;
            }
            // Don't currently have a descriptor delete function in
            // ZeroDMA lib, but if we did, it would be called here.
        }
        dma.free(); // Deallocate DMA channel
    }
#endif // end DMA init
}

/*!
    @brief  Call before issuing command(s) or data to display. Performs
            chip-select (if required) and starts an SPI transaction (if
            using hardware SPI and transactions are supported). Required
            for all display types; not an SPI-specific function.
*/
void Adafruit_SPITFT::startWrite(void) {
    if(_cs >= 0) SPI_CS_LOW();
    SPI_BEGIN_TRANSACTION();
}

/*!
    @brief  Call after issuing command(s) or data to display. Performs
            chip-deselect (if required) and ends an SPI transaction (if
            using hardware SPI and transactions are supported). Required
            for all display types; not an SPI-specific function.
*/
void Adafruit_SPITFT::endWrite(void) {
    SPI_END_TRANSACTION();
    if(_cs >= 0) SPI_CS_HIGH();
}


// -------------------------------------------------------------------------
// Lower-level graphics operations. These functions require a chip-select
// and/or SPI transaction around them (via startWrite(), endWrite() above).
// Higher-level graphics primitives might start a single transaction and
// then make multiple calls to these functions (e.g. circle or text
// rendering might make repeated lines or rects) before ending the
// transaction. It's more efficient than starting a transaction every time.

/*!
    @brief  Draw a single pixel to the display at requested coordinates.
            Not self-contained; should follow a startWrite() call.
    @param  x      Horizontal position (0 = left).
    @param  y      Vertical position   (0 = top).
    @param  color  16-bit pixel color in '565' RGB format.
*/
void Adafruit_SPITFT::writePixel(int16_t x, int16_t y, uint16_t color) {
    if((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
        setAddrWindow(x, y, 1, 1);
        SPI_WRITE16(color);
    }
}

/*!
    @brief  Issue a series of pixels from memory to the display. Not self-
            contained; should follow startWrite() and setAddrWindow() calls.
    @param  colors  Pointer to array of 16-bit pixel values in '565' RGB
                    format.
    @param  len     Number of elements in 'colors' array.
*/
void Adafruit_SPITFT::writePixels(uint16_t *colors, uint32_t len) {

    if(!len) return; // Avoid 0-byte transfers

#if defined(ESP32) // ESP32 has a special SPI pixel-writing function...
    if(connection == TFT_HARD_SPI) {
        hwspi._spi->writePixels(colors, len * 2);
        return;
    }
#elif defined(USE_SPI_DMA)
    if(connection == TFT_HARD_SPI) {
        int     maxSpan     = maxFillLen / 2; // One scanline max
        uint8_t pixelBufIdx = 0;              // Active pixel buffer number
        while(len) {
            int count = (len < maxSpan) ? len : maxSpan;

            // Because TFT and SAMD endianisms are different, must swap bytes
            // from the 'colors' array passed into a DMA working buffer. This
            // can take place while the prior DMA transfer is in progress,
            // hence the need for two pixelBufs.
            for(int i=0; i<count; i++) {
                pixelBuf[pixelBufIdx][i] = __builtin_bswap16(*colors++);
            }
            // The transfers themselves are relatively small, so we don't
            // need a long descriptor list. We just alternate between the
            // first two, sharing pixelBufIdx for that purpose.
            descriptor[pixelBufIdx].SRCADDR.reg       =
              (uint32_t)pixelBuf[pixelBufIdx] + count * 2;
            descriptor[pixelBufIdx].BTCTRL.bit.SRCINC = 1;
            descriptor[pixelBufIdx].BTCNT.reg         = count * 2;
            descriptor[pixelBufIdx].DESCADDR.reg      = 0;

            while(dma_busy); // wait for prior line to complete

            // Move new descriptor into place...
            memcpy(dptr, &descriptor[pixelBufIdx], sizeof(DmacDescriptor));
            dma_busy = true;
            dma.startJob();                // Trigger SPI DMA transfer
            pixelBufIdx = 1 - pixelBufIdx; // Swap DMA pixel buffers

            len -= count;
        }
        lastFillColor = 0x0000; // pixelBuf has been sullied
        lastFillLen   = 0;
        while(dma_busy);        // Wait for last line to complete
#if defined(__SAMD51__)
        hwspi._spi->setDataMode(SPI_MODE0); // See SAMD51 note in writeColor()
#endif
        return;
    }
#endif // end USE_SPI_DMA

    // All other cases (non-DMA hard SPI, bitbang SPI, parallel),
    // use a loop with the normal 16-bit data write function:
    while(len--) {
        SPI_WRITE16(*colors++);
    }
}

/*!
    @brief  Issue a series of pixels, all the same color. Not self-
            contained; should follow startWrite() and setAddrWindow() calls.
    @param  color  16-bit pixel color in '565' RGB format.
    @param  len    Number of pixels to draw.
*/
void Adafruit_SPITFT::writeColor(uint16_t color, uint32_t len) {

    if(!len) return; // Avoid 0-byte transfers

    uint8_t hi = color >> 8, lo = color;

#if defined(ESP32) // ESP32 has a special SPI pixel-writing function...
    if(connection == TFT_HARD_SPI) {
        #define SPI_MAX_PIXELS_AT_ONCE 32
        #define TMPBUF_LONGWORDS       (SPI_MAX_PIXELS_AT_ONCE + 1) / 2
        #define TMPBUF_PIXELS          (TMPBUF_LONGWORDS * 2)
        static uint32_t temp[TMPBUF_LONGWORDS];
        uint32_t        c32    = color * 0x00010001;
        uint16_t        bufLen = (len < TMPBUF_PIXELS) ? len : TMPBUF_PIXELS,
                        xferLen, fillLen;
        // Fill temp buffer 32 bits at a time
        fillLen = (bufLen + 1) / 2; // Round up to next 32-bit boundary
        for(uint32_t t=0; t<fillLen; t++) {
            temp[t] = c32;
        }
        // Issue pixels in blocks from temp buffer
        while(len) {                                 // While pixels remain
            xferLen = (bufLen < len) ? bufLen : len; // How many this pass?
            writePixels((uint16_t *)temp, xferLen);
            len -= xferLen;
        }
        return;
    }
#else  // !ESP32
 #if defined(USE_SPI_DMA)
    if(connection == TFT_HARD_SPI) {
        int i, d, numDescriptors;
        if(hi == lo) { // If high & low bytes are same...
            onePixelBuf = color;
            // Can do this with a relatively short descriptor list,
            // each transferring a max of 32,767 (not 32,768) pixels.
            // This won't run off the end of the allocated descriptor list,
            // since we're using much larger chunks per descriptor here.
            numDescriptors = (len + 32766) / 32767;
            for(d=0; d<numDescriptors; d++) {
                int count = (len < 32767) ? len : 32767;
                descriptor[d].SRCADDR.reg       = (uint32_t)&onePixelBuf;
                descriptor[d].BTCTRL.bit.SRCINC = 0;
                descriptor[d].BTCNT.reg         = count * 2;
                descriptor[d].DESCADDR.reg      = (uint32_t)&descriptor[d+1];
                len -= count;
            }
            descriptor[d-1].DESCADDR.reg        = 0;
        } else {
            // If high and low bytes are distinct, it's necessary to fill
            // a buffer with pixel data (swapping high and low bytes because
            // TFT and SAMD are different endianisms) and create a longer
            // descriptor list pointing repeatedly to this data. We can do
            // this slightly faster working 2 pixels (32 bits) at a time.
            uint32_t *pixelPtr  = (uint32_t *)pixelBuf[0],
                      twoPixels = __builtin_bswap16(color) * 0x00010001;
            // We can avoid some or all of the buffer-filling if the color
            // is the same as last time...
            if(color == lastFillColor) {
                // If length is longer than prior instance, fill only the
                // additional pixels in the buffer and update lastFillLen.
                if(len > lastFillLen) {
                    int fillStart = lastFillLen / 2,
                        fillEnd   = (((len < maxFillLen) ?
                                       len : maxFillLen) + 1) / 2;
                    for(i=fillStart; i<fillEnd; i++) pixelPtr[i] = twoPixels;
                    lastFillLen = fillEnd * 2;
                } // else do nothing, don't set pixels or change lastFillLen
            } else {
                int fillEnd = (((len < maxFillLen) ?
                                 len : maxFillLen) + 1) / 2;
                for(i=0; i<fillEnd; i++) pixelPtr[i] = twoPixels;
                lastFillLen   = fillEnd * 2;
                lastFillColor = color;
            }

            numDescriptors = (len + maxFillLen - 1) / maxFillLen;
            for(d=0; d<numDescriptors; d++) {
                int pixels = (len < maxFillLen) ? len : maxFillLen,
                    bytes  = pixels * 2;
                descriptor[d].SRCADDR.reg       = (uint32_t)pixelPtr + bytes;
                descriptor[d].BTCTRL.bit.SRCINC = 1;
                descriptor[d].BTCNT.reg         = bytes;
                descriptor[d].DESCADDR.reg      = (uint32_t)&descriptor[d+1];
                len -= pixels;
            }
            descriptor[d-1].DESCADDR.reg        = 0;
        }
        memcpy(dptr, &descriptor[0], sizeof(DmacDescriptor));

        dma_busy = true;
        dma.startJob();
        while(dma_busy); // Wait for completion
        // Unfortunately blocking is necessary. An earlier version returned
        // immediately and checked dma_busy on startWrite() instead, but it
        // turns out to be MUCH slower on many graphics operations (as when
        // drawing lines, pixel-by-pixel), perhaps because it's a volatile
        // type and doesn't cache. Working on this.
  #if defined(__SAMD51__)
        // SAMD51: SPI DMA seems to leave the SPI peripheral in a freaky
        // state on completion. Workaround is to explicitly set it back...
        hwspi._spi->setDataMode(SPI_MODE0);
  #endif
        return;
    }
 #endif // end USE_SPI_DMA
#endif // end !ESP32

    // All other cases (non-DMA hard SPI, bitbang SPI, parallel)...

    switch(connection) {
      case TFT_HARD_SPI:
#if defined(ESP8266)
        do {
            uint32_t pixelsThisPass = len;
            if(pixelsThisPass > 50000) pixelsThisPass = 50000;
            len -= pixelsThisPass;
            yield(); // Periodic yield() on long fills
            while(pixelsThisPass--) {
                hwspi._spi->write(hi);
                hwspi._spi->write(lo);
            }
        } while(len);
#else  // !ESP8266
        while(len--) {
 #if defined(__AVR__)
            for(SPDR = hi; !(SPSR & _BV(SPIF)); );
            for(SPDR = lo; !(SPSR & _BV(SPIF)); );
 #elif defined(ESP32)
            hwspi._spi->write(hi);
            hwspi._spi->write(lo);
 #else
            hwspi._spi->transfer(hi);
            hwspi._spi->transfer(lo);
 #endif
        }
#endif // end !ESP8266
        break;
      case TFT_SOFT_SPI:
#if defined(ESP8266)
        do {
            uint32_t pixelsThisPass = len;
            if(pixelsThisPass > 20000) pixelsThisPass = 20000;
            len -= pixelsThisPass;
            yield(); // Periodic yield() on long fills
            while(pixelsThisPass--) {
                for(uint16_t bit=0, x=color; bit<16; bit++) {
                    if(x & 0x8000) SPI_MOSI_HIGH();
                    else           SPI_MOSI_LOW();
                    SPI_SCK_HIGH();
                    SPI_SCK_LOW();
                    x <<= 1;
                }
            }
        } while(len);
#else  // !ESP8266
        while(len--) {
 #if defined(__AVR__)
            for(uint8_t bit=0, x=hi; bit<8; bit++) {
                if(x & 0x80) SPI_MOSI_HIGH();
                else         SPI_MOSI_LOW();
                SPI_SCK_HIGH();
                SPI_SCK_LOW();
                x <<= 1;
            }
            for(uint8_t bit=0, x=lo; bit<8; bit++) {
                if(x & 0x80) SPI_MOSI_HIGH();
                else         SPI_MOSI_LOW();
                SPI_SCK_HIGH();
                SPI_SCK_LOW();
                x <<= 1;
            }
 #else  // !__AVR__
            for(uint16_t bit=0, x=color; bit<16; bit++) {
                if(x & 0x8000) SPI_MOSI_HIGH();
                else           SPI_MOSI_LOW();
                SPI_SCK_HIGH();
                x <<= 1;
                SPI_SCK_LOW();
            }
 #endif // end !__AVR__
        }
#endif // end !ESP8266
        break;
      case TFT_PARALLEL:
        if(hi == lo) {
#if defined(__AVR__)
            len            *= 2;
            *tft8.writePort = hi;
            while(len--) {
                TFT_WR_STROBE();
            }
#elif defined(USE_FAST_PINIO)
            if(!tft8.wide) {
                len            *= 2;
                *tft8.writePort = hi;
            } else {
                *(volatile uint16_t *)tft8.writePort = color;
            }
            while(len--) {
                TFT_WR_STROBE();
            }
#endif
        } else {
            while(len--) {
#if defined(__AVR__)
                *tft8.writePort = hi;
                TFT_WR_STROBE();
                *tft8.writePort = lo;
#elif defined(USE_FAST_PINIO)
                if(!tft8.wide) {
                    *tft8.writePort = hi;
                    TFT_WR_STROBE();
                    *tft8.writePort = lo;
                } else {
                    *(volatile uint16_t *)tft8.writePort = color;
                }
#endif
                TFT_WR_STROBE();
            }
        }
        break;
    }
}

/*!
    @brief  Draw a filled rectangle to the display. Not self-contained;
            should follow startWrite(). Typically used by higher-level
            graphics primitives; user code shouldn't need to call this and
            is likely to use the self-contained fillRect() instead.
            writeFillRect() performs its own edge clipping and rejection;
            see writeFillRectPreclipped() for a more 'raw' implementation.
    @param  x      Horizontal position of first corner.
    @param  y      Vertical position of first corner.
    @param  w      Rectangle width in pixels (positive = right of first
                   corner, negative = left of first corner).
    @param  h      Rectangle height in pixels (positive = below first
                   corner, negative = above first corner).
    @param  color  16-bit fill color in '565' RGB format.
    @note   Written in this deep-nested way because C by definition will
            optimize for the 'if' case, not the 'else' -- avoids branches
            and rejects clipped rectangles at the least-work possibility.
*/
void Adafruit_SPITFT::writeFillRect(int16_t x, int16_t y,
  int16_t w, int16_t h, uint16_t color) {
    if(w && h) {                            // Nonzero width and height?
        if(w < 0) {                         // If negative width...
            x +=  w + 1;                    //   Move X to left edge
            w  = -w;                        //   Use positive width
        }
        if(x < _width) {                    // Not off right
            if(h < 0) {                     // If negative height...
                y +=  h + 1;                //   Move Y to top edge
                h  = -h;                    //   Use positive height
            }
            if(y < _height) {               // Not off bottom
                int16_t x2 = x + w - 1;
                if(x2 >= 0) {               // Not off left
                    int16_t y2 = y + h - 1;
                    if(y2 >= 0) {           // Not off top
                        // Rectangle partly or fully overlaps screen
                        if(x  <  0)       { x = 0; w = x2 + 1; } // Clip left
                        if(y  <  0)       { y = 0; h = y2 + 1; } // Clip top
                        if(x2 >= _width)  { w = _width  - x;   } // Clip right
                        if(y2 >= _height) { h = _height - y;   } // Clip bottom
                        writeFillRectPreclipped(x, y, w, h, color);
                    }
                }
            }
        }
    }
}

/*!
    @brief  Draw a horizontal line on the display. Performs edge clipping
            and rejection. Not self-contained; should follow startWrite().
            Typically used by higher-level graphics primitives; user code
            shouldn't need to call this and is likely to use the self-
            contained drawFastHLine() instead.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  w      Line width in pixels (positive = right of first point,
                   negative = point of first corner).
    @param  color  16-bit line color in '565' RGB format.
*/
void inline Adafruit_SPITFT::writeFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {
    if((y >= 0) && (y < _height) && w) { // Y on screen, nonzero width
        if(w < 0) {                      // If negative width...
            x +=  w + 1;                 //   Move X to left edge
            w  = -w;                     //   Use positive width
        }
        if(x < _width) {                 // Not off right
            int16_t x2 = x + w - 1;
            if(x2 >= 0) {                // Not off left
                // Line partly or fully overlaps screen
                if(x  <  0)       { x = 0; w = x2 + 1; } // Clip left
                if(x2 >= _width)  { w = _width  - x;   } // Clip right
                writeFillRectPreclipped(x, y, w, 1, color);
            }
        }
    }
}

/*!
    @brief  Draw a vertical line on the display. Performs edge clipping and
            rejection. Not self-contained; should follow startWrite().
            Typically used by higher-level graphics primitives; user code
            shouldn't need to call this and is likely to use the self-
            contained drawFastVLine() instead.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  h      Line height in pixels (positive = below first point,
                   negative = above first point).
    @param  color  16-bit line color in '565' RGB format.
*/
void inline Adafruit_SPITFT::writeFastVLine(int16_t x, int16_t y, int16_t h,
  uint16_t color) {
    if((x >= 0) && (x < _width) && h) { // X on screen, nonzero height
        if(h < 0) {                     // If negative height...
            y +=  h + 1;                //   Move Y to top edge
            h  = -h;                    //   Use positive height
        }
        if(y < _height) {               // Not off bottom
            int16_t y2 = y + h - 1;
            if(y2 >= 0) {               // Not off top
                // Line partly or fully overlaps screen
                if(y  <  0)       { y = 0; h = y2 + 1; } // Clip top
                if(y2 >= _height) { h = _height - y;   } // Clip bottom
                writeFillRectPreclipped(x, y, 1, h, color);
            }
        }
    }
}

/*!
    @brief  A lower-level version of writeFillRect(). This version requires
            all inputs are in-bounds, that width and height are positive,
            and no part extends offscreen. NO EDGE CLIPPING OR REJECTION IS
            PERFORMED. If higher-level graphics primitives are written to
            handle their own clipping earlier in the drawing process, this
            can avoid unnecessary function calls and repeated clipping
            operations in the lower-level functions.
    @param  x      Horizontal position of first corner. MUST BE WITHIN
                   SCREEN BOUNDS.
    @param  y      Vertical position of first corner. MUST BE WITHIN SCREEN
                   BOUNDS.
    @param  w      Rectangle width in pixels. MUST BE POSITIVE AND NOT
                   EXTEND OFF SCREEN.
    @param  h      Rectangle height in pixels. MUST BE POSITIVE AND NOT
                   EXTEND OFF SCREEN.
    @param  color  16-bit fill color in '565' RGB format.
    @note   This is a new function, no graphics primitives besides rects
            and horizontal/vertical lines are written to best use this yet.
*/
inline void Adafruit_SPITFT::writeFillRectPreclipped(int16_t x, int16_t y,
  int16_t w, int16_t h, uint16_t color) {
    setAddrWindow(x, y, w, h);
    writeColor(color, (uint32_t)w * h);
}


// -------------------------------------------------------------------------
// Ever-so-slightly higher-level graphics operations. Similar to the 'write'
// functions above, but these contain their own chip-select and SPI
// transactions as needed (via startWrite(), endWrite()). They're typically
// used solo -- as graphics primitives in themselves, not invoked by higher-
// level primitives (which should use the functions above for better
// performance).

/*!
    @brief  Draw a single pixel to the display at requested coordinates.
            Self-contained and provides its own transaction as needed
            (see writePixel(x,y,color) for a lower-level variant).
            Edge clipping is performed here.
    @param  x      Horizontal position (0 = left).
    @param  y      Vertical position   (0 = top).
    @param  color  16-bit pixel color in '565' RGB format.
*/
void Adafruit_SPITFT::drawPixel(int16_t x, int16_t y, uint16_t color) {
    // Clip first...
    if((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
        // THEN set up transaction (if needed) and draw...
        startWrite();
        setAddrWindow(x, y, 1, 1);
        SPI_WRITE16(color);
        endWrite();
    }
}

/*!
    @brief  Draw a filled rectangle to the display. Self-contained and
            provides its own transaction as needed (see writeFillRect() or
            writeFillRectPreclipped() for lower-level variants). Edge
            clipping and rejection is performed here.
    @param  x      Horizontal position of first corner.
    @param  y      Vertical position of first corner.
    @param  w      Rectangle width in pixels (positive = right of first
                   corner, negative = left of first corner).
    @param  h      Rectangle height in pixels (positive = below first
                   corner, negative = above first corner).
    @param  color  16-bit fill color in '565' RGB format.
    @note   This repeats the writeFillRect() function almost in its entirety,
            with the addition of a transaction start/end. It's done this way
            (rather than starting the transaction and calling writeFillRect()
            to handle clipping and so forth) so that the transaction isn't
            performed at all if the rectangle is rejected. It's really not
            that much code.
*/
void Adafruit_SPITFT::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {
    if(w && h) {                            // Nonzero width and height?
        if(w < 0) {                         // If negative width...
            x +=  w + 1;                    //   Move X to left edge
            w  = -w;                        //   Use positive width
        }
        if(x < _width) {                    // Not off right
            if(h < 0) {                     // If negative height...
                y +=  h + 1;                //   Move Y to top edge
                h  = -h;                    //   Use positive height
            }
            if(y < _height) {               // Not off bottom
                int16_t x2 = x + w - 1;
                if(x2 >= 0) {               // Not off left
                    int16_t y2 = y + h - 1;
                    if(y2 >= 0) {           // Not off top
                        // Rectangle partly or fully overlaps screen
                        if(x  <  0)       { x = 0; w = x2 + 1; } // Clip left
                        if(y  <  0)       { y = 0; h = y2 + 1; } // Clip top
                        if(x2 >= _width)  { w = _width  - x;   } // Clip right
                        if(y2 >= _height) { h = _height - y;   } // Clip bottom
                        startWrite();
                        writeFillRectPreclipped(x, y, w, h, color);
                        endWrite();
                    }
                }
            }
        }
    }
}

/*!
    @brief  Draw a horizontal line on the display. Self-contained and
            provides its own transaction as needed (see writeFastHLine() for
            a lower-level variant). Edge clipping and rejection is performed
            here.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  w      Line width in pixels (positive = right of first point,
                   negative = point of first corner).
    @param  color  16-bit line color in '565' RGB format.
    @note   This repeats the writeFastHLine() function almost in its
            entirety, with the addition of a transaction start/end. It's
            done this way (rather than starting the transaction and calling
            writeFastHLine() to handle clipping and so forth) so that the
            transaction isn't performed at all if the line is rejected.
*/
void Adafruit_SPITFT::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {
    if((y >= 0) && (y < _height) && w) { // Y on screen, nonzero width
        if(w < 0) {                      // If negative width...
            x +=  w + 1;                 //   Move X to left edge
            w  = -w;                     //   Use positive width
        }
        if(x < _width) {                 // Not off right
            int16_t x2 = x + w - 1;
            if(x2 >= 0) {                // Not off left
                // Line partly or fully overlaps screen
                if(x  <  0)       { x = 0; w = x2 + 1; } // Clip left
                if(x2 >= _width)  { w = _width  - x;   } // Clip right
                startWrite();
                writeFillRectPreclipped(x, y, w, 1, color);
                endWrite();
            }
        }
    }
}

/*!
    @brief  Draw a vertical line on the display. Self-contained and provides
            its own transaction as needed (see writeFastHLine() for a lower-
            level variant). Edge clipping and rejection is performed here.
    @param  x      Horizontal position of first point.
    @param  y      Vertical position of first point.
    @param  h      Line height in pixels (positive = below first point,
                   negative = above first point).
    @param  color  16-bit line color in '565' RGB format.
    @note   This repeats the writeFastVLine() function almost in its
            entirety, with the addition of a transaction start/end. It's
            done this way (rather than starting the transaction and calling
            writeFastVLine() to handle clipping and so forth) so that the
            transaction isn't performed at all if the line is rejected.
*/
void Adafruit_SPITFT::drawFastVLine(int16_t x, int16_t y, int16_t h,
  uint16_t color) {
    if((x >= 0) && (x < _width) && h) { // X on screen, nonzero height
        if(h < 0) {                     // If negative height...
            y +=  h + 1;                //   Move Y to top edge
            h  = -h;                    //   Use positive height
        }
        if(y < _height) {               // Not off bottom
            int16_t y2 = y + h - 1;
            if(y2 >= 0) {               // Not off top
                // Line partly or fully overlaps screen
                if(y  <  0)       { y = 0; h = y2 + 1; } // Clip top
                if(y2 >= _height) { h = _height - y;   } // Clip bottom
                startWrite();
                writeFillRectPreclipped(x, y, 1, h, color);
                endWrite();
            }
        }
    }
}

/*!
    @brief  Essentially writePixel() with a transaction around it. I don't
            think this is in use by any of our code anymore (believe it was
            for some older BMP-reading examples), but is kept here in case
            any user code relies on it. Consider it DEPRECATED.
    @param  color  16-bit pixel color in '565' RGB format.
*/
void Adafruit_SPITFT::pushColor(uint16_t color) {
    startWrite();
    SPI_WRITE16(color);
    endWrite();
}

/*!
    @brief  Draw a 16-bit image (565 RGB) at the specified (x,y) position.
            For 16-bit display devices; no color reduction performed.
            Adapted from https://github.com/PaulStoffregen/ILI9341_t3
            by Marc MERLIN. See examples/pictureEmbed to use this.
            5/6/2017: function name and arguments have changed for
            compatibility with current GFX library and to avoid naming
            problems in prior implementation.  Formerly drawBitmap() with
            arguments in different order. Handles its own transaction and
            edge clipping/rejection.
    @param  x        Top left corner horizontal coordinate.
    @param  y        Top left corner vertical coordinate.
    @param  pcolors  Pointer to 16-bit array of pixel values.
    @param  w        Width of bitmap in pixels.
    @param  h        Height of bitmap in pixels.
*/
void Adafruit_SPITFT::drawRGBBitmap(int16_t x, int16_t y,
  uint16_t *pcolors, int16_t w, int16_t h) {

    int16_t x2, y2; // Lower-right coord
    if(( x             >= _width ) ||      // Off-edge right
       ( y             >= _height) ||      // " top
       ((x2 = (x+w-1)) <  0      ) ||      // " left
       ((y2 = (y+h-1)) <  0)     ) return; // " bottom

    int16_t bx1=0, by1=0, // Clipped top-left within bitmap
            saveW=w;      // Save original bitmap width value
    if(x < 0) { // Clip left
        w  +=  x;
        bx1 = -x;
        x   =  0;
    }
    if(y < 0) { // Clip top
        h  +=  y;
        by1 = -y;
        y   =  0;
    }
    if(x2 >= _width ) w = _width  - x; // Clip right
    if(y2 >= _height) h = _height - y; // Clip bottom

    pcolors += by1 * saveW + bx1; // Offset bitmap ptr to clipped top-left
    startWrite();
    setAddrWindow(x, y, w, h); // Clipped area
    while(h--) { // For each (clipped) scanline...
      writePixels(pcolors, w); // Push one (clipped) row
      pcolors += saveW; // Advance pointer by one full (unclipped) line
    }
    endWrite();
}


// -------------------------------------------------------------------------
// Miscellaneous class member functions that don't draw anything.

/*!
    @brief  Invert the colors of the display (if supported by hardware).
            Self-contained, no transaction setup required.
    @param  i  true = inverted display, false = normal display.
*/
void Adafruit_SPITFT::invertDisplay(bool i) {
    startWrite();
    writeCommand(i ? invertOnCommand : invertOffCommand);
    endWrite();
}

/*!
    @brief   Given 8-bit red, green and blue values, return a 'packed'
             16-bit color value in '565' RGB format (5 bits red, 6 bits
             green, 5 bits blue). This is just a mathematical operation,
             no hardware is touched.
    @param   red    8-bit red brightnesss (0 = off, 255 = max).
    @param   green  8-bit green brightnesss (0 = off, 255 = max).
    @param   blue   8-bit blue brightnesss (0 = off, 255 = max).
    @return  'Packed' 16-bit color value (565 format).
*/
uint16_t Adafruit_SPITFT::color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}


// -------------------------------------------------------------------------
// Lowest-level hardware-interfacing functions. Many of these are inline and
// compile to different things based on #defines -- typically just a few
// instructions. Others, not so much, those are not inlined.

/*!
    @brief  Start an SPI transaction if using the hardware SPI interface to
            the display. If using an earlier version of the Arduino platform
            (before the addition of SPI transactions), this instead attempts
            to set up the SPI clock and mode. No action is taken if the
            connection is not hardware SPI-based. This does NOT include a
            chip-select operation -- see startWrite() for a function that
            encapsulated both actions.
*/
inline void Adafruit_SPITFT::SPI_BEGIN_TRANSACTION(void) {
    if(connection == TFT_HARD_SPI) {
#if defined(SPI_HAS_TRANSACTION)
        hwspi._spi->beginTransaction(hwspi.settings);
#else // No transactions, configure SPI manually...
 #if defined(__AVR__) || defined(TEENSYDUINO) || defined(ARDUINO_ARCH_STM32F1)
        hwspi._spi->setClockDivider(SPI_CLOCK_DIV2);
 #elif defined(__arm__)
        hwspi._spi->setClockDivider(11);
 #elif defined(ESP8266) || defined(ESP32)
        hwspi._spi->setFrequency(hwspi._freq);
 #elif defined(RASPI) || defined(ARDUINO_ARCH_STM32F1)
        hwspi._spi->setClock(hwspi._freq);
 #endif
        hwspi._spi->setBitOrder(MSBFIRST);
        hwspi._spi->setDataMode(SPI_MODE0);
#endif // end !SPI_HAS_TRANSACTION
    }
}

/*!
    @brief  End an SPI transaction if using the hardware SPI interface to
            the display. No action is taken if the connection is not
            hardware SPI-based or if using an earlier version of the Arduino
            platform (before the addition of SPI transactions). This does
            NOT include a chip-deselect operation -- see endWrite() for a
            function that encapsulated both actions.
*/
inline void Adafruit_SPITFT::SPI_END_TRANSACTION(void) {
#if defined(SPI_HAS_TRANSACTION)
    if(connection == TFT_HARD_SPI) {
        hwspi._spi->endTransaction();
    }
#endif
}

/*!
    @brief  Issue a single 8-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the byte. This is another of
            those functions in the library with a now-not-accurate name
            that's being maintained for compatibility with outside code.
            This function is used even if display connection is parallel.
    @param  b  8-bit value to write.
*/
void Adafruit_SPITFT::spiWrite(uint8_t b) {
    switch(connection) {
      case TFT_HARD_SPI:
#if defined(__AVR__)
        for(SPDR = b; !(SPSR & _BV(SPIF)); );
#elif defined(ESP8266) || defined(ESP32)
        hwspi._spi->write(b);
#else
        hwspi._spi->transfer(b);
#endif
        break;
      case TFT_SOFT_SPI:
        for(uint8_t bit=0; bit<8; bit++) {
            if(b & 0x80) SPI_MOSI_HIGH();
            else         SPI_MOSI_LOW();
            SPI_SCK_HIGH();
            b <<= 1;
            SPI_SCK_LOW();
        }
        break;
      case TFT_PARALLEL:
#if defined(__AVR__)
        *tft8.writePort = b;
#elif defined(USE_FAST_PINIO)
        if(!tft8.wide) *tft8.writePort = b;
        else           *(volatile uint16_t *)tft8.writePort = b;
#endif
        TFT_WR_STROBE();
        break;
    }
}

/*!
    @brief  Write a single command byte to the display. Chip-select and
            transaction must have been previously set -- this ONLY sets
            the device to COMMAND mode, issues the byte and then restores
            DATA mode. There is no corresponding explicit writeData()
            function -- just use spiWrite().
    @param  cmd  8-bit command to write.
*/
void Adafruit_SPITFT::writeCommand(uint8_t cmd) {
    SPI_DC_LOW();
    spiWrite(cmd);
    SPI_DC_HIGH();
}

/*!
    @brief   Read a single 8-bit value from the display. Chip-select and
             transaction must have been previously set -- this ONLY reads
             the byte. This is another of those functions in the library
             with a now-not-accurate name that's being maintained for
             compatibility with outside code. This function is used even if
             display connection is parallel.
    @return  Unsigned 8-bit value read (always zero if USE_FAST_PINIO is
             not supported by the MCU architecture).
*/
uint8_t Adafruit_SPITFT::spiRead(void) {
    uint8_t  b = 0;
    uint16_t w = 0;
    switch(connection) {
      case TFT_HARD_SPI:
        return hwspi._spi->transfer((uint8_t)0);
      case TFT_SOFT_SPI:
        if(swspi._miso >= 0) {
            for(uint8_t i=0; i<8; i++) {
                SPI_SCK_HIGH();
                b <<= 1;
                if(SPI_MISO_READ()) b++;
                SPI_SCK_LOW();
            }
        }
        return b;
      // case TFT_PARALLEL:
      default: // Avoids compiler warning about no return value
        if(tft8._rd >= 0) {
#if defined(USE_FAST_PINIO)
            TFT_RD_LOW();                       // Read line LOW
 #if defined(__AVR__)
            *tft8.portDir = 0x00;               // Set port to input state
            w             = *tft8.readPort;     // Read value from port
            *tft8.portDir = 0xFF;               // Restore port to output
 #else // !__AVR__
            if(!tft8.wide) { // 8-bit TFT connection
  #if defined(HAS_PORT_SET_CLR)
                *tft8.dirClr  = 0xFF;           // Set port to input state
                w             = *tft8.readPort; // Read value from port
                *tft8.dirSet  = 0xFF;           // Restore port to output
  #else  // !HAS_PORT_SET_CLR
                *tft8.portDir = 0x00;           // Set port to input state
                w             = *tft8.readPort; // Read value from port
                *tft8.dirSet  = 0xFF;           // Restore port to output
  #endif // end HAS_PORT_SET_CLR
            } else { // 16-bit TFT connection
  #if defined(HAS_PORT_SET_CLR)
                *(volatile uint16_t *)tft8.dirClr  = 0xFFFF; // Input state
                w = *(volatile uint16_t *)tft8.readPort;     // 16-bit read
                *(volatile uint16_t *)tft8.dirSet  = 0xFFFF; // Output state
  #else  // !HAS_PORT_SET_CLR
                *(volatile uint16_t *)tft8.portDir = 0x0000; // Input state
                w = *(volatile uint16_t *)tft8.readPort;     // 16-bit read
                *(volatile uint16_t *)tft8.portDir = 0xFFFF; // Output state
  #endif // end !HAS_PORT_SET_CLR
            }
            TFT_RD_HIGH();                      // Read line HIGH
 #endif // end !__AVR__
#else  // !USE_FAST_PINIO
            w = 0; // Parallel TFT is NOT SUPPORTED without USE_FAST_PINIO
#endif // end !USE_FAST_PINIO
        }
        return w;
    }
}

/*!
    @brief  Set the software (bitbang) SPI MOSI line HIGH.
*/
inline void Adafruit_SPITFT::SPI_MOSI_HIGH(void) {
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(KINETISK)
    *swspi.mosiPortSet = 1;
  #else  // !KINETISK
    *swspi.mosiPortSet = swspi.mosiPinMask;
  #endif
 #else  // !HAS_PORT_SET_CLR
    *swspi.mosiPort   |= swspi.mosiPinMaskSet;
 #endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
    digitalWrite(swspi._mosi, HIGH);
 #if defined(ESP32)
    for(volatile uint8_t i=0; i<1; i++);
 #endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the software (bitbang) SPI MOSI line LOW.
*/
inline void Adafruit_SPITFT::SPI_MOSI_LOW(void) {
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(KINETISK)
    *swspi.mosiPortClr = 1;
  #else  // !KINETISK
    *swspi.mosiPortClr = swspi.mosiPinMask;
  #endif
 #else  // !HAS_PORT_SET_CLR
    *swspi.mosiPort   &= swspi.mosiPinMaskClr;
 #endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
    digitalWrite(swspi._mosi, LOW);
 #if defined(ESP32)
    for(volatile uint8_t i=0; i<1; i++);
 #endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the software (bitbang) SPI SCK line HIGH.
*/
inline void Adafruit_SPITFT::SPI_SCK_HIGH(void) {
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(KINETISK)
    *swspi.sckPortSet = 1;
  #else  // !KINETISK
    *swspi.sckPortSet = swspi.sckPinMask;
  #endif
 #else  // !HAS_PORT_SET_CLR
    *swspi.sckPort   |= swspi.sckPinMaskSet;
 #endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
    digitalWrite(swspi._sck, HIGH);
 #if defined(ESP32)
    for(volatile uint8_t i=0; i<1; i++);
 #endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the software (bitbang) SPI SCK line LOW.
*/
inline void Adafruit_SPITFT::SPI_SCK_LOW(void) {
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(KINETISK)
    *swspi.sckPortClr = 1;
  #else  // !KINETISK
    *swspi.sckPortClr = swspi.sckPinMask;
  #endif
 #else  // !HAS_PORT_SET_CLR
    *swspi.sckPort   &= swspi.sckPinMaskClr;
 #endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
    digitalWrite(swspi._sck, LOW);
 #if defined(ESP32)
    for(volatile uint8_t i=0; i<1; i++);
 #endif // end ESP32
#endif // end !USE_FAST_PINIO
}

/*!
    @brief   Read the state of the software (bitbang) SPI MISO line.
    @return  true if HIGH, false if LOW.
*/
inline bool Adafruit_SPITFT::SPI_MISO_READ(void) {
#if defined(USE_FAST_PINIO)
 #if defined(KINETISK)
    return *swspi.misoPort;
 #else  // !KINETISK
    return *swspi.misoPort & swspi.misoPinMask;
 #endif // end !KINETISK
#else  // !USE_FAST_PINIO
    return digitalRead(swspi._miso);
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Issue a single 16-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the word. Despite the name,
            this function is used even if display connection is parallel;
            name was maintaned for backward compatibility. Naming is also
            not consistent with the 8-bit version, spiWrite(). Sorry about
            that. Again, staying compatible with outside code.
    @param  w  16-bit value to write.
*/
void Adafruit_SPITFT::SPI_WRITE16(uint16_t w) {
    switch(connection) {
      case TFT_HARD_SPI:
#if defined(__AVR__)
        for(SPDR = (w >> 8); (!(SPSR & _BV(SPIF))); );
        for(SPDR =  w      ; (!(SPSR & _BV(SPIF))); );
#elif defined(ESP8266) || defined(ESP32)
        hwspi._spi->write16(w);
#else
        hwspi._spi->transfer(w >> 8);
        hwspi._spi->transfer(w);
#endif
        break;
      case TFT_SOFT_SPI:
        for(uint8_t bit=0; bit<16; bit++) {
            if(w & 0x8000) SPI_MOSI_HIGH();
            else           SPI_MOSI_LOW();
            SPI_SCK_HIGH();
            SPI_SCK_LOW();
            w <<= 1;
        }
        break;
      case TFT_PARALLEL:
#if defined(__AVR__)
        *tft8.writePort = w >> 8;
        TFT_WR_STROBE();
        *tft8.writePort = w;
#elif defined(USE_FAST_PINIO)
        if(!tft8.wide) {
            *tft8.writePort = w >> 8;
            TFT_WR_STROBE();
            *tft8.writePort = w;
        } else {
            *(volatile uint16_t *)tft8.writePort = w;
        }
#endif
        TFT_WR_STROBE();
        break;
    }
}

/*!
    @brief  Issue a single 32-bit value to the display. Chip-select,
            transaction and data/command selection must have been
            previously set -- this ONLY issues the longword. Despite the
            name, this function is used even if display connection is
            parallel; name was maintaned for backward compatibility. Naming
            is also not consistent with the 8-bit version, spiWrite().
            Sorry about that. Again, staying compatible with outside code.
    @param  l  32-bit value to write.
*/
void Adafruit_SPITFT::SPI_WRITE32(uint32_t l) {
    switch(connection) {
      case TFT_HARD_SPI:
#if defined(__AVR__)
        for(SPDR = (l >> 24); !(SPSR & _BV(SPIF)); );
        for(SPDR = (l >> 16); !(SPSR & _BV(SPIF)); );
        for(SPDR = (l >>  8); !(SPSR & _BV(SPIF)); );
        for(SPDR =  l       ; !(SPSR & _BV(SPIF)); );
#elif defined(ESP8266) || defined(ESP32)
        hwspi._spi->write32(l);
#else
        hwspi._spi->transfer(l >> 24);
        hwspi._spi->transfer(l >> 16);
        hwspi._spi->transfer(l >> 8);
        hwspi._spi->transfer(l);
#endif
        break;
      case TFT_SOFT_SPI:
        for(uint8_t bit=0; bit<32; bit++) {
            if(l & 0x80000000) SPI_MOSI_HIGH();
            else               SPI_MOSI_LOW();
            SPI_SCK_HIGH();
            SPI_SCK_LOW();
            l <<= 1;
        }
        break;
      case TFT_PARALLEL:
#if defined(__AVR__)
        *tft8.writePort = l >> 24;
        TFT_WR_STROBE();
        *tft8.writePort = l >> 16;
        TFT_WR_STROBE();
        *tft8.writePort = l >> 8;
        TFT_WR_STROBE();
        *tft8.writePort = l;
#elif defined(USE_FAST_PINIO)
        if(!tft8.wide) {
            *tft8.writePort = l >> 24;
            TFT_WR_STROBE();
            *tft8.writePort = l >> 16;
            TFT_WR_STROBE();
            *tft8.writePort = l >> 8;
            TFT_WR_STROBE();
            *tft8.writePort = l;
        } else {
            *(volatile uint16_t *)tft8.writePort = l >> 16;
            TFT_WR_STROBE();
            *(volatile uint16_t *)tft8.writePort = l;
        }
#endif
        TFT_WR_STROBE();
        break;
    }
}

/*!
    @brief  Set the RD line LOW, then HIGH. Used for parallel-connected
            interfaces when writing data.
*/
inline void Adafruit_SPITFT::TFT_WR_STROBE(void) {
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
  #if defined(KINETISK)
    *tft8.wrPortClr = 1;
    *tft8.wrPortSet = 1;
  #else  // !KINETISK
    *tft8.wrPortClr = tft8.wrPinMask;
    *tft8.wrPortSet = tft8.wrPinMask;
  #endif // end !KINETISK
 #else  // !HAS_PORT_SET_CLR
    *tft8.wrPort   &= tft8.wrPinMaskClr;
    *tft8.wrPort   |= tft8.wrPinMaskSet;
 #endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
    digitalWrite(tft8._wr, LOW);
    digitalWrite(tft8._wr, HIGH);
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the RD line HIGH. Used for parallel-connected interfaces
            when reading data.
*/
inline void Adafruit_SPITFT::TFT_RD_HIGH(void) {
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
    *tft8.rdPortSet = tft8.rdPinMask;
 #else  // !HAS_PORT_SET_CLR
    *tft8.rdPort   |= tft8.rdPinMaskSet;
 #endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
    digitalWrite(tft8._rd, HIGH);
#endif // end !USE_FAST_PINIO
}

/*!
    @brief  Set the RD line LOW. Used for parallel-connected interfaces
            when reading data.
*/
inline void Adafruit_SPITFT::TFT_RD_LOW(void) {
#if defined(USE_FAST_PINIO)
 #if defined(HAS_PORT_SET_CLR)
    *tft8.rdPortClr = tft8.rdPinMask;
 #else  // !HAS_PORT_SET_CLR
    *tft8.rdPort   &= tft8.rdPinMaskClr;
 #endif // end !HAS_PORT_SET_CLR
#else  // !USE_FAST_PINIO
    digitalWrite(tft8._rd, LOW);
#endif // end !USE_FAST_PINIO
}

#endif // end __AVR_ATtiny85__
