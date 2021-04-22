/*
 * SPI Extended
 *
 * Adds support for transfer32 method.
 * Replacement for transfer16 method. Does SPI transfer16 as a single 16-bit transfer.
 * Replaces SPI Class in local module
 *
 * There are some issues with CS and transfer size:
 *   https://github.com/esp8266/Arduino/pull/6423#issuecomment-523473404
 *   https://github.com/esp8266/Arduino/issues/6417#issuecomment-522224896 - very good desciption of th eissues.
 *   https://github.com/esp8266/Arduino/issues/7926
 *   https://github.com/esp8266/Arduino/issues/2820
 *   https://github.com/esp8266/Arduino/pull/6423 - looks like it was silently closed by author
 *
 *
 * I am just going to do what is needed for this project to succeed and leave
 * the incomplete Arduino SPI API alone!
 *
 * Unrelated SPI issues
 * https://github.com/esp8266/Arduino/issues/5712 - SS issues
 * https://github.com/esp8266/Arduino/issues/5921
 */

#ifndef SPIEX_H_
#define SPIEX_H_
#ifdef _SPI_H_INCLUDED
#error "'SPIClass SPI' global conflict. This SPIEx.h replaces SPI.h previously included in this module."
#endif


#define NO_GLOBAL_SPI
#include <SPI.h>
#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE inline __attribute__ ((always_inline))
#endif

static ALWAYS_INLINE
void __fastDigitalWrite(uint8_t pin, uint8_t val) {
    // Same as core's __digitalWrite, but w/o the Waveform stops
    // We will call the orginal digitalWrite at the start of SPI to get thos things done.
    //
    // stopWaveform(pin); // Disable any Tone or startWaveform on this pin
    // _stopPWM(pin);     // and any analogWrites (PWM)
    if (pin < 16) {
        if (val) GPOS = (1 << pin);
        else GPOC = (1 << pin);
    } else if (pin == 16) {
        if (val) GP16O |= 1;
        else GP16O &= ~1;
    }
}

class SPIExClass : public SPIClass
{
private:
    uint8_t _cs;
      //  This function was not shared, Copied from SPI.cpp,
      inline void setDataBits(uint16_t bits) {
          const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
          bits--;
          SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
      }

public:
    SPIExClass() {
        _cs = 0;
    }

    bool pins(int8_t sck, int8_t miso, int8_t mosi, int8_t ss) {
        _cs = 0;
        bool success = SPIClass::pins(sck, miso, mosi, ss);
        if (success && 0 != ss && 15 != ss) {
            _cs = ss; // Soft CS
        }
        return success;
    }

    uint32_t transfer32(uint32_t data) {
        return transfer32(data, !(SPI1C & (SPICWBO | SPICRBO)));
    }

    uint32_t IRAM_ATTR transfer32(uint32_t data, bool msb) {
        if (msb) {
            // MSBFIRST Byte first
            data = __bswap32(data);
        }

        while(SPI1CMD & SPIBUSY) {}
        // Set to 32Bits transfer
        setDataBits(32);

        if (0 != _cs && LOW == digitalRead(_cs)) {
            // Make CS# inactive for required 200ns to restart process
            digitalWrite(_cs, HIGH);
            // delay200nsMin();              // minimum CS# inactive time
        }
        if (0 != _cs) {
            // Arduino doc says CS is done after SPI.beginTransaction!
            __fastDigitalWrite(_cs, LOW);     // select
            // ESP8266: For softCS first clock rise is ~2us from falling CS# no need for delay function.
            // Due to processor execution time the 100ns delay after CS# is
            // fullfilled by the code that must run to setup the SPI transfer.
            // Hense, these delays are NOOPs for the ESP8266 build.
            // delay100nsMin(); // wait time for first bit valid
        }

        SPI1W0 = data;
        SPI1CMD |= SPIBUSY;
        while(SPI1CMD & SPIBUSY) {}
        data = SPI1W0;

        if (0 != _cs) {
            // For softCS rising CS# is ~2.6us from falling SCK
            __fastDigitalWrite(_cs, HIGH); // deselect slave
        }

        if(msb) {
            data = __bswap32(data);
        }
        return data;
    }
    // uint16_t transfer16(uint16_t data) {
    //     return transfer16(data, !(SPI1C & (SPICWBO | SPICRBO)));
    // }
    //
    // uint16_t transfer16(uint16_t data, bool msb) {
    //     if (msb) {
    //         // MSBFIRST Byte first
    //         data = __bswap16(data);
    //     }
    //
    //     while(SPI1CMD & SPIBUSY) {}
    //     // Set to 32Bits transfer
    //     setDataBits(16);
    //     SPI1W0 = data;
    //     SPI1CMD |= SPIBUSY;
    //     while(SPI1CMD & SPIBUSY) {}
    //     data = (uint16_t)(SPI1W0);
    //     if (msb) {
    //         data = __bswap16(data);
    //     }
    //     return data;
    // }
};

// #if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SPI)
static SPIExClass SPI;
// extern SPIExClass SPIEx;
// #else
#endif
