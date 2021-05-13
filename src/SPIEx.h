/*
 * SPI Extended
 *
 * Adds support for a transfer32 method.
 * Replacement for the transfer16 method. Does SPI transfer16 as a single 16-bit transfer. Not two bytes.
 * SPI Class wrapper to manaage CS
 *
 * This class is intended for use with SPI_PINS_HSPI, normal HSPI mode.
 * Do not use if you need or plan to use SPI_PINS_HSPI_OVERLAP
 * Normal HSPI mode (MISO = GPIO12, MOSI = GPIO13, SCLK = GPIO14);
 * I see no way to support both similtaniously.
 *
 * To support multiple HSPI devices with different CS pins, create a new class
 * instantiation for each CS pin used.
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
 * https://github.com/esp8266/Arduino/discussions/7887#discussioncomment-386274
 */

#ifndef SPIEX_H_
#define SPIEX_H_
#include <SPI.h>
#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE inline __attribute__ ((always_inline))
#endif

static ALWAYS_INLINE
void _fastDigitalWrite(uint8_t pin, uint8_t val) {
    // Same as core's __digitalWrite, but w/o the Waveform stops. We will call
    // the orginal digitalWrite at the start of SPI to get those things done.
    //
    // stopWaveform(pin); // Disable any Tone or startWaveform on this pin
    // _stopPWM(pin);     // and any analogWrites (PWM)
    if (pin < 16) {
        uint32_t valMask = (1u << pin);
        if (val) {
            GPOS = valMask;
        } else {
            GPOC = valMask;
        }
    } else if (pin == 16) {
        if (val) {
            GP16O |= 1;
        } else {
            GP16O &= ~1;
        }
    }
}

class SPIExClass //: public SPIClass
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
    SPIExClass() { _cs = 255u; }
    /*
     * pins acknowleges that the pin assignment specified supports Hardware SPI.
     * Two configurations are available on the ESP8266 for HSPI.  Normal HSPI
     * mode (MISO = GPIO12, MOSI = GPIO13, SCLK = GPIO14) and overlapped HSPI.
     * Overlapped HSPI is not supported by this SPI Extension wrapper Class.
     * On success, this class will always impliment Soft CS support for the
     * specified GPIO value in `ss`.
     */
    bool pins(int8_t sck, int8_t miso, int8_t mosi, int8_t ss) {
        _cs = 255u;
        // Force SoftCS control for all CS pins that might get used for HSPI
        bool success = SPI.pins(sck, miso, mosi, _cs);
        if (success) {
            _cs = ss;
        }
        return success;
        // For now, no support for HSPI overlapped and always do SoftCS.
        // bool success = SPI.pins(sck, miso, mosi, (0u == ss && 6u == sck) ? 0u : 255u );
        // if (success && 6u != sck) {
    }
    void begin() {
        SPI.begin();
        SPI.setHwCs(false);   // We always do SoftCS
    }
    void setHwCs(bool use) { (void)use; }
    void end() { SPI.end(); }
    void beginTransaction(SPISettings settings) {
        if (255u != _cs) {
            digitalWrite(_cs, HIGH);
            pinMode(_cs, OUTPUT);
            // delay200nsMin();              // minimum CS# inactive time
        }
        SPI.beginTransaction(settings); // SPISettings(4000000, MSBFIRST, SPI_MODE0));
    }
    void endTransaction(void) { SPI.endTransaction(); }

    uint32_t transfer32(uint32_t data) {
        return transfer32(data, !(SPI1C & (SPICWBO | SPICRBO)));
    }
    uint32_t IRAM_ATTR transfer32(uint32_t data, bool msb) {
        if (msb) {
            // MSBFIRST Byte first
            data = __bswap32(data);
        }

        while(SPI1CMD & SPIBUSY) {}
        // Set to 32-Bits transfer
        setDataBits(32);
        if (255u != _cs) {
            // Arduino doc says CS is done after SPI.beginTransaction!
            _fastDigitalWrite(_cs, LOW);     // select
            // TODO: ESP8266: Needs retesting for 80/160Mhz CPU clock when using _fastDigitalWrite().
            //?? Due to processor execution time the 100ns delay after CS# is
            //?? fullfilled by the code that must run to setup the SPI transfer.
            //?? Hense, these delays are NOOPs for the ESP8266 build.
            //?? delay100nsMin(); // wait time for first bit valid
        }
        SPI1W0 = data;
        SPI1CMD |= SPIBUSY;
        while(SPI1CMD & SPIBUSY) {}
        data = SPI1W0;
        if (255u != _cs) {
            // For softCS rising CS# is ____us from falling SCK
            _fastDigitalWrite(_cs, HIGH); // deselect slave
        }
        if(msb) {
            data = __bswap32(data);
        }
        return data;
    }

    uint16_t transfer16(uint16_t data) {
        return transfer16(data, !(SPI1C & (SPICWBO | SPICRBO)));
    }
    uint16_t transfer16(uint16_t data, bool msb) {
        if (msb) {
            // MSBFIRST Byte first
            data = __bswap16(data);
        }

        while(SPI1CMD & SPIBUSY) {}
        // Set to 16-Bits transfer
        setDataBits(16);
        if (255u != _cs) {
            _fastDigitalWrite(_cs, LOW);     // select
        }
        SPI1W0 = data;
        SPI1CMD |= SPIBUSY;
        while(SPI1CMD & SPIBUSY) {}
        data = (uint16_t)(SPI1W0);
        if (255u != _cs) {
            _fastDigitalWrite(_cs, HIGH); // deselect slave
        }
        if (msb) {
            data = __bswap16(data);
        }
        return data;
    }
};

#endif
