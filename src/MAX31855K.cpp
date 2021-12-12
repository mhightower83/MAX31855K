/*
 *   Copyright 2020 M Hightower
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

#include <Arduino.h>
#include <interrupts.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include "SPIEx.h"
SPIExClass SPIEx;

static ALWAYS_INLINE
void fastDigitalWrite(uint8_t pin, uint8_t val) {
    // Same as core's __digitalWrite, but w/o the Waveform stops
    // We will call the original digitalWrite at the start of SPI to get those things done.
    //
    // stopWaveform(pin); // Disable any Tone or startWaveform on this pin
    // _stopPWM(pin);     // and any analogWrites (PWM)
    if (pin < 16) {
        uint32_t valMask = (1u << pin);
        if (val) {
            GPOS = valMask;
            GPOS = valMask;
        } else {
            GPOC = valMask;
            GPOC = valMask;
        }
    } else if (pin == 16) {
        if (val) {
            GP16O |= 1;
            GP16O |= 1;
        } else {
            GP16O &= ~1;
            GP16O &= ~1;
        }
    }
}

static ALWAYS_INLINE
uint32_t fastDigitalRead(uint8_t pin) {
    // An inline variation of core's __digitaRead.
    uint32_t valOut = 0;
    if (pin < 16){
        // #define ESP8266_REG(addr) *((volatile uint32_t *)(0x60000000+(addr)))
        // #define GPI    ESP8266_REG(0x318) //GPIO_IN RO (Read Input Level)
        // #define GPIP(p) ((GPI & (1 << ((p) & 0xF))) != 0)
        valOut = GPI >> pin;
    } else if(pin == 16){
        valOut = GP16I;
    }
    asm volatile("excw\n\t");
    // return valOut & 0x01u;
    return valOut;// & 0x01u;
}

#else
#include <SPI.h>
#define SPIEx SPI
static ALWAYS_INLINE
uint32_t fastDigitalRead(uint8_t pin) {
    return (digitalRead(pin)) ? 1 : 0;
}

static ALWAYS_INLINE
void fastDigitalWrite(uint8_t pin, uint8_t val) {
    return digitalWrite(pin, val);
}
#endif
#include <float.h>

// #define DEBUG_SKETCH
// #include <debugHelper.h>
#include "MAX31855K.h"
#include "TypeK_ITS90.h"

#ifndef DEBUGHELPER_H_
// Fail safe defines, for when debugHelper.h is not present
#ifndef CONSOLE_PRINTF
#define DEBUG_PRINTF CONSOLE_PRINTF
#define CONSOLE_PRINTF(a, ...) Serial.printf_P(PSTR(a), ##__VA_ARGS__)
#endif
#ifdef DEBUG_ESP_PORT
#define CONSOLE DEBUG_ESP_PORT
#else
#define CONSOLE Serial
#endif
#endif

///////////////////////////////////////////////////////////////////////////////
// Tuned Delays specific to the ESP8266
//
#if defined(ARDUINO_ARCH_ESP8266)
// The time to execute digitalWrite() exceeds 40ns, 100ns and 200ns timing
// minimums of the MAX31855. No delay functions needed on the ESP8266 between
// digitalWrite() function calls. See `void spiReadTest(void)` for specific timings.
ALWAYS_INLINE void delay100nsMin() {}
ALWAYS_INLINE void delay200nsMin() {}

#else
// These may not be needed as well for other architectures; however, that would
// need to be evaluated. Keep delays until it is otherwise known.
ALWAYS_INLINE void delay200nsMin()
{
    delayMicroseconds(1);
}
ALWAYS_INLINE void delay100nsMin()
{
    delayMicroseconds(1);
}
void testDelay100nsMin(){}

#define IRAM_ATTR
#endif // ARDUINO_ARCH_ESP8266


///////////////////////////////////////////////////////////////////////////////
// SPI Bus - melding Soft and HW based handling
// TODO: This look very localized, maybe make into a separate common library.
//
bool MAX31855K::beginSPI(const SPIPins cfg)
{
    // cs
    // sck
    // miso
    // mosi
    // hw
    // softCs
    _pins = cfg;
    if (_pins.hw) {
        // Try and configure Hardware SPI Bus, returns false if SPI Bus
        // hardware transfers are not supported for the pins chosen.
        _pins.hw = SPIEx.pins(_pins.sck, _pins.miso, _pins.mosi, _pins.cs);
        // Note we can only use SPI Library if SPI.pins return true.
    }
    if (_pins.hw) {
        DEBUG_PRINTF("HW SPI Configured ");
        SPIEx.begin();
    } else {
        DEBUG_PRINTF("Soft SPI Configured");
        digitalWrite(_pins.cs, HIGH);       // deselect
        pinMode(_pins.cs, OUTPUT);
        pinMode(_pins.miso, INPUT_PULLUP); //INPUT);
        digitalWrite(_pins.sck, LOW);       // set inactive clock, starts from low
        pinMode(_pins.sck, OUTPUT);
        delay200nsMin();
        // From here forward we manage all SPI pins through software.
        // We also use fastDigitalWrite to skip over redundant calls stop stop WaveForm etc.
    }
    DEBUG_PRINTF(", Pin Assignment: SCK=%d, MISO=%d, MOSI=%d, CS=%d\r\n",
                  _pins.sck, _pins.miso, _pins.mosi, _pins.cs);
    return read();      // Test pin configuration
}

// Keep timing parts in fast IRAM
NOINLINE IRAM_ATTR static
uint32_t spiRead32(const SPIPins& pins)
{
    union {
      uint32_t u32;
      uint8_t  u8[4];
    } value;

    if (pins.hw) {
#if defined(ARDUINO_ARCH_ESP8266)
        // ESP8266 Hardware SPI notes
        // HW CS# goes to enable state, low, 125ns before SCK's rising edge,
        // sampling MISO. After 32 SCK cycles, SCK begins its falling edge to
        // low 125ns after the last SCK rising edge. Now resting at low. CS#
        // starts its change to the disabled state simultaneously with SCK
        // falling edge. As would be expected with a Clock frequency set to 4MHz.
        // ie. 250ns period.
        // MAX serial clock frequency 5MHz use 4MHz

        SPIEx.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        value.u32 = SPIEx.transfer32(0);
#else
        if (pins.softCs) {
            if (LOW == digitalRead(pins.cs)) {
                // Make CS# inactive for required 200ns to restart process
                fastDigitalWrite(pins.cs, HIGH);
                delay200nsMin();              // minimum CS# inactive time
            }
        } else
        if (SS == pins.cs) {   // GPIO15
            DEBUG_PRINTF("w/HW CS");
            SPIEx.setHwCs(true);
            // From here out we leave all SPI bus pins to the Library to handle.
        }
        // MAX serial clock frequency 5MHz use 4MHz
        SPIEx.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        if (pins.softCs) {
            // Arduino doc says CS is done after SPIEx.beginTransaction!
            digitalWrite(pins.cs, LOW);     // select
            // ESP8266: For softCS first clock rise is ~2us from falling CS# no need for delay function.
            // Due to processor execution time the 100ns delay after CS# is
            // fulfilled by the code that must run to setup the SPI transfer.
            // Hence, these delays are NOOPs for the ESP8266 build.
            delay100nsMin(); // wait time for first bit valid
        }
        for (ssize_t i = 3; i >= 0; i--) {  // MSBFIRST, little-endian
            value.u8[i] = SPIEx.transfer(0);
        }
        if (pins.softCs) {
            // For softCS rising CS# is ~2.6us from falling SCK
            digitalWrite(pins.cs, HIGH); // deselect slave
        }
#endif
        SPIEx.endTransaction();

    } else {
        // This code path is error recovery logic. While it should never happen,
        // just make it right.
        if (LOW == digitalRead(pins.cs) || LOW != digitalRead(pins.sck)) {
            // CS# is in the wrong state, Must be inactive for at least 200ns
            fastDigitalWrite(pins.cs, HIGH);
            // For MAX31855 SPI transfers, SCK must be LOW
            // before CS# is asserted. (Mode 0)
            fastDigitalWrite(pins.sck, LOW);
            delay200nsMin();              // minimum CS# inactive time
        }

        /*
         * The devices involved here have very sharp rise and fall times, which
         * creates ringing and cross-talk. For mode0, MISO is read on the rising
         * edge. Since our GPIO pins are not edge triggered and we are
         * implementing a Soft SPI, we can either sample/read MISO just before
         * or just after the rising edge. If we read immediately after, ringing
         * or cross-talk may still be present in our signal. If we read just
         * before the rising clock, their is time for the ringing to have
         * decayed out from the previous transition. Hence, we will read the
         * state immediate before the rising edge.
         */
        value.u32 = 0;
        {
            // For SPI this disable interrupt should not be needed; however, it
            // is useful during development when evaluating timing results.
            esp8266::InterruptLock lock;

            fastDigitalWrite(pins.cs, LOW);         // CS# enable, 100ns wait time for first bit valid
            // Soft SPI is now ready to read the 1st bit.
            // CS# enable to SCK HIGH 412ns
            for (size_t i = 0; i < 32; i++) {
                delay100nsMin();
                // For HW SPI mode 0, Data is ready on rising edge of SCK
                // For Software SPI, sample the bit, MISO, just before rise.
                uint32_t misoVal = fastDigitalRead(pins.miso);
                // SCK LOW for ??ns on all bits after 1st bit.

                fastDigitalWrite(pins.sck, HIGH);
                delay100nsMin();
                // SCK HIGH duration ??ns
                // At each falling edge of SCK, the MAX31855 updates MISO with new bit
                fastDigitalWrite(pins.sck, LOW);
                // By delaying ORing in the bit read, until SCK goes low, the
                // low state is stretched. This allows more time for MISO data
                // to become valid from the MAX31855 and ringing to settle out
                // before MISO is read.
                value.u32 = (value.u32 << 1) | (1 & misoVal);
            }
            // SCK inactive for SPI mode 0 clock is left LOW!
            fastDigitalWrite(pins.cs, HIGH); // deselect slave
        }
    }
    return value.u32;
}

bool MAX31855K::read() {
    // It is the upper levels responsibility to not call again for 200ns.
    // For the ESP8266 this timing limit is meet through normal code execution time.
    //
    // This should be the natural logic flow. No new converted values will be
    // available for 70-100ms. There is no point in being called sooner!
    // Thus we skip adding a needless hard CPU wasting delay.

    if (_fakeRead) {
        _thermocouple.raw32 = _fakeRead;
        return true;
    }

    _thermocouple.raw32 = spiRead32(_pins);
    if (isValid()) {
        return true;
    }
    _errors++;
    _lastError.raw32 = _thermocouple.raw32;
    return false;
}

//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// MAX31855K Type K Thermocouple library
//
/*
  Overview: We get a die temperature reading from the MAX31855. This is used for
  the Cold-junction temperature in Celsius. That value is changed into mV using
  the ITS90 coefficient based calculation to convert degrees Celsius to an
  equivalent thermocouple mV reading. That value is added to the probe
  thermocouple mV reading. The resulting sum is passed through an ITS90
  coefficient based "inverse" conversion to get the linearized temperature in
  degrees Celsius.

  Useful and instructive info from:
    https://instrumentationtools.com/thermocouple-calculations/
    https://www.ti.com/lit/an/sbaa274/sbaa274.pdf
    https://blog.heypete.com/2016/09/09/max31855-temperature-linearization/

  "thermocouple-calculations" gives a good perspective of how we get the
  hot-thermocouple temperature w/o using an ice bath for the reference end.
  TI's application note SBAA274 adds a lot of design depth to the thermocouple
  discussion.
*/

/*
  The probe temperature (thermocouple) value provided by the MAX31855 is
  compensated for the cold-junction temperature. It also assumes a linear
  response from the non-linear (mostly linear) type K thermocouple.

  To use the ITS90 polynomial to get a linearized voltage reading, we need the
  mV reading of the hot-thermocouple and the mV reading of the mV reading of the
  cold-thermocouple.

  What we get from the MAX31855K is a temperature reading of the chip's die
  (internal) in the MAX31855K in degrees Celsius. This can be used to derive the
  Cold-Junction thermocouple mV output, by using the ITS90 conversion
  coefficients.

  For the Hot-Junction (probe) side of the equation, the MAX31855K gives us an
  approximation of the probe temperature. Based on die temperature plus mV
  reading from the thermocouple (probe) times  a constant based on a straight
  line slope (V/degrees C) for a K thermocouple  calculate from the mV endpoints
  for the range of 0 to 1000 degrees Celsius, 41.276E-06V/degrees C. This value
  can be reversed back to the mV reading by V = (T_C_probe - T_C_internal) *
  41.276E-06V/degrees C
*/

DFLOAT MAX31855K::getITS90(const DFLOAT coldJunctionC, const DFLOAT voutMv) const
{
    /*
      For validating the range of the Cold-junction temperature end. Use the
      MAX31855's operating range of -40 to +125 degrees Celsius. The NIST table
      limits would allowed values far exceeding the MAX31855K operating range.
    */
    if (FLT_MAX != coldJunctionC
    &&  FLT_MAX != voutMv
    &&  isMAX31855InOperationRange(coldJunctionC)) {
        DFLOAT mV = voutMv + type_k_celsius_to_mv(coldJunctionC);
        if (is_type_k_mv_in_range(mV)) {
            return type_k_mv_to_celsius(mV);
        }
    }

    /*
      The request has failed when the input data values do not fall within the
      operating ranges of the hardware or ITS90 table ranges. For cases when
      the range check fails it is probably noise or some other malfunction.
    */
    // I think NAN would need exceptions to handle?
    // Caller must test for FLT_MAX to confirm success.
    return FLT_MAX;
}


/*
  Supports faster integer math when implementing temperature control thresholds
  for turning device on/off. Estimate a probeX10K value for matching a real
  temperature to the uncorrected nonlinear reading from the MAX31855K.

  Wide swings in ambient, Cold Junction, temperature will increase error
  with this method. Influence should be lessened if Hot-junction temperature
  are larger than the Cold Junction temperature. The Cold Junction temperature
  becomes a smaller component in the equation.

  A work-around would be to periodically refresh the threshold calculations.
*/
sint32_t MAX31855K::convertC2ProbeX10K(const DFLOAT hj, const DFLOAT cj) const
{
    if (FLT_MAX == hj || FLT_MAX == cj)
        return INT32_MAX;

    DFLOAT hotJunctionV = milliVolt2Volt(type_k_celsius_to_mv(hj) - type_k_celsius_to_mv(cj));
    DFLOAT coldJunctionV = cj * kTypeKSensitivityVoC;
    return sint32X10K((hotJunctionV + coldJunctionV) / kTypeKSensitivityVoC);
}

/*
  Use reference temperature to calibrate for linearied Temperature
*/
void MAX31855K::setReferenceCalLinearized(const DFLOAT ref, const DFLOAT hj, const DFLOAT cj) {
    setZeroCal(convertC2ProbeX10K(hj, cj) - convertC2ProbeX10K(ref, cj));
}
//
// End - MAX31855K Type K Thermocouple library
///////////////////////////////////////////////////////////////////////////////
