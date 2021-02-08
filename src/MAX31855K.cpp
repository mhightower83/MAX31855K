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
#include <SPI.h>
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
#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE inline __attribute__ ((always_inline))
#endif
#ifndef NOINLINE
#define NOINLINE __attribute__((noinline))
#endif
#ifdef DEBUG_ESP_PORT
#define CONSOLE DEBUG_ESP_PORT
#else
#define CONSOLE Serial
#endif
#endif

//////////////////////////////////////////////////////////////////////////////

extern "C" uint32_t ets_get_cpu_frequency(void);

///////////////////////////////////////////////////////////////////////////////
// Tuned Delays specific to the ESP8266
//
#if defined(ARDUINO_ARCH_ESP8266)

ALWAYS_INLINE static
void _delayCycles(uint32_t delay_count) {
    // ~  3 -  2,  87ns,  7 cycles, +6 => +75ns
    // ~  9 -  8, 162ns, 13 cycles,
    // ~ 10 - 14, 237ns, 19 cycles, +75ns
    // ~ 16 - 20, 312ns, 25 cycles, +75ns
    // ~ 22 - 26, 387ns, 31 cycles, +75ns
    // ~ 28 - 32, 462ns, 37 cycles, +75ns
    uint32_t total, time;
    __asm__ __volatile__ (
        "rsr.ccount %[time]\n\t"
        "extw\n\t"      // 1 or more cycles, maybe 3
        "1:\n\t"        // This loop is 6 cycles long with unavoidable pipeline stalls.
        "rsr.ccount %[total]\n\t"
        "sub        %[total], %[total], %[time]\n\t"
        "blt        %[total], %[delay], 1b\n\t"
        : [total]"=&r"(total), [time]"=&r"(time)
        : [delay]"r"(delay_count)
    );
}

ALWAYS_INLINE static
void _delay12D5nsNop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay25nsNop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay37D5nsNop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay50nsNop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay112D5nsNop() {
    __asm__ __volatile__ (
        "nop.n\n\t"     // 1 or more cycles, maybe 3
        "nop.n\n\t"     // each NOP is 12.5ns at 80MHz
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _IoCommit() {
    __asm__ __volatile__ (
      "extw\n\t");    // 1 or more cycles, maybe 3
}

#endif    // ARDUINO_ARCH_ESP8266
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Optomized delays needed for implimenting a software based SPI Bus for
// communicating with the MAX31855.
//
ALWAYS_INLINE void delay100nsMin()
{
#if defined(ARDUINO_ARCH_ESP8266)
// Approximately 12.5ns/cycle at 80MHz and 6.25ns/cycle for 160MHz.
// It doesn't take long to reach 100ns.
#if defined(F_CPU) && (F_CPU == 160000000L)
    _delayCycles(14);
#else
  // _delayCycles(8);
  _delayCycles(0);   // 87ns,  7 cycles
  _delay25nsNop();  // Net 100 - 112.5ns
  _delay12D5nsNop();  // a little extra for slow rise times cause with resistor pullups
#endif
#else
    delayMicroseconds(1);
#endif
}

ALWAYS_INLINE void delay200nsMin(void)
{
#if defined(ARDUINO_ARCH_ESP8266)
#if defined(F_CPU) && (F_CPU == 160000000L)
  _delayCycles(32);
#else
  _delayCycles(14);
  // _delayCycles(8);   // 162.5ns, 13 cycles
  // _delay50nsNop();  // Net 200 - 212.5ns
  // _delay12D5nsNop();
#endif

#else
    delayMicroseconds(1);
#endif
}

///////////////////////////////////////////////////////////////////////////////
// For tuning and testing SPI delay code
//
#if defined(ARDUINO_ARCH_ESP8266)

ALWAYS_INLINE uint32_t protoSPIDelay100(uint32_t cycles) {
// Approximately 12.5ns/cycle at 80MHz and 6.25ns/cycle for 160MHz.
// It doesn't take long to reach 100ns.
    (void)cycles;
    uint32_t delta, start;
    __asm__ __volatile__ (
      "rsr.ccount %[start]\n\t"
      : [start]"=r"(start)
      ::);

// delay test dejour
    // _delayCycles(cycles);
    delay100nsMin();
    // delay200nsMin();

    __asm__ __volatile__ (
      "rsr.ccount %[delta]\n\t" // These ccount instruction together add 12ns to the result thanks to pipeline
      "sub        %[delta], %[delta], %[start]\n\t"
      "addi       %[delta], %[delta], -1\n\t"
      : [delta]"=&r"(delta)
      : [start]"r"(start)
      :);
    return delta;
}

ALWAYS_INLINE uint32_t protoSPIDelay200(uint32_t cycles) {
// Approximately 12.5ns/cycle at 80MHz and 6.25ns/cycle for 160MHz.
// It doesn't take long to reach 100ns.
    (void)cycles;
    uint32_t delta, start;
    __asm__ __volatile__ (
      "rsr.ccount %[start]\n\t"
      : [start]"=r"(start)
      ::);


// delay test dejour
    // _delayCycles(cycles);
    // delay100nsMin();
    delay200nsMin();


    __asm__ __volatile__ (
      "rsr.ccount %[delta]\n\t" // These ccount instruction together add 12ns to the result thanks to pipeline
      "sub        %[delta], %[delta], %[start]\n\t"
      "addi       %[delta], %[delta], -1\n\t"
      : [delta]"=&r"(delta)
      : [start]"r"(start)
      :);
    return delta;
}
NOINLINE IRAM_ATTR
void testDelay100nsMin() {
  // constexpr uint32_t cycles =  2;  //  87ns,  7 cycles +6 => +75ns
  constexpr uint32_t cycles =  8;  // 162ns, 13 cycles
  // constexpr uint32_t cycles = 14;  // 237ns, 19 cycles, +75ns
  // constexpr uint32_t cycles = 20;  // 312ns, 25 cycles, +75ns
  // constexpr uint32_t cycles = 26;  // 387ns, 31 cycles, +75ns
  // constexpr uint32_t cycles = 32;  // 462ns, 37 cycles, +75ns

  uint32_t freq = ets_get_cpu_frequency();
  uint32_t delay;
  delay = protoSPIDelay100(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_100, %4.2fns, %u cycles\r\n", delay * 1000. / freq, delay);
  delay = protoSPIDelay100(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_100, %4.2fns, %u cycles\r\n", delay * 1000. / freq, delay);
  CONSOLE.println();
  delay = protoSPIDelay200(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_200, %4.2fns, %u cycles\r\n", delay * 1000. / freq, delay);
  delay = protoSPIDelay200(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_200, %4.2fns, %u cycles\r\n", delay * 1000. / freq, delay);
}
#endif // ARDUINO_ARCH_ESP8266
//
// end of Optomized delay and tuning SPI delay code
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// SPI Bus - melding Soft and HW based handling
// TODO: This look very localized, maybe make into a separate common library.
//
bool MAX31855K::beginSPI(const SPIPins cfg)
{
    // _cs         = cfg._cs;
    // _sck        = cfg._sck;
    // _miso       = cfg._miso;
    // _mosi       = cfg._mosi;
    // _hw         = cfg._hw;
    // _softCs     = cfg._softCs;
    _pins = cfg;

    if (_pins.hw) {
        // Try and configure Hardware SPI Bus, returns false if SPI Bus
        // hardware transfers are not supported for the pins chosen.
        _pins.hw = SPI.pins(_pins.sck, _pins.miso, _pins.mosi, _pins.cs);
    }
    if (_pins.hw) {
        DEBUG_PRINTF("HW SPI Configured ");
        if (15 == _pins.cs) {
            DEBUG_PRINTF("w/HW CS");
            SPI.setHwCs(true);
        } else if (0 != _pins.cs) {
            DEBUG_PRINTF("w/SW CS");
            _pins.softCs = true;
            digitalWrite(_pins.cs, HIGH);    // deselect
            pinMode(_pins.cs, OUTPUT);
        }
        SPI.begin();
    } else {
        DEBUG_PRINTF("Soft SPI Configured");
        digitalWrite(_pins.cs, HIGH);    // deselect
        pinMode(_pins.cs, OUTPUT);
        digitalWrite(_pins.sck, LOW);    // clock starts from low
        pinMode(_pins.sck, OUTPUT);
        pinMode(_pins.miso, INPUT);
    }
    DEBUG_PRINTF(", Pin Assignment: SCK=%d, MISO=%d, MOSI=%d, CS=%d\r\n",
                  _pins.sck, _pins.miso, _pins.mosi, _pins.cs);
    delay200nsMin();    // minimum CS# inactive time
    return read();      // Test pin configuration
}

// Keep keep timing parts in fast IRAM
NOINLINE IRAM_ATTR static
//D uint32_t _spiRead32(bool hw, bool softCs, uint8_t cs, uint8_t sck, uint8_t miso)
uint32_t _spiRead32(const SPIPins& pins)
{
    union {
      uint32_t u32;
      uint8_t  u8[4];
    } value;

    if (pins.hw) {
        // MAX serial clock frequency 5MHz
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        if (pins.softCs) {
            // Arduino doc says CS is done after SPI.beginTransaction!
            digitalWrite(pins.cs, LOW);  // select
            delay100nsMin();        // wait time for first bit valid - not sure this is needed here
        }
        for (ssize_t i = 3; i >= 0; i--) {
            value.u8[i] = SPI.transfer(0);
        }
        SPI.endTransaction();
    } else {
        for (size_t i = 0; i < 32; i++) {
            value.u32 <<= 1;
            if (digitalRead(pins.miso)) {
                value.u32 |= 1;
            }
            digitalWrite(pins.sck, HIGH); // data read before rising clock edge
            delay100nsMin();
            digitalWrite(pins.sck, LOW);
            delay100nsMin();
        }
    }
    return value.u32;
}

uint32_t MAX31855K::spiRead32(void)
{
    if (_pins.softCs) {
        if (_pins.hw) {
            if (LOW == digitalRead(_pins.cs)) {
                // make idle to restart process
                digitalWrite(_pins.cs, HIGH);
                delay200nsMin();    // minimum CS# inactive time
            }
        } else {
            // Check for idle state
            if (LOW == digitalRead(_pins.cs)
            ||  LOW != digitalRead(_pins.sck)) {
                // make idle to restart process
                digitalWrite(_pins.cs, HIGH);
                digitalWrite(_pins.sck, LOW);  // Start soft SPI with SCK low before CS#
                delay200nsMin();    // minimum CS# inactive time
            }
            digitalWrite(_pins.cs, LOW);     // select
            delay100nsMin();            // wait time for first bit valid
            // Soft SPI is now ready to read the 1st bit.
        }
    }

    //D uint32_t value = _spiRead32(_pins.hw, _pins.softCs, _pins.cs, _pins.sck, _pins.miso);  // Keep the key parts in fast IRAM
    uint32_t value = _spiRead32(_pins);

    digitalWrite(_pins.cs, HIGH);  // deselect slave
    /*
      TODO: Check timing with scope with attention to the rise/fall times with
      that resister/diode level translater on SCK and CS#.
    */
    return value;
}
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// MAX31855K Type K Thermocouple library
//
/*
  Useful and instructive info from:
    https://instrumentationtools.com/thermocouple-calculations/
    https://www.ti.com/lit/an/sbaa274/sbaa274.pdf
    https://blog.heypete.com/2016/09/09/max31855-temperature-linearization/

  "thermocouple-calculations" gives a good perspective of how we get the
  hot-thermocouple temperature w/o using an ice bath for the reference end.
  SBAA274 adds a lot of design depth to the thermocouple discussion.

  We get a die temerature reading from the MAX31855. This is used for the
  Cold-junction temperature in Celsius. That value is changed into mV using the
  ITS90 coefficient based calculation to convert degrees Celsius to an
  equivalent thermocouple mV reading. That value is added to the probe
  thermocouple mV reading. The resulting sum is passed through an ITS90
  coefficient based "inverse" conversion to get the linearized temperature in
  degrees Celsius.
*/

sint32_t MAX31855K::_getProbeEM04() const
{
    // The probe temperature (thermocouple) value provided by the MAX31855 is
    // compensated for cold junction temperature. It also assumes a linear
    // response from the non-linear (mostly linear) type K thermocouple.
    sint32_t probe   = (parceData().probe - _zero_cal) * 2500;  // 0.25 * 10000
    if (_swap_leads) {
        // Fix miss wired thermocouple, polarity reversed.
        sint32_t internal = _getDeviceEM04();
        probe = (internal - probe) + internal;
    }
    return probe;
}

/*
  This is the isolated hot-junction thermocouple voltage output.
  ie. before the cold-junction block's voltage is added in.
*/
DFLOAT MAX31855K::getHotJunctionVout() const
{
    DFLOAT vout = FLT_MAX;

    if (isValid()) {
        sint32_t probe    = _getProbeEM04();
        sint32_t internal = _getDeviceEM04();
        vout = 41.276E-06 * DFLOAT(probe - internal) * 1.0E-04;
    }
    return vout;
}


DFLOAT MAX31855K::getITS90(const DFLOAT coldJunctionC, const DFLOAT voutMv) const
{
    /*
      For validating the range of the Cold-junction temperature end. Use the
      MAX31855's operating range of -40 to +125 degrees Celsius. The NIST table
      limits would allowed values far exceeding the MAX31855K operating range.
    */
    if (isMAX31855InOperationRange(coldJunctionC)) {  // This is less than NIST table range
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
  Supports faster integer math when implimenting temperature control thresholds
  for tuning device on/off. Generate a probeEM04 value for matching a real
  temperature to the uncorrected nonlinear reading from the MAX31855K.

  Wide swings in ambient, Cold Junction, temperature will increase error
  with this method. Infuence should be lessened if Hot-junction temperature
  are larger than the Cold Junction temperature. The Cold Junction temperature
  becomes a smaller component in the equation.

  A workaround would be to periodicly refresh the threshold calculations.
*/
sint32_t MAX31855K::convertC2ProbeEM04(const DFLOAT hj, const DFLOAT cj) const
{
    constexpr DFLOAT kvc = 41.276E-06; //type_k_celsius_to_mv(1000.) * 1.0E-03 / 1000.;
    DFLOAT hotJunctionV = (type_k_celsius_to_mv(hj) - type_k_celsius_to_mv(cj)) * 1.0E-03;
    DFLOAT coldJunctionV = cj * kvc;
    return ((hotJunctionV + coldJunctionV) / kvc * 1.0E04);  // * 1.0E-04
}
//
// End - MAX31855K Type K Thermocouple library
///////////////////////////////////////////////////////////////////////////////
