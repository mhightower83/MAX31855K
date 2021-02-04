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

#define DEBUG_SKETCH
#include <debugHelper.h>
#include "MAX31855K.h"
#include "TypeK_ITS90.h"

#define DIAG_PRINT

extern "C" uint32_t ets_get_cpu_frequency(void);

///////////////////////////////////////////////////////////////////////////////
// Tuned Delays specific to the ESP8266
//
#if defined(ARDUINO_ARCH_ESP8266)

ALWAYS_INLINE static
void _delay_cycles(uint32_t delay_count) {
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
void _delay_12_5ns_nop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay_25ns_nop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay_37_5ns_nop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay_50ns_nop() {
    __asm__ __volatile__ (
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
        "nop.n\n\t"
    );
}

ALWAYS_INLINE static
void _delay_112_5ns_nop() {
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
void _io_commit() {
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
ALWAYS_INLINE void SPI_DELAY_100NS_MIN()
{
#if defined(ARDUINO_ARCH_ESP8266)
// Approximately 12.5ns/cycle at 80MHz and 6.25ns/cycle for 160MHz.
// It doesn't take long to reach 100ns.
#if defined(F_CPU) && (F_CPU == 160000000L)
    _delay_cycles(14);
#else
  // _delay_cycles(8);
  _delay_cycles(0);   // 87ns,  7 cycles
  _delay_25ns_nop();  // Net 100 - 112.5ns
  _delay_12_5ns_nop();  // a little extra for slow rise times cause with resistor pullups
#endif
#else
    delayMicroseconds(1);
#endif
}

ALWAYS_INLINE void SPI_DELAY_200NS_MIN(void)
{
#if defined(ARDUINO_ARCH_ESP8266)
#if defined(F_CPU) && (F_CPU == 160000000L)
  _delay_cycles(32);
#else
  _delay_cycles(14);
  // _delay_cycles(8);   // 162.5ns, 13 cycles
  // _delay_50ns_nop();  // Net 200 - 212.5ns
  // _delay_12_5ns_nop();
#endif

#else
    delayMicroseconds(1);
#endif
}

///////////////////////////////////////////////////////////////////////////////
// For tuning SPI delay code
//
#if defined(ARDUINO_ARCH_ESP8266)

ALWAYS_INLINE uint32_t proto_SPI_DELAY_100(uint32_t cycles) {
// Approximately 12.5ns/cycle at 80MHz and 6.25ns/cycle for 160MHz.
// It doesn't take long to reach 100ns.
    (void)cycles;
    uint32_t delta, start;
    __asm__ __volatile__ (
      "rsr.ccount %[start]\n\t"
      : [start]"=r"(start)
      ::);

// delay test dejour
    // _delay_cycles(cycles);
    SPI_DELAY_100NS_MIN();
    // SPI_DELAY_200NS_MIN();

    __asm__ __volatile__ (
      "rsr.ccount %[delta]\n\t" // These ccount instruction together add 12ns to the result thanks to pipeline
      "sub        %[delta], %[delta], %[start]\n\t"
      "addi       %[delta], %[delta], -1\n\t"
      : [delta]"=&r"(delta)
      : [start]"r"(start)
      :);
    return delta;
}

ALWAYS_INLINE uint32_t proto_SPI_DELAY_200(uint32_t cycles) {
// Approximately 12.5ns/cycle at 80MHz and 6.25ns/cycle for 160MHz.
// It doesn't take long to reach 100ns.
    (void)cycles;
    uint32_t delta, start;
    __asm__ __volatile__ (
      "rsr.ccount %[start]\n\t"
      : [start]"=r"(start)
      ::);


// delay test dejour
    // _delay_cycles(cycles);
    // SPI_DELAY_100NS_MIN();
    SPI_DELAY_200NS_MIN();


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
void test_SPI_DELAY_100NS_MIN() {
  // constexpr uint32_t cycles =  2;  //  87ns,  7 cycles +6 => +75ns
  constexpr uint32_t cycles =  8;  // 162ns, 13 cycles
  // constexpr uint32_t cycles = 14;  // 237ns, 19 cycles, +75ns
  // constexpr uint32_t cycles = 20;  // 312ns, 25 cycles, +75ns
  // constexpr uint32_t cycles = 26;  // 387ns, 31 cycles, +75ns
  // constexpr uint32_t cycles = 32;  // 462ns, 37 cycles, +75ns

  uint32_t delay;
  delay = proto_SPI_DELAY_100(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_100, %4.2fns, %u cycles\r\n", delay * 1000. / ets_get_cpu_frequency(), delay);
  delay = proto_SPI_DELAY_100(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_100, %4.2fns, %u cycles\r\n", delay * 1000. / ets_get_cpu_frequency(), delay);
  CONSOLE.println();
  delay = proto_SPI_DELAY_200(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_200, %4.2fns, %u cycles\r\n", delay * 1000. / ets_get_cpu_frequency(), delay);
  delay = proto_SPI_DELAY_200(cycles);
  CONSOLE_PRINTF("Elapsed time for SPI_DELAY_200, %4.2fns, %u cycles\r\n", delay * 1000. / ets_get_cpu_frequency(), delay);
}
#endif // ARDUINO_ARCH_ESP8266
//
// end of Optomized delay and tuning SPI delay code
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// MAX31855K Type K Thermocouple library
//
bool MAX31855K::begin(const SPIPins cfg)
{
    // _cs         = cfg._cs;
    // _sck        = cfg._sck;
    // _miso       = cfg._miso;
    // _mosi       = cfg._mosi;
    // _hw         = cfg._hw;
    // _softCs     = cfg._softCs;
    _pins = cfg;

    if (_pins._hw) {
        // Try and configure Hardware SPI Bus, returns false if SPI Bus
        // hardware transfers are not supported for the pins chosen.
        _pins._hw = SPI.pins(_pins._sck, _pins._miso, _pins._mosi, _pins._cs);
    }
    if (_pins._hw) {
        DEBUG_PRINT("HW SPI Configured ");
        if (15 == _pins._cs) {
            DEBUG_PRINT("w/HW CS");
            SPI.setHwCs(true);
        } else if (0 != _pins._cs) {
            DEBUG_PRINT("w/SW CS");
            _pins._softCs = true;
            digitalWrite(_pins._cs, HIGH);    // deselect
            pinMode(_pins._cs, OUTPUT);
        }
        SPI.begin();
    } else {
        DEBUG_PRINT("Soft SPI Configured");
        digitalWrite(_pins._cs, HIGH);    // deselect
        pinMode(_pins._cs, OUTPUT);
        digitalWrite(_pins._sck, LOW);    // clock starts from low
        pinMode(_pins._sck, OUTPUT);
        pinMode(_pins._miso, INPUT);
    }
    DEBUG_PRINTF(", Pin Assignment: SCK=%d, MISO=%d, MOSI=%d, CS=%d\r\n",
                  _pins._sck, _pins._miso, _pins._mosi, _pins._cs);
    SPI_DELAY_200NS_MIN();    // minimum CS# inactive time
    return readSample();      // Test pin configuration
}

NOINLINE IRAM_ATTR static
uint32_t _spi_read32(bool hw, uint8_t sck, uint8_t miso)
{
    union {
      uint32_t u32;
      uint8_t  u8[4];
    } value;

    if (hw) {
        // MAX serial clock frequency 5MHz
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        for (ssize_t i = 3; i >= 0; i--) {
            value.u8[i] = SPI.transfer(0);
        }
        SPI.endTransaction();
    } else {
        for (size_t i = 0; i < 32; i++) {
            value.u32 <<= 1;
            if (digitalRead(miso)) {
                value.u32 |= 1;
            }
            digitalWrite(sck, HIGH);    // data read before rising clock edge
            SPI_DELAY_100NS_MIN();
            digitalWrite(sck, LOW);
            SPI_DELAY_100NS_MIN();
        }
    }
    return value.u32;
}
uint32_t MAX31855K::spi_read32(void)
{
    if (_pins._softCs) {
        if (_pins._hw) {
            if (LOW == digitalRead(_pins._cs)) {
                // make idle to restart process
                digitalWrite(_pins._cs, HIGH);
                SPI_DELAY_200NS_MIN();    // minimum CS# inactive time
            }
        } else {
            // Check for idle state
            if (LOW == digitalRead(_pins._cs)
            ||  LOW != digitalRead(_pins._sck)) {
                // make idle to restart process
                digitalWrite(_pins._cs, HIGH);
                digitalWrite(_pins._sck, LOW);  // Start soft SPI with SCK low before CS#
                SPI_DELAY_200NS_MIN();    // minimum CS# inactive time
            }
        }
        digitalWrite(_pins._cs, LOW);     // select
        SPI_DELAY_100NS_MIN();            // wait time for first bit valid
        // Soft SPI is now ready to read the 1st bit.
    }

    uint32_t value = _spi_read32(_pins._hw, _pins._sck, _pins._miso);  // Keep the key parts in fast IRAM

    digitalWrite(_pins._cs, HIGH);  // deselect slave
    /*
      TODO: Check timing with scope with attention to the rise/fall times with
      that resister/diode level translater on SCK and CS#.
    */
    return value;
}

sint32_t MAX31855K::_getProbeE_04() const
{
    // The probe temperature (thermocouple) value provided by the MAX31855 is
    // compensated for cold junction temperature. It also assumes a linear
    // response from the non-linear (mostly linear) type K thermocouple.
    const struct MAX31855_BITMAP& parse = getData();
    sint32_t probe   = (parse.probe - _zero_cal) * 2500;
    if (_swap_leads) {
        sint32_t internal = parse.internal * 625;
        probe = (internal - probe) + internal;  // Reverse leads - Invert the delta temperature
        return probe;
    }
    return probe;
}
sint32_t MAX31855K::getProbeE_04() const
{
    if (isValid()) {
      return _getProbeE_04();
    }
    return INT32_MAX;
}

sint32_t MAX31855K::_getDeviceE_04() const
{
      return getData().internal * 625;
}
sint32_t MAX31855K::getDeviceE_04() const
{
    if (isValid()) {
        return _getDeviceE_04();
    }
    return INT32_MAX;
}

sint32_t MAX31855K::getVoutE_09() const // nV
{
    if (isValid()) {
        sint32_t probe    = _getProbeE_04();
        sint32_t internal = _getDeviceE_04();
        // temperatures were in milli degrees - convert to degrees
        sint32_t vout = 41276 * (probe - internal) / 10000; // nV
        return vout;
    }
    return INT32_MAX;
}

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
inline constexpr bool is_max31855k_operation_range(const float t) {
  return (125. >= t && -40. <= t);
}

float MAX31855K::getITS90()   // aka float getLinearizedTemp()
{
    // Check 1st for a good sample
    if (!isValid()) {
        return FLT_MAX;   // I think NAN would need exceptions to handle
                          // Caller must test for FLT_MAX to confirm success.
    }

    const float cold_junction_c = (float)getDeviceE_04() / 10000.;   // cold-junction termerature
    const float vout_mv = (float)getVoutE_09() / 1000000.;  // voltage reading of thermocouple
                                          // getVoutE_09() returns int32_t in nV convert to mV.

    /*
      For validating the range of the Cold-junction temperature end. Use the
      MAX31855's operating range of -40 to +125 degrees Celsius. The NIST table
      limits would allowed values far exceeding the MAX31855K operating range.
    */
    if (is_max31855k_operation_range(cold_junction_c)) {  // This is less than NIST table range
        float mV = vout_mv + type_k_celsius_to_mv(cold_junction_c);
        if (is_type_k_mv_in_range(mV)) {
            return type_k_mv_to_celsius(mV);
        }
    }

    /*
      The request is failed when the input data values do not fall within the
      operating ranges of the hardware or ITS90 table ranges. For cases when
      the range check fails it is probably noise or some other malfunction.
    */
    return FLT_MAX;
}
//
// End - MAX31855K Type K Thermocouple library
///////////////////////////////////////////////////////////////////////////////
