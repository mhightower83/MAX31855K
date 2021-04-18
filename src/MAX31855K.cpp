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

extern "C" uint32_t ets_get_cpu_frequency(void);

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

///////////////////////////////////////////////////////////////////////////////
// Optomized delays needed for implimenting a software based SPI Bus for
// communicating with the MAX31855.
//
ALWAYS_INLINE void delay100nsMin()
{
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
}

ALWAYS_INLINE void delay200nsMin()
{
#if defined(F_CPU) && (F_CPU == 160000000L)
  _delayCycles(32);
#else
  _delayCycles(14);
  //D+ Next line
  _delayCycles(14);     // debug
  // _delayCycles(8);   // 162.5ns, 13 cycles
  // _delay50nsNop();  // Net 200 - 212.5ns
  // _delay12D5nsNop();
#endif
}

///////////////////////////////////////////////////////////////////////////////
// For tuning and testing SPI delay code
//
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
//
// end of Optomized delay and tuning SPI delay code
///////////////////////////////////////////////////////////////////////////////

#else
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
        _pins.hw = SPI.pins(_pins.sck, _pins.miso, _pins.mosi, _pins.cs);
    }
    if (_pins.hw) {
        DEBUG_PRINTF("HW SPI Configured ");
        if (SS == _pins.cs) {   // GPIO15
            DEBUG_PRINTF("w/HW CS");
            SPI.setHwCs(true);
        } else if (0 != _pins.cs) {
            DEBUG_PRINTF("w/SW CS");
            _pins.softCs = true;
            digitalWrite(_pins.cs, HIGH);    // deselect
            pinMode(_pins.cs, OUTPUT);
        }
    delay200nsMin();    // minimum CS# inactive time
        SPI.begin();
    } else {
        DEBUG_PRINTF("Soft SPI Configured");
        pinMode(_pins.miso, INPUT_PULLUP); //INPUT);
        digitalWrite(_pins.cs, HIGH);       // deselect
        pinMode(_pins.cs, OUTPUT);
        delay200nsMin();
        digitalWrite(_pins.sck, LOW);       // set inactive clock, starts from low
        pinMode(_pins.sck, OUTPUT);
        delay100nsMin();
    }
    DEBUG_PRINTF(", Pin Assignment: SCK=%d, MISO=%d, MOSI=%d, CS=%d\r\n",
                  _pins.sck, _pins.miso, _pins.mosi, _pins.cs);
    return read();      // Test pin configuration
}

// Keep timing parts in fast IRAM
NOINLINE IRAM_ATTR static
uint32_t _spiRead32(const SPIPins& pins)
{
    union {
      uint32_t u32;
      uint8_t  u8[4];
    } value;

    if (pins.hw) {
        // MAX serial clock frequency 5MHz
        //+ SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        if (pins.softCs) {
            // Arduino doc says CS is done after SPI.beginTransaction!
            digitalWrite(pins.cs, LOW);     // select
            //+ delay100nsMin(); // wait time for first bit valid - not sure this is needed here
            delay200nsMin(); // wait time for first bit valid - not sure this is needed here
        }
        for (ssize_t i = 3; i >= 0; i--) {  // MSBFIRST little-endian
            value.u8[i] = SPI.transfer(0);
        }
        SPI.endTransaction();
    } else {
        digitalWrite(pins.cs, LOW);      // CS# enable
        ///+ delay100nsMin();                  // wait time for first bit valid
        delay200nsMin();                  // overkill - wait time for first bit valid
        // Soft SPI is now ready to read the 1st bit.
        for (size_t i = 0; i < 32; i++) {
            value.u32 = value.u32 << 1 | digitalRead(pins.miso);
            digitalWrite(pins.sck, HIGH);   // data read before rising clock edge
            //+ delay100nsMin();
            delay200nsMin();
            digitalWrite(pins.sck, LOW);
            //+ delay100nsMin();
            delay200nsMin();
        }
        // SCK inactive clock left LOW!
    }
    return value.u32;
}

uint32_t MAX31855K::spiRead32(void)
{
    // Prep/confirm CS#, and SCK inactive state.
    if (_pins.softCs) {
        // This code path is error recovery logic. While it should never happen,
        // just make it right.
        if (LOW == digitalRead(_pins.cs)) {
            // Make CS# inactive for required 200ns to restart process
            digitalWrite(_pins.cs, HIGH);
            delay200nsMin();              // minimum CS# inactive time
        }
        if (LOW != digitalRead(_pins.sck)) {
            // For MAX31855 SPI transfers, SCK must be in the inactive state
            // before CS# is asserted. (Mode 0)
            digitalWrite(_pins.sck, LOW);
            delay100nsMin();
        }
    }

    uint32_t value = _spiRead32(_pins);

    // No harm if HW SPI, this may have already been done.
    digitalWrite(_pins.cs, HIGH); // deselect slave
    delay200nsMin();              // minimum CS# inactive time
    // It is the upper levels responsibility to not call again for 200ns.
    // This should be the natural logic flow. No new converted values will be
    // available for 70-100ms. There is no point in being called sooner!
    // Thus we skip adding a needless hard CPU wasting delay.
    /*
      TODO: Check timing with scope with attention to the rise/fall times with
      that resister/diode level translater on SCK and CS#.
    */
    if (_fakeRead) {
        value = _fakeRead;
    }
    return value;
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

  To use the ITS90 polynomial to get a linearied voltage reading, we need the
  mV reading of the hot-thermocouple and the mV reading of the mV reading of the
  cold-thermocouple.

  What we get from the MAX31855K is a temperature reading of the chip's die
  (internal) in the MAX31855K in degrees Celsius. This can be used to derive the
  Cold-Junction thermocouple mV output, by using the ITS90 coversion
  coefficients.

  For the Hot-Junction (probe) side of the equation, the MAX31855K gives us an
  approxamation of the probe temperature. Based on die temperature plus mV
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
  Supports faster integer math when implimenting temperature control thresholds
  for turning device on/off. Estamate a probeX10K value for matching a real
  temperature to the uncorrected nonlinear reading from the MAX31855K.

  Wide swings in ambient, Cold Junction, temperature will increase error
  with this method. Infuence should be lessened if Hot-junction temperature
  are larger than the Cold Junction temperature. The Cold Junction temperature
  becomes a smaller component in the equation.

  A work-around would be to periodicly refresh the threshold calculations.
*/
sint32_t MAX31855K::convertC2ProbeX10K(const DFLOAT hj, const DFLOAT cj) const
{
    if (FLT_MAX == hj || FLT_MAX == cj)
        return INT32_MAX;

    DFLOAT hotJunctionV = milliVolt2Volt(type_k_celsius_to_mv(hj) - type_k_celsius_to_mv(cj));
    DFLOAT coldJunctionV = cj * kTypeKSensitivityVoC;
    return sint32X10K((hotJunctionV + coldJunctionV) / kTypeKSensitivityVoC);
}
//
// End - MAX31855K Type K Thermocouple library
///////////////////////////////////////////////////////////////////////////////
