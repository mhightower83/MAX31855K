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
/*
  Clarification on word usage: "corrected" vs "compensated".

  I use (linear) "corrected" when referring to the linearized correction applied
  to the thermocouple's non-linear response.

  "compensated" is used when referring to the adding in of the Cold-Junction
  temperature measurement to the  Hot-Junction measurement to make it relative to a
  Cold-Junction temperature of 0 degrees Celsius.
*/
#ifndef MAX31855K_H
#define MAX31855K_H
#include <cmath>
#include <cfloat>

#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE inline __attribute__ ((always_inline))
#endif
#ifndef NOINLINE
#define NOINLINE __attribute__((noinline))
#endif

#define DFLOAT float //double

//////////////////////////////////////////////////////////////////////////////
// Things the linter cannot find  ¯\_(ツ)_/¯
#ifdef GCC_LINTER
#define FLT_MAX		__FLT_MAX__
#endif


//////////////////////////////////////////////////////////////////////////////
// SPI Bus API helper
//
class SPIPins {
public:
    uint8_t cs;
    uint8_t sck;
    uint8_t miso;
    uint8_t mosi;    // Not used, HW SPI will require it to match and must be reserved for SPI hardware
                     // For Software SPI you can use 0
    uint8_t hw;
    uint8_t softCs;
#if defined(ARDUINO_ARCH_ESP8266)
    // These defaults are for HSPI on the ESP8266
    // If a CS other than 15 is used, soft chip select is used with HW SPI.
    // If any of the other pins are not default the library switches over to
    // Software SPI.
    SPIPins(uint32_t _cs=15, uint8_t _sck=14, uint8_t _miso=12, uint8_t _mosi=13, bool _autoHw=true)
    : cs(_cs)
    , sck(_sck)
    , miso(_miso)
    , mosi(_mosi) {
        hw         = false;
        softCs     = true;
        // Auto select HW SPI if pins are correct.
        // MAX31855 is a read-only SPI device, MOSI is only needed to complete
        // the Hardware SPI interface selection vs Software driven SPI interface.
        if (_autoHw && 14 == sck && 12 == miso && 13 == mosi) {
            hw = true;
            if (15 == cs) {
              softCs = false;
            }
        }
    }

#else
    SPIPins(uint32_t _cs, uint8_t _sck, uint8_t _miso,  uint8_t _mosi, bool _autoHw=true))
    : cs(_cs)
    , sck(_sck)
    , miso(_miso)
    , mosi(_mosi) {
      (void)_autoHw;
        hw     = false;
        softCs = true;
        if (UINT8_MAX == _cs) {
            softCs = false;
        }
        if (UINT8_MAX == _sck) {
            hw = true;
        }
    }
    SPIPins() {
        SPIPins(UINT8_MAX, UINT8_MAX, UINT8_MAX, UINT8_MAX);
    }
    SPIPins(uint32_t _cs) {
        SPIPins(_cs, UINT8_MAX, UINT8_MAX, UINT8_MAX);
    }
#endif
};

///////////////////////////////////////////////////////////////////////////////
// MAX31855K Type K Thermocouple library
//
ALWAYS_INLINE constexpr bool isMAX31855InOperationRange(const DFLOAT t) {
    return (125. >= t && -40. <= t);
}

// inline conversions
ALWAYS_INLINE constexpr DFLOAT floatX10K(const sint32_t t) { if (INT32_MAX == t) return FLT_MAX; return DFLOAT(t * 1E-04); }
ALWAYS_INLINE constexpr sint32_t sint32X10K(const DFLOAT t) { if (FLT_MAX == t) return INT32_MAX; return sint32_t(round(t * 1E04)); }
ALWAYS_INLINE constexpr sint32_t sint32X10K(const sint32_t t) { if (INT32_MAX == t) return INT32_MAX; return sint32_t(t * 10000); }
ALWAYS_INLINE constexpr DFLOAT milliVolt2Volt(const DFLOAT mV) { if (FLT_MAX == mV) return FLT_MAX; return mV * 1.0E-03; }
ALWAYS_INLINE constexpr DFLOAT volt2MilliVolt(const DFLOAT mV) { if (FLT_MAX == mV) return FLT_MAX; return mV * 1.0E03; }
ALWAYS_INLINE constexpr DFLOAT celsius2Fahrenheit(const DFLOAT c) { if (FLT_MAX == c) return FLT_MAX; return c * 1.8 + 32; }
ALWAYS_INLINE constexpr DFLOAT fahrenheit2Celsius(const DFLOAT f) { if (FLT_MAX == f) return FLT_MAX; return (f - 32) / 1.8; }

constexpr sint32_t kProbeX10K    = sint32X10K(DFLOAT(0.25));    // Thermocouple data resolution * 10000
constexpr sint32_t kInternalX10K = sint32X10K(DFLOAT(0.0625));  // Cold-Junction temperature data resolution * 10000
// Over the range: 0 to 1000 degrees for Type K thermocouple. "kTypeKSensitivityVoC"
// could be calculated with type_k_Celsius_to_mv(1000.) * 1.0E-03 / 1000.;
// This is a straight line fit to nonlinear data.
constexpr DFLOAT kTypeKSensitivityVoC = 41.276E-06; // Use constant specified in the MAX313855K datasheet.

constexpr uint32_t kFakeRawSample = 0x3E801900u;    // Probe 1000 degrees C and 25 degrees ambient

struct Meter31855X10K {
    sint32_t hot;     // aka. Hot-Junction, hot-probe, probe, thermocouple
    sint32_t cold;    // aka. Cold-Junction, cold-block, device, ambient, internal, reference
    Meter31855X10K() {
        hot = 0;
        cold = 0;
    }
};
struct Meter31855 {
    DFLOAT hot;
    DFLOAT cold;
    Meter31855() {
      hot = 0;
      cold = 0;
    }
};
ALWAYS_INLINE Meter31855X10K get31855X10K(const sint32_t probe, const sint32_t internal, const sint32_t zero_cal_X10K=0, const bool swap_leads=false) {
    Meter31855X10K valueX10K;
    valueX10K.hot  = (probe * kProbeX10K)  - zero_cal_X10K;  // 0.25 * 10000
    valueX10K.cold = internal * kInternalX10K;         // 0.0625 * 10000
    if (swap_leads) {
        // Fix cross wired thermocouple, polarity reversed.
        valueX10K.hot = (valueX10K.cold - valueX10K.hot) + valueX10K.cold;
    }
    return valueX10K;
}


struct MAX31855_BITMAP {
    bool     oc:1;            // =1, thermocouple connection is open
    bool     scg:1;           // =1, short-circuited to GND
    bool     scv:1;           // =1, short-circuited to VCC
    uint32_t res:1;           // always 0
    sint32_t internal:12;     // Device internal temperature, cold-junction side
    bool     fault:1;         // set if any of { oc | scg | scv } are set
    uint32_t res2:1;          // always 0
    sint32_t probe:14;        // Cold-junction "compensated" probe temperature, Thermocouple.
};

class MAX31855K {
public:
    /*
      D2 - SCV Fault
        This bit is a 1 when the thermocouple is short-circuited to VCC.
        Default value is 0.

      D1 - SCG Fault
        This bit is a 1 when the thermocouple is short-circuited to GND.
        Default value is 0.

      D0 - OC Fault
        This bit is a 1 when the thermocouple is open (no connections).
        Default value is 0

      D16 - Fault
        This bit is set when any of the above bits are set.
    */
    enum FAULTS {
        OK  = 0,
        SCV = BIT2,
        SCG = BIT1,
        OC  = BIT0,
        ANY = (SCV | SCG | OC),
        FAULT = BIT16,
        RES = (BIT3 | BIT17),
        ERROR_MASK = (ANY | FAULT | RES)
    };

    union Thermocouple {
        MAX31855_BITMAP parse;
        uint32_t raw32;
    };

private:
    SPIPins _pins;
    bool    _swap_leads;
    sint32_t _zero_cal_X10K;       // add to MAX31855_BITMAP.probe value to get zero for 0 degrees Celsius
    uint32_t _errors;
    uint32_t _fakeRead;
    Thermocouple _thermocouple;
    Thermocouple _lastError;
    sint32_t _getProbeX10K() const {
        return get31855X10K(parceData().probe, parceData().internal, _zero_cal_X10K, _swap_leads).hot;
    }
    sint32_t _getDeviceX10K() const { return parceData().internal * kInternalX10K; }

public:
    const SPIPins& getSPIPins() const { return _pins; }
    bool beginSPI(const SPIPins cfg);
    void endSPI() {};

    MAX31855K() { _swap_leads = false; _zero_cal_X10K = 0; _errors = 0; _fakeRead = 0;};
    bool begin(const SPIPins cfg) { return beginSPI(cfg); }
    void end() {}
    void setSwapLeads(bool b) { _swap_leads = b; }
    bool getSwapLeads() const { return _swap_leads; }
    void setZeroCal(sint32_t cal) { _zero_cal_X10K = cal; }
    void setZeroCal() { setZeroCal(parceData().probe * kProbeX10K); }
    void setReferenceCalLinearized(const DFLOAT ref, const DFLOAT hj, const DFLOAT cj);
    void setZeroCalLinearized(const DFLOAT hj, const DFLOAT cj) { setReferenceCalLinearized(0.0, hj, cj); }

    sint32_t getZeroCal() const { return _zero_cal_X10K; }
    //D sint32_t getZeroCalX10K() const { return getZeroCal() * kProbeX10K; }
    void setFakeRead(uint32_t val) { _fakeRead = val; }

    bool isValid(uint32_t raw_u32) const {
        return
            0u == (ERROR_MASK & raw_u32)
        &&  0u != raw_u32    // This is a possible value; however, not very plausible.
        && ~0u != raw_u32;
    }
    bool isValid() const {
        return isValid(_thermocouple.raw32);
    }
    bool read();

    // Access to unprocessed sample data
    const struct MAX31855_BITMAP& parceData() const { return _thermocouple.parse; }
    const union Thermocouple& getData() const { return _thermocouple; }
    const union Thermocouple& getLastError() const { return _lastError; }

    // Scaled Integer result for quick processing
    // ...X10K values were multiplied by 10000 to preserve the decimal parts.
    sint32_t getProbeX10K() const { // * 1.0E-04
        if (isValid()) {
          return _getProbeX10K();
        }
        return INT32_MAX;
    }
    sint32_t getDeviceX10K() const { // * 1.0E-04
        if (isValid()) {
            return _getDeviceX10K();
        }
        return INT32_MAX;
    }
    sint32_t convertC2ProbeX10K(const DFLOAT hj, const DFLOAT cj) const;

    // These float values are not "corrected" for the non-linear properties of
    // thermocouple
    DFLOAT getColdJunction() const { return floatX10K(getDeviceX10K()); }
    DFLOAT getHotJunction() const { return floatX10K(getProbeX10K()); }
    // This is the isolated hot-junction thermocouple voltage output.
    // ie. before the cold-junction block's voltage is added in.
    DFLOAT getHotJunctionVout(const sint32_t probeX10K, const sint32_t deviceX10K) const {
        if (INT32_MAX == probeX10K || INT32_MAX == deviceX10K) return FLT_MAX;
        return kTypeKSensitivityVoC * floatX10K(probeX10K - deviceX10K);
    }
    DFLOAT getHotJunctionVout() const {
        if (isValid()) {
            return getHotJunctionVout(_getProbeX10K(), _getDeviceX10K());
        }
        return FLT_MAX;
    }
    DFLOAT getColdJunctionVout() const { return getColdJunction() * kTypeKSensitivityVoC; };

    // Float values "corrected" for non-linear properties of Type K thermocouple
    // using polynomial coefficients defined by ITS90
    // returns FLT_MAX, for conditions that do not allow processing
    DFLOAT getITS90(const DFLOAT coldJunctionC, const DFLOAT voutMv) const;
    DFLOAT getITS90() const {
        if (isValid()) {
            return getITS90(getColdJunction(), volt2MilliVolt(getHotJunctionVout())); // (C, mV)
        }
        return FLT_MAX;
    }
    DFLOAT getITS90(const sint32_t probeX10K, const sint32_t deviceX10K) const {
        return getITS90(
            floatX10K(deviceX10K),
            volt2MilliVolt(getHotJunctionVout(probeX10K, deviceX10K))); // (C, mV)
    };

    // wrappers for linear corrected values
    DFLOAT getC() const { return getITS90(); };
    DFLOAT getF() const { return celsius2Fahrenheit(getITS90()); };
    DFLOAT getCelsius() const { return getITS90(); };
    DFLOAT getFahrenheit() const { return celsius2Fahrenheit(getITS90()); };

    uint32_t getErrorCount() const { return _errors; }
    void clearErrorCount() { _errors = 0; }

};
//
// End - MAX31855K Type K Thermocouple library
///////////////////////////////////////////////////////////////////////////////
#endif
