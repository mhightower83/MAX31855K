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

#ifndef MAX31855K_H
#define MAX31855K_H
#include <cmath>
#include <cfloat>

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

    SPIPins(uint32_t _cs=15, uint8_t _sck=14, uint8_t _miso=12, uint8_t _mosi=13, bool _autoHw=true)
    : cs(_cs)
    , sck(_sck)
    , miso(_miso)
    , mosi(_mosi) {
        hw         = false;
        softCs     = true;
        // Auto select HW SPI if pins are correct.
        // MAX31855 is a read-only SPI device, MOSI is only needed to complete
        // the Hardware SPI interface selection vs Sofware driven SPI interface.
        if (_autoHw && 14 == sck && 12 == miso && 13 == mosi) {
            hw = true;
            if (15 == cs) {
              softCs = false;
            }
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
// MAX31855K Type K Thermocouple library
//
struct MAX31855_BITMAP {
    bool     oc:1;            // =1, thermocouple connection is open
    bool     scg:1;           // =1, short-circuited to GND
    bool     scv:1;           // =1, short-circuited to VCC
    uint32_t res:1;           // always 0
    sint32_t internal:12;     // Device internal temperature, cold-junction side
    bool     fault:1;         // set if any of { oc | scg | scv } are set
    uint32_t res2:1;          // always 0
    sint32_t probe:14;        // Cold-junction compensated probe temperature, Thermocouple.
};

class MAX31855K {
public:
    union Thermocouple {
        struct MAX31855_BITMAP parse;
        uint32_t raw32;
    };
private:
    bool    _swap_leads;
    SPIPins _pins;
    sint32_t _zero_cal;       // add to MAX31855_BITMAP.probe value to get zero for 0 degrees Celsius
    uint32_t _errors;
    union Thermocouple _thermocouple;
    union Thermocouple _lastError;
    sint32_t _getProbeEM04() const;  // * 1.0E-04
    sint32_t _getDeviceEM04() const;  // * 1.0E-04

public:
    const SPIPins& getSPIPins() const { return _pins; }
    bool beginSPI(const SPIPins cfg);
    void endSPI() {};

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


    MAX31855K() { _swap_leads = false; _zero_cal = 0; _errors = 0; };
    bool begin(const SPIPins cfg);
    void end() {}
    void setSwapLeads(bool b) { _swap_leads = b; }
    sint32_t getSwapLeads() { return _swap_leads; }

    void setZeroCal(sint32_t cal) { _zero_cal = cal; }
    void setZeroCal() { setZeroCal(_thermocouple.parse.probe); }
    sint32_t getZeroCal() const { return _zero_cal; }
    sint32_t getZeroCalEM04() const { return getZeroCal() * 2500; }

    uint32_t spi_read32();
    bool isValid(uint32_t raw_u32) const {
        return
            0u == (ERROR_MASK & raw_u32)
        &&  0u != raw_u32
        && ~0u != raw_u32;
    }
    bool isValid() const {
        return isValid(_thermocouple.raw32);
    }
    bool read() {
        _thermocouple.raw32 = spi_read32();
        if (!isValid()) {
          _errors++;
          _lastError.raw32 = _thermocouple.raw32;
          return false;
        }
        return true;
    }

    // Access to unprocessed sample data
    const struct MAX31855_BITMAP& getData() const { return _thermocouple.parse; }
    const union Thermocouple& getRawData() const { return _thermocouple; }
    const union Thermocouple& getLastError() const { return _lastError; }

    // Scaled Integer result for quick processing
    sint32_t getDeviceEM04() const;  // * 1.0E-04
    sint32_t getProbeEM04() const;  // * 1.0E-04
    //D sint32_t getVoutEM09() const;  // nV for Volts: * 1.0E-09

    // Float values not corrected for non-linear properties of thermocouple
    float getColdJunction() const { return (float)getDeviceEM04() * 1.0E-04; }
    float getHotJunction() const { return (float)getProbeEM04() * 1.0E-04; }
    //D float getVout() const { return (float)getVoutEM09() * 1.0E-09; }
    float getVout() const;

    // Float values corrected for non-linear properties of Type K thermocouple
    // using polynomial coefficients defined by ITS90
    // returns FLT_MAX, for conditions that do not allow processing
    float getITS90(const float cold_junction_c, const float vout_mv);
    float getITS90();
    float getC() { return getITS90(); };
    float getF() { return getITS90() * 1.8 + 32; };
    float getCelsius() { return getITS90(); };
    float getFahrenheit() { return getITS90() * 1.8 + 32; };

    uint32_t getErrorCount() const { return _errors; }
    void clearErrorCount() { _errors = 0; }

};
//
// End - MAX31855K Type K Thermocouple library
///////////////////////////////////////////////////////////////////////////////
#endif
