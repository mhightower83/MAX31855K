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

///////////////////////////////////////////////////////////////////////////////
// MAX31855K Type K Thermocoupler library
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

class SPIPins {
public:
    uint8_t _cs;
    uint8_t _sck;
    uint8_t _miso;
    uint8_t _mosi;    // Not used, HW SPI will require it to match and be reserved.
    uint8_t _hw;
    uint8_t _softCs;

    SPIPins(uint32_t cs=15, uint8_t sck=14, uint8_t miso=12, uint8_t mosi=13, bool autoHw=true)
    : _cs(cs)
    , _sck(sck)
    , _miso(miso)
    , _mosi(mosi) {
        _hw         = false;
        _softCs     = true;
        // Auto select HW SPI if pins are correct.
        // MAX31855 is a read-only SPI device, MOSI is only needed to complete
        // the Hardware SPI interface selection vs Sofware driven SPI interface.
        if (autoHw && 14 == _sck && 12 == _miso && 13 == mosi) {
            _hw = true;
            if (15 == _cs) {
              _softCs = false;
            }
        }
    }
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
    sint32_t _getProbeE_04() const;  // * 1.0E-04
    sint32_t _getDeviceE_04() const;  // * 1.0E-04

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


    MAX31855K() { _swap_leads = false; _zero_cal = 0; _errors = 0; };
    bool begin(const SPIPins cfg);
    void setSwapLeads(bool b) { _swap_leads = b; }
    sint32_t getSwapLeads() { return _swap_leads; }
    void setZeroCal(sint32_t cal) { _zero_cal = cal; }
    void setZeroCal() { _zero_cal = _thermocouple.parse.probe; }
    sint32_t getZeroCal() const { return _zero_cal; }
    sint32_t getZeroCalE_04() const { return _zero_cal * 2500; }
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
    bool readSample() {
        _thermocouple.raw32 = spi_read32();
        if (!isValid()) {
          _errors++;
          _lastError.raw32 = _thermocouple.raw32;
          return false;
        }
        return true;
    }
    const SPIPins& getSPIPins() const { return _pins; }
    const struct MAX31855_BITMAP& getData() const { return _thermocouple.parse; }
    const union Thermocouple& getSample() const { return _thermocouple; }
    const union Thermocouple& getLastError() const { return _lastError; }
    uint32_t getRaw32() const { return _thermocouple.raw32; }
    sint32_t getDeviceE_04() const;  // * 1.0E-04
    sint32_t getProbeE_04() const;  // * 1.0E-04
    sint32_t getVoutE_09() const;  // nV for Volts: * 1.0E-09
    uint32_t getErrorCount() const { return _errors; }
    void clearErrorCount() { _errors = 0; }
    float getITS90();         // returns FLT_MAX, for conditions that do not allow processing.
    float getLinearizedTemp() { return getITS90(); };
    float getC() { return getLinearizedTemp(); };
    float getF() { return getLinearizedTemp() * 1.8 + 32; };
    float getCelsius() { return getC(); };
    float getFahrenheit() { return getF(); };

};
//
// extern MAX31855K max31855k;

//
// End - MAX31855K Type K Thermocoupler library
///////////////////////////////////////////////////////////////////////////////
#endif
