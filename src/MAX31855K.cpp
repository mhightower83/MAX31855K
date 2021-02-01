#include <Arduino.h>
#include <SPI.h>

#define DEBUG_SKETCH
#include <debugHelper.h>
#include "MAX31855K.h"

#define DIAG_PRINT

extern "C" uint32_t ets_get_cpu_frequency(void);

///////////////////////////////////////////////////////////////////////////////
// MAX31855K Type K Thermocoupler library
//
ALWAYS_INLINE void SPI_DELAY_100NS_MIN(void)
{
#if defined(ARDUINO_ARCH_ESP8266)
    // Approximately 12.5ns/cycle at 80MHz and 6.25ns/cycle for 160MHz.
    // It doesn't take long to reach 100ns.

    // This should put us on the high side for now.
    __asm__("extw; _nop; _nop;"); // each NOP is 37.5ns at 80MHz, +112.5ns
#if defined(F_CPU) && (F_CPU == 160000000L)
    __asm__("_nop; _nop; _nop;"); // each NOP is 18.75ns at 160MHz, +56.25ns
#endif
#else
    delayMicroseconds(1);
#endif
}

ALWAYS_INLINE void SPI_DELAY_200NS_MIN(void)
{
#if defined(ARDUINO_ARCH_ESP8266)
    SPI_DELAY_100NS_MIN();
    SPI_DELAY_100NS_MIN();
#else
    delayMicroseconds(1);
#endif
}

// MAX31855K::MAX31855K(){}

bool MAX31855K::begin(const SPIPins cfg)
{
    _cs         = cfg._cs;
    _sck        = cfg._sck;
    _miso       = cfg._miso;
    _mosi       = cfg._mosi;
    _hw         = cfg._hw;
    _softCs     = cfg._softCs;

    if (_hw) {
        _hw = SPI.pins(_sck, _miso, _mosi, _cs);
    }
    if (_hw) {
        DEBUG_PRINT("HW SPI Configured ");
        if (15 == _cs) {
            DEBUG_PRINT("w/HW CS");
            SPI.setHwCs(true);
        } else if (0 != _cs) {
            DEBUG_PRINT("w/SW CS");
            _softCs = true;
            digitalWrite(_cs, HIGH);
            pinMode(_cs, OUTPUT);
        }
        SPI.begin();
    } else {
        DEBUG_PRINT("Soft SPI Configured");
        digitalWrite(_cs, HIGH);
        pinMode(_cs, OUTPUT);
        digitalWrite(_sck, LOW);
        pinMode(_sck, OUTPUT);
        pinMode(_miso, INPUT);
    }
    DEBUG_PRINTF(", Pin Assignment: SCK=%d, MISO=%d, MOSI=%d, CS=%d\r\n", _sck, _miso, _mosi, _cs);
    SPI_DELAY_200NS_MIN();    // minimum CS# inactive time
    return readSample();
}

NOINLINE ICACHE_RAM_ATTR
static uint32_t _spi_read32(uint8_t _hw, uint8_t _sck, uint8_t _miso)
{
    union {
      uint32_t u32;
      uint8_t  u8[4];
    } value;

    if (_hw) {
        // MAX serial clock frequency 5MHz
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        for (ssize_t i = 3; i >= 0; i--) {
            value.u8[i] = SPI.transfer(0);
        }
        SPI.endTransaction();
    } else {
        for (size_t i = 0; i < 32; i++) {
            value.u32 <<= 1;
            if (digitalRead(_miso)) {
                value.u32 |= 1;
            }
            digitalWrite(_sck, HIGH);
            SPI_DELAY_100NS_MIN();
            digitalWrite(_sck, LOW);
            SPI_DELAY_100NS_MIN();
        }
    }
    return value.u32;
}
uint32_t MAX31855K::spi_read32(void)
{
    if (_softCs) {
        if (_hw) {
            if (LOW == digitalRead(_cs)) {
              digitalWrite(_cs, HIGH);
              SPI_DELAY_200NS_MIN();    // minimum CS# inactive time
            }
        } else {
            // Check if not in start state
            if (LOW == digitalRead(_cs)
            ||  LOW != digitalRead(_sck)) {
                digitalWrite(_cs, HIGH);
                digitalWrite(_sck, LOW);  // Start soft SPI with SCK low before CS#
                SPI_DELAY_200NS_MIN();    // minimum CS# inactive time
            }
        }
        digitalWrite(_cs, LOW);
        SPI_DELAY_100NS_MIN();
    }

    uint32_t value = _spi_read32(_hw, _sck, _miso);  // Keep the key parts in fast IRAM

    digitalWrite(_cs, HIGH);  // deselect slave
    /*
    TODO: check that 200ns minimum CS# inactive time will have passed if called twice
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
////////////////////////////////////////////////////////////////////////////////
// https://blog.heypete.com/2016/09/09/max31855-temperature-linearization/
// https://blog.heypete.com/wp-content/uploads/2016/09/MAX31855-Linearized-Thermocouple-Temperature.pdf
//
#define PRECISION double

void MAX31855K::sumCoPowT_I(PRECISION& v, const PRECISION& t, const PRECISION c[], const size_t& csz )
{
    // Number of coefficients.
    size_t sz = csz / sizeof(PRECISION);

    // For below 0 degrees Celsius, the NIST formula is simpler and has no
    // exponential components: E = sum(i=0 to n) c_i t^i
    for (size_t i = 0; i < sz; i++) {
        v += c[i] * pow(t, i);
    }
}

PRECISION MAX31855K::getLinearizedTemp(void)
{
    // Check to make sure thermocouple is working correctly.
    if (!isValid()) {
        return NAN;
    }

    const PRECISION internalTemp = (PRECISION)getDeviceE_04() / 10000; // Read the internal temperature of the MAX31855.
    // PRECISION rawTemp = (PRECISION)getProbeE_04() / 1000;       // Read the temperature of the thermocouple. This temp is compensated for cold junction temperature.
    PRECISION internalVoltage = 0;
    PRECISION correctedTemp = 0;
    PRECISION totalVoltage = 0;


    // Steps 1 & 2.
    // Subtract cold junction temperature from the raw  thermocouple
    // temperature.
    // thermocoupleVoltage = (rawTemp - internalTemp)*0.041276;  // C * mv/C = mV
    const PRECISION thermocoupleVoltage = (PRECISION)getVoutE_09() / 1000000.; // getVoutE_09() returns int in nV convert to mV.


    // Step 3. Calculate the cold junction equivalent thermocouple voltage.
    if (internalTemp >= 0) {
        // For positive temperatures use appropriate NIST coefficients
        // Coefficients and equations available from
        // http://srdata.nist.gov/its90/download/type_k.tab
        constexpr PRECISION c[] = {
            -0.176004136860E-01,  0.389212049750E-01,  0.185587700320E-04,
            -0.994575928740E-07,  0.318409457190E-09, -0.560728448890E-12,
             0.560750590590E-15, -0.320207200030E-18,  0.971511471520E-22,
            -0.121047212750E-25};

        // Count the the number of coefficients. There are 10 coefficients
        // for positive temperatures (plus three exponential coefficients),
        // but there are 11 coefficients for negative temperatures.
          // int cLength = sizeof(c) / sizeof(c[0]);

        // Exponential coefficients. Only used for positive temperatures.
        constexpr PRECISION a[] = {
             0.118597600000E+00, -0.118343200000E-03, 0.126968600000E+03};


        // From NIST: E = sum(i=0 to n) c_i t^i + a0 exp(a1 (t - a2)^2),
        // where E is the thermocouple voltage in mV and t is the
        // temperature in degrees C.
        //
        // In this case, E is the cold junction equivalent thermocouple voltage.
        // Alternative form:
        //   C0 + C1*internalTemp + C2*internalTemp^2 + C3*internalTemp^3 +
        //       ... + C10*internaltemp^10 + A0*e^(A1*(internalTemp - A2)^2)
        //
        // This loop sums up the c_i t^i components.
        sumCoPowT_I(internalVoltage, internalTemp, c, sizeof(c));

        // This section adds the a0 exp(a1 (t - a2)^2) components.
        internalVoltage += a[0] * exp(a[1] * pow((internalTemp - a[2]), 2));

    } else
    if (internalTemp < 0)
    { // for negative temperatures
        constexpr PRECISION c[] = {
             0.000000000000E+00,  0.394501280250E-01,  0.236223735980E-04,
            -0.328589067840E-06, -0.499048287770E-08, -0.675090591730E-10,
            -0.574103274280E-12, -0.310888728940E-14, -0.104516093650E-16,
            -0.198892668780E-19, -0.163226974860E-22};

        // Below 0 degrees Celsius, the NIST formula is simpler and has no
        // exponential components: E = sum(i=0 to n) c_i t^i
        sumCoPowT_I(internalVoltage, internalTemp, c, sizeof(c));
    }



    // Step 4. Add the cold junction equivalent thermocouple voltage calculated
    // in step 3 to the thermocouple voltage calculated in step 2.
    totalVoltage = thermocoupleVoltage + internalVoltage;



    // Step 5. Use the result of step 4 and the NIST voltage-to-temperature
    // (inverse) coefficients to calculate the cold junction compensated,
    // linearized temperature value.
    // The equation is in the form correctedTemp = d_0 + d_1*E + d_2*E^2 + ...
    //   + d_n*E^n,
    //   where E is the totalVoltage in mV and correctedTemp is in degrees C.
    // NIST uses different coefficients for different temperature subranges:
    //   (-200 to 0C), (0 to 500C) and (500 to 1372C).
    if (totalVoltage < 0)
    { // Temperature is between -200 and 0C.
        constexpr PRECISION d[] = {
             0.0000000E+00,  2.5173462E+01, -1.1662878E+00, -1.0833638E+00,
            -8.9773540E-01, -3.7342377E-01, -8.6632643E-02, -1.0450598E-02,
            -5.1920577E-04, 0.0000000E+00};

        sumCoPowT_I(correctedTemp, totalVoltage, d, sizeof(d));

    } else
    if (totalVoltage < 20.644)
    { // Temperature is between 0C and 500C.
        constexpr PRECISION d[] = {
             0.000000E+00,  2.508355E+01, 7.860106E-02, -2.503131E-01,
             8.315270E-02, -1.228034E-02, 9.804036E-04, -4.413030E-05,
             1.057734E-06, -1.052755E-08};

        sumCoPowT_I(correctedTemp, totalVoltage, d, sizeof(d));

    } else
    if (totalVoltage < 54.886 )
    { // Temperature is between 500C and 1372C.
        constexpr PRECISION d[] = {
            -1.318058E+02, 4.830222E+01, -1.646031E+00, 5.464731E-02,
            -9.650715E-04, 8.802193E-06, -3.110810E-08, 0.000000E+00,
             0.000000E+00, 0.000000E+00};

        sumCoPowT_I(correctedTemp, totalVoltage, d, sizeof(d));

    } else {
        // NIST only has data for K-type thermocouples from -200C to +1372C.
        // If the temperature is not in that range, set temp to impossible
        // value.
        // Error handling should be improved.
        Serial.print("Temperature is out of range. This should never happen.");
        correctedTemp = NAN;
    }

    return correctedTemp;
}
///////////////////////////////////////////////////////////////////////////////

void MAX31855K::sumCoPowT_I_f(PRECISION2& v, const PRECISION2& t, const PRECISION2 c[], const size_t& csz )
{
    // Number of coefficients.
    size_t sz = csz / sizeof(PRECISION2);

    // For below 0 degrees Celsius, the NIST formula is simpler and has no
    // exponential components: E = sum(i=0 to n) c_i t^i
    for (size_t i = 0; i < sz; i++) {
      if (i > 5) {
        v += (c[i] * pow(t, 4)) * pow(t, i-4);
      } else {
        v += c[i] * pow(t, i);
      }
    }
}

PRECISION2 MAX31855K::getLinearizedTemp_f(void)
{
    // Check to make sure thermocouple is working correctly.
    if (!isValid()) {
        return NAN;
    }

    const PRECISION2 internalTemp = (PRECISION2)getDeviceE_04() / 10000; // Read the internal temperature of the MAX31855.
    // PRECISION2 rawTemp = (PRECISION2)getProbeE_04() / 10000;       // Read the temperature of the thermocouple. This temp is compensated for cold junction temperature.
    PRECISION2 internalVoltage = 0;
    PRECISION2 correctedTemp = 0;
    PRECISION2 totalVoltage = 0;

    // Steps 1 & 2.
    // Subtract cold junction temperature from the raw  thermocouple
    // temperature.
    // thermocoupleVoltage = (rawTemp - internalTemp)*0.041276;  // C * mv/C = mV
    const PRECISION2 thermocoupleVoltage= (PRECISION2)getVoutE_09() / 1000000.; // getVoutE_09() returns nV convert to mV.

    // Step 3. Calculate the cold junction equivalent thermocouple voltage.

    if (internalTemp >= 0) {
        // For positive temperatures use appropriate NIST coefficients
        // Coefficients and equations available from
        // http://srdata.nist.gov/its90/download/type_k.tab
        constexpr PRECISION2 c[] = {
            -0.176004136860E-01,  0.389212049750E-01,  0.185587700320E-04,
            -0.994575928740E-07,  0.318409457190E-09, -0.560728448890E-12,
             0.560750590590E-15, -0.320207200030E-18,  0.971511471520E-22,
            -0.121047212750E-25};

        // Count the the number of coefficients. There are 10 coefficients
        // for positive temperatures (plus three exponential coefficients),
        // but there are 11 coefficients for negative temperatures.
          // int cLength = sizeof(c) / sizeof(c[0]);

        // Exponential coefficients. Only used for positive temperatures.
        constexpr PRECISION2 a[] = {
             0.118597600000E+00, -0.118343200000E-03, 0.126968600000E+03};


        // From NIST: E = sum(i=0 to n) c_i t^i + a0 exp(a1 (t - a2)^2),
        // where E is the thermocouple voltage in mV and t is the
        // temperature in degrees C.
        //
        // In this case, E is the cold junction equivalent thermocouple voltage.
        // Alternative form:
        //   C0 + C1*internalTemp + C2*internalTemp^2 + C3*internalTemp^3 +
        //       ... + C10*internaltemp^10 + A0*e^(A1*(internalTemp - A2)^2)
        //
        // This loop sums up the c_i t^i components.
        sumCoPowT_I_f(internalVoltage, internalTemp, c, sizeof(c));

        // This section adds the a0 exp(a1 (t - a2)^2) components.
        internalVoltage += a[0] * exp(a[1] * pow((internalTemp - a[2]), 2));

    } else
    if (internalTemp < 0) { // for negative temperatures
        constexpr PRECISION2 c[] = {
             0.000000000000E+00,  0.394501280250E-01,  0.236223735980E-04,
            -0.328589067840E-06, -0.499048287770E-08, -0.675090591730E-10,
            -0.574103274280E-12, -0.310888728940E-14, -0.104516093650E-16,
            -0.198892668780E-19, -0.163226974860E-22};

        // Below 0 degrees Celsius, the NIST formula is simpler and has no
        // exponential components: E = sum(i=0 to n) c_i t^i
        sumCoPowT_I_f(internalVoltage, internalTemp, c, sizeof(c));
    }

    // Step 4. Add the cold junction equivalent thermocouple voltage
    // calculated in step 3 to the thermocouple voltage calculated in step
    // 2.
    totalVoltage = thermocoupleVoltage + internalVoltage;

    // Step 5. Use the result of step 4 and the NIST voltage-to-temperature
    // (inverse) coefficients to calculate the cold junction compensated,
    // linearized temperature value.
    // The equation is in the form correctedTemp = d_0 + d_1*E + d_2*E^2 + ...
    //   + d_n*E^n,
    //   where E is the totalVoltage in mV and correctedTemp is in degrees C.
    // NIST uses different coefficients for different temperature subranges:
    //   (-200 to 0C), (0 to 500C) and (500 to 1372C).
    if (totalVoltage < 0) { // Temperature is between -200 and 0C.
        constexpr PRECISION2 d[] = {
             0.0000000E+00,  2.5173462E+01, -1.1662878E+00, -1.0833638E+00,
            -8.9773540E-01, -3.7342377E-01, -8.6632643E-02, -1.0450598E-02,
            -5.1920577E-04, 0.0000000E+00};

        sumCoPowT_I_f(correctedTemp, totalVoltage, d, sizeof(d));

    } else
    if (totalVoltage < 20.644) { // Temperature is between 0C and 500C.
        constexpr PRECISION2 d[] = {
             0.000000E+00,  2.508355E+01, 7.860106E-02, -2.503131E-01,
             8.315270E-02, -1.228034E-02, 9.804036E-04, -4.413030E-05,
             1.057734E-06, -1.052755E-08};

        sumCoPowT_I_f(correctedTemp, totalVoltage, d, sizeof(d));

    } else
    if (totalVoltage < 54.886 ) { // Temperature is between 500C and 1372C.
        constexpr PRECISION2 d[] = {
            -1.318058E+02, 4.830222E+01, -1.646031E+00, 5.464731E-02,
            -9.650715E-04, 8.802193E-06, -3.110810E-08, 0.000000E+00,
             0.000000E+00, 0.000000E+00};

        sumCoPowT_I_f(correctedTemp, totalVoltage, d, sizeof(d));

    } else {
        // NIST only has data for K-type thermocouples from -200C to +1372C.
        // If the temperature is not in that range, set temp to impossible
        // value.
        // Error handling should be improved.
        Serial.print("Temperature is out of range. This should never happen.");
        correctedTemp = NAN;
    }

    return correctedTemp;
}

MAX31855K max31855k;

///////////////////////////////////////////////////////////////////////////////
//
// End - MAX31855K Type K Thermocoupler library
///////////////////////////////////////////////////////////////////////////////

#if 0

static uint32_t last;

bool max31855Init(const uint8_t cs, const uint8_t sck,
                  const uint8_t miso, const bool swap_leads)
{
    // bool success =
    // max31855k.begin(SPIPins());              // HW SPI
    // max31855k.begin(SPIPins(cs));            // HW SPI w/software CS

    // This should now select HW SPI when the pin assignments match up
    bool success =
    max31855k.begin(SPIPins(cs, sck, miso, 0)); //13)); // Software SPI
    max31855k.setSwapLeads(swap_leads);
    max31855k.setZeroCal(0);
    if (!success) {
        CONSOLE_PRINTLN("MAX31855K did not start, check pin configuration. Will try later.");
    }
    CONSOLE.println();
    last = millis();
    return success;
}

bool max31855Loop(uint32_t interval_ms) {
    if (interval_ms) {
        uint32_t now = millis();
        if ((now - last) < interval_ms) {
            return false;
        }
        last = now;
    }

    bool success = max31855k.readSample();
    if (!success) {
      CONSOLE_PRINTLN("MAX31855 sample read erroro");
    }
    return true;
}

#define DIAG_PRINT
void printReport() {
    const struct MAX31855_BITMAP& parse = max31855k.getData();
    if (max31855k.isValid()) {
        CONSOLE_PRINTLN2("Probe Temperature is  ", (max31855k.getProbeE_04()) + SF(" oC / 10000"));
        CONSOLE_PRINTLN2("Device Temperature is ", (max31855k.getDeviceE_04()) + SF(" oC / 10000")); // degree symbol "\xC2\xB0" causes print delay
        CONSOLE.println();

        // sint32_t probe   = parse.probe * 250;
        // sint32_t internal = parse.internal * 625 / 10;
        // probe = (internal - probe) + internal;  // Reverse leads - Invert the delta temperature
        // CONSOLE_PRINTLN2("Probe Temperature with leads reversed is ", (probe) + SF(" oC / 1000"));

        sint32_t vout = 41276 * (max31855k.getProbeE_04() - max31855k.getDeviceE_04()) / 10000;
        CONSOLE_PRINTLN2("Vout: ", (vout) + SF(" nV"));
        CONSOLE_PRINTLN2("Vout: ", (max31855k.getVoutE_09()) + SF(" nV"));
        CONSOLE_PRINTLN2("Error Count: ", (max31855k.getErrorCount()) );

        CONSOLE_PRINTLN2("Corrected Probe Temperature PRECISION  ", String(max31855k.getLinearizedTemp(), 9) + SF(" oC "));
        CONSOLE_PRINTLN2("Corrected Probe Temperature PRECISION2 ", String(max31855k.getLinearizedTemp_f(), 9) + SF(" oC "));
        CONSOLE.println();
        #if 0
        CONSOLE_PRINTLN2("Device Temperature is ", String((float)(parse.internal * 625 /10) / 1000, 3) + SF("\xC2\xB0""C"));
        CONSOLE_PRINTLN2("Probe Temperature is  ", String((float)(parse.probe * 250      ) / 1000, 3) + SF("\xC2\xB0""C"));
        #endif
    }
#ifdef DIAG_PRINT
    else {
        if (~0u == max31855k.getRaw32()) {
            CONSOLE_PRINTLN("Internal: Check SPI Bus pin assignments and connections, reading all ones.");
        } else
        if (0u == max31855k.getRaw32()) {
            CONSOLE_PRINTLN("Internal: Check SPI Bus pin assignments and connections, reading all zeros.");
        } else
        if (!(0 == parse.res && 0 == parse.res2)) {
            CONSOLE_PRINTLN("Internal: Check SPI Bus pin assignments and connections, parse.res == 0 && parse.res2 == 0 test failed.");
        } else
        if (parse.fault) {
            if (parse.oc) {
                // Onece design is working, this is the only error that should be seen.
                CONSOLE_PRINTLN("Fault: Open-circuit, check thermocouple connection and wiring.");
            }
            if (parse.scg) {
                CONSOLE_PRINTLN("Internal Fault: Short-circuit to ground, check internal wiring");
            }
            if (parse.scv) {
                CONSOLE_PRINTLN("Internal Fault: Short-circuit to VCC, check internal wiring");
            }
            if (!(parse.oc || parse.scg || parse.scv)) {
                CONSOLE_PRINTLN("Internal: Fantom fault set without parse.oc || parse.scg || parse.scv being set");
            }
        } else
        if (!(parse.oc || parse.scg || parse.scv)) {
            CONSOLE_PRINTLN("Internal: unknown error, parse.fault not set");
        }
    }
#endif
}
#endif
