#include <Arduino.h>
//#include <debugHelper.h>
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

#include <MAX31855K.h>
#include <TypeK_ITS90.h>
MAX31855K max31855k;

static uint32_t last_sample;
static uint32_t last;
static bool sample_available;       // A valid sample reading is available
static bool sample_update = false;  // Set when new sample is read, success or fail
static uint32_t sample_count = 0;   // number of samples read
static uint32_t sample_bad_count = 0;

bool diagPrintError(Print& out, const union MAX31855K::Thermocouple& sample);
void printReport(Print& out);
void processKey(Print& out, int hotKey);

bool max31855Init(const uint8_t cs, const uint8_t sck,
                  const uint8_t miso, const bool swap_leads)
{
    // bool success =
    // max31855k.begin(SpiPinConfig());              // HW SPI
    // max31855k.begin(SpiPinConfig(cs));            // HW SPI w/software CS

    // This should now select HW SPI when the pin assignments match up
    bool success =
    max31855k.begin(SPIPins(cs, sck, miso, 0)); //13)); // Software SPI
    max31855k.setSwapLeads(swap_leads);
    max31855k.setZeroCal(0);
    if (!success) {
        CONSOLE_PRINTLN("MAX31855K did not start, check pin configuration. Will try later.");
    }
    CONSOLE.println();
    last_sample = last = millis();
    return success;
}

bool max31855Loop(uint32_t interval_ms) {
    /*
      Temperature conversion time takes 70ms to 100ms
    */
    constexpr uint32_t tConv = 100; // Conversion time in ms

    if (tConv > interval_ms) {
        interval_ms = tConv;
    }

    uint32_t now = millis();
    if ((now - last_sample) < interval_ms) {
        return false;
    }

    sample_available = max31855k.read();
    sample_update = true;
    if (sample_available) {
        sample_count++;
    } else {
        sample_bad_count++;
    }
    last_sample = millis();

    return true;
}

// uint32_t lastErrorCount = 0;
// void monitorErrorCount() {
//     uint32_t currentCount = max31855k.getErrorCount();
//     if (currentCount != lastErrorCount) {
//         lastErrorCount = currentCount;
//         diagPrintError(CONSOLE, max31855k.getLastError());
//     }
// }

void printPins(Print& out) {
    SPIPins pins = max31855k.getSPIPins();
    out._PRINTLN("SPI Bus configuration:");
    out._PRINTF( "  Pin Assignment:      SCK=%d, MISO=%d, MOSI=%d, CS=%d\r\n",
                   pins.sck, pins.miso, pins.mosi, pins.cs);
    out._PRINTF( "  Bus support:         %s\r\n", (pins.hw) ? "Hardware" : "Software");
    out._PRINTF( "  Chip Select support: %s\r\n", (pins.softCs) ? "Software" : "Hardware");
}

void setup() {
    CONSOLE.begin(115200);
    delay(15);               // Give ESP8266 Modified CONSOLE Monitor time to enable

    CONSOLE.printf("\r\nLet the Show begin:\r\n");

    LOG_FAIL(
        max31855Init(/* CS */ 5, /* SCK */ 14, /* MISO */ 12, /* SwapLeads */ true),
        "max31855Init: Failed: 0x%08X", max31855k.getData().raw32
    );

    last = millis();

}

uint32_t report_interval_ms = 5000;

void loop() {
    max31855Loop(0);

    // Handle inferred read errors as they occur.
    if (!sample_available && sample_update) {
        diagPrintError(CONSOLE, max31855k.getData());
        sample_update = false;
    }

    if (report_interval_ms && sample_available) {
        uint32_t now = millis();
        if ((now - last) > report_interval_ms) {
            printReport(CONSOLE);
            sample_available = false;
            sample_update = false;
            last = now;
        }
    }

    // monitorErrorCount();

    if (CONSOLE.available() > 0) {
        int hotKey = CONSOLE.read();
        processKey(CONSOLE, hotKey);
    }
    // ...
}

bool diagPrintError(Print& out, const union MAX31855K::Thermocouple& sample) {
    const struct MAX31855_BITMAP& parse = sample.parse;

    if (max31855k.isValid(sample.raw32)) {
        out._PRINTLN("No errors detected.");
        return false;     // No error, nothing was printed

    } else {
        if (~0u == sample.raw32) {
            out._PRINTLN("Internal: Check SPI Bus pin assignments and connections, reading all ones.");
        } else
        if (0u == sample.raw32) {
            out._PRINTLN("Internal: Check SPI Bus pin assignments and connections, reading all zeros.");
        } else
        if (!(0 == parse.res && 0 == parse.res2)) {
            out._PRINTLN("Internal: Check SPI Bus pin assignments and connections, parse.res == 0 && parse.res2 == 0 test failed.");
        } else
        if (parse.fault) {
            if (parse.oc) {
                // Onece design is working, this is the only error that should be seen.
                out._PRINTLN("Fault: OC, Open-circuit, check thermocouple connection and wiring.");
            }
            if (parse.scg) {
                out._PRINTLN("Internal Fault: SCG, Short-circuit to ground, check internal wiring");
            }
            if (parse.scv) {
                out._PRINTLN("Internal Fault: SCV, Short-circuit to VCC, check internal wiring");
            }
            if (!(parse.oc || parse.scg || parse.scv)) {
                out._PRINTLN("Internal: Fantom fault set without flags OC, SCG, or SCV being set");
            }
        } else
        if (!(parse.oc || parse.scg || parse.scv)) {
            out._PRINTLN("Internal: unknown error, parse.fault not set");
        }
        out._PRINTLN2("Total Error Count: ", (max31855k.getErrorCount()) );
    }
    return true;
}


#define DIAG_PRINT
#ifdef DIAG_PRINT
#endif

void printReport(Print& out) {
    // const struct MAX31855_BITMAP& parse = max31855k.parceData();
    if (max31855k.isValid()) {
        out._PRINTLN("Report on current MAX31855K sample.");
        out._PRINTF("  Compensated Probe (Thermocouple) Temperature %8.4f ""\xC2\xB0""C\r\n", (DFLOAT)max31855k.getProbeX10K()  / 10000.);
        out._PRINTF("  Device Internal (Cold-Junction) Temperature  %8.4f ""\xC2\xB0""C\r\n", (DFLOAT)max31855k.getDeviceX10K() / 10000.);
        DFLOAT probeTempC = max31855k.getCelsius();
        if (FLT_MAX == probeTempC) {
            out._PRINTF("  Linearized Probe Temperature                 NaN\r\n");
        } else {
            out._PRINTF("  Linearized Probe Temperature                 %8.4f ""\xC2\xB0""C\r\n", probeTempC);
            out._PRINTF("  Linearized Probe Temperature                 %8.4f ""\xC2\xB0""F\r\n", probeTempC * 1.8 + 32);
        }
        out.println();
        DFLOAT mV = max31855k.getHotJunctionVout() * 1.0E03;
        out._PRINTF("  Hot-Junction thermocouple Vout:     %9.4f mV\r\n", mV);
        DFLOAT cold_junction_mV = type_k_celsius_to_mv(max31855k.getColdJunction()); // same as ?? max31855k.getDeviceX10K() / 10000.);
        out._PRINTF("  Cold-Junction thermocouple Vout:    %9.4f mV\r\n", cold_junction_mV);
        mV += cold_junction_mV;
        out._PRINTF("  Total Vout:                         %9.4f mV\r\n", mV);
        DFLOAT c = type_k_mv_to_celsius(mV);
        out._PRINTF("  type_k_mv_to_celsius(%9.4f)     %9.4f ""\xC2\xB0""C\r\n", mV, c);
{
        sint32_t nonlinearC = max31855k.convertC2ProbeX10K(c, max31855k.getColdJunction());
        DFLOAT nonlinearCf = nonlinearC / 10000.;
        out._PRINTF("    convertC2ProbeX10K(  %9.4f)     %9.4f ""\xC2\xB0""C, type_k_mv_to_celsius(%9.4f)\r\n",
                    c, nonlinearCf, mV);
        DFLOAT hot_junction_mV = 41.276E-06 * (nonlinearCf - max31855k.getColdJunction()) * 1.0E03;
        DFLOAT cold_junction_mV = type_k_celsius_to_mv(max31855k.getColdJunction());
        DFLOAT totalVoutMv = hot_junction_mV + cold_junction_mV;
        DFLOAT c2 = type_k_mv_to_celsius(totalVoutMv);
        out._PRINTF("    type_k_mv_to_celsius(%9.4f)     %9.4f ""\xC2\xB0""C\r\n", totalVoutMv, c2);


        const DFLOAT vc = type_k_celsius_to_mv(1000.) * 1.0E-03 / 1000.;
        out._PRINTF("    Range: 0 - 1000 ""\xC2\xB0""C,           %14.8f uV/""\xC2\xB0""C\r\n", vc * 1.0E06);
}
    // // const DFLOAT vc = type_k_celsius_to_mv(1000.) * 1.0E-03 / 1000.;
    //     DFLOAT ct = type_k_celsius_to_mv(c) * 1.0E-03 / 41.276E-06 * 1.0E04;  // * 1.0E-04
    //     out._PRINTF("  convertC2ProbeX10K(  %9.4f)     %9.4f ""\xC2\xB0""C, type_k_mv_to_celsius(%9.4f)\r\n", c, ct / 10000., mV);


        mV = type_k_celsius_to_mv(c);
        out._PRINTF("  type_k_celsius_to_mv(%9.4f)     %9.4f mV\r\n", c, mV);
        c = type_k_mv_to_celsius(mV);
        out._PRINTF("  type_k_mv_to_celsius(%9.4f)     %9.4f ""\xC2\xB0""C\r\n", mV, c);
        mV = type_k_celsius_to_mv(c);
        out._PRINTF("  type_k_celsius_to_mv(%9.4f)     %9.4f mV\r\n", c, mV);
        DFLOAT vout = 41.276E-06 * (max31855k.getProbeX10K() - max31855k.getDeviceX10K()) * 1.0E-04;
        out._PRINTF("  Hot-Junction thermocouple Vout_cmp: %9.4f mV\r\n", (vout * 1.0E03));
        out.println();

        out._PRINTLN2("  sample_count:      ", (sample_count) );
        out._PRINTLN2("  sample_bad_count:  ", (sample_bad_count) );
        out._PRINTLN2("  getErrorCount():   ", (max31855k.getErrorCount()) );
        out.println();
    } else {
      diagPrintError(out, max31855k.getData());
    }
}
