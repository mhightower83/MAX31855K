#include <Arduino.h>
#include <debugHelper.h>
#include <MAX31855K.h>
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
        "max31855Init: Failed: 0x%08X", max31855k.getRawData().raw32
    );

    last = millis();

}

uint32_t report_interval_ms = 5000;

void loop() {
    max31855Loop(0);

    // Handle inferred read errors as they occur.
    if (!sample_available && sample_update) {
        diagPrintError(CONSOLE, max31855k.getRawData());
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
    // const struct MAX31855_BITMAP& parse = max31855k.getData();
    if (max31855k.isValid()) {
        out._PRINTLN("///////////////////////////////////////////////////////////////////////////////////////////////////");
        out._PRINTLN2("Compensated Probe (Thermocouple) Temperature ", String((float)max31855k.getProbeEM04()  / 10000., 8) + SF(" oC"));
        out._PRINTLN2("Device Internal (Cold-Junction) Temperature  ", String((float)max31855k.getDeviceEM04() / 10000., 8) + SF(" oC")); // degree symbol "\xC2\xB0" causes print delay
        float probeTempC = max31855k.getC();
        if (FLT_MAX == probeTempC) {
            out._PRINTLN2("Linearized Probe Temperature                 ", F("-overflow-"));
        } else {
            out._PRINTLN2("Linearized Probe Temperature                 ", String(probeTempC, 8) + SF(" oC "));
            out._PRINTLN2("Linearized Probe Temperature                 ", String(probeTempC*1.8 + 32, 8) + SF(" oF "));
        }
        out.println();

        float vout = 41.276E-06 * (max31855k.getProbeEM04() - max31855k.getDeviceEM04()) / 1.0E-04;
        out._PRINTLN2("Vout: ", (vout*1000, 4) + SF(" mV"));
        out._PRINTLN2("Vout: ", (max31855k.getVout()*1000, 4) + SF(" mV"));
        out.println();

        out._PRINTLN2("sample_count:      ", (sample_count) );
        out._PRINTLN2("Ssample_bad_count: ", (sample_bad_count) );
        out._PRINTLN2("getErrorCount():   ", (max31855k.getErrorCount()) );
        out.println();
    } else {
      diagPrintError(out, max31855k.getRawData());
    }
}
