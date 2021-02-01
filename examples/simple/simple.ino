#include <Arduino.h>
#include <debugHelper.h>
#include <MAX31855K.h>


static uint32_t last;
bool diagPrintError(const union MAX31855K::Thermocouple& sample);
void printReport();

bool max31855Init(const uint8_t cs, const uint8_t sck,
                  const uint8_t miso, const bool swap_leads)
{
    // bool success =
    // max31855k.begin(SpiPinConfig());              // HW SPI
    // max31855k.begin(SpiPinConfig(cs));            // HW SPI w/software CS

    // This should now select HW SPI when the pin assignments match up
    bool success =
    max31855k.begin(SpiPinConfig(cs, sck, miso, 0)); //13)); // Software SPI
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

uint32_t lastErrorCount = 0;
void monitorErrorCount() {
    uint32_t currentCount = max31855k.getErrorCount();
    if (currentCount != lastErrorCount) {
        lastErrorCount = currentCount;
        diagPrintError(max31855k.getLastError());
    }
}


void setup() {
    Serial.begin(115200);
    delay(15);               // Give ESP8266 Modified Serial Monitor time to enable

    Serial.printf("\r\nLet the Show begin:\r\n");

    LOG_FAIL(
        max31855Init(/* CS */ 5, /* SCK */ 14, /* MISO */ 12, /* SwapLeads */ true),
        "max31855Init: Failed: 0x%08X", max31855k.getRaw32()
    );

    last = millis();

}

uint32_t interval_ms = 5000;

void loop() {
    max31855Loop(0);

    if (interval_ms) {
        uint32_t now = millis();
        if ((now - last) > interval_ms) {
            printReport();
            last = now;
        }
    }

    monitorErrorCount();

    if (Serial.available() > 0) {
        int hotKey = Serial.read();
        processKey(Serial, hotKey);
    }
    // ...
}

bool diagPrintError(const union MAX31855K::Thermocouple& sample) {
    const struct MAX31855_BITMAP& parse = sample.parse;

    if (max31855k.isValid(sample.raw32)) {
        CONSOLE_PRINTLN("No errors detected.");
        return false;     // No error, nothing was printed

    } else {
        if (~0u == sample.raw32) {
            CONSOLE_PRINTLN("Internal: Check SPI Bus pin assignments and connections, reading all ones.");
        } else
        if (0u == sample.raw32) {
            CONSOLE_PRINTLN("Internal: Check SPI Bus pin assignments and connections, reading all zeros.");
        } else
        if (!(0 == parse.res && 0 == parse.res2)) {
            CONSOLE_PRINTLN("Internal: Check SPI Bus pin assignments and connections, parse.res == 0 && parse.res2 == 0 test failed.");
        } else
        if (parse.fault) {
            if (parse.oc) {
                // Onece design is working, this is the only error that should be seen.
                CONSOLE_PRINTLN("Fault: OC, Open-circuit, check thermocouple connection and wiring.");
            }
            if (parse.scg) {
                CONSOLE_PRINTLN("Internal Fault: SCG, Short-circuit to ground, check internal wiring");
            }
            if (parse.scv) {
                CONSOLE_PRINTLN("Internal Fault: SCV, Short-circuit to VCC, check internal wiring");
            }
            if (!(parse.oc || parse.scg || parse.scv)) {
                CONSOLE_PRINTLN("Internal: Fantom fault set without flags OC, SCG, or SCV being set");
            }
        } else
        if (!(parse.oc || parse.scg || parse.scv)) {
            CONSOLE_PRINTLN("Internal: unknown error, parse.fault not set");
        }
        CONSOLE_PRINTLN2("Total Error Count: ", (max31855k.getErrorCount()) );
    }
    return true;
}


#define DIAG_PRINT
#ifdef DIAG_PRINT
#endif

void printReport() {
    // const struct MAX31855_BITMAP& parse = max31855k.getData();
    if (max31855k.isValid()) {
        CONSOLE_PRINTLN("///////////////////////////////////////////////////////////////////////////////////////////////////");
        CONSOLE_PRINTLN2("Compensated Probe (Thermocouple) Temperature ", String((float)max31855k.getProbeE_04()  / 10000., 8) + SF(" oC"));
        CONSOLE_PRINTLN2("Device Internal (Cold-Junction) Temperature  ", String((float)max31855k.getDeviceE_04() / 10000., 8) + SF(" oC")); // degree symbol "\xC2\xB0" causes print delay
        CONSOLE_PRINTLN2("Linearized Probe Temperature PRECISION       ", String(max31855k.getLinearizedTemp(), 8) + SF(" oC "));
        CONSOLE_PRINTLN2("Linearized Probe Temperature PRECISION2      ", String(max31855k.getLinearizedTemp_f(), 8) + SF(" oC "));
        CONSOLE_PRINTLN2("Linearized Probe Temperature PRECISION       ", String(max31855k.getLinearizedTemp()*1.8 + 32, 8) + SF(" oF "));
        CONSOLE_PRINTLN2("Linearized Probe Temperature PRECISION2      ", String(max31855k.getLinearizedTemp_f()*1.8 + 32, 8) + SF(" oF "));
        CONSOLE.println();

        sint32_t vout = 41276 * (max31855k.getProbeE_04() - max31855k.getDeviceE_04()) / 10000;
        CONSOLE_PRINTLN2("Vout: ", (vout) + SF(" nV"));
        CONSOLE_PRINTLN2("Vout: ", (max31855k.getVoutE_09()) + SF(" nV"));
        CONSOLE.println();

        CONSOLE_PRINTLN2("Error Count: ", (max31855k.getErrorCount()) );
        CONSOLE.println();
    } else {
      diagPrintError(max31855k.getSample());
    }
}
