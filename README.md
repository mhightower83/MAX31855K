# WIP
## MAX31855K Library
Cloning not recommended at this time, may have a rebase in the future. Use the zip file for now.


At this time I leave all the minutiae details of this MAX31855K Library to the source code. Start with the `MAX31855K.h` file.

This library provides general access to probe and device temperatures as an integer value of the real value times 10K. Look for the functions ending in ...`X10K`. By multiplying the decimal value by 10,000, the result preserves the full precision of the value. This allows for the use of quick integer computations when needed. Remember to multiply by `1E-04` when used in a _float_ expression.

Floating-point values are also available. Functions are provided for converting the straight-line approximation slope-based values returned by the MAX31955K to a non-linear data corrected value based on the ITS90 polynomial [table](https://srdata.nist.gov/its90/download/allcoeff.tab) for the Type-K Thermocouple. See [NIST ITS-90 Thermocouple Database](https://srdata.nist.gov/its90/main/its90_main_page.html) for more details.

This Library is coded to support Arduino or Arduino ESP8266; however, I have only used it with the ESP8266.

Both hardware and software SPI Bus implementations are supported. 

On the ESP8266 when using HSPI with hardware CS, be very careful with GPIO15 as ChipSelect. For this to work, you must **not** have a pull-up resistor on that pin! If you do, your device most likely will not boot. See Troubleshooting - [ESP8266 Will Not Boot](https://github.com/mhightower83/MAX31855K/wiki/Troubleshooting#esp8266-will-not-boot) for specifics.

[Presision](https://github.com/mhightower83/MAX31855K/wiki/Precision)

[Boards](https://github.com/mhightower83/MAX31855K/wiki/Boards)

[Troubleshooting](Troubleshooting#troubleshooting)

[References](References)
