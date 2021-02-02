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

#include <Arduino.h>    // needed for printing
#include <cmath>
#include <cfloat>
#include <pgmspace.h>  // When present, ITS90 tables will be stored in PROGMEM.
// #include <debugHelper.h>
#include <TypeK_ITS90.h>
#include <TypeK_ITS90_table.h>

// Fail safe defines, for when debugHelper.h is not present
#ifndef CONSOLE_PRINTF
#define CONSOLE_PRINTF(a, ...) Serial.printf_P(PSTR(a), ##__VA_ARGS__)
#endif

/*
  The equation for the above 0 °C case is of the form
  E = sum(i=0 to n) c_i t^i + a0 exp(a1 (t - a2)^2).
*/
static DFLOAT exponential_part(DFLOAT t)
{
  return a0 * exp(a1 * pow((t - a2), 2));
}

/*
  This summation function is common to converting Celsius to mV and
  the inverse mV to Celsius.
    E = sum(i=0 to n) c_i t^i for degrees C less than 0 or
    t_90 = d_0 + d_1*E + d_2*E^2 + ... + d_n*E^n,
    etc.
*/
static DFLOAT sum_C_i_X_pow_T_i(const DFLOAT *c, const DFLOAT t, const size_t n)
{
    DFLOAT sum = c[0];
    for (size_t i = 1; i < n; ++i) {
        sum += c[i] * pow(t, i);
    }
    return sum;
}

DFLOAT type_k_celsius_to_mv(const DFLOAT t)
{
    const DFLOAT *c = c_m270;    // -270.000 to 0.000 °C
    DFLOAT n = sizeof(c_m270) / sizeof(DFLOAT);
    DFLOAT mv = 0;

    if (0 <= t) {
        c = c_0;                // 0.000 to 1372.000 °C
        n = sizeof(c_0) / sizeof(DFLOAT);
        mv = exponential_part(t);
    }

    mv += sum_C_i_X_pow_T_i(c, t, n);
    return mv;
}

DFLOAT type_k_mv_to_celsius(const DFLOAT mv)
{
    const DFLOAT *d = d_0_21;    //  0.000 to 20.644 mV | 0. to 500. °C
    DFLOAT n = sizeof(d_0_21) / sizeof(DFLOAT);

    if (0 > mv) {
        d = d_m6_0;             // -5.891 to 0.000 | -200. to 0. °C
        n = sizeof(d_m6_0) / sizeof(DFLOAT);
    } else
    if (20.644 < mv) {
        d = d_21_55;            // 20.644 to 54.886 mV | 500. to 1372. °C
        n = sizeof(d_21_55) / sizeof(DFLOAT);
    }
    return sum_C_i_X_pow_T_i(d, mv, n);
}


///////////////////////////////////
// Test data
constexpr DFLOAT c_mv_data[] PROGMEM = {
//  degrees C     mV
                          // only C to mV is handled this low
    -270.001,  -6.458,
    -270.000,  -6.458,
    -260.000,  -6.441,
    -210.000,  -6.035,
                          // not accurate for mV to C conversion below -5.891 mV
                          // From here down both directions are supported
    -200.000,  -5.891,
    -100.000,  -3.554,
     -10.000,  -0.392,
       0.000,   0.000,
      10.000,   0.397,
      20.000,   0.798,
      30.000,   1.203,
      40.000,   1.612,
      50.000,   2.023,
      60.000,   2.436,
      70.000,   2.851,
      80.000,   3.267,
      90.000,   3.682,
     100.000,   4.096,
     200.000,   8.138,
     300.000,  12.209,
     310.000,  12.624,
     400.000,  16.397,
     500.000,  20.644,
     600.000,  24.905,
     610.000,  25.330,
     700.000,  29.129,
     800.000,  33.275,
     900.000,  37.326,
     910.000,  37.725,
    1000.000,  41.276,
    1100.000,  45.119,
    1200.000,  48.838,
    1210.000,  49.202,
    1300.000,  52.410,
    1372.000,  54.886,
    1372.001,  54.886
};
bool test_with_edge_data() {
    constexpr size_t n = sizeof(c_mv_data)/sizeof(DFLOAT);

    for (size_t i = 0; i < n; i += 2) {
      DFLOAT mV = type_k_celsius_to_mv(c_mv_data[i]);
      DFLOAT t = type_k_mv_to_celsius(c_mv_data[i + 1]);
      CONSOLE_PRINTF("  %8.3f Celsius to %8.3f mV, expected %8.3f mV, convert expected mV to %8.3f Celsius\r\n",
                      c_mv_data[i], mV, c_mv_data[i + 1], t);
    }

    return true;
}

//
///////////////////////////////////////////////////////////////////////////////
