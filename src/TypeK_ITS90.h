#ifndef TYPEK_ITS90_H
#define TYPEK_ITS90_H
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

inline constexpr bool is_type_k_mv_in_range(const float mV) {
  // Note these equations only handle temperatures down to -200 not -270
  return (54.886 >= mV && -5.891 <= mV);
}

inline constexpr bool is_type_k_t_in_range(const float t) {
  return (1372.000 >= t && -270.000 <= t);
}

float type_k_mv_to_celsius(const float mV);

float type_k_celsius_to_mv(const float t);

bool test_with_edge_data();

#endif
