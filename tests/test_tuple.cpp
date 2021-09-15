// Copyright 2021 Francois Chabot

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "abu/types.h"
#include "gtest/gtest.h"

TEST(tuple, empty_comparisons) {
  using T = abu::tuple<>;

  static_assert(abu::tuple_size<T>::value == 0);

  T x;
  T y;

  EXPECT_EQ(x, y);
}

TEST(tuple, single_value_comparisons) {
  using T = abu::tuple<int>;

  static_assert(abu::tuple_size<T>::value == 1);

  T x{0};
  T y{12};

  EXPECT_EQ(get<0>(x), 0);
  EXPECT_EQ(get<0>(y), 12);

  EXPECT_EQ(get<int>(x), 0);
  EXPECT_EQ(get<int>(y), 12);

  EXPECT_NE(x, y);
  EXPECT_GT(y, x);
}

TEST(tuple, multiple_value_comparisons) {
  using T = abu::tuple<int, float>;

  T defaulted;
  T explicitely_defaulted{};
  T explicitely_list_defaulted = {};

  EXPECT_EQ(defaulted, explicitely_defaulted);
  EXPECT_EQ(defaulted, explicitely_list_defaulted);

  T x{1, 2.0f};
  T y{1, 3.0f};
  T z{2, 0.0f};

  EXPECT_EQ(get<0>(x), 1);
  EXPECT_EQ(get<1>(x), 2.0f);
  EXPECT_EQ(get<int>(x), 1);
  EXPECT_EQ(get<float>(x), 2.0f);

  EXPECT_NE(x, y);
  EXPECT_LT(x, y);
  EXPECT_LT(x, z);
  EXPECT_LT(y, z);
}

TEST(tuple, structured_bindings) {
  using T = abu::tuple<int, char>;

  T x{1, 'a'};

  auto [a, b] = x;

  EXPECT_EQ(a, 1);
  EXPECT_EQ(b, 'a');
}