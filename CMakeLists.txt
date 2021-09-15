cmake_minimum_required(VERSION 3.16)
project(abu_types VERSION 0.1.0)

include(.abu/build-utils/abu_lib.cmake)

# abu::types
add_library(abu_types INTERFACE)
target_compile_features(abu_types INTERFACE cxx_std_20)
target_include_directories(abu_types INTERFACE include)
add_library(abu::types ALIAS abu_types)
add_library(abu::checked::types ALIAS abu_types)

abu_announce(types)

if(ABU_TYPES_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

