add_executable(abu_types_tests
    test_tuple.cpp
)
target_link_libraries(abu_types_tests PRIVATE abu::checked::types)
abu_configure_test_target(abu_types_tests)
add_test(abu_types_tests abu_types_tests)

