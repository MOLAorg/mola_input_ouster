# Tests for mola_input_ouster

# Unit test: module registration, YAML validation, helpers
add_executable(test-ouster-direct-input test-ouster-direct-input.cpp)

target_link_libraries(test-ouster-direct-input
    PRIVATE
        ${PROJECT_NAME}
        mola::mola_kernel
        mrpt::obs
)

add_test(NAME test-ouster-direct-input COMMAND test-ouster-direct-input)
