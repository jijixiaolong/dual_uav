cmake_minimum_required(VERSION 3.16)
project(docking_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(include)

# Core Static Library (pure C++17, no external dependencies)
add_library(docking_core STATIC
    src/geometric_se3.cpp
    src/allocator.cpp
    src/fsm.cpp
    src/compensator.cpp
    src/research_compensation.cpp
    src/safety.cpp
)

# C++ Test Executable
add_executable(docking_core_tests tests/test_core.cpp)
target_link_libraries(docking_core_tests PRIVATE docking_core)
# These tests use assert, which must remain enabled in Release builds too.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(docking_core_tests PRIVATE -UNDEBUG)
endif()
enable_testing()
add_test(NAME docking_core_tests COMMAND docking_core_tests)

# Research compensation is tested separately from the validated nominal core.
add_executable(docking_research_compensation_tests tests/test_research_compensation.cpp)
target_link_libraries(docking_research_compensation_tests PRIVATE docking_core)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(docking_research_compensation_tests PRIVATE -UNDEBUG)
endif()
add_test(NAME docking_research_compensation_tests COMMAND docking_research_compensation_tests)

# pybind11 Python Module
find_package(pybind11 QUIET)
if (NOT pybind11_FOUND)
    # Fallback to python3 execution check
    execute_process(
        COMMAND python3 -m pybind11 --includes
        OUTPUT_VARIABLE PYBIND11_INCLUDES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (PYBIND11_INCLUDES)
        separate_arguments(PYBIND11_INC_LIST UNIX_COMMAND "${PYBIND11_INCLUDES}")
        add_library(docking_core_py MODULE src/bindings.cpp)
        target_link_libraries(docking_core_py PRIVATE docking_core)
        target_compile_options(docking_core_py PRIVATE ${PYBIND11_INC_LIST})
        set_target_properties(docking_core_py PROPERTIES PREFIX "" SUFFIX ".so")
    endif()
else()
    pybind11_add_module(docking_core_py src/bindings.cpp)
    target_link_libraries(docking_core_py PRIVATE docking_core)
endif()
