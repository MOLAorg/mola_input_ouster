# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping (MOLA)
#
#                   https://github.com/MOLAorg/mola
#
# Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria, and
#                          individual contributors.
# SPDX-License-Identifier: GPL-3.0
# See LICENSE for full license information.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

# Tell CMake we'll use C++ for checkLibraryExists, etc.
if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(mola_input_ouster VERSION 0.1.0 LANGUAGES CXX)

# ---- Require C++17 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Find MOLA cmake helpers (mola_cmake_functions) and dependencies
# ---------------------------------------------------------------------------
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-maps REQUIRED)

# ---------------------------------------------------------------------------
# Find the Ouster SDK (ouster_client, ouster_pcap)
#
# Search order:
#   1. A system-installed or user-installed OusterSDK (OusterSDK_DIR hint).
#   2. The bundled git submodule at <source>/ouster-sdk (zero-install path).
#
# To force the bundled copy even when a system SDK is present, pass:
#   -DUSE_BUNDLED_OUSTER_SDK=ON
# ---------------------------------------------------------------------------
option(USE_BUNDLED_OUSTER_SDK
    "Always use the bundled ouster-sdk git submodule" OFF)

set(_OUSTER_SDK_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ouster-sdk")
set(_OUSTER_SDK_SUBMODULE_PRESENT
    EXISTS "${_OUSTER_SDK_SUBMODULE_DIR}/CMakeLists.txt")

if(NOT USE_BUNDLED_OUSTER_SDK)
    find_package(OusterSDK QUIET)
endif()

if(NOT OusterSDK_FOUND)
    if(${_OUSTER_SDK_SUBMODULE_PRESENT})
        message(STATUS
            "OusterSDK not found on system — using bundled submodule: "
            "${_OUSTER_SDK_SUBMODULE_DIR}")
        # Mirror the options used by ouster-ros so only what we need is built
        set(BUILD_VIZ     OFF CACHE BOOL "" FORCE)
        set(BUILD_OSF     ON  CACHE BOOL "" FORCE)
        set(BUILD_MAPPING OFF CACHE BOOL "" FORCE)
        set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(BUILD_TESTING  OFF CACHE BOOL "" FORCE)
        set(BUILD_PCAP    ON  CACHE BOOL "" FORCE)
        # Let FindOusterSDK.cmake (inside the submodule) drive add_subdirectory
        list(APPEND CMAKE_MODULE_PATH "${_OUSTER_SDK_SUBMODULE_DIR}/cmake")
        find_package(OusterSDK REQUIRED)

        # Explicit include paths for the bundled submodule.
        # ALIAS targets built in a subdirectory do not always propagate
        # BUILD_INTERFACE include dirs to consumers in the parent scope,
        # so we list them directly here.
        set(_OUSTER_SDK_INCLUDE_DIRS
            "${_OUSTER_SDK_SUBMODULE_DIR}/ouster_sensor/include"
            "${_OUSTER_SDK_SUBMODULE_DIR}/ouster_client/include"
            "${_OUSTER_SDK_SUBMODULE_DIR}/ouster_client/include/optional-lite"
            "${_OUSTER_SDK_SUBMODULE_DIR}/ouster_pcap/include"
            "${_OUSTER_SDK_SUBMODULE_DIR}/ouster_osf/include"
        )

        # Workaround: some versions of libtins (system package) omit
        # '#include <cstdint>' in ip_address.h, causing 'std::uint32_t'
        # errors when building ouster_pcap against them.  Force-include
        # the header for every TU that belongs to the ouster_pcap target.
        if(TARGET ouster_pcap)
            target_compile_options(ouster_pcap PRIVATE
                $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-include;cstdint>)
        endif()
    else()
        message(FATAL_ERROR
            "OusterSDK not found and the bundled submodule is missing.\n"
            "Either install the Ouster SDK system-wide, set OusterSDK_DIR,\n"
            "or initialise the submodule:\n"
            "  git submodule update --init --recursive\n"
            "See: https://github.com/ouster-lidar/ouster-sdk")
    endif()
endif()

# ---------------------------------------------------------------------------
# Declare the library
# ---------------------------------------------------------------------------
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

add_library(${PROJECT_NAME} SHARED ${LIB_SRCS} ${LIB_PUBLIC_HDRS})

# Public include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${_OUSTER_SDK_INCLUDE_DIRS}   # populated when using the bundled submodule
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        mola::mola_kernel
    PRIVATE
        mola::mola_yaml
        mrpt::obs
        mrpt::maps
        # ouster_sensor was split out in SDK >= 0.16; present in the bundled submodule
        $<TARGET_NAME_IF_EXISTS:OusterSDK::ouster_sensor>
        $<TARGET_NAME_IF_EXISTS:OusterSDK::ouster_sensor>
        OusterSDK::ouster_client
        OusterSDK::ouster_pcap
        OusterSDK::ouster_osf
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install mola-cli launch YAML files
install(DIRECTORY mola-cli-launchs/
    DESTINATION share/${PROJECT_NAME}/mola-cli-launchs
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# Export targets
install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE mola::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
