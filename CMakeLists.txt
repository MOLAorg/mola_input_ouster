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

# Minimum CMake version required
cmake_minimum_required(VERSION 3.9)

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
# Find the Ouster SDK (ouster_client)
# The Ouster SDK can be installed via:
#   - System package manager
#   - vcpkg
#   - Building from source with `cmake --install`
#   - As a subdirectory via add_subdirectory() or FetchContent
# ---------------------------------------------------------------------------
find_package(OusterSDK REQUIRED COMPONENTS ouster_client ouster_pcap)

# If OusterSDK config is not found, fall back to manual find:
if(NOT TARGET OusterSDK::ouster_client)
    # Try pkg-config or a manual find path
    find_library(OUSTER_CLIENT_LIB NAMES ouster_client)
    find_library(OUSTER_PCAP_LIB NAMES ouster_pcap)
    find_path(OUSTER_CLIENT_INCLUDE_DIR NAMES ouster/client.h)
    if(OUSTER_CLIENT_LIB AND OUSTER_PCAP_LIB AND OUSTER_CLIENT_INCLUDE_DIR)
        add_library(OusterSDK::ouster_client UNKNOWN IMPORTED)
        set_target_properties(OusterSDK::ouster_client PROPERTIES
            IMPORTED_LOCATION "${OUSTER_CLIENT_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${OUSTER_CLIENT_INCLUDE_DIR}"
        )
        add_library(OusterSDK::ouster_pcap UNKNOWN IMPORTED)
        set_target_properties(OusterSDK::ouster_pcap PROPERTIES
            IMPORTED_LOCATION "${OUSTER_PCAP_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${OUSTER_CLIENT_INCLUDE_DIR}"
        )
    else()
        message(FATAL_ERROR
            "OusterSDK not found. Please install the Ouster SDK or set "
            "OusterSDK_DIR to its cmake config directory.\n"
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
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        mola::mola_kernel
    PRIVATE
        mola::mola_yaml
        mrpt::obs
        mrpt::maps
        OusterSDK::ouster_client
        OusterSDK::ouster_pcap
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
