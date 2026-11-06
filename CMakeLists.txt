# SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Red Wire Technologies
cmake_minimum_required(VERSION 3.22)

project(gr4-rwt LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(ENABLE_TESTING "Enable test targets" ON)
option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

include(cmake/Dependencies.cmake)
gr4_rwt_resolve_dependencies()

if(ENABLE_TESTING)
  include(CTest)
  enable_testing()
endif()

if(WARNINGS_AS_ERRORS)
  if(MSVC)
    add_compile_options(/WX)
  else()
    add_compile_options(-Werror)
  endif()
endif()

add_subdirectory(blocks)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gr4_rwtConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gr4_rwtConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gr4_rwt"
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gr4_rwtConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)
install(
  EXPORT gr4_rwtTargets
  FILE gr4_rwtTargets.cmake
  NAMESPACE gr4_rwt::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gr4_rwt"
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/gr4_rwtConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/gr4_rwtConfigVersion.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/FindLibIIO.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/FindLibAD9361.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/Gr4RwtHttplib.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gr4_rwt"
)
