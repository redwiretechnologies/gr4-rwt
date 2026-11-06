# SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Red Wire Technologies
include_guard(GLOBAL)

# libad9361 is optional: it loads the AD9361 FIR filters that baseband rates
# below 2.083 MS/s need. Released libad9361 versions are built on libiio 0.x and
# cannot be loaded next to libiio v1; only its libiio v1 port can.
set(ENABLE_LIBAD9361 "AUTO" CACHE STRING "Use libad9361 for AD9361 FIR loading (AUTO, ON, OFF)")
set_property(CACHE ENABLE_LIBAD9361 PROPERTY STRINGS AUTO ON OFF)

function(gr4_rwt_resolve_dependencies)
  find_package(gnuradio4 CONFIG REQUIRED)
  find_package(GnuRadioBlockLib CONFIG REQUIRED)
  # GnuRadioBlockLibConfig.cmake sets PARSER_EXECUTABLE (path to the installed
  # gnuradio_4_0_parse_registrations); it must escape this function's scope or
  # the blocklib macros fall back to a gnuradio4 build-tree path.
  set(PARSER_EXECUTABLE "${PARSER_EXECUTABLE}" PARENT_SCOPE)
  # GnuRadioBlockLibMacros.cmake looks up Python3 while it is included here, but
  # gr_add_block_library() runs the Python registration generator later, from
  # the caller's scope. Without these the generator is run without an
  # interpreter and the configure fails with an empty error message.
  set(Python3_EXECUTABLE "${Python3_EXECUTABLE}" PARENT_SCOPE)
  set(Python3_Interpreter_FOUND "${Python3_Interpreter_FOUND}" PARENT_SCOPE)
  set(
    GR4_OOT_GNURADIO4_TARGET
    "gnuradio4::gnuradio-core;gnuradio4::gnuradio-blocklib-core"
    PARENT_SCOPE
  )

  # --- libiio v1 (required) -------------------------------------------------
  find_package(LibIIO 1.0 REQUIRED)
  message(STATUS "libiio ${LibIIO_VERSION}: ${LibIIO_LIBRARIES}")

  # --- cpp-httplib (required): the Oxygen management API ---------------------
  include(Gr4RwtHttplib)
  message(STATUS "cpp-httplib: httplib::httplib")

  # --- libad9361 (optional, libiio v1 build only) ----------------------------
  set(_use_ad9361 OFF)
  if(NOT ENABLE_LIBAD9361 STREQUAL "OFF")
    find_package(LibAD9361 QUIET)
    if(LibAD9361_FOUND)
      # ad9361.h does not say which libiio it was built for; its NEEDED entries do.
      get_filename_component(_ad9361_real "${LibAD9361_LIBRARIES}" REALPATH)
      find_program(_readelf NAMES readelf ${CMAKE_READELF})
      set(_ad9361_iio "unknown")
      if(_readelf AND EXISTS "${_ad9361_real}")
        execute_process(COMMAND ${_readelf} -d "${_ad9361_real}" OUTPUT_VARIABLE _dyn ERROR_QUIET)
        if(_dyn MATCHES "libiio\\.so\\.1")
          set(_ad9361_iio "v1")
        elseif(_dyn MATCHES "libiio\\.so\\.0")
          set(_ad9361_iio "v0")
        endif()
      endif()
      message(STATUS "libad9361: ${LibAD9361_LIBRARIES} (libiio ${_ad9361_iio})")
      if(_ad9361_iio STREQUAL "v1" OR (ENABLE_LIBAD9361 STREQUAL "ON" AND NOT _ad9361_iio STREQUAL "v0"))
        set(_use_ad9361 ON)
      elseif(_ad9361_iio STREQUAL "v0")
        message(WARNING "libad9361 at ${LibAD9361_LIBRARIES} is built on libiio 0.x and cannot be used with "
                        "libiio v1; build its libiio v1 port to get FIR loading")
      endif()
    endif()
    if(ENABLE_LIBAD9361 STREQUAL "ON" AND NOT _use_ad9361)
      message(FATAL_ERROR "ENABLE_LIBAD9361=ON but no libiio v1 compatible libad9361 was found")
    endif()
  endif()
  if(_use_ad9361 AND NOT TARGET LibAD9361::ad9361)
    add_library(LibAD9361::ad9361 UNKNOWN IMPORTED)
    set_target_properties(LibAD9361::ad9361 PROPERTIES
      IMPORTED_LOCATION "${LibAD9361_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${LibAD9361_INCLUDE_DIRS}"
      INTERFACE_COMPILE_DEFINITIONS "GR4_RWT_HAS_AD9361=1")
  endif()
  set(GR4_RWT_USE_AD9361 ${_use_ad9361} PARENT_SCOPE)
  if(_use_ad9361)
    message(STATUS "Using libad9361 for AD9361 FIR loading")
  else()
    message(STATUS "Not using libad9361: AD9361 rates below 2.083 MS/s get no FIR, and the RX source "
                   "starts the FPGA CIC decimator below 2.083 MS/s instead of 600 kS/s")
  endif()

  if(ENABLE_TESTING)
    find_package(boost_ut CONFIG QUIET)
    if(TARGET boost_ut::ut)
      set(GR4_OOT_BOOST_UT_TARGET boost_ut::ut PARENT_SCOPE)
    else()
      find_path(_boost_ut_include boost/ut.hpp PATH_SUFFIXES boost-ut/include include)
      if(NOT _boost_ut_include)
        message(FATAL_ERROR "boost-ut headers not found (boost/ut.hpp). Install system boost-ut.")
      endif()
      add_library(gr4_rwt_boost_ut INTERFACE)
      target_include_directories(gr4_rwt_boost_ut INTERFACE "${_boost_ut_include}")
      add_library(gr4_rwt::boost_ut ALIAS gr4_rwt_boost_ut)
      set(GR4_OOT_BOOST_UT_TARGET gr4_rwt::boost_ut PARENT_SCOPE)
    endif()
  endif()
endfunction()

function(gr4_modtool_add_ut_test target_name source_file)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name} PRIVATE ${GR4_OOT_GNURADIO4_TARGET} ${GR4_OOT_BOOST_UT_TARGET})
  add_test(NAME ${target_name} COMMAND ${target_name})
endfunction()
