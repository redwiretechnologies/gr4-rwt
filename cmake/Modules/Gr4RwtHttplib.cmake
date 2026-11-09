# SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Red Wire Technologies
# cpp-httplib (header-only) for the RWT Hardware Manager: its CMake package where
# installed (httplib::httplib), otherwise httplib.h, made into the same imported target.
include_guard(GLOBAL)
if(NOT TARGET httplib::httplib)
  find_package(httplib CONFIG QUIET)
endif()
if(NOT TARGET httplib::httplib)
  find_package(Threads REQUIRED)
  find_path(GR4_RWT_HTTPLIB_INCLUDE_DIR httplib.h)
  if(NOT GR4_RWT_HTTPLIB_INCLUDE_DIR)
    message(FATAL_ERROR "cpp-httplib not found (httplib.h): it is needed for the RWT Hardware Manager")
  endif()
  add_library(httplib::httplib INTERFACE IMPORTED)
  set_target_properties(httplib::httplib PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GR4_RWT_HTTPLIB_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES Threads::Threads)
endif()
