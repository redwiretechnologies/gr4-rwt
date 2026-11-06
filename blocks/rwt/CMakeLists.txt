# SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Red Wire Technologies
add_library(gr4_rwt_blocks_rwt_headers INTERFACE)
add_library(gr4_rwt::blocks_rwt_headers ALIAS gr4_rwt_blocks_rwt_headers)

target_include_directories(gr4_rwt_blocks_rwt_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(gr4_rwt_blocks_rwt_headers INTERFACE cxx_std_23)
target_link_libraries(gr4_rwt_blocks_rwt_headers INTERFACE ${GR4_OOT_GNURADIO4_TARGET} LibIIO::iio httplib::httplib
  $<$<BOOL:${GR4_RWT_USE_AD9361}>:LibAD9361::ad9361>)
# consumers of the installed package use the same name as this build tree
set_target_properties(gr4_rwt_blocks_rwt_headers PROPERTIES EXPORT_NAME blocks_rwt_headers)
install(TARGETS gr4_rwt_blocks_rwt_headers EXPORT gr4_rwtTargets)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/rwt
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0)

file(GLOB gr4_rwt_rwt_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/rwt/*.hpp"
)
if(gr4_rwt_rwt_HEADERS)
  gr_add_block_library(
    GrGr4RwtRwtBlocks
    MAKE_SHARED_LIBRARY
    HEADERS
      ${gr4_rwt_rwt_HEADERS}
    LINK_LIBRARIES
      gr4_rwt_blocks_rwt_headers
  )
endif()

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
