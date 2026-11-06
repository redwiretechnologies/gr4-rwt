# SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Red Wire Technologies
# Tests for rwt blocks
#
# qa_RWTSource, qa_RWTSink and qa_Oxygen run against libiio v1's emulated
# AD9361 (emu backend, libiio's FMCOMMS2 test description) and skip those
# cases when libiio was built without WITH_EMU_BACKEND.
#
# The paths compiled into the tests can be overridden when the tests are
# installed elsewhere (e.g. as a Yocto ptest package on the target).
set(GR4_RWT_TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data" CACHE PATH
  "Directory holding fmcomms2.xml, as seen by the test binaries")

gr4_modtool_add_ut_test(qa_RWTSource qa_RWTSource.cpp)
target_link_libraries(qa_RWTSource PRIVATE gr4_rwt::blocks_rwt_headers)
target_compile_definitions(qa_RWTSource PRIVATE GR4_RWT_TEST_DATA_DIR="${GR4_RWT_TEST_DATA_DIR}")
gr4_modtool_add_ut_test(qa_RWTSink qa_RWTSink.cpp)
target_link_libraries(qa_RWTSink PRIVATE gr4_rwt::blocks_rwt_headers)
target_compile_definitions(qa_RWTSink PRIVATE GR4_RWT_TEST_DATA_DIR="${GR4_RWT_TEST_DATA_DIR}")
gr4_modtool_add_ut_test(qa_Oxygen qa_Oxygen.cpp)
target_link_libraries(qa_Oxygen PRIVATE gr4_rwt::blocks_rwt_headers)
target_compile_definitions(qa_Oxygen PRIVATE GR4_RWT_TEST_DATA_DIR="${GR4_RWT_TEST_DATA_DIR}")

# settings sent as messages to running blocks, as gnuradio4-control-plane sends them
gr4_modtool_add_ut_test(qa_LiveSettings qa_LiveSettings.cpp)
target_link_libraries(qa_LiveSettings PRIVATE gr4_rwt::blocks_rwt_headers)
target_compile_definitions(qa_LiveSettings PRIVATE GR4_RWT_TEST_DATA_DIR="${GR4_RWT_TEST_DATA_DIR}")

# the Oxygen management API (connection_control), against a mock of its HTTP server
gr4_modtool_add_ut_test(qa_RadioApi qa_RadioApi.cpp)
target_link_libraries(qa_RadioApi PRIVATE gr4_rwt::blocks_rwt_headers)
target_compile_definitions(qa_RadioApi PRIVATE GR4_RWT_TEST_DATA_DIR="${GR4_RWT_TEST_DATA_DIR}")

# loads the compiled block library and creates the blocks through the registry
gr4_modtool_add_ut_test(qa_Registry qa_Registry.cpp)
target_link_libraries(qa_Registry PRIVATE gr4_rwt::blocks_rwt_headers GrGr4RwtRwtBlocksShared)

# runs examples/oxygen_repeater.grc with GR4's rungraph tool against the emulator
get_filename_component(_gr4_prefix "${gnuradio4_DIR}/../../.." ABSOLUTE)
option(GR4_RWT_TEST_EXAMPLE "Test examples/oxygen_repeater.grc with rungraph" ON)
find_program(GR4_RUNGRAPH NAMES rungraph gr4-rungraph HINTS "${_gr4_prefix}/bin")
set(GR4_RWT_TEST_EXAMPLE_FILE "${PROJECT_SOURCE_DIR}/examples/oxygen_repeater.grc" CACHE FILEPATH
  "oxygen_repeater.grc, as seen by qa_Example")
set(GR4_RWT_TEST_PLUGIN_DIR "" CACHE PATH
  "Directory of the block library, as seen by qa_Example (default: the build tree)")
if(GR4_RWT_TEST_PLUGIN_DIR)
  set(_rwt_plugin_dir "${GR4_RWT_TEST_PLUGIN_DIR}")
else()
  set(_rwt_plugin_dir "$<TARGET_FILE_DIR:GrGr4RwtRwtBlocksShared>")
endif()
if(GR4_RWT_TEST_EXAMPLE AND GR4_RUNGRAPH)
  gr4_modtool_add_ut_test(qa_Example qa_Example.cpp)
  target_link_libraries(qa_Example PRIVATE gr4_rwt::blocks_rwt_headers)
  add_dependencies(qa_Example GrGr4RwtRwtBlocksShared)
  target_compile_definitions(qa_Example PRIVATE
    GR4_RWT_TEST_DATA_DIR="${GR4_RWT_TEST_DATA_DIR}"
    GR4_RWT_RUNGRAPH="${GR4_RUNGRAPH}"
    GR4_RWT_EXAMPLE="${GR4_RWT_TEST_EXAMPLE_FILE}"
    GR4_RWT_PLUGIN_DIR="${_rwt_plugin_dir}")
elseif(GR4_RWT_TEST_EXAMPLE)
  message(STATUS "rungraph not found: the example graph is not tested")
endif()
