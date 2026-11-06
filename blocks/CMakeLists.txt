# SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Red Wire Technologies
add_subdirectory(rwt)

add_library(gr4_rwt_blocks_headers INTERFACE)
add_library(gr4_rwt::blocks_headers ALIAS gr4_rwt_blocks_headers)

target_link_libraries(gr4_rwt_blocks_headers INTERFACE
  gr4_rwt::blocks_rwt_headers
)
set_target_properties(gr4_rwt_blocks_headers PROPERTIES EXPORT_NAME blocks_headers)
install(TARGETS gr4_rwt_blocks_headers EXPORT gr4_rwtTargets)
