# SPDX-License-Identifier: LicenseRef-RWT-Hardware-Apache-2.0 OR GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Red Wire Technologies
# - Find libiio v1.x
#
#  LibIIO_FOUND        - system has libiio v1
#  LibIIO_INCLUDE_DIRS - directory containing iio/iio.h
#  LibIIO_LIBRARIES    - libraries to link
#  LibIIO_VERSION      - libiio version
#
# libiio v1 installs its header as <iio/iio.h>; v0.x installed <iio.h>
# and is not API compatible, so it is rejected here.

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_LibIIO QUIET libiio)
endif()

find_path(LibIIO_INCLUDE_DIR iio/iio.h
    HINTS ${PC_LibIIO_INCLUDEDIR} ${PC_LibIIO_INCLUDE_DIRS}
    PATHS /usr/local/include /opt/local/include)

find_library(LibIIO_LIBRARY NAMES iio libiio
    HINTS ${PC_LibIIO_LIBDIR} ${PC_LibIIO_LIBRARY_DIRS}
    PATHS /usr/local/lib /opt/local/lib)

# A v0.x libiio.pc (e.g. meta-oe's libiio 0.26 next to a v1 install) is ignored.
if(PC_LibIIO_VERSION AND PC_LibIIO_VERSION VERSION_LESS 1.0)
    message(STATUS "Ignoring libiio ${PC_LibIIO_VERSION} from pkg-config: libiio >= 1.0 is required")
    unset(PC_LibIIO_VERSION)
endif()

if(PC_LibIIO_VERSION)
    set(LibIIO_VERSION ${PC_LibIIO_VERSION})
elseif(LibIIO_INCLUDE_DIR)
    # No pkg-config: an <iio/iio.h> header is only shipped by v1.
    set(LibIIO_VERSION "1.0")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibIIO
    REQUIRED_VARS LibIIO_LIBRARY LibIIO_INCLUDE_DIR
    VERSION_VAR LibIIO_VERSION)

if(LibIIO_FOUND)
    set(LibIIO_INCLUDE_DIRS ${LibIIO_INCLUDE_DIR})
    set(LibIIO_LIBRARIES ${LibIIO_LIBRARY})
    if(NOT TARGET LibIIO::iio)
        add_library(LibIIO::iio UNKNOWN IMPORTED)
        set_target_properties(LibIIO::iio PROPERTIES
            IMPORTED_LOCATION "${LibIIO_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LibIIO_INCLUDE_DIR}")
    endif()
endif()

mark_as_advanced(LibIIO_INCLUDE_DIR LibIIO_LIBRARY)
