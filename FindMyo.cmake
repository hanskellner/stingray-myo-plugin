set(MYO_VERSION "0.9.0")
set(MYO_REVISION "")

# Check for root directory existence. Can be overwritten with environment variable SR_<LIBRARY NAME>_ROOT.
if( PLATFORM_WINDOWS )
	find_root(MYO_ROOT "${ENGINE_LIB_DIR}/myo-sdk-win-${MYO_VERSION}")
else()
	message(FATAL_ERROR "Unknown platform!")
endif()

# Set definitions
set(MYO_DEFINITIONS)

# Set directories
set(MYO_INCLUDE_DIRS "${MYO_ROOT}/include")
set(MYO_LIB_DIR "${MYO_ROOT}/lib")
set(MYO_BIN_DIR "${MYO_ROOT}/bin")

# Set libraries
set(VARIANT $<$<CONFIG:DEBUG>:Debug>$<$<NOT:$<CONFIG:DEBUG>>:Release>)
if( PLATFORM_WINDOWS )
	if( PLATFORM_64BIT )
		set(MYO_LIBRARIES "${MYO_LIB_DIR}/myo64.lib")
	else()
		set(MYO_LIBRARIES "${MYO_LIB_DIR}/myo32.lib")
	endif()
else()
	message(FATAL_ERROR "Unknown platform!")
endif()

# Set binaries
if( PLATFORM_WINDOWS )
	if( PLATFORM_64BIT )
		set(MYO_BINARIES "${MYO_BIN_DIR}/myo64.dll")
	else()
		set(MYO_BINARIES "${MYO_BIN_DIR}/myo32.dll")
	endif()
else()
	set(MYO_BINARIES)
endif()

# Check if package setup is successful
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Myo DEFAULT_MSG MYO_ROOT MYO_INCLUDE_DIRS MYO_LIBRARIES)
mark_as_advanced(MYO_INCLUDE_DIRS MYO_LIBRARIES)
