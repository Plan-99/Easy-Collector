
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was igris_sdk-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Note: Cyclone DDS (ddsc and ddscxx) are statically linked into libigris_sdk.a
# Thirdparty headers and libraries are provided for compiling against IDL-generated types

# Get SDK root directory
get_filename_component(IGRIS_SDK_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(IGRIS_SDK_ROOT "${IGRIS_SDK_CMAKE_DIR}/../../.." ABSOLUTE)

# Set thirdparty paths
set(IGRIS_SDK_THIRDPARTY_INCLUDE_DIR "${IGRIS_SDK_ROOT}/thirdparty/include")
set(IGRIS_SDK_THIRDPARTY_LIB_DIR "${IGRIS_SDK_ROOT}/thirdparty/lib")

# Find thirdparty libraries (Cyclone DDS statically linked dependencies)
find_library(DDSC_LIB NAMES ddsc libddsc PATHS "${IGRIS_SDK_THIRDPARTY_LIB_DIR}" NO_DEFAULT_PATH)
find_library(DDSCXX_LIB NAMES ddscxx libddscxx PATHS "${IGRIS_SDK_THIRDPARTY_LIB_DIR}" NO_DEFAULT_PATH)

# Find system libraries required by Cyclone DDS
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

# Create imported targets for thirdparty libraries
if(NOT TARGET ddsc)
    add_library(ddsc STATIC IMPORTED)
    set_target_properties(ddsc PROPERTIES
        IMPORTED_LOCATION "${DDSC_LIB}"
        INTERFACE_LINK_LIBRARIES "OpenSSL::SSL;OpenSSL::Crypto"
    )
endif()

if(NOT TARGET ddscxx)
    add_library(ddscxx STATIC IMPORTED)
    set_target_properties(ddscxx PROPERTIES IMPORTED_LOCATION "${DDSCXX_LIB}")
endif()

# Include the exported targets
include("${CMAKE_CURRENT_LIST_DIR}/igris_sdk-targets.cmake")

# Add thirdparty includes and link dependencies
if(TARGET igris_sdk::igris_sdk)
    target_include_directories(igris_sdk::igris_sdk INTERFACE
        "${IGRIS_SDK_ROOT}/include"
        "${IGRIS_SDK_THIRDPARTY_INCLUDE_DIR}"
        "${IGRIS_SDK_THIRDPARTY_INCLUDE_DIR}/ddscxx"
    )

    # Link Cyclone DDS libraries and system dependencies
    target_link_libraries(igris_sdk::igris_sdk INTERFACE
        ddscxx
        ddsc
        Threads::Threads
        dl
        rt
    )
endif()

check_required_components(igris_sdk)
