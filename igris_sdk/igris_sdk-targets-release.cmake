#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "igris_sdk::igris_sdk" for configuration "Release"
set_property(TARGET igris_sdk::igris_sdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(igris_sdk::igris_sdk PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libigris_sdk.a"
  )

list(APPEND _cmake_import_check_targets igris_sdk::igris_sdk )
list(APPEND _cmake_import_check_files_for_igris_sdk::igris_sdk "${_IMPORT_PREFIX}/lib/libigris_sdk.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
