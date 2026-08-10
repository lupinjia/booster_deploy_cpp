#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "data_tamer::data_tamer" for configuration "Release"
set_property(TARGET data_tamer::data_tamer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(data_tamer::data_tamer PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdata_tamer.a"
  )

list(APPEND _cmake_import_check_targets data_tamer::data_tamer )
list(APPEND _cmake_import_check_files_for_data_tamer::data_tamer "${_IMPORT_PREFIX}/lib/libdata_tamer.a" )

# Import target "data_tamer::mcap_lib" for configuration "Release"
set_property(TARGET data_tamer::mcap_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(data_tamer::mcap_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmcap_lib.a"
  )

list(APPEND _cmake_import_check_targets data_tamer::mcap_lib )
list(APPEND _cmake_import_check_files_for_data_tamer::mcap_lib "${_IMPORT_PREFIX}/lib/libmcap_lib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
