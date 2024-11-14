#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "serial::serial_lib" for configuration ""
set_property(TARGET serial::serial_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(serial::serial_lib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libserial_lib.so"
  IMPORTED_SONAME_NOCONFIG "libserial_lib.so"
  )

list(APPEND _cmake_import_check_targets serial::serial_lib )
list(APPEND _cmake_import_check_files_for_serial::serial_lib "${_IMPORT_PREFIX}/lib/libserial_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
