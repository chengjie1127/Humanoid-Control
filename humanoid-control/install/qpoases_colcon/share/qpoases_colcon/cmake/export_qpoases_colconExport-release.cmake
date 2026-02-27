#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qpoases_colcon::qpOASES" for configuration "Release"
set_property(TARGET qpoases_colcon::qpOASES APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(qpoases_colcon::qpOASES PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libqpOASES.so.3.2"
  IMPORTED_SONAME_RELEASE "libqpOASES.so.3.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS qpoases_colcon::qpOASES )
list(APPEND _IMPORT_CHECK_FILES_FOR_qpoases_colcon::qpOASES "${_IMPORT_PREFIX}/lib/libqpOASES.so.3.2" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
