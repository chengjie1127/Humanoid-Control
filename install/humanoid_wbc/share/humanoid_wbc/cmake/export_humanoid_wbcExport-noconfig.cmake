#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "humanoid_wbc::humanoid_wbc" for configuration ""
set_property(TARGET humanoid_wbc::humanoid_wbc APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(humanoid_wbc::humanoid_wbc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhumanoid_wbc.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS humanoid_wbc::humanoid_wbc )
list(APPEND _IMPORT_CHECK_FILES_FOR_humanoid_wbc::humanoid_wbc "${_IMPORT_PREFIX}/lib/libhumanoid_wbc.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
