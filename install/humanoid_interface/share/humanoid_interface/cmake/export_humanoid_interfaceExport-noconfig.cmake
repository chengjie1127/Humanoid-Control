#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "humanoid_interface::humanoid_interface" for configuration ""
set_property(TARGET humanoid_interface::humanoid_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(humanoid_interface::humanoid_interface PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhumanoid_interface.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS humanoid_interface::humanoid_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_humanoid_interface::humanoid_interface "${_IMPORT_PREFIX}/lib/libhumanoid_interface.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
