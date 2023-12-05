#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "eufs_models::eufs_models" for configuration ""
set_property(TARGET eufs_models::eufs_models APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(eufs_models::eufs_models PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libeufs_models.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS eufs_models::eufs_models )
list(APPEND _IMPORT_CHECK_FILES_FOR_eufs_models::eufs_models "${_IMPORT_PREFIX}/lib/libeufs_models.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
