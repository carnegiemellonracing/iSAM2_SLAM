#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "eufs_rviz_plugins::eufs_rviz_plugins" for configuration ""
set_property(TARGET eufs_rviz_plugins::eufs_rviz_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(eufs_rviz_plugins::eufs_rviz_plugins PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libeufs_rviz_plugins.so"
  IMPORTED_SONAME_NOCONFIG "libeufs_rviz_plugins.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS eufs_rviz_plugins::eufs_rviz_plugins )
list(APPEND _IMPORT_CHECK_FILES_FOR_eufs_rviz_plugins::eufs_rviz_plugins "${_IMPORT_PREFIX}/lib/libeufs_rviz_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
